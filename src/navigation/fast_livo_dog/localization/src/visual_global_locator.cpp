#include <cv_bridge/cv_bridge.h>
#ifdef ROS_HUMBLE
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#endif

#include <deque>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <Eigen/Dense>
#include <vector>
#include <atomic>
#include <opencv2/core.hpp>
#include <DBoW3/DBoW3.h>

struct HistoricalPose {
    double x, y, z, roll, pitch, yaw;
};

class VisualGlobalLocator : public rclcpp::Node {
public:
    VisualGlobalLocator() : Node("visual_global_locator"), is_backend_localized_(false) {
        cv::setNumThreads(4);
        this->declare_parameter<std::string>("visual_db_path", "./src/navigation/fast_livo_dog/data/visual_descriptors.bin");
        this->declare_parameter<std::string>("pose_db_path", "./src/navigation/fast_livo_dog/data/sc_database.txt");
        this->declare_parameter<std::string>("voc_path", "./src/navigation/fast_livo_dog/data/orb_vocabulary.dbow3");
        this->declare_parameter<int>("min_good_matches", 20);
        this->declare_parameter<int>("common.img_en", 1);

        std::string visual_db = this->get_parameter("visual_db_path").as_string();
        std::string pose_db = this->get_parameter("pose_db_path").as_string();
        std::string voc_path = this->get_parameter("voc_path").as_string();
        min_good_matches_ = this->get_parameter("min_good_matches").as_int();
        int img_en_int = 1;
        this->get_parameter("common.img_en", img_en_int);
        img_en_ = (img_en_int != 0);

        if (!img_en_) {
            RCLCPP_WARN(this->get_logger(), ">>> [GLOBAL RELOCALIZATION]   : Visual Global Locator DISABLED by common.img_en=0 (Pure LIO mode).");
            return;
        }

        pub_initial_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/initialpose_visual", 1);
        pub_fail_count_ = this->create_publisher<std_msgs::msg::Int32>("/visual_fail_count", 10);

        sub_status_ = this->create_subscription<std_msgs::msg::Bool>(
            "/localization_status", 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                bool was_localized = this->is_backend_localized_;
                this->is_backend_localized_ = msg->data;
                if (!this->is_backend_localized_ && was_localized) {
                    this->recent_visual_ids_.clear();  // 唤醒时清空历史匹配缓存
                    RCLCPP_INFO(this->get_logger(), "Backend lost tracking. Visual module awakened.");
                }
            });

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->current_odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            });

        sub_radius_ = this->create_subscription<std_msgs::msg::Float64>(
            "/global_localization/adaptive_search_radius", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->active_search_radius_.store(msg->data);
            });

        if (loadDatabases(visual_db, pose_db, voc_path)) {
            RCLCPP_INFO(this->get_logger(),
                        "Visual database and BoW loaded successfully. Waiting for incoming image topics.");
            sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/rgb_img", 10, std::bind(&VisualGlobalLocator::imageCallback, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load visual, pose, or vocabulary database files.");
        }

        orb_ = cv::ORB::create(ORB_FEATURES_LIMIT_);
        clahe_ = cv::createCLAHE(CLAHE_CLIP_LIMIT_, cv::Size(CLAHE_GRID_SIZE_, CLAHE_GRID_SIZE_));
    }

private:
    // --- 视觉匹配与物理学连续性检验配置 ---
    const int REQUIRED_CONSISTENT_FRAMES_ = 3;  // 触发重定位所需的连续一致帧数量下限
    const double MAX_SPATIAL_JUMP_M_ = 3.0;     // 允许的最大物理空间欧氏距离跳变约束 (米)
    const int ORB_FEATURES_LIMIT_ = 500;        // 图像中 ORB 特征点提取的最大点数限制
    const double ORB_RATIO_TEST_ = 0.75;
    const double VISUAL_AMBIGUITY_DIST_M_ = 2.5;
    const double VISUAL_AMBIGUITY_RATIO_ = 0.75;
    const int VISUAL_AMBIGUITY_DIFF_ = 5;        // KNN 描述子粗匹配结果 of Lowe's Ratio Test
    const double CLAHE_CLIP_LIMIT_ = 3.0;       // CLAHE clip limit
    const int CLAHE_GRID_SIZE_ = 8;             // CLAHE grid size

    // 加载相机帧描述子字典与对应的物理坐标先验关联库，以及词袋向量
    bool loadDatabases(const std::string& visual_path, const std::string& pose_path, const std::string& voc_path) {
        std::ifstream ifs(pose_path);
        if (!ifs.is_open()) return false;

        int num_frames_pose;
        if (!(ifs >> num_frames_pose)) return false;
        for (int i = 0; i < num_frames_pose; ++i) {
            int id;
            HistoricalPose p;
            ifs >> id >> p.x >> p.y >> p.z >> p.roll >> p.pitch >> p.yaw;
            db_poses_.push_back(p);

            int rows, cols;
            ifs >> rows >> cols;
            double dummy;
            for (int k = 0; k < rows * cols; ++k) ifs >> dummy;
        }
        ifs.close();

        // 1. 加载标准字典
        voc_.load(voc_path);
        if (voc_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load vocabulary from: %s", voc_path.c_str());
            return false;
        }
        db_.setVocabulary(voc_, true, 0);

        std::ifstream ifs_vis(visual_path, std::ios::binary);
        if (!ifs_vis.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open visual_db: %s", visual_path.c_str());
            return false;
        }

        int num_frames_visual;
        ifs_vis.read(reinterpret_cast<char*>(&num_frames_visual), sizeof(int));
        int loaded_bow_count = 0;

        for (int i = 0; i < num_frames_visual; ++i) {
            int rows, cols, type;
            ifs_vis.read(reinterpret_cast<char*>(&rows), sizeof(int));
            ifs_vis.read(reinterpret_cast<char*>(&cols), sizeof(int));
            ifs_vis.read(reinterpret_cast<char*>(&type), sizeof(int));

            cv::Mat desc;
            if (rows > 0 && cols > 0) {
                desc.create(rows, cols, type);
                size_t data_size = rows * cols * desc.elemSize();
                ifs_vis.read(reinterpret_cast<char*>(desc.data), data_size);
            }
            db_descriptors_.push_back(desc);

            DBoW3::BowVector bow_vec;
            if (!desc.empty()) {
                voc_.transform(desc, bow_vec);
                loaded_bow_count++;
            }
            db_.add(bow_vec);
        }
        ifs_vis.close();

        RCLCPP_INFO(this->get_logger(),
                    "Poses, descriptors and BoW database loaded (Poses: %zu, Descriptors: %zu, BoW Vectors: %d/%d).",
                    db_poses_.size(), db_descriptors_.size(), loaded_bow_count, num_frames_visual);
        return !db_descriptors_.empty();
    }

    // 图像帧的回调，进行特征检测匹配以及空间连续性的严格校验
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!img_en_) return;
        // 核心性能优化：如果后端已经定位成功，视觉节点直接休眠，跳过昂贵的 ORB 提取
        if (is_backend_localized_) {
            return;
        }

        static std::atomic<bool> is_searching{false};
        if (is_searching.load()) {
            return;
        }
        is_searching.store(true);

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            is_searching.store(false);
            return;
        }

        cv::Mat gray, clahe_img;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        clahe_->apply(gray, clahe_img);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_->detectAndCompute(clahe_img, cv::noArray(), keypoints, descriptors);

        if (descriptors.empty()) {
            is_searching.store(false);
            return;
        }

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        int best_id = -1;
        int max_good = 0;

        // 第一阶段（BoW 闪电战）：将当前图像描述子转化为 BowVector，并在词袋中查询最相似的 Top 100 候选帧
        DBoW3::BowVector current_bow;
        voc_.transform(descriptors, current_bow);

        DBoW3::QueryResults ret_matches;
        db_.query(current_bow, ret_matches, 100);

        // 第二阶段（局部精准死磕）：仅针对这 100 个候选帧进行精确的 Knn 匹配
        std::vector<std::pair<int, int>> candidate_scores; // <good_cnt, id>

        for (const auto& match : ret_matches) {
            int i = match.Id;
            if (i < 0 || i >= static_cast<int>(db_descriptors_.size())) continue;
            if (db_descriptors_[i].empty()) continue;

            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher.knnMatch(descriptors, db_descriptors_[i], knn_matches, 2);

            int good_cnt = 0;
            for (size_t m = 0; m < knn_matches.size(); m++) {
                if (knn_matches[m].size() < 2) continue;
                if (knn_matches[m][0].distance < ORB_RATIO_TEST_ * knn_matches[m][1].distance) {
                    good_cnt++;
                }
            }

            candidate_scores.push_back({good_cnt, i});

            if (good_cnt > max_good) {
                max_good = good_cnt;
                best_id = i;
            }
        }

        // 排序并打印 Top 10 内点数
        std::sort(candidate_scores.begin(), candidate_scores.end(), 
                  [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                      return a.first > b.first;
                  });



        // --- 视觉歧义性排斥 (Ambiguity Rejection) ---
        int ambiguous_good = -1;
        if (!candidate_scores.empty() && candidate_scores[0].first > 0) {
            HistoricalPose best_pose = db_poses_[candidate_scores[0].second];
            for (size_t k = 1; k < candidate_scores.size(); ++k) {
                HistoricalPose p = db_poses_[candidate_scores[k].second];
                double dx = p.x - best_pose.x;
                double dy = p.y - best_pose.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > VISUAL_AMBIGUITY_DIST_M_) {
                    ambiguous_good = candidate_scores[k].first;
                    break;
                }
            }
        }

        bool is_ambiguous = false;
        if (ambiguous_good != -1 && max_good > 0) {
            double ratio = (double)ambiguous_good / (double)max_good;
            int diff = max_good - ambiguous_good;
            if (ratio > VISUAL_AMBIGUITY_RATIO_ || diff < VISUAL_AMBIGUITY_DIFF_) {
                is_ambiguous = true;
                RCLCPP_DEBUG(this->get_logger(),
                             "Ambiguity Rejected! Best: %d, Ambiguous: %d. High risk of mismatch.",
                             max_good, ambiguous_good);
            }
        }

        static rclcpp::Time last_success_time = this->now();
        if (max_good <= min_good_matches_ || is_ambiguous) {
            consecutive_bow_failures_++;
            // 避免单帧模糊导致队列清空：只有连续 2 秒没有匹配上，才清空历史缓存
            if ((this->now() - last_success_time).seconds() > 2.0) {
                recent_visual_ids_.clear();
            }
            if (!is_ambiguous) {
                RCLCPP_DEBUG(this->get_logger(),
                             "DBoW3 candidate matching failed. Max Good: %d, Threshold: %d.",
                             max_good, min_good_matches_);
            }
        } else {
            last_success_time = this->now();

            // Throttle publishing to 1Hz maximum
            static rclcpp::Time last_pushed_time = this->now();
            if ((this->now() - last_pushed_time).seconds() >= 1.0) {
                last_pushed_time = this->now();

                consecutive_bow_failures_ = 0; // 真正通过了初筛，清零计数器！
                RCLCPP_INFO(this->get_logger(),
                            "Visual strict anchor verified (Best: %d, Ambiguous: %d). Forwarding proposal to backend.",
                            max_good, ambiguous_good);

                HistoricalPose p = db_poses_[best_id];
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = msg->header.stamp;
                pose_array_msg.header.frame_id = "map";

                geometry_msgs::msg::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = p.z;

                tf2::Quaternion q;
                q.setRPY(p.roll, p.pitch, p.yaw);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();

                pose_array_msg.poses.push_back(pose);
                pub_initial_pose_array_->publish(pose_array_msg);
            }
    }
        
        std_msgs::msg::Int32 count_msg;
        count_msg.data = consecutive_bow_failures_;
        pub_fail_count_->publish(count_msg);
        is_searching.store(false);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_status_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_initial_pose_array_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_fail_count_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_radius_;
    std::optional<Eigen::Vector3d> current_odom_pos_;
    std::atomic<double> active_search_radius_{-1.0};

    std::vector<HistoricalPose> db_poses_;
    std::vector<cv::Mat> db_descriptors_;

    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::CLAHE> clahe_;

    int min_good_matches_;
    std::deque<int> recent_visual_ids_;
    bool is_backend_localized_;

    DBoW3::Vocabulary voc_;
    DBoW3::Database db_;
    int consecutive_bow_failures_ = 0;
    bool img_en_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualGlobalLocator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
