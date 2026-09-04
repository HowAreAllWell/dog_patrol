#include <cv_bridge/cv_bridge.h>
#include <DBoW3/DBoW3.h>
#include <future>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#else
#include <opencv2/aruco.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>

class VisualLoopNode : public rclcpp::Node {
public:
    VisualLoopNode() : Node("visual_loop_node") {
        this->declare_parameter<int>("common.img_en", 1);
        int img_en = this->get_parameter("common.img_en").as_int();
        if (img_en == 0) {
            img_en_ = false;
            RCLCPP_WARN(this->get_logger(), ">>> [BACKEND LOOP DET.]      : LiDAR ScanContext ONLY (Visual Loop DISABLED by img_en=0)");
            return;
        }
        img_en_ = true;

        this->declare_parameter<int>("orb_nfeatures", DEFAULT_ORB_NFEATURES_);
        this->declare_parameter<double>("clahe_clip_limit", DEFAULT_CLAHE_CLIP_LIMIT_);
        this->declare_parameter<int>("min_loop_id_diff", DEFAULT_MIN_LOOP_ID_DIFF_);
        this->declare_parameter<double>("lowe_ratio", DEFAULT_LOWE_RATIO_);
        this->declare_parameter<int>("min_good_matches", DEFAULT_MIN_GOOD_MATCHES_);
        this->declare_parameter<int>("min_matches_for_ransac", DEFAULT_MIN_MATCHES_FOR_RANSAC_);
        this->declare_parameter<int>("min_inliers_for_loop", DEFAULT_MIN_INLIERS_FOR_LOOP_);
        this->declare_parameter<std::string>("save_directory", "./src/navigation/fast_livo_dog/data/");

        orb_nfeatures_ = this->get_parameter("orb_nfeatures").as_int();
        clahe_clip_limit_ = this->get_parameter("clahe_clip_limit").as_double();
        min_loop_id_diff_ = this->get_parameter("min_loop_id_diff").as_int();
        lowe_ratio_ = this->get_parameter("lowe_ratio").as_double();
        min_good_matches_ = this->get_parameter("min_good_matches").as_int();
        min_matches_for_ransac_ = this->get_parameter("min_matches_for_ransac").as_int();
        min_inliers_for_loop_ = this->get_parameter("min_inliers_for_loop").as_int();
        save_directory_ = this->get_parameter("save_directory").as_string();
        if (!save_directory_.empty() && save_directory_.back() != '/') {
            save_directory_ += "/";
        }
        img_directory_ = save_directory_ + "Image/";
        std::string cmd_mkdir = "mkdir -p " + img_directory_;
        std::string cmd_rm = "rm -rf " + img_directory_ + "*";
        int ret_mkdir = system(cmd_mkdir.c_str());
        int ret_rm = system(cmd_rm.c_str());
        (void)ret_mkdir;
        (void)ret_rm;

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left_camera/image", 10, std::bind(&VisualLoopNode::imageCallback, this, std::placeholders::_1));

        sub_kf_id_ = this->create_subscription<std_msgs::msg::Header>(
            "/key_frames_ids", 10, std::bind(&VisualLoopNode::keyframeCallback, this, std::placeholders::_1));

        pub_candidate_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/visual_loop_candidate", 10);

        // ArUco 回环订阅 (只在 img_en=1 && aruco_en=true 时激活)
        this->declare_parameter<bool>("aruco.aruco_en", false);
        this->declare_parameter<double>("aruco.marker_size", 0.0);
        aruco_en_ = this->get_parameter("aruco.aruco_en").as_bool();
        aruco_marker_size_ = this->get_parameter("aruco.marker_size").as_double();
        // 加载相机内参 (用于 ArUco 解算及 Visual PGO 本质矩阵)
        this->declare_parameter<int>("camera.width", 640);
        this->declare_parameter<int>("camera.height", 512);
        this->declare_parameter<double>("camera.fx", 0.0);
        this->declare_parameter<double>("camera.fy", 0.0);
        this->declare_parameter<double>("camera.cx", 0.0);
        this->declare_parameter<double>("camera.cy", 0.0);
        this->declare_parameter<double>("camera.d0", 0.0);
        this->declare_parameter<double>("camera.d1", 0.0);
        this->declare_parameter<double>("camera.d2", 0.0);
        this->declare_parameter<double>("camera.d3", 0.0);
        double fx = this->get_parameter("camera.fx").as_double();
        double fy = this->get_parameter("camera.fy").as_double();
        double cx = this->get_parameter("camera.cx").as_double();
        double cy = this->get_parameter("camera.cy").as_double();
        cam_K_ = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cam_dist_ = (cv::Mat_<double>(4,1) <<
            this->get_parameter("camera.d0").as_double(),
            this->get_parameter("camera.d1").as_double(),
            this->get_parameter("camera.d2").as_double(),
            this->get_parameter("camera.d3").as_double());

        // 加载 T_camera_lidar 外参，用于坐标系转换
        std::vector<double> T_cl_vec(16, 0.0);
        this->declare_parameter<std::vector<double>>("extrin_calib.T_camera_lidar", T_cl_vec);
        this->get_parameter("extrin_calib.T_camera_lidar", T_cl_vec);
        if (T_cl_vec.size() == 16) {
            T_camera_lidar_ = (cv::Mat_<double>(4,4) <<
                T_cl_vec[0],  T_cl_vec[1],  T_cl_vec[2],  T_cl_vec[3],
                T_cl_vec[4],  T_cl_vec[5],  T_cl_vec[6],  T_cl_vec[7],
                T_cl_vec[8],  T_cl_vec[9],  T_cl_vec[10], T_cl_vec[11],
                T_cl_vec[12], T_cl_vec[13], T_cl_vec[14], T_cl_vec[15]);
            // T_lidar_camera = inv(T_camera_lidar)
            T_lidar_camera_ = T_camera_lidar_.inv();
        }

        pub_visual_pgo_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/visual_pgo_constraint", 10);

        if (aruco_en_) {
            // latched publisher，发布给后端保存 aruco_landmarks.yaml
            pub_aruco_landmarks_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/aruco_landmark_obs", rclcpp::QoS(10).transient_local());

            sub_aruco_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/aruco_detections", 10,
                std::bind(&VisualLoopNode::arucoCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "ArUco: Visual loop: subscribed to /aruco_detections");
        }

        clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(CLAHE_GRID_SIZE_, CLAHE_GRID_SIZE_));
        orb_ = cv::ORB::create(orb_nfeatures_);

        voc_ = std::make_shared<DBoW3::Vocabulary>(save_directory_ + "orb_vocabulary.dbow3");
        db_ = std::make_shared<DBoW3::Database>(*voc_);

        if (voc_->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load vocabulary from: %s", (save_directory_ + "orb_vocabulary.dbow3").c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Vocabulary loaded successfully. In-memory database initialized.");
        }

        RCLCPP_WARN(this->get_logger(), ">>> [BACKEND LOOP DET.]      : LiDAR ScanContext + Visual ORB DBoW3 (Both ENABLED)");
        RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
        RCLCPP_INFO(this->get_logger(), "  orb_nfeatures: %d", orb_nfeatures_);
        RCLCPP_INFO(this->get_logger(), "  min_good_matches: %d", min_good_matches_);
        RCLCPP_INFO(this->get_logger(), "  min_matches_for_ransac: %d", min_matches_for_ransac_);
        RCLCPP_INFO(this->get_logger(), "  min_inliers_for_loop: %d", min_inliers_for_loop_);
        RCLCPP_INFO(this->get_logger(), "  save_directory: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "  img_directory: %s", img_directory_.c_str());
    }

    ~VisualLoopNode() {
        if (!img_en_) return;
        RCLCPP_INFO(this->get_logger(),
                    "Node is shutting down. Saving visual descriptors database to Binary format.");
        saveVisualDatabase(save_directory_ + "visual_descriptors.bin");
    }

    void saveVisualDatabase(const std::string& filename) {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to open visual database file for writing (Path: %s).",
                         filename.c_str());
            return;
        }

        int num_frames = history_descriptors_.size();
        ofs.write(reinterpret_cast<const char*>(&num_frames), sizeof(int));

        for (size_t i = 0; i < history_descriptors_.size(); ++i) {
            const cv::Mat& desc = history_descriptors_[i];
            int rows = desc.rows;
            int cols = desc.cols;
            int type = desc.type();
            
            ofs.write(reinterpret_cast<const char*>(&rows), sizeof(int));
            ofs.write(reinterpret_cast<const char*>(&cols), sizeof(int));
            ofs.write(reinterpret_cast<const char*>(&type), sizeof(int));
            
            if (rows > 0 && cols > 0) {
                size_t data_size = rows * cols * desc.elemSize();
                ofs.write(reinterpret_cast<const char*>(desc.data), data_size);
            }
        }
        
        ofs.close();
        RCLCPP_INFO(this->get_logger(), "Global visual Bag-of-Words library serialized successfully into binary format.");
        RCLCPP_INFO(this->get_logger(),
                    "Visual descriptors database saved successfully (Path: %s, Frames count: %zu).",
                    filename.c_str(), history_descriptors_.size());
    }

private:
    // --- Visual Loop Detection and Matching Thresholds ---
    const int CLAHE_GRID_SIZE_ = 8;                // CLAHE grid block size (pixels)
    const int DEFAULT_ORB_NFEATURES_ = 500;        // ORB feature limit
    const double DEFAULT_CLAHE_CLIP_LIMIT_ = 3.0;  // CLAHE local clip limit
    const int DEFAULT_MIN_LOOP_ID_DIFF_ = 50;      // Minimum keyframe interval to prevent self-loop
    const double DEFAULT_LOWE_RATIO_ = 0.75;       // Lowe's ratio test threshold
    const int DEFAULT_MIN_GOOD_MATCHES_ = 30;      // Minimum good matches to trigger loop closure
    const int DEFAULT_MIN_MATCHES_FOR_RANSAC_ = 15; // Minimum matches required to attempt RANSAC
    const int DEFAULT_MIN_INLIERS_FOR_LOOP_ = 25;  // Minimum RANSAC inliers to accept the loop closure
    const double DEFAULT_BOW_TRASH_SCORE_ = 0.015; // Trash filter threshold for BoW query

    bool has_calibrated_offset_ = false;
    double base_time_offset_ = 0.0;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(img_mutex_);
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            int target_width = this->get_parameter("camera.width").as_int();
            int target_height = this->get_parameter("camera.height").as_int();
            if (target_width > 0 && target_height > 0 && (img.cols != target_width || img.rows != target_height)) {
                cv::resize(img, img, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            }
            image_buffer_[timestamp] = img;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to convert image topic via cv_bridge (Error: %s).",
                         e.what());
            return;
        }

        if (image_buffer_.size() > 100) {
            image_buffer_.erase(image_buffer_.begin());
        }
    }

    void keyframeCallback(const std_msgs::msg::Header::SharedPtr msg) {
        double kf_time = rclcpp::Time(msg->stamp).seconds();
        cv::Mat kf_img;

        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (image_buffer_.empty()) {
                RCLCPP_WARN(this->get_logger(),
                            "Image buffer is currently empty. Cannot associate keyframe.");
                return;
            }

            auto it = image_buffer_.lower_bound(kf_time);
            if (it == image_buffer_.end()) {
                it--;
            } else if (it != image_buffer_.begin()) {
                auto prev_it = std::prev(it);
                if (std::abs(it->first - kf_time) > std::abs(prev_it->first - kf_time)) {
                    it = prev_it;
                }
            }

            double raw_img_time = it->first;
            if (!has_calibrated_offset_) {
                base_time_offset_ = raw_img_time - kf_time;
                has_calibrated_offset_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Camera/LiDAR clock epoch offset calibrated: %.3f seconds.", base_time_offset_);
            }

            double corrected_img_time = raw_img_time - base_time_offset_;
            double real_time_diff = std::abs(corrected_img_time - kf_time);

            if (real_time_diff > 0.1) {
                it = std::prev(image_buffer_.end());
            }

            kf_img = it->second.clone();
        }

        int curr_id = history_descriptors_.size();
        if (!kf_img.empty() && !img_directory_.empty()) {
            std::string img_path = img_directory_ + padZeros(curr_id) + ".png";
            cv::imwrite(img_path, kf_img);
        }

        // --- 使用 std::thread 彻底后台分离（detach），主线程 0 微秒阻塞！ ---
        std::thread([this, kf_img, curr_id]() {
            if (kf_img.empty()) {
                std::lock_guard<std::mutex> lock(img_mutex_);
                history_descriptors_.push_back(cv::Mat());
                history_keypoints_.push_back(std::vector<cv::KeyPoint>());
                return;
            }

            cv::Mat gray_img, clahe_img;
            cv::cvtColor(kf_img, gray_img, cv::ColorConversionCodes::COLOR_BGR2GRAY);
            clahe_->apply(gray_img, clahe_img);

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb_->detectAndCompute(clahe_img, cv::noArray(), keypoints, descriptors);

            if (descriptors.empty()) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to extract any ORB features. Inserting empty placeholder.");
                std::lock_guard<std::mutex> lock(img_mutex_);
                history_descriptors_.push_back(cv::Mat());
                history_keypoints_.push_back(std::vector<cv::KeyPoint>());
                return;
            }

            {
                std::lock_guard<std::mutex> lock(img_mutex_);
                history_descriptors_.push_back(descriptors);
                history_keypoints_.push_back(keypoints);
                db_->add(descriptors);
                dbow_to_hist_id_.push_back(curr_id);
            }

            if (curr_id > min_loop_id_diff_) {
                // DBoW3 Coarse Search
                DBoW3::QueryResults ret;
                {
                    std::lock_guard<std::mutex> lock(img_mutex_);
                    db_->query(descriptors, ret, 100, curr_id - min_loop_id_diff_);
                }

                cv::BFMatcher matcher(cv::NORM_HAMMING);
                for (size_t i = 0; i < ret.size(); ++i) {
                    int dbow_id = ret[i].Id;
                    int hist_id = 0;
                    cv::Mat target_descriptors;
                    std::vector<cv::KeyPoint> target_keypoints;

                    {
                        std::lock_guard<std::mutex> lock(img_mutex_);
                        if (dbow_id >= (int)dbow_to_hist_id_.size()) continue;
                        hist_id = dbow_to_hist_id_[dbow_id];
                        target_descriptors = history_descriptors_[hist_id];
                        target_keypoints = history_keypoints_[hist_id];
                    }
                    
                    if (ret[i].Score < DEFAULT_BOW_TRASH_SCORE_) continue;

                    std::vector<std::vector<cv::DMatch>> knn_matches;
                    matcher.knnMatch(descriptors, target_descriptors, knn_matches, 2);

                    std::vector<cv::DMatch> good_matches;
                    for (size_t m = 0; m < knn_matches.size(); m++) {
                        if (knn_matches[m].size() < 2) continue;
                        if (knn_matches[m][0].distance < lowe_ratio_ * knn_matches[m][1].distance) {
                            good_matches.push_back(knn_matches[m][0]);
                        }
                    }

                    if (int(good_matches.size()) > min_matches_for_ransac_) {
                        std::vector<cv::Point2f> pts_curr, pts_hist;
                        for (size_t k = 0; k < good_matches.size(); k++) {
                            pts_curr.push_back(keypoints[good_matches[k].queryIdx].pt);
                            pts_hist.push_back(target_keypoints[good_matches[k].trainIdx].pt);
                        }

                        std::vector<uchar> status;
                        cv::Mat E = cv::findEssentialMat(pts_curr, pts_hist, cam_K_, cv::RANSAC, 0.999, 1.0, status);
                        int inlier_count = 0;
                        for (size_t k = 0; k < status.size(); k++) {
                            if (status[k]) inlier_count++;
                        }

                        if (inlier_count >= min_inliers_for_loop_) {
                            cv::Mat R_cam, t_cam;
                            cv::recoverPose(E, pts_curr, pts_hist, cam_K_, R_cam, t_cam, status);

                            RCLCPP_INFO(this->get_logger(),
                                "Visual loop candidate proposed. Hist ID: %d, Curr ID: %d, BoW: %.4f, RANSAC Inliers: %d. Running 2D-2D Epipolar Geometry verification...",
                                hist_id, curr_id, ret[i].Score, inlier_count);

                            cv::Mat rvec;
                            cv::Rodrigues(R_cam, rvec);
                            
                            auto pgo_msg = std_msgs::msg::Float64MultiArray();
                            pgo_msg.data = {
                                (double)hist_id, (double)curr_id, 
                                t_cam.at<double>(0,0), t_cam.at<double>(1,0), t_cam.at<double>(2,0),
                                rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)
                            };
                            pub_visual_pgo_->publish(pgo_msg);

                            auto candidate_msg = std_msgs::msg::Int32MultiArray();
                            candidate_msg.data = {hist_id, curr_id};
                            pub_candidate_->publish(candidate_msg);
                            break;
                        }
                    }
                }
            }
        }).detach();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_kf_id_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_aruco_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_candidate_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_aruco_landmarks_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_visual_pgo_;

    // ArUco 回环状态
    bool aruco_en_ = false;
    double aruco_marker_size_ = 0.20;

    // 最佳观测记录: marker_id -> {best_kf_id, max_area, T_lidar_marker(4x4)}
    struct ArucoBestObs {
        int kf_id;
        float max_area;
        cv::Mat T_lidar_marker; // marker 在该帧激光雷达坐标系下的位姿 (4x4 double)
    };
    std::map<int, ArucoBestObs> aruco_best_obs_;       // marker_id -> 最佳观测
    std::map<int, int>          aruco_marker_first_kf_; // marker_id -> 首次观测关键帧 ID (用作回环锚点)
    std::map<int, int>          aruco_marker_last_loop_kf_; // marker_id -> 上次触发回环的关键帧 ID
    std::mutex aruco_mutex_;

    // 相机内参与外参（PnP 用）
    cv::Mat cam_K_;
    cv::Mat cam_dist_;
    cv::Mat T_lidar_camera_; // 4x4 double
    cv::Mat T_camera_lidar_; // 4x4 double

    // 发布全量 landmark 观测到 latched 话题
    // 格式: [n_markers, (marker_id, kf_id, marker_size, T_lidar_marker[16]) x n]
    void publishLandmarkObs() {
        std_msgs::msg::Float64MultiArray lm_msg;
        lm_msg.data.push_back(static_cast<double>(aruco_best_obs_.size()));
        for (auto& [id, obs] : aruco_best_obs_) {
            lm_msg.data.push_back(static_cast<double>(id));
            lm_msg.data.push_back(static_cast<double>(obs.kf_id));
            lm_msg.data.push_back(aruco_marker_size_);  // marker 物理尺寸
            // 展平 4x4 矩阵
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    lm_msg.data.push_back(obs.T_lidar_marker.at<double>(r, c));
        }
        pub_aruco_landmarks_->publish(lm_msg);
    }

    void arucoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // 格式: [kf_counter, n_markers, (marker_id, u0,v0, u1,v1, u2,v2, u3,v3) x n]
        if (msg->data.size() < 2) return;
        int kf_id = static_cast<int>(msg->data[0]);
        int n     = static_cast<int>(msg->data[1]);
        size_t idx = 2;
        bool landmark_updated = false;
        std::lock_guard<std::mutex> lock(aruco_mutex_);
        for (int i = 0; i < n; ++i) {
            if (idx + 9 > msg->data.size()) break;
            int marker_id = static_cast<int>(msg->data[idx++]);

            // Step 1: 解析 4 个角点像素坐标 (u,v)*4，不再 skip
            std::vector<cv::Point2f> corners(4);
            for (int c = 0; c < 4; ++c) {
                corners[c].x = static_cast<float>(msg->data[idx++]);
                corners[c].y = static_cast<float>(msg->data[idx++]);
            }

            // 计算 marker 投影面积 (近似认为是用角点构成的四边形面积，简化为对角线长度相乘)
            float d1 = cv::norm(corners[0] - corners[2]);
            float d2 = cv::norm(corners[1] - corners[3]);
            float area = d1 * d2 * 0.5f;

            bool is_first = (aruco_marker_first_kf_.find(marker_id) == aruco_marker_first_kf_.end());
            if (is_first) {
                aruco_marker_first_kf_[marker_id] = kf_id;
                aruco_marker_last_loop_kf_[marker_id] = kf_id;
                RCLCPP_INFO(this->get_logger(),
                    "ArUco: First observation of marker ID=%d at KF=%d (anchor)", marker_id, kf_id);
            }

            int first_kf = aruco_marker_first_kf_[marker_id];
            int last_loop_kf = aruco_marker_last_loop_kf_[marker_id];

            bool has_best = (aruco_best_obs_.find(marker_id) != aruco_best_obs_.end());
            if (!has_best || area > aruco_best_obs_[marker_id].max_area) {
                // Step 2: PnP 解算，得到 T_camera_marker
                // 3D marker 角点坐标（marker 物理坐标系，Z=0 平面，顺序: TL TR BR BL）
                float half = static_cast<float>(aruco_marker_size_ / 2.0);
                std::vector<cv::Point3f> obj_pts = {
                    {-half,  half, 0.f},
                    { half,  half, 0.f},
                    { half, -half, 0.f},
                    {-half, -half, 0.f}
                };
                cv::Mat rvec, tvec;
                bool pnp_ok = false;
                if (!cam_K_.empty() && !T_lidar_camera_.empty()) {
                    pnp_ok = cv::solvePnP(obj_pts, corners, cam_K_, cam_dist_, rvec, tvec);
                }

                if (pnp_ok) {
                    ArucoBestObs obs;
                    obs.kf_id = kf_id;
                    obs.max_area = area;
                    // 构造 T_camera_marker (4x4)
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    cv::Mat T_camera_marker = cv::Mat::eye(4, 4, CV_64F);
                    R.convertTo(T_camera_marker(cv::Rect(0,0,3,3)), CV_64F);
                    tvec.convertTo(T_camera_marker(cv::Rect(3,0,1,3)), CV_64F);
                    // T_lidar_marker = T_lidar_camera * T_camera_marker
                    obs.T_lidar_marker = T_lidar_camera_ * T_camera_marker;
                    
                    aruco_best_obs_[marker_id] = obs;
                    landmark_updated = true;
                    RCLCPP_INFO(this->get_logger(),
                        "ArUco: Updated BEST observation for marker ID=%d at KF=%d (area: %.1f px^2)",
                        marker_id, kf_id, area);
                } else if (!has_best) {
                    // 如果 PnP 失败且是第一次，先存个单位阵垫底
                    ArucoBestObs obs;
                    obs.kf_id = kf_id;
                    obs.max_area = 0.0f;
                    obs.T_lidar_marker = cv::Mat::eye(4, 4, CV_64F);
                    aruco_best_obs_[marker_id] = obs;
                    RCLCPP_WARN(this->get_logger(),
                        "ArUco: PnP failed for marker ID=%d at KF=%d", marker_id, kf_id);
                }
            }

            // Step 3: 回环触发逻辑（保持不变，锚定初始帧 first_kf）
            if (!is_first && kf_id - last_loop_kf > 5) {
                RCLCPP_INFO(this->get_logger(),
                    "ArUco Loop: Marker ID=%d re-observed: KF %d <-> KF %d -> sending loop candidate",
                    marker_id, first_kf, kf_id);
                auto loop_msg = std_msgs::msg::Int32MultiArray();
                loop_msg.data = {first_kf, kf_id};
                pub_candidate_->publish(loop_msg);
                aruco_marker_last_loop_kf_[marker_id] = kf_id;
            }
        }

        // Step 3: 有新 landmark 时更新 latched 话题
        if (landmark_updated && pub_aruco_landmarks_) {
            publishLandmarkObs();
        }
    }

    std::map<double, cv::Mat> image_buffer_;
    static std::string padZeros(int val, int num_digits = 6) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(num_digits) << val;
        return ss.str();
    }

    std::vector<cv::Mat> history_descriptors_;
    std::vector<std::vector<cv::KeyPoint>> history_keypoints_;
    std::vector<int> dbow_to_hist_id_;
    std::mutex img_mutex_;

    cv::Ptr<cv::CLAHE> clahe_;
    cv::Ptr<cv::ORB> orb_;

    bool img_en_ = true;
    int orb_nfeatures_;
    double clahe_clip_limit_;
    int min_loop_id_diff_;
    double lowe_ratio_;
    int min_good_matches_;
    int min_matches_for_ransac_;
    int min_inliers_for_loop_;
    std::string save_directory_;
    std::string img_directory_;

    std::shared_ptr<DBoW3::Vocabulary> voc_;
    std::shared_ptr<DBoW3::Database> db_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualLoopNode>());
    rclcpp::shutdown();
    return 0;
}
