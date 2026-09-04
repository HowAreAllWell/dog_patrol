#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_HUMBLE
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#endif

#include <algorithm>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

#include "scancontext/Scancontext.h"

using namespace std::chrono_literals;

struct HistoricalPose {
    double x, y, z, roll, pitch, yaw;
};

struct SCResult {
    double score;
    int id;
    int yaw_shift;
    bool operator<(const SCResult& other) const { return score < other.score; }
};

class LidarGlobalLocator : public rclcpp::Node {
public:
    LidarGlobalLocator() : Node("lidar_global_locator") {
        this->declare_parameter<std::string>("sc_db_path", "./src/navigation/fast_livo_dog/data/sc_database.txt");
        this->declare_parameter<double>("match_threshold", SC_ACCEPT_THRESHOLD_);
        this->declare_parameter<int>("common.img_en", 1);

        this->get_parameter("sc_db_path", db_path_);
        this->get_parameter("match_threshold", threshold_);
        int img_en_int = 1;
        this->get_parameter("common.img_en", img_en_int);
        img_en_ = (img_en_int != 0);

        pub_initial_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/initialpose_lidar", 1);

        if (loadDatabase(db_path_)) {
            RCLCPP_INFO(this->get_logger(),
                        "ScanContext database loaded successfully. Waiting for incoming laser scan messages.");
            sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered", 1, std::bind(&LidarGlobalLocator::scanCallback, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ScanContext database file.");
        }

        sub_status_ = this->create_subscription<std_msgs::msg::Bool>(
            "/localization_status", 10, std::bind(&LidarGlobalLocator::statusCallback, this, std::placeholders::_1));

        sub_visual_fail_count_ = this->create_subscription<std_msgs::msg::Int32>(
            "/visual_fail_count", 10, std::bind(&LidarGlobalLocator::visualFailCountCallback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                Eigen::Quaternionf q(msg->pose.pose.orientation.w, 
                                     msg->pose.pose.orientation.x, 
                                     msg->pose.pose.orientation.y, 
                                     msg->pose.pose.orientation.z);
                Eigen::Vector3f t(msg->pose.pose.position.x, 
                                  msg->pose.pose.position.y, 
                                  msg->pose.pose.position.z);
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                pose.block<3,3>(0,0) = q.toRotationMatrix();
                pose.block<3,1>(0,3) = t;
                latest_odom_pose_ = pose;
                odom_received_ = true;
            });

        // (Removed force_lidar logic)

        lost_time_ = this->now();
    }

private:
    // --- 全局雷达检索与匹配参数 ---
    const double SC_ACCEPT_THRESHOLD_ = 0.15;  // 上限限制，只有低于该阈值才触发有效匹配
    const int SC_SECTORS_ = 60;                // ScanContext 的环形扇区总量划分配置 (360度 / 60 扇区 = 6度/扇区)
    const int MAX_CANDIDATES_ = 10;            // 取前10个候选，用于进行歧义性验证
    const double SC_AMBIGUITY_DIST_M_ = 2.5;   // 空间隔离半径 (米)，用于寻找可能造成歧义的另一个相似地标
    const double SC_AMBIGUITY_RATIO_ = 0.75;   // 歧义判断的得分比例阈值
    const double SC_AMBIGUITY_DIFF_ = 0.025;   // 歧义判断的绝对分差阈值
    const double SC_ABERRANT_LIMIT_ = 0.99;    // 用于异常值检验与非空检查的最大无效描述子匹配得分上限

    // 加载全局 ScanContext 索引先验数据库
    bool loadDatabase(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) return false;

        int num_frames;
        if (!(ifs >> num_frames)) return false;

        for (int i = 0; i < num_frames; ++i) {
            int id;
            HistoricalPose p;
            ifs >> id >> p.x >> p.y >> p.z >> p.roll >> p.pitch >> p.yaw;
            db_poses_.push_back(p);

            int rows, cols;
            ifs >> rows >> cols;
            Eigen::MatrixXd sc(rows, cols);
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    ifs >> sc(r, c);
                }
            }
            db_scs_.push_back(sc);
        }
        ifs.close();
        return true;
    }

    // 处理接收到的当前激光雷达帧，执行 ScanContext
    // 全局描述符匹配并解算最优偏差与候选位姿假设
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        if (is_backend_locked_.load() || !odom_received_) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *tempCloudRGB);

        // Transform from World to Body using Odometry
        Eigen::Matrix4f t_world_to_body = latest_odom_pose_.inverse();
        pcl::transformPointCloud(*tempCloudRGB, *tempCloudRGB, t_world_to_body);

        // Downsample to match mapping (HISTORY_CLOUD_DOWNSAMPLE_ = 0.4)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
        downSizeFilter.setLeafSize(0.4f, 0.4f, 0.4f);
        downSizeFilter.setInputCloud(tempCloudRGB);
        downSizeFilter.filter(*dsCloudRGB);

        pcl::PointCloud<SCPointType>::Ptr current_cloud(new pcl::PointCloud<SCPointType>());
        current_cloud->points.resize(dsCloudRGB->points.size());
        for (size_t i = 0; i < dsCloudRGB->points.size(); ++i) {
            current_cloud->points[i].x = dsCloudRGB->points[i].x;
            current_cloud->points[i].y = dsCloudRGB->points[i].y;
            current_cloud->points[i].z = dsCloudRGB->points[i].z;
            current_cloud->points[i].intensity = 0.0f;
        }

        Eigen::MatrixXd current_sc = scManager_.makeScancontext(*current_cloud);

        std::vector<SCResult> results;

        for (size_t i = 0; i < db_scs_.size(); ++i) {
            auto sc_dist_result = scManager_.distanceBtnScanContext(current_sc, db_scs_[i]);
            results.push_back({sc_dist_result.first, (int)i, sc_dist_result.second});
        }

        std::sort(results.begin(), results.end());

        static rclcpp::Time last_success_time = this->now();
        if (results.empty() || results[0].score >= threshold_ || results[0].score >= SC_ABERRANT_LIMIT_) {
            double best_score = results.empty() ? 999.0 : results[0].score;
            // 避免单帧扫描遮挡导致队列清空：只有连续 2 秒没有匹配上，才清空历史缓存
            if ((this->now() - last_success_time).seconds() > 2.0) {
                recent_lidar_ids_.clear();
            }
            RCLCPP_DEBUG(this->get_logger(),
                         "ScanContext matching failed. Best Score: %.3f, Threshold: %.2f.",
                         best_score, threshold_);
            return;
        }

        // --- 歧义性抑制 (Ambiguity Rejection) ---
        HistoricalPose best_pose = db_poses_[results[0].id];
        double best_score = results[0].score;
        bool ambiguity_detected = false;
        double ambiguous_score = 999.0;

        for (size_t i = 1; i < results.size() && i < (size_t)MAX_CANDIDATES_; ++i) {
            HistoricalPose candidate_pose = db_poses_[results[i].id];
            // 计算欧氏距离
            double dx = candidate_pose.x - best_pose.x;
            double dy = candidate_pose.y - best_pose.y;
            double dz = candidate_pose.z - best_pose.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // 如果物理距离超过 SC_AMBIGUITY_DIST_M_
            if (dist > SC_AMBIGUITY_DIST_M_) {
                ambiguous_score = results[i].score;
                // 检查得分比例：如果 Top 1 和远处的候选帧得分比例极其接近
                // 或者是绝对差值非常小，说明当前环境极度相似
                if ((best_score / ambiguous_score > SC_AMBIGUITY_RATIO_) || (ambiguous_score - best_score < SC_AMBIGUITY_DIFF_)) {
                    ambiguity_detected = true;
                }
                break; // 只看第一个物理分离的候选
            }
        }

        if (ambiguity_detected) {
            RCLCPP_DEBUG(this->get_logger(),
                        "Ambiguity Rejected! Best: %.3f, Ambiguous: %.3f. High risk of mismatch.", 
                        best_score, ambiguous_score);
            return;
        }

        last_success_time = this->now();
        int best_id = results[0].id;
        double best_yaw_shift = results[0].yaw_shift;

        // Throttle publishing to 1Hz maximum
        static rclcpp::Time last_pushed_time = this->now();
        if ((this->now() - last_pushed_time).seconds() >= 1.0) {
            last_pushed_time = this->now();

            RCLCPP_INFO(this->get_logger(),
                        "LiDAR strict anchor verified (Best: %.3f, Ambiguous: %.3f). Forwarding proposal to backend.", 
                        best_score, ambiguous_score == 999.0 ? -1.0 : ambiguous_score);

            HistoricalPose p = db_poses_[best_id];
            double sector_res = 2.0 * M_PI / static_cast<double>(SC_SECTORS_);
            double yaw_offset = best_yaw_shift * sector_res;

            geometry_msgs::msg::PoseArray pose_array_msg;
            pose_array_msg.header.stamp = msg->header.stamp;
            pose_array_msg.header.frame_id = "map";

            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = p.z;

            tf2::Quaternion q;
            double final_yaw = p.yaw + yaw_offset;
            q.setRPY(p.roll, p.pitch, final_yaw);

            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            pose_array_msg.poses.push_back(pose);
            pub_initial_pose_array_->publish(pose_array_msg);
        }


    }

    void statusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            is_backend_locked_.store(true);
        } else {
            if (is_backend_locked_.load()) {
                lost_time_ = this->now();

            }
            is_backend_locked_.store(false);
        }
    }

    void visualFailCountCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        consecutive_bow_failures_.store(msg->data);
    }

    std::string db_path_;
    double threshold_;
    SCManager scManager_;
    std::vector<HistoricalPose> db_poses_;
    std::vector<Eigen::MatrixXd> db_scs_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_initial_pose_array_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_status_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_visual_fail_count_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    Eigen::Matrix4f latest_odom_pose_ = Eigen::Matrix4f::Identity();
    bool odom_received_ = false;

    std::atomic<bool> is_backend_locked_{false};
    std::atomic<int> consecutive_bow_failures_{0};
    rclcpp::Time lost_time_;
    bool img_en_{true};
    std::deque<int> recent_lidar_ids_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarGlobalLocator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
