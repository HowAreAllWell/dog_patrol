#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <optional>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

struct ICPResult {
    bool valid;
    double fitness;
    Eigen::Matrix4f T_final;
    double degenerate_ratio = 1.0;
};

class GlobalLocalization : public rclcpp::Node {
public:
    GlobalLocalization() : Node("global_localization") {
        RCLCPP_WARN(this->get_logger(), ">>> [SLAM LOCALIZATION START] : NDT / ICP Global Relocalization");
        RCLCPP_WARN(this->get_logger(), ">>> [GLOBAL RELOCALIZATION]   : LiDAR PointCloud Match (ENABLED)");
        RCLCPP_WARN(this->get_logger(), ">>> [INITIAL TRACKING MODE]   : LIVO / LIO Odometry Waiting for Lock...");

        initialized_ = false;
        map_received_ = false;
        T_map_to_odom_ = Eigen::Matrix4f::Identity();

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cur_scan_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        auto qos_map = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

        pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
        pub_pc_in_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 10);
        pub_status_ = this->create_publisher<std_msgs::msg::Bool>("/localization_status", 1);
        pub_adaptive_radius_ = this->create_publisher<std_msgs::msg::Float64>("~/adaptive_search_radius", 10);
        adaptive_submap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/adaptive_submap_highlight", rclcpp::SystemDefaultsQoS());
        pub_force_lidar_ = this->create_publisher<std_msgs::msg::Bool>("/localization/force_lidar", 1);

        sub_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map", qos_map, std::bind(&GlobalLocalization::cb_global_map, this, std::placeholders::_1));

        last_visual_time_ = this->now();
        t_lost_start_ = this->now();

        sub_initialpose_visual_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/initialpose_visual", 1,
            std::bind(&GlobalLocalization::cb_initialpose_visual, this, std::placeholders::_1));

        sub_initialpose_lidar_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/initialpose_lidar", 1, std::bind(&GlobalLocalization::cb_initialpose_lidar, this, std::placeholders::_1));

        sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&GlobalLocalization::cb_initialpose, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&GlobalLocalization::cb_save_cur_scan, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&GlobalLocalization::cb_odom, this, std::placeholders::_1));

        sub_smoothed_localization_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization", 10,
            std::bind(&GlobalLocalization::cb_smoothed_localization, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalLocalization::timer_localization_callback, this));
        adaptive_timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalLocalization::adaptive_timer_callback, this));

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cur_scan", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
        pub_submap_highlight_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/submap_highlight", 10);
        pub_ui_projected_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("~/ui_horizontal_pose", 10);
        pub_ui_clear_trigger_ = this->create_publisher<std_msgs::msg::Bool>("~/ui_clear_trigger", 10);

        // 首先加载 body -> base_link 外参矩阵以自动计算旋转
        this->declare_parameter<std::vector<double>>("T_lidar_base", std::vector<double>(16, 0.0));
        std::vector<double> ext_matrix = this->get_parameter("T_lidar_base").as_double_array();
        if (ext_matrix.size() == 16 && ext_matrix[0] != 0.0) {
            T_body_to_baselink_ << ext_matrix[0], ext_matrix[1], ext_matrix[2], ext_matrix[3],
                                  ext_matrix[4], ext_matrix[5], ext_matrix[6], ext_matrix[7],
                                  ext_matrix[8], ext_matrix[9], ext_matrix[10], ext_matrix[11],
                                  ext_matrix[12], ext_matrix[13], ext_matrix[14], ext_matrix[15];
        } else {
            T_body_to_baselink_ = Eigen::Matrix4f::Identity();
        }

        loadArucoLandmarks();
    }

private:
    const double VISUAL_DEGENERATE_THRESHOLD_ = 0.05;
    const double LIDAR_DEGENERATE_THRESHOLD_ = 0.05;
    // --- 算法阈值与常量配置 ---
    const double SCORE_THRESHOLD_INITIAL_ = 0.20;    // 视觉初值重定位的严格匹配得分 (MSE)
                                                     // 上限，防止走廊等相似环境的假阳性
    const double SCORE_THRESHOLD_TRACKING_ = 0.20;   // 局部低频跟踪维护 of 严格匹配得分 (MSE) 上限，低于此值允许修正轨迹
    const double SCORE_THRESHOLD_COASTING_ = 0.40;   // 局部低频跟踪维护 of 宽松匹配得分 (MSE) 上限，介于两阈值之间维持现状，超过此值触发倒计时
    const double VISUAL_MAX_INITIAL_SHIFT_M_ = 2.0;  // 视觉初始重定位阶段允许的最大平移偏移量 (米)
    const double MAX_TRACKING_SHIFT_M_ = 1.0;        // 低频跟踪阶段允许的最大平移补偿量
                                                     // (米)，用以吸收系统数据交互的时间差
    const double MAX_TRACKING_ROTATION_DEG_ = 10.0;  // 低频跟踪阶段允许的最大异常偏航角度补偿限制 (度)
    const int MAX_TRACKING_LOSS_FRAMES_ = 10;        // 容忍连续里程计递推算的最大帧数上限
    const double SCAN_VOXEL_SIZE_ = 0.2;             // 雷达实时单帧点云下采样的体素尺寸大小 (米)
    const double MAP_VOXEL_SIZE_ = 0.4;              // 全局地图降采样采样的体素尺寸大小 (米)
    const double TIMEOUT_SEC_B_ = 15.0;              // 视觉节点特征信号失联的最长超时时限 (秒)
    const int MAX_REJECTS_A_ = 5;                    // 允许连续拒绝视觉初始位姿建议 of 次数上限

    // --- 裁剪与匹配算法内部常量 ---
    const double CROP_RADIUS_M_ = 40.0;                   // 局部地图裁剪以提高计算实时性的球形区域半径 (米)
    const double ICP_RESOLUTION_ = 0.4;                   // VGICP 的体素空间分辨率 (米)
    const int ICP_MAX_ITERATIONS_ = 30;                   // 限制 VGICP 的最大收敛迭代步数，确保低延迟耗时
    const double ICP_MAX_CORRESPONDENCE_DISTANCE_ = 1.5;  // VGICP 特征匹配搜索半径，保障较大的收敛范围与扭矩修正能力
    const size_t MIN_SCAN_POINTS_ = 100;                  // 参与匹配计算的实时雷达点云的最小点数限制
    const size_t MIN_MAP_POINTS_ = 500;                   // 局部裁剪地图的最小点数限制，防止出现数学退化
    const double DEFAULT_FITNESS_SCORE_ = 1000.0;         // 默认失效时输出的极大匹配得分
    const double FITNESS_SCORE_MAX_DIST_ = 2.0;           // 计算最终得分时只对特定距离内的点对进行累加 (米)
    const double LIDAR_SCORE_THRESHOLD_INITIAL_ = 0.20;   // 纯雷达初始化方案的严格匹配得分 (MSE) 上限
    const double LIDAR_MAX_INITIAL_SHIFT_M_ = 2.0;        // 纯雷达模式下防止误锁定的平移偏差阈值 (米)

    // 矩阵安全求逆以避免数值不稳定
    Eigen::Matrix4f safeInverse(const Eigen::Matrix4f& T) {
        Eigen::Matrix4f T_inv = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f R = T.block<3, 3>(0, 0);
        Eigen::Vector3f t = T.block<3, 1>(0, 3);
        T_inv.block<3, 3>(0, 0) = R.transpose();
        T_inv.block<3, 1>(0, 3) = -R.transpose() * t;
        return T_inv;
    }

    // 将几何位姿消息转化为四维同次变换矩阵，包含输入有效性正则化
    Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose& pose) {
        Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        if (q.norm() > 1e-6) {
            q.normalize();
        } else {
            q = Eigen::Quaternionf::Identity();
        }
        Eigen::Vector3f t(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        mat.block<3, 1>(0, 3) = t;
        return mat;
    }

    // 从 3x3 旋转矩阵中计算 Yaw 角度，处理万向锁与归一化
    double getYaw(const Eigen::Matrix3d& R) {
        double pitch = -std::asin(std::max(-1.0, std::min(1.0, R(2, 0))));
        double yaw = 0.0;
        if (std::abs(std::cos(pitch)) > 1e-4) {
            yaw = std::atan2(R(1, 0), R(0, 0));
        } else {
            if (R(2, 0) < 0) {
                // 俯仰角为 90 度
                yaw = std::atan2(-R(0, 1), R(1, 1));
            } else {
                // 俯仰角为 -90 度
                yaw = std::atan2(R(0, 1), R(1, 1));
            }
        }
        return yaw; // 弧度
    }

    // 将相对旋转矩阵拆解为欧拉角 (Roll, Pitch, Yaw)，做万向锁处理及绝对偏差归一化 (0~180度)
    void extractEulerAngles(const Eigen::Matrix3d& R, double& roll_deg, double& pitch_deg, double& yaw_deg) {
        double pitch = -std::asin(std::max(-1.0, std::min(1.0, R(2, 0))));
        double roll = 0.0;
        double yaw = 0.0;

        if (std::abs(std::cos(pitch)) > 1e-4) {
            roll = std::atan2(R(2, 1), R(2, 2));
            yaw = std::atan2(R(1, 0), R(0, 0));
        } else {
            roll = 0.0;
            if (R(2, 0) < 0) {
                // 俯仰角为 90 度
                yaw = std::atan2(-R(0, 1), R(1, 1));
            } else {
                // 俯仰角为 -90 度
                yaw = std::atan2(R(0, 1), R(1, 1));
            }
        }

        // 转换为角度
        roll_deg = roll * 180.0 / M_PI;
        pitch_deg = pitch * 180.0 / M_PI;
        yaw_deg = yaw * 180.0 / M_PI;

        // 将绝对差值归一化到 [0, 180] 度
        auto normalize_deg = [](double deg) {
            deg = std::fmod(deg, 360.0);
            if (deg > 180.0) deg -= 360.0;
            if (deg < -180.0) deg += 360.0;
            return std::abs(deg);
        };

        roll_deg = normalize_deg(roll_deg);
        pitch_deg = normalize_deg(pitch_deg);
        yaw_deg = normalize_deg(yaw_deg);
    }

    // 体素滤波器以降低点云规模
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(*cloud_filtered);
        return cloud_filtered;
    }

    // 接收全局完整先验点云地图并进行降采样处理
    void cb_global_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (map_received_) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *raw_map);
        global_map_ = voxelDownSample(raw_map, MAP_VOXEL_SIZE_);

        // 永久缓存全局 KD-Tree，避免定位丢失期间 1Hz 高频重建引发的严重卡顿
        global_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        global_kdtree_->setInputCloud(global_map_);

        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Global map downsampled and KD-Tree built successfully.");
    }

    // 过滤里程计计算结果并缓存最新输入
    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (cur_odom_) {
            rclcpp::Time current_time(msg->header.stamp);
            rclcpp::Time prev_time(cur_odom_->header.stamp);
            if (current_time.nanoseconds() < prev_time.nanoseconds()) {
                RCLCPP_WARN(this->get_logger(), "Time jump backward detected in global_localization! Clearing TF buffer.");
                tf_buffer_->clear();
                {
                    std::lock_guard<std::mutex> lock(path_mutex_);
                    path_record_.poses.clear();
                }
            }
        }

        // 维护基础跟踪指标参数
        double yaw_diff = 0.0;
        bool has_prev_odom = false;
        if (cur_odom_) {
            Eigen::Matrix4f T_prev = poseToMatrix(cur_odom_->pose.pose);
            Eigen::Matrix4f T_curr = poseToMatrix(msg->pose.pose);
            double prev_yaw = getYaw(T_prev.block<3, 3>(0, 0).cast<double>());
            double curr_yaw = getYaw(T_curr.block<3, 3>(0, 0).cast<double>());
            yaw_diff = curr_yaw - prev_yaw;
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
            has_prev_odom = true;
        }

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (has_prev_odom) {
                if (tracking_loss_cnt_ > 0) {
                    accumulated_yaw_since_lost_deg_ += std::abs(yaw_diff) * 180.0 / M_PI;
                } else {
                    accumulated_yaw_since_lost_deg_ = 0.0;
                    accumulated_distance_since_lost_ = 0.0;
                    coasting_budget_distance_ = 0.0;
                }
            }

            if (initialized_) {
                Eigen::Matrix4f T = poseToMatrix(msg->pose.pose);
                if (last_locked_odom_pose_) {
                    Eigen::Vector3f curr_pos(T(0, 3), T(1, 3), T(2, 3));
                    double ds = (curr_pos - *last_locked_odom_pose_).norm();
                    
                    // [静止死区滤波与轨迹平滑 (Spatial Step-Accumulator)]
                    // 利用 2cm 的阶跃累加机制，完美过滤底盘高频震动带来的零均值位移积分爆炸，
                    // 无损保留机器狗的低速真实运动。
                    if (ds >= 0.02) {
                        delta_s_ += ds;
                        if (tracking_loss_cnt_ > 0) {
                            accumulated_distance_since_lost_ += ds;
                            coasting_budget_distance_ += ds;
                        }
                        last_locked_odom_pose_ = curr_pos; // 只有发生真实物理位移才更新锚点
                    }
                } else {
                    last_locked_odom_pose_ = Eigen::Vector3f(T(0, 3), T(1, 3), T(2, 3));
                }
            } else {
                last_locked_odom_pose_ = std::nullopt;
            }
        }

        cur_odom_ = msg;

        // 核心同步：将机器人位姿先验挪至门禁前更新，保证外部视觉节点大循环圆心始终随动
        Eigen::Matrix4f T_odom_current = poseToMatrix(msg->pose.pose);
        Eigen::Matrix4f local_last_known_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_known_pose_ = T_map_to_odom_ * T_odom_current * T_body_to_baselink_;
            local_last_known_pose = last_known_pose_;
        }

        // 暂态时序保护门禁
        if (!initialized_) {
            stable_stream_delay_idx_.store(0);
            return;
        }

        if (stable_stream_delay_idx_.load() < 10) {
            stable_stream_delay_idx_++;
            return; 
        }

        // 严重 Bug 修复：RViz 轨迹线和 GUI 界面轨迹必须取自 transform_fusion 平滑过后的真值 (/localization)。
        // 原先这里直接使用未经平滑的 raw local_last_known_pose，导致每当 ICP 修正产生跳变时，
        // 虽然机器狗 TF 树平滑移动，但 RViz 绿线和界面轨迹却瞬间跳跃折弯（Z字形）。
        // 现将可视化发布的职责移至平滑后的回调函数 cb_smoothed_localization 中。

        // 实时发布 map -> camera_init 高频变换，确保外部节点（如 odom_bridge）查表时时间戳绝对对齐不延迟
        publish_tf_from_matrix();
    }

    void cb_smoothed_localization(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!initialized_) {
            return;
        }

        Eigen::Matrix4f local_last_known_pose_smoothed = poseToMatrix(msg->pose.pose);
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_known_pose_smoothed_ = local_last_known_pose_smoothed;
        }

        // 组装并发布平展平面下的可视化路径
        Eigen::Vector3f t_map = local_last_known_pose_smoothed.block<3, 1>(0, 3);
        Eigen::Quaternionf q_map(local_last_known_pose_smoothed.block<3, 3>(0, 0));

        geometry_msgs::msg::PoseStamped map_pose_stamped;
        map_pose_stamped.header.stamp = msg->header.stamp;
        map_pose_stamped.header.frame_id = "map";
        map_pose_stamped.pose.position.x = t_map.x();
        map_pose_stamped.pose.position.y = t_map.y();
        map_pose_stamped.pose.position.z = t_map.z();
        map_pose_stamped.pose.orientation.w = q_map.w();
        map_pose_stamped.pose.orientation.x = q_map.x();
        map_pose_stamped.pose.orientation.y = q_map.y();
        map_pose_stamped.pose.orientation.z = q_map.z();

        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path_record_.header.stamp = msg->header.stamp;
            path_record_.header.frame_id = "map";
            path_record_.poses.push_back(map_pose_stamped);
            pub_path_->publish(path_record_);
        }

        // 发送给交互终端界面
        geometry_msgs::msg::Pose ui_pose = map_pose_stamped.pose;
        pub_ui_projected_pose_->publish(ui_pose);
    }

    // 对接收到的传感器实时注册点云进行去除无效数据、去极值、几何转换和映射发布
    void cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        
        // 核心解耦：无条件缓存最新的高频点云数据，确保后端评估函数永远能获取有效点云
        cur_scan_ = cloud;

        // 之后再针对初始化状态进行可视化话题的拦截拦截
        Eigen::Matrix4f local_T_map_to_odom;
        Eigen::Matrix4f local_T_map_to_baselink_smoothed;
        Eigen::Matrix4f local_T_odom_current;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            local_T_map_to_odom = T_map_to_odom_;
            local_T_map_to_baselink_smoothed = last_known_pose_smoothed_;
            if (cur_odom_) {
                local_T_odom_current = poseToMatrix(cur_odom_->pose.pose);
            } else {
                local_T_odom_current = Eigen::Matrix4f::Identity();
            }
        }

        if (!initialized_ || !cur_odom_ || !local_T_map_to_odom.allFinite()) {
            // 如果未定位，发布空的点云话题维持总线活跃
            sensor_msgs::msg::PointCloud2 empty_msg;
            empty_msg.header.stamp = msg->header.stamp;
            empty_msg.header.frame_id = "map";
            pub_pc_in_map_->publish(empty_msg);
            pub_scan_->publish(empty_msg);
            return;
        }

        // 修复：为了让 Scan 点云的显示与平滑后的轨迹线完全贴合，不产生虚影和分离，
        // 我们需要使用平滑过后的 T_map_to_odom 来对 Scan 进行坐标系变换，而不是原生的 raw local_T_map_to_odom。
        Eigen::Matrix4f T_map_to_odom_smoothed = local_T_map_to_baselink_smoothed * T_body_to_baselink_.inverse() * local_T_odom_current.inverse();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud, *cloud_in_map, T_map_to_odom_smoothed);

        sensor_msgs::msg::PointCloud2 pub_msg;
        pcl::toROSMsg(*cloud_in_map, pub_msg);
        pub_msg.header.stamp = msg->header.stamp;
        pub_msg.header.frame_id = "map";
        pub_pc_in_map_->publish(pub_msg);
        pub_scan_->publish(pub_msg);
    }

    // 在预估基座位姿的球形范围内裁剪全局先验地图，有效抑制错误纹理并保障运算效率
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_global_map_in_FOV(Eigen::Matrix4f pose_estimation) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_in_radius(new pcl::PointCloud<pcl::PointXYZ>());
        
        float px = pose_estimation(0, 3);
        float py = pose_estimation(1, 3);
        float pz = pose_estimation(2, 3);
        float r2 = CROP_RADIUS_M_ * CROP_RADIUS_M_;

        // 避免全图做昂贵的 pcl::transformPointCloud，直接利用欧氏距离提取目标
        for (const auto& pt : global_map_->points) {
            float dx = pt.x - px;
            float dy = pt.y - py;
            float dz = pt.z - pz;
            if (dx * dx + dy * dy + dz * dz < r2) {
                map_in_radius->points.push_back(pt);
            }
        }
        
        map_in_radius->width = map_in_radius->points.size();
        map_in_radius->height = 1;
        map_in_radius->is_dense = true;
        return map_in_radius;
    }

    // 利用 Fast-VGICP 评估输入预测位姿，返回精细对齐后的几何变换及其评估指标
    ICPResult evaluate_icp(Eigen::Matrix4f pose_estimation, bool is_lidar = false) {
        (void)is_lidar;
        ICPResult res;
        res.valid = false;
        res.fitness = DEFAULT_FITNESS_SCORE_;
        res.T_final = Eigen::Matrix4f::Identity();

        if (!cur_scan_ || cur_scan_->empty() || !cur_odom_ || global_map_->empty()) return res;

        // 通过反向变换将原本已经投射至里程计坐标系的点云还原至车辆真实物理几何坐标系中
        Eigen::Matrix4f T_odom_to_base_link = poseToMatrix(cur_odom_->pose.pose);
        Eigen::Matrix4f T_base_link_to_odom = safeInverse(T_odom_to_base_link);

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_base_link(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cur_scan_, *scan_base_link, T_base_link_to_odom);

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_down = voxelDownSample(scan_base_link, SCAN_VOXEL_SIZE_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        if (!initialized_ && has_been_localized_once_ && global_kdtree_) {
            pcl::PointXYZ searchPoint;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                searchPoint.x = last_known_pose_(0, 3);
                searchPoint.y = last_known_pose_(1, 3);
                searchPoint.z = last_known_pose_(2, 3);
            }
            
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr sub_map(new pcl::PointCloud<pcl::PointXYZ>());
            if (global_kdtree_->radiusSearch(searchPoint, r_current_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                sub_map->points.reserve(pointIdxRadiusSearch.size());
                for (int idx : pointIdxRadiusSearch) {
                    sub_map->points.push_back(global_map_->points[idx]);
                }
                sub_map->width = sub_map->points.size();
                sub_map->height = 1;
                sub_map->is_dense = true;
            }
            
            RCLCPP_INFO(this->get_logger(),
                        "Local pointcloud submap extracted successfully. Region volume points: %lu. Evaluating registration convergence...",
                        sub_map->points.size());

            target_cloud = voxelDownSample(sub_map, 0.4f);
        } else {
            target_cloud = crop_global_map_in_FOV(pose_estimation);
            // 对局部追踪裁剪出的稠密目标地图进行降采样，大幅降低 VGICP 运算负荷
            target_cloud = voxelDownSample(target_cloud, 0.2f);
        }

        if (scan_down->empty() || target_cloud->empty()) {
            return res;
        }

        fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setNumThreads(omp_get_max_threads());
        icp.setResolution(ICP_RESOLUTION_);
        icp.setInputSource(scan_down);
        icp.setInputTarget(target_cloud);

        icp.setMaximumIterations(ICP_MAX_ITERATIONS_);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE_);

        auto tic = std::chrono::steady_clock::now();

        // 异常拦截：如果可用计算点数过少，直接规避可能引发除零数学崩溃的情况
        if (scan_down->points.size() < MIN_SCAN_POINTS_ || target_cloud->points.size() < MIN_MAP_POINTS_) {
            RCLCPP_ERROR(this->get_logger(),
                         "Local point cloud too sparse (Scan points: %lu, Map points: %lu). Abandoning ICP to prevent division by zero.",
                         scan_down->points.size(), target_cloud->points.size());
            res.fitness = DEFAULT_FITNESS_SCORE_;
            res.valid = false;
            res.T_final = Eigen::Matrix4f::Identity();
            return res;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>());
        icp.align(*Final, pose_estimation);

        res.fitness = icp.getFitnessScore(FITNESS_SCORE_MAX_DIST_);
        res.valid = icp.hasConverged();
        res.T_final = icp.getFinalTransformation();

        if (res.valid && Final->size() > 0) {
            Eigen::Matrix3d H_trans = Eigen::Matrix3d::Zero();
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(target_cloud);

            int count = 0;
            // 抽样策略：每 5 个点抽取 1 个算协方差，特征值比例恒定，但计算量锐减 80%
            for (size_t i = 0; i < Final->size(); i += 5) {
                const auto& pi = Final->points[i];
                std::vector<int> pointIdxNKNSearch(10);
                std::vector<float> pointNKNSquaredDistance(10);

                if (kdtree.nearestKSearch(pi, 10, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    if (pointNKNSquaredDistance[0] > FITNESS_SCORE_MAX_DIST_ * FITNESS_SCORE_MAX_DIST_) {
                        continue;
                    }

                    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                    for (int k = 0; k < 10; ++k) {
                        const auto& pk = target_cloud->points[pointIdxNKNSearch[k]];
                        mean += Eigen::Vector3d(pk.x, pk.y, pk.z);
                    }
                    mean /= 10.0;

                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (int k = 0; k < 10; ++k) {
                        const auto& pk = target_cloud->points[pointIdxNKNSearch[k]];
                        Eigen::Vector3d diff = Eigen::Vector3d(pk.x, pk.y, pk.z) - mean;
                        cov += diff * diff.transpose();
                    }
                    cov /= 10.0;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                    Eigen::Vector3d eigenvalues = es.eigenvalues();
                    double e1 = eigenvalues(0);
                    double e2 = eigenvalues(1);
                    double e3 = eigenvalues(2);

                    double sum_e = e1 + e2 + e3;
                    if (sum_e < 1e-6) {
                        continue;
                    }

                    double curvature = e1 / sum_e;
                    double linearity = (e3 - e2) / e3;

                    // 只提取曲率极大的边缘点（Edge）和法向量极度明确的平整面点（Surf）计算信息矩阵
                    bool is_surf = (curvature < 0.05);
                    bool is_edge = (linearity > 0.6);

                    if (!is_surf && !is_edge) {
                        continue;
                    }

                    Eigen::Vector3d normal = es.eigenvectors().col(0);

                    Eigen::Vector3d J_trans = normal;
                    H_trans += J_trans * J_trans.transpose();
                    count++;
                }
            }

            if (count > 10) {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_trans(H_trans);
                double lam_min_trans = es_trans.eigenvalues()(0);
                double lam_max_trans = es_trans.eigenvalues()(2);
                if (lam_max_trans > 1e-6) {
                    res.degenerate_ratio = lam_min_trans / lam_max_trans;
                }
            }
        }

        if (!res.T_final.allFinite()) {
            RCLCPP_ERROR(this->get_logger(),
                         "ICP convergence resulted in invalid numerical transformation matrix (NaN/Inf). Intercepted.");
            res.fitness = DEFAULT_FITNESS_SCORE_;
            res.valid = false;
            return res;
        }

        auto toc = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = toc - tic;

        RCLCPP_INFO(this->get_logger(),
                    "ICP alignment completed (Time: %.3fs, Converged: %d, MSE: %.4f).",
                    time_used.count(), res.valid, res.fitness);

        return res;
    }

    // 变换树消息组合发布
    void publish_tf_from_matrix() {
        if (!has_been_localized_once_) {
            return;
        }
        Eigen::Matrix4f local_T_map_to_odom;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            local_T_map_to_odom = T_map_to_odom_;
        }
        if (!local_T_map_to_odom.allFinite()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Attempted to publish non-finite transformation matrix. Intercepted.");
            return;
        }
        Eigen::Matrix3f R = local_T_map_to_odom.block<3, 3>(0, 0);
        if (!R.allFinite()) return;

        Eigen::Quaternionf q(R);
        if (q.norm() > 1e-6 && std::isfinite(q.norm())) {
            q.normalize();
        } else {
            q = Eigen::Quaternionf::Identity();
        }
        Eigen::Vector3f t = local_T_map_to_odom.block<3, 1>(0, 3);

        nav_msgs::msg::Odometry map_to_odom;
        map_to_odom.header.stamp = cur_odom_->header.stamp;
        map_to_odom.header.frame_id = "map";
        map_to_odom.pose.pose.position.x = t.x();
        map_to_odom.pose.pose.position.y = t.y();
        map_to_odom.pose.pose.position.z = t.z();
        map_to_odom.pose.pose.orientation.x = q.x();
        map_to_odom.pose.pose.orientation.y = q.y();
        map_to_odom.pose.pose.orientation.z = q.z();
        map_to_odom.pose.pose.orientation.w = q.w();
        pub_map_to_odom_->publish(map_to_odom);

        // 移除原有的 tf_broadcaster_->sendTransform(tf_msg)
        // 严重 Bug 修复：map -> camera_init 的 TF 树只能由 transform_fusion 这个专门负责平滑的节点来发布。
        // 如果这里也直接发布，会导致 TF 树发生 2 个节点争抢，RViz 里会出现极其剧烈的 Z 字形瞬间跳变（锯齿）。
        // 本节点只负责通过 /map_to_odom 话题向外输出目标位姿即可。
    }

    // 校验视觉位姿估算，通过双向限制与滤波，判定是否达成全局初始化
    void cb_initialpose_visual(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (initialized_ || tracking_state_visual_ == TrackingState::PROVISIONAL_TRACKING) return;
        if (!map_received_ || !cur_odom_ || msg->poses.empty()) return;

        last_visual_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(),
                     "Received visual initial pose proposal. Starting ICP cross-validation.");

        Eigen::Matrix4f T_map_to_base_link_visual = poseToMatrix(msg->poses[0]);
        Eigen::Matrix4f T_odom_to_base_link = poseToMatrix(cur_odom_->pose.pose);

        ICPResult res = evaluate_icp(T_map_to_base_link_visual, false);
        float shift_x = res.T_final(0, 3) - T_map_to_base_link_visual(0, 3);
        float shift_y = res.T_final(1, 3) - T_map_to_base_link_visual(1, 3);
        float shift_dist = std::sqrt(shift_x * shift_x + shift_y * shift_y);

        RCLCPP_INFO(this->get_logger(), "Environment Degenerate Factor (3-DoF XYZ): %.4f (Strict Threshold: 0.05)", res.degenerate_ratio);

        if (res.fitness < SCORE_THRESHOLD_INITIAL_ && shift_dist < VISUAL_MAX_INITIAL_SHIFT_M_ && res.T_final.allFinite() &&
            T_odom_to_base_link.allFinite()) {
            if (res.degenerate_ratio <= 0.05) {
                RCLCPP_DEBUG(this->get_logger(), "Visual proposal initialization rejected: Axis-degradation verified. Current ratio: %.4f, Threshold: 0.05.", res.degenerate_ratio);
                if (visual_reject_cnt_ < MAX_REJECTS_A_) {
                    visual_reject_cnt_++;
                    if (visual_reject_cnt_.load() == MAX_REJECTS_A_) {
                        is_lidar_allowed_.store(true); // 唯一次单向解锁雷达双通道竞争
                        RCLCPP_DEBUG(this->get_logger(), "Visual rejection limit reached. Current: %d, Threshold: %d. Global Lidar channel UNLOCKED.", visual_reject_cnt_.load(), MAX_REJECTS_A_);
                        std_msgs::msg::Bool force_msg;
                        force_msg.data = true;
                        pub_force_lidar_->publish(force_msg);
                    }
                }
            } else {
                visual_reject_cnt_ = 0;
                tracking_loss_cnt_ = 0;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    odom_pos_at_last_correct_ = Eigen::Vector3f(cur_odom_->pose.pose.position.x,
                                                                 cur_odom_->pose.pose.position.y,
                                                                 cur_odom_->pose.pose.position.z);
                    odom_yaw_at_last_correct_ = getYaw(T_odom_to_base_link.block<3, 3>(0, 0).cast<double>());
                    accumulated_yaw_since_lost_deg_ = 0.0;
                    accumulated_distance_since_lost_ = 0.0;
                    coasting_budget_distance_ = 0.0;
                    T_map_to_odom_ = res.T_final * safeInverse(T_odom_to_base_link);
                    delta_s_ = 0.0;
                    last_known_pose_ = res.T_final;
                }
                tracking_state_visual_ = TrackingState::PROVISIONAL_TRACKING;
                provisional_success_count_visual_ = 1;
                provisional_timer_visual_ = 0;
                provisional_odom_pose_visual_ = T_odom_to_base_link;
                provisional_map_pose_visual_ = res.T_final;
                
                if (timer_) timer_->cancel();
                timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalLocalization::timer_localization_callback, this));
                
                RCLCPP_INFO(this->get_logger(),
                            "PROVISIONAL_TRACKING [1/%d]: Passed 1st ICP (Visual). Waiting for continuous blind-push validation over spatial gaps.", REQUIRED_PROVISIONAL_FRAMES_);
            }
        } else {
            if (res.fitness >= SCORE_THRESHOLD_INITIAL_) {
                RCLCPP_DEBUG(this->get_logger(),
                             "Visual proposal geometric matching degraded. Current MSE: %.4f, Threshold: %.2f.",
                             res.fitness, SCORE_THRESHOLD_INITIAL_);
            } else {
                RCLCPP_DEBUG(this->get_logger(),
                             "Visual proposal translation shift triggered. Current Shift: %.2fm, Threshold: %.2fm.",
                             shift_dist, VISUAL_MAX_INITIAL_SHIFT_M_);
            }
            if (visual_reject_cnt_ < MAX_REJECTS_A_) {
                visual_reject_cnt_++;
                if (visual_reject_cnt_.load() == MAX_REJECTS_A_) {
                    is_lidar_allowed_.store(true); // 唯一次单向解锁雷达双通道竞争
                    RCLCPP_DEBUG(this->get_logger(), "Visual rejection limit reached. Current: %d, Threshold: %d. Global Lidar channel UNLOCKED.", visual_reject_cnt_.load(), MAX_REJECTS_A_);
                    std_msgs::msg::Bool force_msg;
                    force_msg.data = true;
                    pub_force_lidar_->publish(force_msg);
                }
            }
        }
    }

    // 在视觉信号大范围丢帧或退化情况下，采用备用的雷达提案引导全局对齐
    void cb_initialpose_lidar(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (initialized_ || tracking_state_lidar_ == TrackingState::PROVISIONAL_TRACKING) return;
        if (!map_received_ || !cur_odom_ || msg->poses.empty()) return;



        // 只有锁打开后（5次之后），雷达提案才允许并存、进入下方的 ICP 交叉几何校验
        RCLCPP_DEBUG(this->get_logger(), "GlobalLoc: Processing incoming Pure-LiDAR global proposal under dual-channel competition mode.");

        Eigen::Matrix4f T_map_to_base_link_lidar = poseToMatrix(msg->poses[0]);
        Eigen::Matrix4f T_odom_to_base_link = poseToMatrix(cur_odom_->pose.pose);

        ICPResult res = evaluate_icp(T_map_to_base_link_lidar, true);
        float shift_x = res.T_final(0, 3) - T_map_to_base_link_lidar(0, 3);
        float shift_y = res.T_final(1, 3) - T_map_to_base_link_lidar(1, 3);
        float shift_dist = std::sqrt(shift_x * shift_x + shift_y * shift_y);

        RCLCPP_INFO(this->get_logger(), "Environment Degenerate Factor (3-DoF XYZ): %.4f (Strict Threshold: 0.05)", res.degenerate_ratio);

        if (res.fitness < LIDAR_SCORE_THRESHOLD_INITIAL_ && shift_dist < LIDAR_MAX_INITIAL_SHIFT_M_ &&
            res.T_final.allFinite() && T_odom_to_base_link.allFinite()) {
            if (res.degenerate_ratio <= 0.05) {
                RCLCPP_DEBUG(this->get_logger(), "LiDAR proposal initialization rejected: Axis-degradation verified. Current ratio: %.4f, Threshold: 0.05.", res.degenerate_ratio);
            } else {
                tracking_loss_cnt_ = 0;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    odom_pos_at_last_correct_ = Eigen::Vector3f(cur_odom_->pose.pose.position.x,
                                                                 cur_odom_->pose.pose.position.y,
                                                                 cur_odom_->pose.pose.position.z);
                    odom_yaw_at_last_correct_ = getYaw(T_odom_to_base_link.block<3, 3>(0, 0).cast<double>());
                    accumulated_yaw_since_lost_deg_ = 0.0;
                    accumulated_distance_since_lost_ = 0.0;
                    coasting_budget_distance_ = 0.0;
                    T_map_to_odom_ = res.T_final * safeInverse(T_odom_to_base_link);
                    delta_s_ = 0.0;
                    last_known_pose_ = res.T_final;
                }
                
                tracking_state_lidar_ = TrackingState::PROVISIONAL_TRACKING;
                provisional_success_count_lidar_ = 1;
                provisional_timer_lidar_ = 0;
                provisional_odom_pose_lidar_ = T_odom_to_base_link;
                provisional_map_pose_lidar_ = res.T_final;

                if (timer_) timer_->cancel();
                timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalLocalization::timer_localization_callback, this));

                RCLCPP_INFO(this->get_logger(),
                            "PROVISIONAL_TRACKING [1/%d]: Passed 1st ICP (LiDAR). Waiting for continuous blind-push validation over spatial gaps.", REQUIRED_PROVISIONAL_FRAMES_);
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                         "LiDAR proposal geometric matching degraded or shift triggered. Current MSE: %.4f, Shift: %.2fm.",
                         res.fitness, shift_dist);
        }
    }

    void cb_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Received 2D initial pose arrow from RViz.");

        // 1. 重置状态标志并激活 10 帧流延迟滤波器
        initialized_ = false;
        stable_stream_delay_idx_.store(0);

        // 2. 清空 RViz 路径并发送清除触发信号给 UI 以抹除旧轨迹线
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path_record_.poses.clear();
            pub_path_->publish(path_record_);
        }
        
        std_msgs::msg::Bool ui_clear_msg;
        ui_clear_msg.data = true;
        pub_ui_clear_trigger_->publish(ui_clear_msg);

        if (!cur_odom_) {
            RCLCPP_WARN(this->get_logger(), "Missing base odometry, reject manual override.");
            return;
        }

        // 3. 解析输入位姿并对齐到 map 坐标系
        Eigen::Matrix4f T_target_in_msg = poseToMatrix(msg->pose.pose);
        T_manual_base_target_ = T_target_in_msg;

        // 4. 将 Z 轴偏移强制归零以尊重手动放置
        T_manual_base_target_(2, 3) = 0.0f;

        // 5. 将目标位姿设为 map 坐标系下的位姿
        Eigen::Matrix4f T_map_to_base_target = T_manual_base_target_;
        Eigen::Matrix4f T_odom_current = poseToMatrix(cur_odom_->pose.pose);
        
        // 通过引入外参矩阵 T_body_to_baselink_ 来更新基准变换 T_map_to_odom_
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            T_map_to_odom_ = T_map_to_base_target * safeInverse(T_odom_current * T_body_to_baselink_);
            delta_s_ = 0.0; // 重置累计距离，以便 UI 从 0 开始计算距离
            last_known_pose_ = T_map_to_base_target;
        }
        
        // 6. 重置跟踪计数器并清空累计距离
        tracking_loss_cnt_ = 0;
        accumulated_yaw_since_lost_deg_ = 0.0;
        accumulated_distance_since_lost_ = 0.0;
        coasting_budget_distance_ = 0.0;
        has_been_localized_once_ = true;

        clear_highlight_submap(false);
        initialized_ = true;

        // 激活定位检查定时器并发布状态以使视觉重定位休眠
        if (timer_) timer_->cancel();
        timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalLocalization::timer_localization_callback, this));
        publish_tf_from_matrix();

        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        pub_status_->publish(status_msg);

        RCLCPP_INFO(this->get_logger(), "Success! Updated T_map_to_odom_. Tracking initialized.");
        RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");
    }

    void set_localization_timer_interval(int interval_ms) {
        if (current_timer_interval_ms_ == interval_ms) return;
        current_timer_interval_ms_ = interval_ms;
        if (timer_) {
            timer_->cancel();
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&GlobalLocalization::timer_localization_callback, this));
        RCLCPP_INFO(this->get_logger(),
                    "Adaptive Localization Timer: Frequency switched to %dms interval.", interval_ms);
    }

    // 定期执行局部精调对齐，跟踪全局位置变化，并在持续偏离较大时回收定位锁
    void timer_localization_callback() {
        if ((!initialized_ && tracking_state_visual_ != TrackingState::PROVISIONAL_TRACKING && tracking_state_lidar_ != TrackingState::PROVISIONAL_TRACKING) || !cur_odom_) return;

        Eigen::Matrix4f T_odom_to_base_link = poseToMatrix(cur_odom_->pose.pose);
        
        auto process_provisional = [&](TrackingState& state, int& count, int& timer_count, Eigen::Matrix4f& odom_pose, Eigen::Matrix4f& map_pose, bool is_visual, const std::string& name) {
            if (state == TrackingState::PROVISIONAL_TRACKING && count < REQUIRED_PROVISIONAL_FRAMES_) {
                float dx = T_odom_to_base_link(0,3) - odom_pose(0,3);
                float dy = T_odom_to_base_link(1,3) - odom_pose(1,3);
                float dz = T_odom_to_base_link(2,3) - odom_pose(2,3);
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                timer_count++;

                if (dist < PROVISIONAL_DISTANCE_GAP_ && timer_count < PROVISIONAL_TIMEOUT_SEC_) {
                    return;
                }

                Eigen::Matrix4f guess_base_link = map_pose * safeInverse(odom_pose) * T_odom_to_base_link;
                ICPResult res = evaluate_icp(guess_base_link, !is_visual);
                
                double degen_th = is_visual ? VISUAL_DEGENERATE_THRESHOLD_ : LIDAR_DEGENERATE_THRESHOLD_;
                if (res.fitness < SCORE_THRESHOLD_INITIAL_ && res.degenerate_ratio > degen_th) {
                    count++;
                    timer_count = 0; // 重置定时器，进入下一帧考察
                    RCLCPP_INFO(this->get_logger(),
                                "PROVISIONAL_TRACKING [%d/%d]: Validation passed (%s). (Dist: %.2fm, MSE: %.4f, Degen: %.4f)", 
                                count, REQUIRED_PROVISIONAL_FRAMES_, name.c_str(), dist, res.fitness, res.degenerate_ratio);
                    
                    odom_pose = T_odom_to_base_link;
                    map_pose = res.T_final;
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "PROVISIONAL_TRACKING (%s) failed at count %d. (Dist: %.2fm, MSE: %.4f, Degen: %.4f). Revoking...",
                                name.c_str(), count, dist, res.fitness, res.degenerate_ratio);
                    state = TrackingState::SEARCHING;
                    count = 0;
                    timer_count = 0;
                    revoke_localization("Spatial/Temporal provisional failed", guess_base_link);
                }
            }
        };

        if (!initialized_) {
            if (tracking_state_visual_ == TrackingState::PROVISIONAL_TRACKING) {
                process_provisional(tracking_state_visual_, provisional_success_count_visual_, provisional_timer_visual_, provisional_odom_pose_visual_, provisional_map_pose_visual_, true, "Visual");
            }
            if (tracking_state_lidar_ == TrackingState::PROVISIONAL_TRACKING) {
                process_provisional(tracking_state_lidar_, provisional_success_count_lidar_, provisional_timer_lidar_, provisional_odom_pose_lidar_, provisional_map_pose_lidar_, false, "LiDAR");
            }
            
            bool visual_ready = (tracking_state_visual_ == TrackingState::PROVISIONAL_TRACKING && provisional_success_count_visual_ >= REQUIRED_PROVISIONAL_FRAMES_);
            bool lidar_ready = (tracking_state_lidar_ == TrackingState::PROVISIONAL_TRACKING && provisional_success_count_lidar_ >= REQUIRED_PROVISIONAL_FRAMES_);
            
            bool visual_active = (tracking_state_visual_ == TrackingState::PROVISIONAL_TRACKING);
            bool lidar_active = (tracking_state_lidar_ == TrackingState::PROVISIONAL_TRACKING);

            auto execute_lock = [&](Eigen::Matrix4f final_map_pose, Eigen::Matrix4f final_odom_pose, const std::string& winner_name, const std::string& reason) {
                initialized_.store(true);
                tracking_state_visual_ = TrackingState::LOCKED;
                tracking_state_lidar_ = TrackingState::LOCKED;
                is_lidar_allowed_.store(false);
                has_been_localized_once_ = true;
                last_locked_odom_pose_ = std::nullopt;
                if (adaptive_timer_) {
                    adaptive_timer_->cancel();
                }
                clear_highlight_submap();
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    T_map_to_odom_ = final_map_pose * safeInverse(final_odom_pose);
                    delta_s_ = 0.0;
                    last_known_pose_ = final_map_pose;
                    
                    odom_pos_at_last_correct_ = Eigen::Vector3f(final_odom_pose(0, 3),
                                                                final_odom_pose(1, 3),
                                                                final_odom_pose(2, 3));
                    odom_yaw_at_last_correct_ = getYaw(final_odom_pose.block<3, 3>(0, 0).cast<double>());
                    accumulated_yaw_since_lost_deg_ = 0.0;
                    accumulated_distance_since_lost_ = 0.0;
                    coasting_budget_distance_ = 0.0;
                }
                publish_tf_from_matrix();
                
                std_msgs::msg::Bool status_msg;
                status_msg.data = true;
                pub_status_->publish(status_msg);
                RCLCPP_INFO(this->get_logger(),
                            "\033[1;32m%s Localization LOCKED\033[0m - %s. Broadcasting status.", 
                            winner_name.c_str(), reason.c_str());
            };

            if (visual_ready && lidar_ready) {
                float dx = provisional_map_pose_visual_(0,3) - provisional_map_pose_lidar_(0,3);
                float dy = provisional_map_pose_visual_(1,3) - provisional_map_pose_lidar_(1,3);
                float dz = provisional_map_pose_visual_(2,3) - provisional_map_pose_lidar_(2,3);
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                if (dist <= 3.0f) {
                    RCLCPP_INFO(this->get_logger(), "Cross-Veto: Visual and LiDAR AGREE (dist: %.2fm <= 3.0m). Approving Visual Pose.", dist);
                    execute_lock(provisional_map_pose_visual_, provisional_odom_pose_visual_, "Visual+LiDAR", "Cross-Veto dual validation passed");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Cross-Veto: CONFLICT! Visual and LiDAR differ by %.2fm. Vetoing BOTH!", dist);
                    tracking_state_visual_ = TrackingState::SEARCHING;
                    provisional_success_count_visual_ = 0;
                    provisional_timer_visual_ = 0;
                    tracking_state_lidar_ = TrackingState::SEARCHING;
                    provisional_success_count_lidar_ = 0;
                    provisional_timer_lidar_ = 0;
                }
            } else if (visual_ready && !lidar_active) {
                RCLCPP_INFO(this->get_logger(), "Cross-Veto: Visual ready, LiDAR abstained/failed. Approving Visual Pose.");
                execute_lock(provisional_map_pose_visual_, provisional_odom_pose_visual_, "Visual", "Single modality fallback (LiDAR inactive)");
            } else if (lidar_ready && !visual_active) {
                RCLCPP_INFO(this->get_logger(), "Cross-Veto: LiDAR ready, Visual abstained/failed. Approving LiDAR Pose.");
                execute_lock(provisional_map_pose_lidar_, provisional_odom_pose_lidar_, "LiDAR", "Single modality fallback (Visual inactive)");
            } else if (visual_ready && lidar_active) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cross-Veto: Visual ready, waiting for LiDAR to finish validation...");
            } else if (lidar_ready && visual_active) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cross-Veto: LiDAR ready, waiting for Visual to finish validation...");
            }

            return;
        }

        Eigen::Matrix4f guess_base_link;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            guess_base_link = T_map_to_odom_ * T_odom_to_base_link;
        }

        // [定位核心重构] 楼梯退化场景感知 (Degeneracy-Aware Coasting)
        // 严格对齐 UI 界面的真实水平俯仰角 (消除地图初始倾斜与雷达硬件外参安装倾角)
        Eigen::Matrix4f ui_chassis_pose = guess_base_link * T_body_to_baselink_;
        Eigen::Matrix3d R_ui = ui_chassis_pose.block<3, 3>(0, 0).cast<double>();

        double global_roll_deg = 0.0;
        double global_pitch_deg = 0.0;
        double global_yaw_deg = 0.0;
        extractEulerAngles(R_ui, global_roll_deg, global_pitch_deg, global_yaw_deg);

        // 设定楼梯判别阈值：超过此物理倾斜角即视为进入楼梯退化地形
        const double STAIR_PITCH_THRESHOLD_DEG = 15.0;
        bool is_physically_on_stairs = (global_pitch_deg > STAIR_PITCH_THRESHOLD_DEG);

        if (is_physically_on_stairs) {
            auto clock = this->get_clock();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000,
                        "Degeneracy-Aware: Staircase mode ACTIVE. Global ICP tracking PAUSED. Coasting purely on Odometry.");
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                coasting_budget_distance_ = 0.0; // 楼梯物理护盾：重置物理生存预算，不累加跟踪丢失计数
            }
            return; // 直接返回，完全屏蔽本帧 ICP 修正，彻底消除楼梯处的 Z 轴退化拉扯
        }

        ICPResult res = evaluate_icp(guess_base_link);

        float dx = res.T_final(0, 3) - guess_base_link(0, 3);
        float dy = res.T_final(1, 3) - guess_base_link(1, 3);
        float dz = res.T_final(2, 3) - guess_base_link(2, 3);
        float trans_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

        Eigen::Matrix3d R_guess = guess_base_link.block<3, 3>(0, 0).cast<double>();
        Eigen::Matrix3d R_final = res.T_final.block<3, 3>(0, 0).cast<double>();
        Eigen::Matrix3d R_diff = R_guess.inverse() * R_final;

        double roll_diff_deg = 0.0;
        double pitch_diff_deg = 0.0;
        double yaw_diff_deg = 0.0;
        extractEulerAngles(R_diff, roll_diff_deg, pitch_diff_deg, yaw_diff_deg);

        // 计算里程计盲推的路程积分（自上次有效纠正定位后的实际移动轨迹长度）
        double dist_since_lost = 0.0;
        double budget_dist_since_lost = 0.0;
        if (tracking_loss_cnt_ > 0) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            dist_since_lost = accumulated_distance_since_lost_;
            budget_dist_since_lost = coasting_budget_distance_;
        }

        // ====================================================================
        // [动态平移阈值参数]
        // 使用指数饱和模型控制重定位的平移容忍度，防止阈值随时间无界膨胀
        // 公式：阈值 = D_base + D_span * (1 - exp(-k * dist))
        // ====================================================================
        const double D_base = 1.0;  // 基础底噪容忍度(m)：吸收传感器本底噪声与原地微小晃动
        const double D_span = 1.5;  // 最大膨胀跨度(m)：系统能容忍的额外物理拉扯极限
        const double k = 0.10;      // 漂移膨胀率：控制容忍度随盲推距离增长的快慢

        // 计算当前动态平移容忍上限
        double dynamic_trans_threshold = D_base + D_span * (1.0 - std::exp(-k * dist_since_lost));

        // --- [旋转阈值：重力锁 + 动态偏航角] ---
        const double MAX_ROLL_PITCH_DEG = 10.0; // 严格的重力约束：不允许超过 10 度的倾角突变

        // Yaw 动态阈值参数 (指数饱和模型)
        const double YAW_R_BASE = 10.0;  // 基础底噪容忍度
        const double YAW_R_SPAN = 10.0; // 甩头最大拉扯膨胀跨度
        const double YAW_K = 0.10;      // 膨胀速率

        double local_accumulated_yaw = 0.0;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            local_accumulated_yaw = accumulated_yaw_since_lost_deg_;
        }
        double dynamic_yaw_threshold = YAW_R_BASE + YAW_R_SPAN * (1.0 - std::exp(-YAW_K * local_accumulated_yaw));
        bool rotation_valid = (roll_diff_deg < MAX_ROLL_PITCH_DEG) &&
                              (pitch_diff_deg < MAX_ROLL_PITCH_DEG) &&
                              (yaw_diff_deg < dynamic_yaw_threshold);

        // 探针修复：因为 VGICP 的 MaxCorrespondenceDistance 设置为 1.5m，
        // 所以单点最大可能误差(MSE)理论上限为 1.5^2 = 2.25。
        // 将爆炸阈值设置在 10.0 会导致物理上永远无法触发。根据要求，设定为 2.0，代表接近理论极限的极度发散。
        const double SCORE_THRESHOLD_EXPLOSION = 2.0;

        if (res.fitness < SCORE_THRESHOLD_TRACKING_) {
            // 分支 A: MSE < 0.20 - 满足严格精度，允许修正轨迹
            if (trans_diff < dynamic_trans_threshold && rotation_valid) {
                RCLCPP_INFO(this->get_logger(),
                            "Low-frequency tracking correction applied (MSE: %.4f, Trans: %.3fm (< %.3fm), "
                            "Roll: %.1f° (< 10.0°), Pitch: %.1f° (< 10.0°), Yaw: %.1f° (< %.1f°)).",
                            res.fitness, trans_diff, dynamic_trans_threshold,
                            roll_diff_deg, pitch_diff_deg, yaw_diff_deg, dynamic_yaw_threshold);

                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    odom_pos_at_last_correct_ = Eigen::Vector3f(cur_odom_->pose.pose.position.x,
                                                                 cur_odom_->pose.pose.position.y,
                                                                 cur_odom_->pose.pose.position.z);
                    odom_yaw_at_last_correct_ = getYaw(T_odom_to_base_link.block<3, 3>(0, 0).cast<double>());
                    accumulated_yaw_since_lost_deg_ = 0.0;
                    accumulated_distance_since_lost_ = 0.0;
                    coasting_budget_distance_ = 0.0;
                    T_map_to_odom_ = res.T_final * safeInverse(T_odom_to_base_link);
                    last_known_pose_ = res.T_final;
                }
                tracking_loss_cnt_ = 0;
                publish_tf_from_matrix();
                set_localization_timer_interval(2000); // 成功高精度匹配：降频至 2s一次 节省算力
            } else {
                set_localization_timer_interval(1000); // 平移/旋转超限：恢复 1s一次 密切监管
                tracking_loss_cnt_++;
                if (tracking_loss_cnt_ >= MAX_TRACKING_LOSS_FRAMES_) {
                    revoke_localization("Trans/Rot exceeded", guess_base_link);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Accurate MSE (%.4f < 0.20) but translation/rotation check failed "
                                "(Trans: %.3fm %s %.3fm, Roll: %.1f° %s 10.0°, Pitch: %.1f° %s 10.0°, Yaw: %.1f° %s %.1f°). "
                                "Coasting on Odometry (%d/%d).",
                                res.fitness,
                                trans_diff, (trans_diff >= dynamic_trans_threshold ? ">=" : "<"), dynamic_trans_threshold,
                                roll_diff_deg, (roll_diff_deg >= MAX_ROLL_PITCH_DEG ? ">=" : "<"),
                                pitch_diff_deg, (pitch_diff_deg >= MAX_ROLL_PITCH_DEG ? ">=" : "<"),
                                yaw_diff_deg, (yaw_diff_deg >= dynamic_yaw_threshold ? ">=" : "<"), dynamic_yaw_threshold,
                                tracking_loss_cnt_, MAX_TRACKING_LOSS_FRAMES_);
                }
            }
        } else if (res.fitness <= SCORE_THRESHOLD_COASTING_) {
            set_localization_timer_interval(1000);
            // 分支 B: 0.20 <= MSE <= 0.40 - 折中缓冲区域：拒绝轨迹修正，维持现状，同时冻结倒计时计数器
            if (trans_diff < dynamic_trans_threshold && rotation_valid) {
                RCLCPP_INFO(this->get_logger(),
                            "Intermediate tracking quality (0.20 <= MSE: %.4f <= 0.40). "
                            "Rejecting correction, coasting on Odometry (Countdown frozen at %d/%d).",
                            res.fitness, tracking_loss_cnt_, MAX_TRACKING_LOSS_FRAMES_);
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    coasting_budget_distance_ = 0.0; // 过渡状态：重置 5m 物理生存预算
                }
            } else {
                // 平移或旋转超限，这绝对不是过渡，而是假匹配！必须按失败处理，累加计数！
                tracking_loss_cnt_++;
                const double MAX_COASTING_DISTANCE_M = 5.0; // 盲推允许的最大物理路程
                if (tracking_loss_cnt_ >= MAX_TRACKING_LOSS_FRAMES_) {
                    if (budget_dist_since_lost <= MAX_COASTING_DISTANCE_M) {
                        RCLCPP_WARN(this->get_logger(),
                                    "Spatio-Temporal Shield: Coasting frames maxed out (%d), but robot only moved %.2fm (<= %.1fm). Holding state.",
                                    tracking_loss_cnt_, budget_dist_since_lost, MAX_COASTING_DISTANCE_M);
                        tracking_loss_cnt_ = MAX_TRACKING_LOSS_FRAMES_ - 1; // 冻结在阈值边缘
                    } else {
                        revoke_localization("Trans/Rot exceeded during coasting", guess_base_link);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Coasting MSE (%.4f) but physical geometry rejected "
                                "(Trans: %.3fm %s %.3fm, Roll: %.1f° %s 10.0°, Pitch: %.1f° %s 10.0°, Yaw: %.1f° %s %.1f°). "
                                "Coasting aborted, count: %d/%d.",
                                res.fitness,
                                trans_diff, (trans_diff >= dynamic_trans_threshold ? ">=" : "<"), dynamic_trans_threshold,
                                roll_diff_deg, (roll_diff_deg >= MAX_ROLL_PITCH_DEG ? ">=" : "<"),
                                pitch_diff_deg, (pitch_diff_deg >= MAX_ROLL_PITCH_DEG ? ">=" : "<"),
                                yaw_diff_deg, (yaw_diff_deg >= dynamic_yaw_threshold ? ">=" : "<"), dynamic_yaw_threshold,
                                tracking_loss_cnt_, MAX_TRACKING_LOSS_FRAMES_);
                }
            }
        } else if (res.fitness < SCORE_THRESHOLD_EXPLOSION) {
            set_localization_timer_interval(1000);
            // 分支 C: 0.40 < MSE < 2.0 - 严重失配，拒绝轨迹修正，触发倒计时
            tracking_loss_cnt_++;
            const double MAX_COASTING_DISTANCE_M = 5.0; // 盲推允许的最大物理路程
            if (tracking_loss_cnt_ >= MAX_TRACKING_LOSS_FRAMES_) {
                if (budget_dist_since_lost <= MAX_COASTING_DISTANCE_M) {
                    RCLCPP_WARN(this->get_logger(),
                                "Spatio-Temporal Shield: Tracking degraded frames maxed out (%d), but robot only moved %.2fm (<= %.1fm). Holding state.",
                                tracking_loss_cnt_, budget_dist_since_lost, MAX_COASTING_DISTANCE_M);
                    tracking_loss_cnt_ = MAX_TRACKING_LOSS_FRAMES_ - 1; // 冻结在阈值边缘
                } else {
                    revoke_localization("MSE > 0.40", guess_base_link);
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Local geometric tracking degraded (MSE: %.4f > 0.40). "
                            "Coasting on Odometry (%d/%d), dist/yaw since lost: %.3fm / %.1f°.",
                            res.fitness, tracking_loss_cnt_, MAX_TRACKING_LOSS_FRAMES_, dist_since_lost, local_accumulated_yaw);
            }
        } else {
            // 分支 D: 退化爆炸 (MSE >= 10.0) - 前端 LIO 发生 NaN 或极大值漂移
            RCLCPP_ERROR(this->get_logger(), 
                         "Degeneracy Explosion: MSE (%.4f) >= %.1f! Front-end LIVO odometry severely corrupted. Immediate revocation.", 
                         res.fitness, SCORE_THRESHOLD_EXPLOSION);
            revoke_localization("MSE Degeneracy Explosion", guess_base_link);
        }
    }

    // --- 自适应重定位状态机与子图高亮发布 ---
    void revoke_localization(const std::string& reason, const Eigen::Matrix4f& current_pose) {
        if (!initialized_) return;
        
        double old_delta_s = 0.0;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_known_pose_ = current_pose;
            old_delta_s = delta_s_;
            last_track_odometer_ = delta_s_; // 保存最后一次成功跟踪的距离用于计算置信度
            delta_s_ = 0.0; // 跟踪失败时重置累计距离
        }
        
        RCLCPP_ERROR(this->get_logger(),
                     "GlobalLoc: Tracking severely degraded (%s). Revoking localization state. Success track odometer delta_s: %.2fm.",
                     reason.c_str(), old_delta_s);
        
        has_been_localized_once_ = true; // 固化曾成功定位标志
        initialized_ = false;
        is_lidar_allowed_.store(true);  // 跟踪失败时立即解锁雷达通道以启动局部子图搜索
        
        t_lost_start_ = this->now();
        
        if (timer_) {
            timer_->cancel();
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ dummy_pt;
        dummy_pt.x = 0.0f; dummy_pt.y = 0.0f; dummy_pt.z = -999.0f; // 投射至物理深渊
        empty_cloud->points.push_back(dummy_pt);
        empty_cloud->width = 1; empty_cloud->height = 1; empty_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 clear_msg;
        pcl::toROSMsg(*empty_cloud, clear_msg);
        clear_msg.header.stamp = this->now();
        clear_msg.header.frame_id = "map";
        pub_pc_in_map_->publish(clear_msg);

        // 状态自愈撤回：跟踪丢失瞬间，强行抹除 RViz 飘移残留
        pub_scan_->publish(clear_msg);

        // 抹除 RViz 飘移路径残留，撤销绿色轨迹
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path_record_.poses.clear();
            pub_path_->publish(path_record_);
        }

        std_msgs::msg::Bool status_msg;
        status_msg.data = false;
        pub_status_->publish(status_msg);

        stable_stream_delay_idx_.store(0); // 丢失定位时重置计数器
        if (adaptive_timer_) {
            adaptive_timer_->reset();
        }
    }

    void TriggerLidarGlobalSearch() {
        RCLCPP_INFO(this->get_logger(), "GlobalLoc: Lidar global search active. Awaiting Lidar proposals from external scan-context node.");
    }

    void clear_highlight_submap(bool reset_manual = true) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI dummy_pt;
        dummy_pt.x = 0.0f; dummy_pt.y = 0.0f; dummy_pt.z = -999.0f; // 物理深渊投射
        dummy_pt.intensity = 0.0f;
        empty_cloud->points.push_back(dummy_pt);
        empty_cloud->width = 1; empty_cloud->height = 1; empty_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 clear_msg;
        pcl::toROSMsg(*empty_cloud, clear_msg);
        clear_msg.header.stamp = this->now();
        clear_msg.header.frame_id = "map";

        // 双链路同步清空：强行阻断任何残留
        adaptive_submap_pub_->publish(clear_msg);          // 原生红球自毁
        pub_submap_highlight_->publish(clear_msg);         // 影子红球自毁

        if (reset_manual) {
            is_manual_override_mode_ = false; // 切回算法原生模式
            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                path_record_.poses.clear();
                pub_path_->publish(path_record_); // 发空轨迹，让 RViz 画面上的绿线彻底干净消失
            }
            // 同步通知 UI 物理中断清空画布
            std_msgs::msg::Bool ui_clear_msg;
            ui_clear_msg.data = true;
            pub_ui_clear_trigger_->publish(ui_clear_msg);
        }
    }

    void trigger_heavy_global_relocalization() {
        has_been_localized_once_ = false;
        clear_highlight_submap();
    }

    void adaptive_timer_callback() {
        // 刚性防线 1：如果目前已经处于锁死定位状态，不执行子图检索
        if (initialized_) {
            return;
        }
        // 刚性防线 2：如果是刚开机、还从来没有成功定位过，坚决不允许越权执行与打印
        if (!has_been_localized_once_) {
            return;
        }

        double delta_t_prime = (this->now() - t_lost_start_).seconds();

        // 刚性防线：如果系统未定位，且由于视觉前端完全无信号导致计数器无法累加
        // 一旦自丢失/开机起流逝的时间 delta_t_prime 超过了 10.0 秒，后端直接刚性强行解封雷达通道！
        if (!initialized_ && !is_lidar_allowed_.load() && delta_t_prime > 10.0) {
            is_lidar_allowed_.store(true);
            RCLCPP_WARN(this->get_logger(), 
                        "GlobalLoc: Visual frontend is SILENT for %.1fs. Time-break guard triggered! Forced UNLOCKING Lidar global channel for dual-competition mode.", delta_t_prime);
        }
        double local_delta_s = 0.0;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            local_delta_s = last_track_odometer_; // 使用断配前积攒的有效跟踪里程
        }
        double r_initial = 20.0 / std::log10(std::max(local_delta_s, 10.0));
        // 完美对齐机器狗物理运动速度的 1.0m/s 膨胀方程
        double r_current = r_initial + 1.0 * delta_t_prime;
        
        r_current_ = r_current;

        std_msgs::msg::Float64 radius_msg;
        radius_msg.data = r_current_;
        pub_adaptive_radius_->publish(radius_msg);

        const double MAX_SEARCH_RADIUS = 40.0;
        RCLCPP_WARN(this->get_logger(),
                    "Coasting elapsed time: %.1fs | Dynamic search radius r: %.2fm / %.1fm (Initial: %.2fm)",
                    delta_t_prime, r_current, MAX_SEARCH_RADIUS, r_initial);

        if (r_current >= MAX_SEARCH_RADIUS) {
            RCLCPP_ERROR(this->get_logger(),
                         "Search radius breached upper bound (>= %.1fm)! Local spatial confidence bankrupted. Seamlessly switching to HEAVY GLOBAL RELOCALIZATION.", MAX_SEARCH_RADIUS);
            
            // 发布 -1.0 以通知视觉前端局部空间置信度已破产
            std_msgs::msg::Float64 radius_msg;
            radius_msg.data = -1.0;
            pub_adaptive_radius_->publish(radius_msg);

            clear_highlight_submap(); // 刚性补齐：确保超时熔断瞬间，极限状态的 30m 红球在视窗中无缝蒸发
            trigger_heavy_global_relocalization();
            return;
        }

        // 从全局地图中提取子图并以自适应着色发布
        if (!global_map_->empty() && global_kdtree_) {
            pcl::PointXYZ searchPoint;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                searchPoint.x = last_known_pose_(0, 3);
                searchPoint.y = last_known_pose_(1, 3);
                searchPoint.z = last_known_pose_(2, 3);
            }
            
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr sub_map(new pcl::PointCloud<pcl::PointXYZ>());
            if (global_kdtree_->radiusSearch(searchPoint, r_current_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                sub_map->points.reserve(pointIdxRadiusSearch.size());
                for (int idx : pointIdxRadiusSearch) {
                    sub_map->points.push_back(global_map_->points[idx]);
                }
                sub_map->width = sub_map->points.size();
                sub_map->height = 1;
                sub_map->is_dense = true;
            }

            if (!is_lidar_allowed_.load()) {
                clear_highlight_submap();
                return;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr sub_map_highlight(new pcl::PointCloud<pcl::PointXYZI>());
            sub_map_highlight->points.reserve(sub_map->points.size());
            
            for (const auto& pt : sub_map->points) {
                pcl::PointXYZI p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                p.intensity = pt.x; // 使用X坐标作为强度（intensity），在单层地图中呈现鲜艳的 X 轴方向彩虹渐变，与地图本身的 Z 轴着色形成鲜明对比
                sub_map_highlight->points.push_back(p);
            }
            sub_map_highlight->width = sub_map_highlight->points.size();
            sub_map_highlight->height = 1;
            sub_map_highlight->is_dense = true;

            sensor_msgs::msg::PointCloud2 highlight_msg;
            pcl::toROSMsg(*sub_map_highlight, highlight_msg);
            highlight_msg.header.stamp = this->now();
            highlight_msg.header.frame_id = "map";
            adaptive_submap_pub_->publish(highlight_msg);
            pub_submap_highlight_->publish(highlight_msg);
        }
    }

    enum class TrackingState {
        SEARCHING,
        PROVISIONAL_TRACKING,
        LOCKED
    };
    TrackingState tracking_state_visual_ = TrackingState::SEARCHING;
    Eigen::Matrix4f provisional_map_pose_visual_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f provisional_odom_pose_visual_ = Eigen::Matrix4f::Identity();
    int provisional_success_count_visual_ = 0;
    int provisional_timer_visual_ = 0;

    TrackingState tracking_state_lidar_ = TrackingState::SEARCHING;
    Eigen::Matrix4f provisional_map_pose_lidar_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f provisional_odom_pose_lidar_ = Eigen::Matrix4f::Identity();
    int provisional_success_count_lidar_ = 0;
    int provisional_timer_lidar_ = 0;

    const int REQUIRED_PROVISIONAL_FRAMES_ = 3;
    const double PROVISIONAL_DISTANCE_GAP_ = 2.0;
    const int PROVISIONAL_TIMEOUT_SEC_ = 10;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> map_received_{false};
    Eigen::Matrix4f T_map_to_odom_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr global_kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_;
    nav_msgs::msg::Odometry::SharedPtr cur_odom_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_adaptive_radius_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_, sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_smoothed_localization_;

    rclcpp::Time last_visual_time_;
    std::atomic<int> visual_reject_cnt_{0};
    std::atomic<bool> is_lidar_allowed_{false};
    int tracking_loss_cnt_ = 0;
    rclcpp::Time last_stair_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3f odom_pos_at_last_correct_ = Eigen::Vector3f::Zero();
    double odom_yaw_at_last_correct_ = 0.0;
    double accumulated_yaw_since_lost_deg_ = 0.0;
    double accumulated_distance_since_lost_ = 0.0;
    double last_track_odometer_ = 0.0;
    double coasting_budget_distance_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_initialpose_visual_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_initialpose_lidar_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
    int current_timer_interval_ms_ = 1000;
    rclcpp::TimerBase::SharedPtr timer_;

    // 新增私有变量
    double delta_s_ = 0.0;
    rclcpp::Time t_lost_start_;
    rclcpp::TimerBase::SharedPtr adaptive_timer_;
    double r_current_ = 10.0;
    bool has_been_localized_once_ = false;
    Eigen::Matrix4f last_known_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_known_pose_smoothed_ = Eigen::Matrix4f::Identity();
    std::optional<Eigen::Vector3f> last_locked_odom_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr adaptive_submap_pub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_highlight_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_force_lidar_;
    
    // ui data hooks
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_ui_projected_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ui_clear_trigger_;
    nav_msgs::msg::Path path_record_; // 用于同步缓存的轨迹
    std::mutex path_mutex_; // 轨迹写入多线程保护锁
    std::mutex pose_mutex_; // 机器人位姿/矩阵读写多线程锁
    tf2::Quaternion static_q;
    // Removed T_map_to_rviz
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::map<int, Eigen::Matrix4f> aruco_world_poses_; // T_world_marker
    std::map<int, double> aruco_marker_sizes_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_aruco_detections_;
    rclcpp::Time aruco_last_init_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cv::Mat cam_K_;
    cv::Mat cam_dist_;
    cv::Mat T_lidar_camera_;

    void loadArucoLandmarks() {
        this->declare_parameter<std::string>("aruco.landmarks_file", "./src/navigation/fast_livo_dog/data/aruco_landmarks.yaml");
        std::string landmarks_file = this->get_parameter("aruco.landmarks_file").as_string();
        try {
            YAML::Node config = YAML::LoadFile(landmarks_file);
            if (config["landmarks"]) {
                for (const auto& lm : config["landmarks"]) {
                    int id = lm["id"].as<int>();
                    double size = lm["marker_size"].as<double>();
                    std::vector<double> pose_vec = lm["pose"].as<std::vector<double>>();
                    if (pose_vec.size() == 16) {
                        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                        for (int r = 0; r < 4; ++r) {
                            for (int c = 0; c < 4; ++c) {
                                T(r, c) = pose_vec[r * 4 + c];
                            }
                        }
                        aruco_world_poses_[id] = T;
                        aruco_marker_sizes_[id] = size;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "ArUco: Loaded %zu landmarks from %s", aruco_world_poses_.size(), landmarks_file.c_str());
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "ArUco: Failed to load landmarks file %s. Error: %s", landmarks_file.c_str(), e.what());
        }

        // 仅在有 landmarks 时才初始化相机参数并订阅检测结果
        if (!aruco_world_poses_.empty()) {
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

            std::vector<double> T_cl_vec(16, 0.0);
            this->declare_parameter<std::vector<double>>("extrin_calib.T_camera_lidar", T_cl_vec);
            this->get_parameter("extrin_calib.T_camera_lidar", T_cl_vec);
            if (T_cl_vec.size() == 16) {
                cv::Mat T_camera_lidar = (cv::Mat_<double>(4,4) <<
                    T_cl_vec[0],  T_cl_vec[1],  T_cl_vec[2],  T_cl_vec[3],
                    T_cl_vec[4],  T_cl_vec[5],  T_cl_vec[6],  T_cl_vec[7],
                    T_cl_vec[8],  T_cl_vec[9],  T_cl_vec[10], T_cl_vec[11],
                    T_cl_vec[12], T_cl_vec[13], T_cl_vec[14], T_cl_vec[15]);
                T_lidar_camera_ = T_camera_lidar.inv();
            } else {
                T_lidar_camera_ = cv::Mat::eye(4, 4, CV_64F);
            }

            sub_aruco_detections_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/aruco_detections", 10,
                std::bind(&GlobalLocalization::cb_aruco_detections, this, std::placeholders::_1));
        }
    }

    void cb_aruco_detections(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (initialized_ || tracking_state_visual_ == TrackingState::PROVISIONAL_TRACKING || tracking_state_lidar_ == TrackingState::PROVISIONAL_TRACKING) {
            return;
        }

        // 5秒冷却保护
        if ((this->now() - aruco_last_init_time_).seconds() < 5.0) {
            return;
        }

        const auto& data = msg->data;
        if (data.size() < 2) return;
        int num_markers = static_cast<int>(data[1]);
        if (num_markers <= 0) return;

        int idx = 2;
        for (int i = 0; i < num_markers; ++i) {
            if (idx + 9 > static_cast<int>(data.size())) break;
            int marker_id = static_cast<int>(data[idx++]);
            
            // 读取 4 个角点
            std::vector<cv::Point2f> image_points(4);
            for (int j = 0; j < 4; ++j) {
                image_points[j] = cv::Point2f(data[idx], data[idx + 1]);
                idx += 2;
            }

            // 检查这个 id 是否在 YAML 中有记录
            if (aruco_world_poses_.find(marker_id) != aruco_world_poses_.end()) {
                double size = aruco_marker_sizes_[marker_id];
                std::vector<cv::Point3f> object_points = {
                    cv::Point3f(-size / 2, size / 2, 0),
                    cv::Point3f(size / 2, size / 2, 0),
                    cv::Point3f(size / 2, -size / 2, 0),
                    cv::Point3f(-size / 2, -size / 2, 0)
                };

                cv::Mat rvec, tvec;
                if (cv::solvePnP(object_points, image_points, cam_K_, cam_dist_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    cv::Mat T_camera_marker = cv::Mat::eye(4, 4, CV_64F);
                    R.copyTo(T_camera_marker(cv::Rect(0, 0, 3, 3)));
                    tvec.copyTo(T_camera_marker(cv::Rect(3, 0, 1, 3)));

                    cv::Mat T_lidar_marker_cv = T_lidar_camera_ * T_camera_marker;
                    Eigen::Matrix4f T_lidar_marker = Eigen::Matrix4f::Identity();
                    for (int r = 0; r < 4; ++r) {
                        for (int c = 0; c < 4; ++c) {
                            T_lidar_marker(r, c) = T_lidar_marker_cv.at<double>(r, c);
                        }
                    }

                    // T_world_lidar = T_world_marker * inv(T_lidar_marker)
                    Eigen::Matrix4f T_world_marker = aruco_world_poses_[marker_id];
                    Eigen::Matrix4f T_world_lidar = T_world_marker * safeInverse(T_lidar_marker);

                    RCLCPP_WARN(this->get_logger(), "ArUco: Found known marker ID=%d. Generating global initialization proposal!", marker_id);

                    // 构造 PoseArray 发送给现有的 ICP 验证通道
                    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseArray>();
                    pose_msg->header.stamp = this->now();
                    pose_msg->header.frame_id = "map";

                    geometry_msgs::msg::Pose pose;
                    pose.position.x = T_world_lidar(0, 3);
                    pose.position.y = T_world_lidar(1, 3);
                    pose.position.z = T_world_lidar(2, 3);

                    Eigen::Matrix3f R_world_lidar = T_world_lidar.block<3, 3>(0, 0);
                    Eigen::Quaternionf q(R_world_lidar);
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();

                    pose_msg->poses.push_back(pose);

                    // 记录触发时间，并调用验证通道 (使用 visual 的通道)
                    aruco_last_init_time_ = this->now();
                    cb_initialpose_visual(pose_msg);
                    return; // 只要算出一个就跳出，等待验证
                }
            }
        }
    }
    std::atomic<int> stable_stream_delay_idx_{0};
    Eigen::Matrix4f T_user_initial_guess_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_odom_reset_anchor_ = Eigen::Matrix4f::Identity();
    bool is_manual_override_mode_ = false;
    Eigen::Matrix4f T_map_to_odom_user_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_manual_base_target_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_manual_odom_anchor_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_body_to_baselink_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalLocalization>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
