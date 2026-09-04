/*
 * [TF TREE TOPOLOGY]
 * map -> camera_init -> body -> base_link
 *
 * 1. map -> camera_init:
 * 吸收全局漂移误差。应用轻度平滑插值滤波，防止定位突变跳跃对控制节点造成冲击。
 * 2. camera_init -> body:
 * 高频局域里程计，由前端节点发布，保障局域相对运动的平滑与连续。
 * 3. body -> base_link:
 * 由本节点发布，表示传感器外参到车辆中心物理位置的静态标定关系。
 */
#ifdef ROS_HUMBLE
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#endif
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class TransformFusion : public rclcpp::Node {
public:
    TransformFusion() : Node("transform_fusion") {
        RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
        T_map_to_odom_current_.setIdentity();
        T_map_to_odom_target_.setIdentity();
        T_map_to_odom_shadow_.setIdentity();
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 声明与读取外参矩阵 T_lidar_base 并还原为 tf2::Transform
        this->declare_parameter<std::vector<double>>("T_lidar_base", std::vector<double>(16, 0.0));
        std::vector<double> ext_matrix = this->get_parameter("T_lidar_base").as_double_array();

        // 提取旋转矩阵 R (3x3) 和平移向量 t (3x1) 构建 tf2::Transform
        tf2::Matrix3x3 R_base(ext_matrix[0], ext_matrix[1], ext_matrix[2], ext_matrix[4], ext_matrix[5], ext_matrix[6],
                              ext_matrix[8], ext_matrix[9], ext_matrix[10]);
        tf2::Vector3 t_base(ext_matrix[3], ext_matrix[7], ext_matrix[11]);

        t_body_to_baselink_.setBasis(R_base);
        t_body_to_baselink_.setOrigin(t_base);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&TransformFusion::cb_save_cur_odom, this, std::placeholders::_1));

        sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/map_to_odom", 1, std::bind(&TransformFusion::cb_save_map_to_odom, this, std::placeholders::_1));

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", qos);
        pub_global_path_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", qos);
        global_path_.header.frame_id = "map";

        sub_status_ = this->create_subscription<std_msgs::msg::Bool>(
            "/localization_status", 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data && this->is_localized_) {
                    RCLCPP_WARN(this->get_logger(),
                                "Localization revoke command received. Clearing global path and resetting state.");
                    this->is_localized_ = false;
                    this->global_path_.poses.clear();
                    this->pub_global_path_->publish(this->global_path_);

                    this->T_map_to_odom_current_.setIdentity();
                    this->T_map_to_odom_target_.setIdentity();
                    this->T_map_to_odom_shadow_.setIdentity();
                }
            });

        // timer_ has been removed; logic moved to cb_save_cur_odom to be fully data-driven
    }

private:
    // --- 算法常量与静态标定外参配置 ---
    const double SMOOTH_FACTOR_ = 0.015;   // 数据驱动级联二阶滤波因子。调整为 0.015 后，收敛时间约为 1.58 秒
    tf2::Transform t_body_to_baselink_;    // 激光雷达至车辆中心(body->base_link)的4x4外参变换
    const int PATH_PUBLISH_INTERVAL_ = 20; // 改为数据驱动后频率高达200Hz，调高抽取间隔防RViz卡顿

    tf2::Transform poseToTransform(const geometry_msgs::msg::Pose& pose) {
        tf2::Transform t;
        t.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
        t.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        return t;
    }

    void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (cur_odom_to_baselink_) {
            rclcpp::Time current_time(msg->header.stamp);
            rclcpp::Time prev_time(cur_odom_to_baselink_->header.stamp);
            if (current_time.nanoseconds() < prev_time.nanoseconds()) {
                RCLCPP_WARN(this->get_logger(), "Time jump backward detected in transform_fusion! Clearing path.");
                global_path_.poses.clear();
            }
        }
        cur_odom_to_baselink_ = msg;
        
        // 直接在此处执行插值与发布（完美的数据驱动时空同步）
        process_fusion_and_publish();
    }

    void cb_save_map_to_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Transform new_target = poseToTransform(msg->pose.pose);

        if (!is_localized_) {
            T_map_to_odom_current_ = new_target;
            T_map_to_odom_target_ = new_target;
            T_map_to_odom_shadow_ = new_target;
            is_localized_ = true;
            global_path_.poses.clear();
            RCLCPP_INFO(this->get_logger(),
                        "Initial localization received. Applying transform immediately.");
        } else {
            // 计算当前位姿和目标新位姿之间的平移偏差
            double dx = new_target.getOrigin().x() - T_map_to_odom_current_.getOrigin().x();
            double dy = new_target.getOrigin().y() - T_map_to_odom_current_.getOrigin().y();
            double dz = new_target.getOrigin().z() - T_map_to_odom_current_.getOrigin().z();
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // 结合 global_localization 中指数自适应阈值的绝对上限 (D_base + D_span = 1.0 + 1.5 = 2.5m)
            // 任何大于此值的坐标跳变绝对不可能是 ICP 追踪产生的，必定是用户大范围手动重定位。
            const double MAX_ICP_TRACKING_SHIFT_M = 2.5;
            
            if (distance > MAX_ICP_TRACKING_SHIFT_M) {
                // 发生超越算法容忍极限的超大坐标跳变（通常由用户手动拖拽给出），
                // 直接切断弹簧滤波，强制瞬间空降（Teleport），防止出现恐怖的长距离滑坡运动。
                T_map_to_odom_current_ = new_target;
                T_map_to_odom_target_ = new_target;
                T_map_to_odom_shadow_ = new_target;
                global_path_.poses.clear();
                RCLCPP_WARN(this->get_logger(),
                            "Massive localization jump detected (%.2fm > %.1fm). Teleporting immediately to prevent dangerous glides.", 
                            distance, MAX_ICP_TRACKING_SHIFT_M);
            } else {
                // 常规算法后台 ICP 微调修正，交由二阶阻尼弹簧进行 1.58s 平滑吸收
                T_map_to_odom_target_ = new_target;
            }
        }
    }

    void process_fusion_and_publish() {
        if (!cur_odom_to_baselink_) {
            return;
        }

        // 1. 数据驱动的高频缓动插值计算（跟随前端原生频率，完美免疫倍速播包）
        if (is_localized_) {
            // ----- 级联低通滤波（二阶阻尼弹簧模型 / S曲线平滑） -----
            // Step 1: 虚拟影子去追赶目标 (一阶，起步快)
            tf2::Vector3 shadow_origin = T_map_to_odom_shadow_.getOrigin();
            tf2::Vector3 tar_origin = T_map_to_odom_target_.getOrigin();
            tf2::Vector3 next_shadow_origin = shadow_origin.lerp(tar_origin, SMOOTH_FACTOR_);
            
            tf2::Quaternion shadow_quat = T_map_to_odom_shadow_.getRotation();
            tf2::Quaternion tar_quat = T_map_to_odom_target_.getRotation();
            tf2::Quaternion next_shadow_quat = shadow_quat.slerp(tar_quat, SMOOTH_FACTOR_);
            
            T_map_to_odom_shadow_.setOrigin(next_shadow_origin);
            T_map_to_odom_shadow_.setRotation(next_shadow_quat);

            // Step 2: 真实当前位姿去追赶影子 (二阶，起步为0，完美 S 曲线)
            tf2::Vector3 cur_origin = T_map_to_odom_current_.getOrigin();
            tf2::Vector3 step_origin = cur_origin.lerp(next_shadow_origin, SMOOTH_FACTOR_);

            tf2::Quaternion cur_quat = T_map_to_odom_current_.getRotation();
            tf2::Quaternion step_quat = cur_quat.slerp(next_shadow_quat, SMOOTH_FACTOR_);

            T_map_to_odom_current_.setOrigin(step_origin);
            T_map_to_odom_current_.setRotation(step_quat);
        }

        // 3. 定位未完成初始化前，发布单位变换矩阵，锁定当前树状态以防止前端节点漂移
        if (!is_localized_) {
            geometry_msgs::msg::TransformStamped t_static;
            t_static.header.stamp = cur_odom_to_baselink_->header.stamp;
            t_static.header.frame_id = "map";
            t_static.child_frame_id = "camera_init";
            t_static.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t_static);
        } else {
            geometry_msgs::msg::TransformStamped t_msg;
            t_msg.header.stamp = cur_odom_to_baselink_->header.stamp;
            t_msg.header.frame_id = "map";
            t_msg.child_frame_id = "camera_init";

            t_msg.transform.translation.x = T_map_to_odom_current_.getOrigin().x();
            t_msg.transform.translation.y = T_map_to_odom_current_.getOrigin().y();
            t_msg.transform.translation.z = T_map_to_odom_current_.getOrigin().z();
            t_msg.transform.rotation.x = T_map_to_odom_current_.getRotation().x();
            t_msg.transform.rotation.y = T_map_to_odom_current_.getRotation().y();
            t_msg.transform.rotation.z = T_map_to_odom_current_.getRotation().z();
            t_msg.transform.rotation.w = T_map_to_odom_current_.getRotation().w();

            tf_broadcaster_->sendTransform(t_msg);
        }

        // 1. 获取 map -> body 的变换
        tf2::Transform t_odom_to_body = poseToTransform(cur_odom_to_baselink_->pose.pose);
        tf2::Transform t_map_to_body = T_map_to_odom_current_ * t_odom_to_body;

        // 3. 计算最终的 map -> base_link 真值位姿
        tf2::Transform t_map_to_baselink_final = t_map_to_body * t_body_to_baselink_;

        // 4. 构建并发布修正后的 Odometry 消息
        nav_msgs::msg::Odometry localization_msg;
        localization_msg.header.stamp = cur_odom_to_baselink_->header.stamp;
        localization_msg.header.frame_id = "map";
        localization_msg.child_frame_id = "base_link";  // 明确声明发布的是机器人本体中心坐标

        localization_msg.pose.pose.position.x = t_map_to_baselink_final.getOrigin().x();
        localization_msg.pose.pose.position.y = t_map_to_baselink_final.getOrigin().y();
        localization_msg.pose.pose.position.z = t_map_to_baselink_final.getOrigin().z();

        localization_msg.pose.pose.orientation.x = t_map_to_baselink_final.getRotation().x();
        localization_msg.pose.pose.orientation.y = t_map_to_baselink_final.getRotation().y();
        localization_msg.pose.pose.orientation.z = t_map_to_baselink_final.getRotation().z();
        localization_msg.pose.pose.orientation.w = t_map_to_baselink_final.getRotation().w();

        localization_msg.twist = cur_odom_to_baselink_->twist;

        if (is_localized_) {
            pub_localization_->publish(localization_msg);
        }

        // 使用里程计最新合法时间戳同步广播静态标定的 body 到 base_link
        // 外参，避免由于 TF 时序不同步引发的运行期崩溃
        geometry_msgs::msg::TransformStamped t_base;
        t_base.header.stamp = cur_odom_to_baselink_->header.stamp;
        t_base.header.frame_id = "aft_mapped";
        t_base.child_frame_id = "base_link";

        t_base.transform.translation.x = t_body_to_baselink_.getOrigin().x();
        t_base.transform.translation.y = t_body_to_baselink_.getOrigin().y();
        t_base.transform.translation.z = t_body_to_baselink_.getOrigin().z();

        tf2::Quaternion q_base = t_body_to_baselink_.getRotation();

        t_base.transform.rotation.x = q_base.x();
        t_base.transform.rotation.y = q_base.y();
        t_base.transform.rotation.z = q_base.z();
        t_base.transform.rotation.w = q_base.w();

        tf_broadcaster_->sendTransform(t_base);

        if (!is_localized_) {
            global_path_.poses.clear();
            return;
        }

        if (is_localized_) {
            static int path_cnt = 0;
            if (++path_cnt % PATH_PUBLISH_INTERVAL_ == 0) {
                geometry_msgs::msg::PoseStamped current_pose;
                current_pose.header.stamp = cur_odom_to_baselink_->header.stamp;
                current_pose.header.frame_id = "map";
                current_pose.pose = localization_msg.pose.pose;

                global_path_.header.stamp = current_pose.header.stamp;
                global_path_.poses.push_back(current_pose);
                pub_global_path_->publish(global_path_);
            }
        }
    }

    nav_msgs::msg::Odometry::SharedPtr cur_odom_to_baselink_;
    tf2::Transform T_map_to_odom_current_;
    tf2::Transform T_map_to_odom_target_;
    tf2::Transform T_map_to_odom_shadow_;
    bool is_localized_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_status_;
    nav_msgs::msg::Path global_path_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Timer removed in favor of data-driven processing
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}