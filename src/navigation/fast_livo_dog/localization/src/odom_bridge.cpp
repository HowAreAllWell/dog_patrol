#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

class OdomBridge : public rclcpp::Node {
public:
    OdomBridge() : Node("odom_bridge") {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("source_odom_topic", "/aft_mapped_to_init");
        this->declare_parameter<std::string>("output_odom_topic", "/odom");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "camera_init");
        this->declare_parameter<std::string>("nav_odom_frame", "camera_init_footprint");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("nav_base_frame", "base_footprint");
        this->declare_parameter<bool>("publish_tf", true);
        
        // Read extrinsics directly from device_parameters.yaml (T_lidar_base)
        this->declare_parameter<std::vector<double>>("T_lidar_base", std::vector<double>(16, 0.0));

        source_odom_topic_ = this->get_parameter("source_odom_topic").as_string();
        output_odom_topic_ = this->get_parameter("output_odom_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        nav_odom_frame_ = this->get_parameter("nav_odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        nav_base_frame_ = this->get_parameter("nav_base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        std::vector<double> ext_matrix = this->get_parameter("T_lidar_base").as_double_array();
        if (ext_matrix.size() == 16) {
            tf2::Matrix3x3 R_base(ext_matrix[0], ext_matrix[1], ext_matrix[2], 
                                  ext_matrix[4], ext_matrix[5], ext_matrix[6],
                                  ext_matrix[8], ext_matrix[9], ext_matrix[10]);
            tf2::Vector3 t_base(ext_matrix[3], ext_matrix[7], ext_matrix[11]);
            T_body_base_.setBasis(R_base);
            T_body_base_.setOrigin(t_base);
            RCLCPP_INFO(this->get_logger(), "Extrinsics loaded successfully from T_lidar_base.");
        } else {
            RCLCPP_WARN(this->get_logger(), "T_lidar_base size is not 16. Using identity matrix.");
            T_body_base_.setIdentity();
        }

        // Initialize TF listener and broadcaster
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);

        // Subscription
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            source_odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&OdomBridge::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "OdomBridge node initialized successfully. Bridging %s to %s.",
                    source_odom_topic_.c_str(), output_odom_topic_.c_str());

        // Publish fallback identity TF until first real odom is received to prevent Nav2 startup TF lookup timeout
        first_odom_received_ = false;
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomBridge::publish_default_tf, this)
        );
    }

private:
    void publish_default_tf() {
        if (first_odom_received_) {
            init_timer_->cancel();
            return;
        }

        rclcpp::Time stamp = this->now();

        // Publish map -> camera_init_footprint as identity (fallback)
        geometry_msgs::msg::TransformStamped tf_map_camera_init_footprint;
        tf_map_camera_init_footprint.header.stamp = stamp;
        tf_map_camera_init_footprint.header.frame_id = map_frame_;
        tf_map_camera_init_footprint.child_frame_id = nav_odom_frame_;
        tf_map_camera_init_footprint.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_map_camera_init_footprint);

        // Publish camera_init_footprint -> base_footprint as identity
        geometry_msgs::msg::TransformStamped tf_camera_init_footprint_base;
        tf_camera_init_footprint_base.header.stamp = stamp;
        tf_camera_init_footprint_base.header.frame_id = nav_odom_frame_;
        tf_camera_init_footprint_base.child_frame_id = nav_base_frame_;
        tf_camera_init_footprint_base.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_camera_init_footprint_base);
    }

    tf2::Transform planarize(const tf2::Transform& T) {
        double yaw, pitch, roll;
        tf2::Matrix3x3(T.getRotation()).getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        return tf2::Transform(q, tf2::Vector3(T.getOrigin().x(), T.getOrigin().y(), 0.0));
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;
        if (!first_odom_received_) {
            first_odom_received_ = true;
            tf_buffer_->clear();
        } else {
            if (stamp.nanoseconds() < static_cast<int64_t>(prev_stamp_ns_)) {
                RCLCPP_WARN(this->get_logger(), "Time jump backward detected in odom_bridge! Clearing TF buffer.");
                tf_buffer_->clear();
            }
        }

        // 1. Get 3D pose of body in camera_init (from odometry input)
        tf2::Transform T_camera_init_body;
        T_camera_init_body.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        T_camera_init_body.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

        // 2. Apply body to base transform to get 3D base_link in camera_init
        tf2::Transform T_camera_init_base = T_camera_init_body * T_body_base_;

        // 3. 严格遵循 REP-105：纯净平滑的 2D 局部里程计，绝对不能受后端全局校准 (T_map_odom) 的污染！
        // 直接由前端 3D 连续里程计降维得到，保证绝对的局部平滑和速度积分的准确性。
        tf2::Transform T_camera_init_footprint_base_footprint = planarize(T_camera_init_base);
        tf2::Transform T_map_camera_init_footprint;
        T_map_camera_init_footprint.setIdentity();



        // 4. Look up map -> camera_init (odom_frame_) transform from TF
        try {
            geometry_msgs::msg::TransformStamped tf_map_odom = tf_buffer_->lookupTransform(
                map_frame_,
                odom_frame_,
                tf2::TimePointZero
            );
            tf2::Transform T_map_odom;
            T_map_odom.setOrigin(tf2::Vector3(tf_map_odom.transform.translation.x, tf_map_odom.transform.translation.y, tf_map_odom.transform.translation.z));
            T_map_odom.setRotation(tf2::Quaternion(tf_map_odom.transform.rotation.x, tf_map_odom.transform.rotation.y, tf_map_odom.transform.rotation.z, tf_map_odom.transform.rotation.w));

            // 计算当前机器人的 3D 真实全局绝对位姿，并降维至 2D 平面
            tf2::Transform T_map_base = T_map_odom * T_camera_init_base;

            tf2::Transform T_map_base_nav = planarize(T_map_base);

            // 反推 map -> camera_init_footprint，使得 T_map_camera_init_footprint * T_camera_init_footprint_base_footprint == T_map_base_nav 始终成立
            T_map_camera_init_footprint = T_map_base_nav * T_camera_init_footprint_base_footprint.inverse();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "TF lookup map -> %s failed: %s. Using local fallback.",
                                 odom_frame_.c_str(), ex.what());
        }

        // 5. Calculate twist velocities by differentiation
        uint64_t stamp_ns = stamp.nanoseconds();

        double vx = 0.0, vy = 0.0, wz = 0.0;
        if (prev_stamp_ns_ != 0) {
            double dt = (stamp_ns - prev_stamp_ns_) * 1e-9;
            if (dt >= 0.001 && dt <= 0.5) {
                tf2::Vector3 delta_world = T_camera_init_footprint_base_footprint.getOrigin() - prev_pos_;
                tf2::Vector3 delta_base = tf2::quatRotate(T_camera_init_footprint_base_footprint.getRotation().inverse(), delta_world);
                vx = delta_base.x() / dt;
                vy = delta_base.y() / dt;

                double roll, pitch, yaw;
                tf2::Matrix3x3(T_camera_init_footprint_base_footprint.getRotation()).getRPY(roll, pitch, yaw);
                double delta_yaw = std::atan2(std::sin(yaw - prev_yaw_), std::cos(yaw - prev_yaw_));
                wz = delta_yaw / dt;
            }
        }

        prev_stamp_ns_ = stamp_ns;
        prev_pos_ = T_camera_init_footprint_base_footprint.getOrigin();
        double roll, pitch, yaw;
        tf2::Matrix3x3(T_camera_init_footprint_base_footprint.getRotation()).getRPY(roll, pitch, yaw);
        prev_yaw_ = yaw;

        // 6. Publish 2D Odometry message
        nav_msgs::msg::Odometry out_odom;
        out_odom.header.stamp = stamp;
        out_odom.header.frame_id = nav_odom_frame_;
        out_odom.child_frame_id = nav_base_frame_;
        out_odom.pose.pose.position.x = T_camera_init_footprint_base_footprint.getOrigin().x();
        out_odom.pose.pose.position.y = T_camera_init_footprint_base_footprint.getOrigin().y();
        out_odom.pose.pose.position.z = T_camera_init_footprint_base_footprint.getOrigin().z();
        out_odom.pose.pose.orientation.x = T_camera_init_footprint_base_footprint.getRotation().x();
        out_odom.pose.pose.orientation.y = T_camera_init_footprint_base_footprint.getRotation().y();
        out_odom.pose.pose.orientation.z = T_camera_init_footprint_base_footprint.getRotation().z();
        out_odom.pose.pose.orientation.w = T_camera_init_footprint_base_footprint.getRotation().w();
        out_odom.pose.covariance = msg->pose.covariance;

        out_odom.twist.twist.linear.x = vx;
        out_odom.twist.twist.linear.y = vy;
        out_odom.twist.twist.angular.z = wz;
        out_odom.twist.covariance[0] = 0.05;
        out_odom.twist.covariance[7] = 0.05;
        out_odom.twist.covariance[35] = 0.05;

        odom_pub_->publish(out_odom);

        // 7. Publish TF transforms
        if (publish_tf_) {
            // map -> camera_init_footprint (always publish to keep TF tree connected and prevent Nav2 blocking timeouts)
            geometry_msgs::msg::TransformStamped tf_map_camera_init_footprint;
            tf_map_camera_init_footprint.header.stamp = stamp;
            tf_map_camera_init_footprint.header.frame_id = map_frame_;
            tf_map_camera_init_footprint.child_frame_id = nav_odom_frame_;
            tf_map_camera_init_footprint.transform.translation.x = T_map_camera_init_footprint.getOrigin().x();
            tf_map_camera_init_footprint.transform.translation.y = T_map_camera_init_footprint.getOrigin().y();
            tf_map_camera_init_footprint.transform.translation.z = T_map_camera_init_footprint.getOrigin().z();
            tf_map_camera_init_footprint.transform.rotation.x = T_map_camera_init_footprint.getRotation().x();
            tf_map_camera_init_footprint.transform.rotation.y = T_map_camera_init_footprint.getRotation().y();
            tf_map_camera_init_footprint.transform.rotation.z = T_map_camera_init_footprint.getRotation().z();
            tf_map_camera_init_footprint.transform.rotation.w = T_map_camera_init_footprint.getRotation().w();
            tf_broadcaster_->sendTransform(tf_map_camera_init_footprint);

            // camera_init_footprint -> base_footprint
            geometry_msgs::msg::TransformStamped tf_camera_init_footprint_base;
            tf_camera_init_footprint_base.header.stamp = stamp;
            tf_camera_init_footprint_base.header.frame_id = nav_odom_frame_;
            tf_camera_init_footprint_base.child_frame_id = nav_base_frame_;
            tf_camera_init_footprint_base.transform.translation.x = T_camera_init_footprint_base_footprint.getOrigin().x();
            tf_camera_init_footprint_base.transform.translation.y = T_camera_init_footprint_base_footprint.getOrigin().y();
            tf_camera_init_footprint_base.transform.translation.z = T_camera_init_footprint_base_footprint.getOrigin().z();
            tf_camera_init_footprint_base.transform.rotation.x = T_camera_init_footprint_base_footprint.getRotation().x();
            tf_camera_init_footprint_base.transform.rotation.y = T_camera_init_footprint_base_footprint.getRotation().y();
            tf_camera_init_footprint_base.transform.rotation.z = T_camera_init_footprint_base_footprint.getRotation().z();
            tf_camera_init_footprint_base.transform.rotation.w = T_camera_init_footprint_base_footprint.getRotation().w();
            tf_broadcaster_->sendTransform(tf_camera_init_footprint_base);

            // (Removed base_footprint_z since we use true base_link for slicing now)

            // odom_frame -> base_frame (camera_init -> base_link)
            // 注释掉此处的 3D TF 发布，以防与 transform_fusion + fast_livo 发布的三维 TF 树 (camera_init -> body -> base_link)
            // 产生多父节点回路冲突 (TF tree loop conflict)。
            /*
            geometry_msgs::msg::TransformStamped tf_odom_base;
            tf_odom_base.header.stamp = stamp;
            tf_odom_base.header.frame_id = odom_frame_;
            tf_odom_base.child_frame_id = base_frame_;
            tf_odom_base.transform.translation.x = T_camera_init_base.getOrigin().x();
            tf_odom_base.transform.translation.y = T_camera_init_base.getOrigin().y();
            tf_odom_base.transform.translation.z = T_camera_init_base.getOrigin().z();
            tf_odom_base.transform.rotation.x = T_camera_init_base.getRotation().x();
            tf_odom_base.transform.rotation.y = T_camera_init_base.getRotation().y();
            tf_odom_base.transform.rotation.z = T_camera_init_base.getRotation().z();
            tf_odom_base.transform.rotation.w = T_camera_init_base.getRotation().w();
            tf_broadcaster_->sendTransform(tf_odom_base);
            */
        }
    }

    std::string source_odom_topic_;
    std::string output_odom_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string nav_odom_frame_;
    std::string base_frame_;
    std::string nav_base_frame_;
    bool publish_tf_;
    tf2::Transform T_body_base_;

    // ROS 2 components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Temporal states for twist calculation
    uint64_t prev_stamp_ns_ = 0;
    tf2::Vector3 prev_pos_;
    double prev_yaw_ = 0.0;

    bool first_odom_received_ = false;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
