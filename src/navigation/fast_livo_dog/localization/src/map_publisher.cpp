#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<std::string>("frame_id", "map");

        std::string map_path;
        std::string frame_id;
        this->get_parameter("map_path", map_path);
        this->get_parameter("frame_id", frame_id);

        if (map_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide a valid map_path parameter!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading global map from: %s", map_path.c_str());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load global map from: %s", map_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Dense global map loaded successfully (Points: %lu).",
                    cloud->points.size());

        // 对齐可靠的 TransientLocal QoS 保证后挂载的 RViz 客户端能稳定检索到地图
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();
        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", qos);

        // --- [RViz Optimization] Downsample for Visualization ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_for_vis(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterVis;
        downSizeFilterVis.setInputCloud(cloud);
        downSizeFilterVis.setLeafSize(0.2, 0.2, 0.2); // 0.2m resolution
        downSizeFilterVis.filter(*cloud_for_vis);

        RCLCPP_INFO(this->get_logger(), "Downsampled visualization map points: %lu",
                    cloud_for_vis->points.size());

        sensor_msgs::msg::PointCloud2 pc2_msg;
        pcl::toROSMsg(*cloud_for_vis, pc2_msg);

        // 强制重置 Frame ID 并做空值兜底，确保与全局坐标系绝对一致，RViz 能够解析
        if (frame_id.empty()) {
            frame_id = "map";
        }
        pc2_msg.header.frame_id = frame_id;
        pc2_msg.header.stamp = this->now();

        // 静态点云地图采用 TransientLocal 机制仅发布一次，DDS
        // 将自动为新加入的节点推送缓存数据
        pub_map_->publish(pc2_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Global map published successfully with Reliable + TransientLocal QoS.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}