#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include "mapping/common.h"

typedef pcl::PointXYZRGB PointTypeRGB;

/**
 * @brief 地图铺设与导出引擎类，负责将优化后的 3D 关键帧点云拼接调平、高度切片、去噪并导出为 2D 栅格地图与 3D PCD 图
 */
class MapPavingEngine {
public:
    struct Config {
        std::string save_directory;
        Eigen::Matrix4f T_body_to_baselink;
        double pcd2pgm_map_resolution;
        double pcd2pgm_thre_z_min;
        double pcd2pgm_thre_z_max;
        double pcd2pgm_thre_radius;
        int pcd2pgm_thres_point_count;
    };

    MapPavingEngine(const Config& config, const rclcpp::Logger& logger);
    ~MapPavingEngine();

    // 拼接 3D 关键帧点云，保存为 3D PCD，调平并切片导出为 2D PGM/YAML 地图
    void buildAndSaveMaps(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                          const std::vector<Pose6D>& keyframePosesUpdated,
                          int recentIdxUpdated);

private:
    // 将按帧动态调平的点云切片、去噪并保存为 2D 地图文件
    void save2DMap(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                   const std::vector<Pose6D>& keyframePosesUpdated,
                   int recentIdxUpdated);

    // 局部转换点云位姿投射到世界参考系 (带颜色信息)
    pcl::PointCloud<PointTypeRGB>::Ptr local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn,
                                                       const Pose6D& tf);

    Config config_;
    rclcpp::Logger logger_;
};
