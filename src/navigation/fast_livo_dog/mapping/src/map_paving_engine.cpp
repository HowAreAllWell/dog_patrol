#include "mapping/map_paving_engine.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <algorithm>

MapPavingEngine::MapPavingEngine(const Config& config, const rclcpp::Logger& logger)
    : config_(config), logger_(logger) {}

MapPavingEngine::~MapPavingEngine() {}

pcl::PointCloud<PointTypeRGB>::Ptr MapPavingEngine::local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn,
                                                                   const Pose6D& tf) {
    pcl::PointCloud<PointTypeRGB>::Ptr cloudOut(new pcl::PointCloud<PointTypeRGB>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    cloudOut->header = cloudIn->header;

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

#pragma omp parallel for num_threads(16)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].r = pointFrom.r;
        cloudOut->points[i].g = pointFrom.g;
        cloudOut->points[i].b = pointFrom.b;
    }
    return cloudOut;
}

void MapPavingEngine::buildAndSaveMaps(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                                      const std::vector<Pose6D>& keyframePosesUpdated,
                                      int recentIdxUpdated) {
    pcl::PointCloud<PointTypeRGB>::Ptr full_map(new pcl::PointCloud<PointTypeRGB>());
    int num_keyframes = keyframeLaserClouds.size();
    
    for (int i = 0; i <= recentIdxUpdated && i < num_keyframes; i++) {
        *full_map += *local2globalRGB(keyframeLaserClouds[i], keyframePosesUpdated[i]);
    }

    if (!full_map->empty()) {
        pcl::io::savePCDFileBinary(config_.save_directory + "map_3d.pcd", *full_map);
        RCLCPP_INFO(logger_,
                    "Full global map saved to file (Directory: %s, Frames count: %d).",
                    config_.save_directory.c_str(), recentIdxUpdated + 1);

        // 过滤出真彩色点（相机可见点），过滤掉灰度退化点（r==g==b 为 intensity 填充的无色点）
        pcl::PointCloud<PointTypeRGB>::Ptr colored_map(new pcl::PointCloud<PointTypeRGB>());
        colored_map->reserve(full_map->size());
        for (const auto& pt : full_map->points) {
            if (pt.r != pt.g || pt.r != pt.b) {
                colored_map->push_back(pt);
            }
        }
        if (!colored_map->empty()) {
            pcl::io::savePCDFileBinary(config_.save_directory + "map_3d_colored.pcd", *colored_map);
            RCLCPP_INFO(logger_,
                        "Colored point cloud map saved (total: %zu pts -> colored: %zu pts).",
                        full_map->size(), colored_map->size());
        } else {
            RCLCPP_WARN(logger_, "No colored points found; map_3d_colored.pcd not saved.");
        }

        save2DMap(keyframeLaserClouds, keyframePosesUpdated, recentIdxUpdated);
    } else {
        RCLCPP_WARN(logger_, "Cannot build map: global point cloud is empty.");
    }
}


void MapPavingEngine::save2DMap(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                               const std::vector<Pose6D>& keyframePosesUpdated,
                               int recentIdxUpdated) {
    if (keyframeLaserClouds.empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>());

    // 1. 逐帧投影并进行动态躯干切片 (Per-Frame Local Base-Link Height Slicing)
    int num_keyframes = keyframeLaserClouds.size();
    for (int i = 0; i <= recentIdxUpdated && i < num_keyframes; i++) {
        const auto& pose = keyframePosesUpdated[i];
        Eigen::Affine3f transCur = pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
        Eigen::Matrix4f T_body_to_base = config_.T_body_to_baselink.inverse();
        pcl::PointCloud<PointTypeRGB> base_local;
        pcl::transformPointCloud(*keyframeLaserClouds[i], base_local, T_body_to_base);
        
        // 由于开启重力对齐后全局坐标系原生绝对水平，直接将局部切片转换至全局 map/camera_init 坐标系
        Eigen::Matrix4f T_base_to_global_map = transCur.matrix() * config_.T_body_to_baselink;

        for (const auto& pt : base_local.points) {
            if (pt.z >= config_.pcd2pgm_thre_z_min && pt.z <= config_.pcd2pgm_thre_z_max) {
                pcl::PointXYZ p;
                p.x = T_base_to_global_map(0,0)*pt.x + T_base_to_global_map(0,1)*pt.y + T_base_to_global_map(0,2)*pt.z + T_base_to_global_map(0,3);
                p.y = T_base_to_global_map(1,0)*pt.x + T_base_to_global_map(1,1)*pt.y + T_base_to_global_map(1,2)*pt.z + T_base_to_global_map(1,3);
                p.z = T_base_to_global_map(2,0)*pt.x + T_base_to_global_map(2,1)*pt.y + T_base_to_global_map(2,2)*pt.z + T_base_to_global_map(2,3);
                cloud_pass_through->points.push_back(p);
            }
        }
    }

    cloud_pass_through->width = cloud_pass_through->points.size();
    cloud_pass_through->height = 1;
    cloud_pass_through->is_dense = true;

    RCLCPP_INFO(logger_, "2D Map: after dynamic height filter, %lu points remain.", cloud_pass_through->points.size());

    if (cloud_pass_through->empty()) {
        RCLCPP_WARN(logger_, "No points remain after dynamic Z height filtering!");
        return;
    }

    // 2.5 体素滤波下采样 (Voxel Grid Filter)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_pass_through);
    voxel_filter.setLeafSize(config_.pcd2pgm_map_resolution, config_.pcd2pgm_map_resolution, config_.pcd2pgm_map_resolution);
    voxel_filter.filter(*cloud_downsampled);

    RCLCPP_INFO(logger_, "2D Map: after voxel downsampling, %lu points remain.", cloud_downsampled->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius = cloud_downsampled;

    // 如果 VoxelGrid 因为边界过大发生溢出（点数没变），或者体素滤波后点数依然极为庞大（>100万），
    // 此时强行跑 RadiusOutlierRemoval 会导致进程卡死，因此必须跳过去噪步骤。
    if (cloud_downsampled->points.size() < 1000000) {
        // 3. 半径噪点滤波移除
        cloud_after_radius.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
        radius_outlier.setInputCloud(cloud_downsampled);
        radius_outlier.setRadiusSearch(config_.pcd2pgm_thre_radius);
        radius_outlier.setMinNeighborsInRadius(config_.pcd2pgm_thres_point_count);
        radius_outlier.filter(*cloud_after_radius);
        
        RCLCPP_INFO(logger_, "2D Map: after radius outlier filter, %lu points remain.", cloud_after_radius->points.size());
    } else {
        RCLCPP_WARN(logger_, "2D Map: Point cloud too large (%lu points after voxel), skipping RadiusOutlierRemoval to prevent freezing.", cloud_downsampled->points.size());
    }

    if (cloud_after_radius->empty()) {
        RCLCPP_WARN(logger_, "No points remain after filtering!");
        return;
    }

    // 4. 边界范围计算
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto & point : cloud_after_radius->points) {
        x_min = std::min(x_min, static_cast<double>(point.x));
        x_max = std::max(x_max, static_cast<double>(point.x));
        y_min = std::min(y_min, static_cast<double>(point.y));
        y_max = std::max(y_max, static_cast<double>(point.y));
    }

    int width = std::ceil((x_max - x_min) / config_.pcd2pgm_map_resolution);
    int height = std::ceil((y_max - y_min) / config_.pcd2pgm_map_resolution);

    if (width <= 0 || height <= 0) {
        RCLCPP_WARN(logger_, "Invalid 2D map size: %d x %d", width, height);
        return;
    }

    // 5. 投影与 2D 连通域二次去噪 (Two-Stage 2D Connected Component Denoising)
    cv::Mat map_img(height, width, CV_8UC1, cv::Scalar(254));

    for (const auto & point : cloud_after_radius->points) {
        int i = std::floor((point.x - x_min) / config_.pcd2pgm_map_resolution);
        int j = std::floor((point.y - y_min) / config_.pcd2pgm_map_resolution);

        if (i >= 0 && i < width && j >= 0 && j < height) {
            int img_j = height - 1 - j;
            map_img.at<uchar>(img_j, i) = 0;
        }
    }

    // 第二阶段去噪：使用 8-连通域剔除面积小 < 10 像素的悬空/乱飞离群噪点块
    cv::Mat binary_mask;
    cv::threshold(map_img, binary_mask, 50, 255, cv::THRESH_BINARY_INV);
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8);

    int removed_blobs = 0;
    int removed_pixels = 0;
    const int MIN_BLOB_PIXELS = 10; // 连通小黑点剔除阈值 (< 10 px)

    for (int label = 1; label < num_labels; ++label) {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < MIN_BLOB_PIXELS) {
            removed_blobs++;
            removed_pixels += area;

            int left = stats.at<int>(label, cv::CC_STAT_LEFT);
            int top = stats.at<int>(label, cv::CC_STAT_TOP);
            int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);

            for (int r = top; r < top + h; ++r) {
                for (int c = left; c < left + w; ++c) {
                    if (labels.at<int>(r, c) == label) {
                        map_img.at<uchar>(r, c) = 254;
                    }
                }
            }
        }
    }

    RCLCPP_INFO(logger_,
                "2D Map 2nd Stage: Cleared %d isolated noise blobs (< %d px, total %d pixels).",
                removed_blobs, MIN_BLOB_PIXELS, removed_pixels);

    // 写入 PGM 文件
    std::string pgm_file = config_.save_directory + "map_2d.pgm";
    std::ofstream pgm(pgm_file, std::ios::binary);
    if (pgm.is_open()) {
        pgm << "P5\n" << width << " " << height << "\n255\n";
        pgm.write(reinterpret_cast<const char*>(map_img.data), map_img.total());
        pgm.close();
        RCLCPP_INFO(logger_, "Successfully saved 2D PGM map to: %s", pgm_file.c_str());
    } else {
        RCLCPP_ERROR(logger_, "Failed to open PGM file for writing: %s", pgm_file.c_str());
    }

    // 6. 写入 YAML 配置文件
    std::string yaml_file = config_.save_directory + "map_2d.yaml";
    std::ofstream yaml(yaml_file);
    if (yaml.is_open()) {
        yaml << "image: map_2d.pgm\n";
        yaml << "resolution: " << config_.pcd2pgm_map_resolution << "\n";
        yaml << "origin: [" << x_min << ", " << y_min << ", 0.0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.196\n";
        yaml.close();
        RCLCPP_INFO(logger_, "Successfully saved 2D YAML config to: %s", yaml_file.c_str());
    } else {
        RCLCPP_ERROR(logger_, "Failed to open YAML file for writing: %s", yaml_file.c_str());
    }
}
