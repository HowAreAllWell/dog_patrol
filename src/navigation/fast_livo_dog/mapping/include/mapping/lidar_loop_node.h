#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <fstream>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <gtsam/nonlinear/Values.h>

#include "mapping/common.h"
#include "mapping/fg_manager.h"
#include "mapping/loop_detector.h"
#include "mapping/map_paving_engine.h"

/**
 * @brief LidarLoopNode 主节点类，充当 ROS 2 与后台多线程算法总线枢纽
 */
class LidarLoopNode : public rclcpp::Node {
public:
    LidarLoopNode();
    ~LidarLoopNode();

private:
    void initParameters();
    void initFileSave();
    void initPublishers();
    void initSubscribers();

    // 传感器回调函数
    void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void visualLoopHandler(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void arucoLandmarkObsHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void visualPGOConstraintHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void saveArucoLandmarks();

    // 位姿与转换工具函数
    std::string padZeros(int val, int num_digits = 6);
    gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p);
    Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    Pose6D diffTransformation(const Pose6D& p1, const Pose6D& p2);
    Eigen::Affine3f Pose6dToAffine3f(const Pose6D& pose);

    // 数据保存与发布函数
    void saveOdometryVerticesKITTIformat(const std::string& filename);
    void saveOptimizedVerticesKITTIformat(const gtsam::Values& estimates, const std::string& filename);
    void pubPath();
    void visualizeLoopClosure();
    void updatePoses(const gtsam::Values& isamCurrentEstimate);

    // 线程主执行函数
    void process_pg();
    void process_lcd();
    void process_icp();
    void process_isam();

    // 点云转换辅助函数
    void RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut);

    // --- 内部变量与状态管理 ---
    std::string saveDirectory_;
    double keyframeMeterGap_;
    double keyframeDegGap_;
    double keyframeRadGap_;
    double scDistThres_;
    double scMaximumRadius_;
    double historyKeyframeSearchRadius_;
    double historyKeyframeSearchTimeDiff_;
    int historyKeyframeSearchNum_;
    double loopNoiseScore_;
    int graphUpdateTimes_;
    double loopFitnessScoreThreshold_;
    double speedFactor_;
    double loopClosureFrequency_;
    double graphUpdateFrequency_;
    double vizPathFrequency_;
    double mapVizFilterSize_;

    std::string pgKITTIformat_;
    std::string odomKITTIformat_;
    std::string pgScansDirectory_;
    std::string pgImageDirectory_;
    std::fstream pgTimeSaveStream_;

    // 发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO_;           // 默认 PGO 轨迹
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathOdom_;              // 自测: 原始前端 Odometry 轨迹
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathPGOArUco_;         // 自测: 额外带 ArUco 闭环修正的轨迹
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocalRegisted_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pubKeyFramesId_;

    Eigen::Matrix4f T_body_to_baselink_ = Eigen::Matrix4f::Identity();

    double pcd2pgm_map_resolution_;
    double pcd2pgm_thre_z_min_;
    double pcd2pgm_thre_z_max_;
    double pcd2pgm_thre_radius_;
    int pcd2pgm_thres_point_count_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subVisualLoop_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subArucoLandmarkObs_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subVisualPGO_;

    // ArUco landmark 缓存: marker_id -> {first_kf_id, T_lidar_marker[16]}
    struct ArucoLandmarkObs {
        int kf_id;
        double marker_size;
        std::array<double, 16> T_lidar_marker;
    };
    std::map<int, ArucoLandmarkObs> arucoLandmarkObs_;
    std::mutex arucoLandmarkMutex_;

    // 自测三轨迹 Path 缓存
    nav_msgs::msg::Path pathOdomMsg_;
    nav_msgs::msg::Path pathPGOArUcoMsg_;
    bool arucoLoopTriggered_{false};

    // 数据缓存
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf_;
    std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsBuf_;
    std::queue<std_msgs::msg::Float64MultiArray> visualPGOQueue_;

    // 互斥锁
    std::mutex mBuf_;
    std::mutex mtxRecentPose_;
    std::mutex mVisualPGO_;

    // 里程计运动累计状态
    double translationAccumulated_ = 1000000.0;
    double rotaionAccumulated_ = 1000000.0;
    bool isNowKeyFrame_ = false;
    Pose6D odom_pose_prev_;
    Pose6D odom_pose_curr_;
    double timeLaserOdometry_ = 0.0;
    double timeLaser_ = 0.0;

    int recentIdxUpdated_ = 0;

    // 后端核心解耦类实例组件
    std::unique_ptr<FactorGraphManager> factorGraphManager_;
    std::unique_ptr<LoopDetector> loopDetector_;
    std::unique_ptr<MapPavingEngine> mapPavingEngine_;

    // 关键帧滤波
    pcl::VoxelGrid<PointType> downSizeFilterScancontext_;

    // GPS 约束状态
    bool useGPS_ = true;
    sensor_msgs::msg::NavSatFix::SharedPtr currGPS_;
    bool hasGPSforThisKF_ = false;
    bool gpsOffsetInitialized_ = false;
    double gpsAltitudeInitOffset_ = 0.0;
    double recentOptimizedX_ = 0.0;
    double recentOptimizedY_ = 0.0;

    // 线程对象
    std::thread posegraphThread_;
    std::thread lcDetectionThread_;
    std::thread icpThread_;
    std::thread isamThread_;
    std::atomic<bool> stopThreads_{false};
    std::atomic<int> nodesAddedToGraph_{0};

    const double HISTORY_CLOUD_DOWNSAMPLE_ = 0.4;
    const double LOOP_NODE_MARKER_SCALE_ = 0.3;
    const double LOOP_EDGE_MARKER_SCALE_ = 0.1;
    const size_t MAX_ICP_QUEUE_SIZE_ = 30;

    // --- GTSAM 先验及里程计噪声协方差方差常量 ---
    const double PRIOR_NOISE_VAR_ = 1e-12;
    const double ODOM_NOISE_VAR_TRANS_ = 1e-6;
    const double ODOM_NOISE_VAR_ROT_ = 1e-4;
    const double GPS_XY_NOISE_VAR_ = 1000000000.0;
    const double GPS_Z_NOISE_VAR_ = 250.0;
};
