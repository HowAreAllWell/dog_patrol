#pragma once

#include <mutex>
#include <queue>
#include <vector>
#include <map>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <gtsam/geometry/Pose3.h>

#include "scancontext/Scancontext.h"
#include "mapping/common.h"

typedef pcl::PointXYZRGB PointTypeRGB;

/**
 * @brief 回环检测器类，负责 ScanContext 指纹检索与基于 Fast-VGICP 的回环校验逻辑
 */
class LoopDetector {
public:
    LoopDetector(rclcpp::Node* node);

    // 录入新的关键帧传感器观测并提取指纹特征
    void addKeyframe(const pcl::PointCloud<PointTypeRGB>::Ptr& cloud, const Pose6D& odom_pose, double time, pcl::PointCloud<PointType>& cloud_ds);

    // 缓存来自视觉节点的潜在闭环匹配帧候选建议
    void addVisualLoopCandidate(int prev_node_idx, int curr_node_idx);

    // 保存特征匹配 ScanContext 数据库到文件
    void saveSCDatabase(const std::string& filename);

    // 点云格式转换辅助函数
    void RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut);

    // 点云转正世界参考系投影转换
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf);

    // 点云转正世界参考系投影转换 (带颜色)
    pcl::PointCloud<PointTypeRGB>::Ptr local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, const Pose6D& tf);

    // 拼接临近帧产生局部的精细匹配子图
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);

    // 运行 Fast-VGICP 进行精细化闭环评估对准，计算相对约束偏移
    gtsam::Pose3 doICPVirtualRelative(int loop_kf_idx, int curr_kf_idx,
                                      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocal,
                                      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopSubmapLocal,
                                      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocalRegisted);

    // 将关键帧数据结构转换为 KdTree 节点位姿集合
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D>& vectorPose6d);

    // 基于空间几何物理距离寻找闭环检测候选帧
    bool detectLoopClosureDistance(int* loopKeyCur, int* loopKeyPre);

    // 基于 ScanContext 执行指纹检索
    void performSCLoopClosure();

    // 基于几何拓扑距离评估潜在闭环
    void performRSLoopClosure();

    // 提供内部互斥锁与核心参数接口，保证多线程调度安全
    std::mutex& getKFMutex();
    std::mutex& getBufMutex();

    std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& getKeyframeLaserClouds();
    std::vector<Pose6D>& getKeyframePoses();
    std::vector<Pose6D>& getKeyframePosesUpdated();
    std::vector<double>& getKeyframeTimes();
    std::map<int, int>& getLoopIndexContainer();
    std::queue<std::pair<int, int>>& getScLoopICPBuf();

private:
    rclcpp::Node* node_;
    SCManager scManager_;
    pcl::VoxelGrid<PointType> downSizeFilterICP_;
    
    std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> keyframeLaserClouds_;
    std::vector<Pose6D> keyframePoses_;
    std::vector<Pose6D> keyframePosesUpdated_;
    std::vector<double> keyframeTimes_;

    std::map<int, int> loopIndexContainer_;
    std::queue<std::pair<int, int>> scLoopICPBuf_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses_;

    double historyKeyframeSearchRadius_;
    double historyKeyframeSearchTimeDiff_;
    int historyKeyframeSearchNum_;
    double loopFitnessScoreThreshold_;

    std::mutex mKF_;
    std::mutex mBuf_;
};
