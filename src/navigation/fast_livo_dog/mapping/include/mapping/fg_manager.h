#pragma once

#include <mutex>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

/**
 * @brief 因子图管理器类，管理 GTSAM 优化库中的位姿图增量求解
 */
class FactorGraphManager {
public:
    FactorGraphManager();
    ~FactorGraphManager();

    // 初始化优化中各类约束的协方差噪声
    void initNoises(double priorNoiseVar, double odomNoiseVarTrans, double odomNoiseVarRot, 
                    double loopNoiseScore, double gpsXYNoiseVar, double gpsZNoiseVar);

    // 添加里程计先验约束（起始帧）
    bool addPriorFactor(int index, const gtsam::Pose3& pose);

    // 添加相邻帧里程计相对位姿约束
    void addOdomFactor(int prev_idx, int curr_idx, const gtsam::Pose3& relative_pose, const gtsam::Pose3& pose_to);

    // 添加GPS全球绝对坐标定位约束
    void addGPSFactor(int index, const gtsam::Point3& gps_point);

    // 添加非相邻帧的回环检测相对位姿约束
    void addLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose);

    // 添加纯视觉 PGO 兜底约束
    void addVisualLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose);

    // 触发图优化计算，更新因子图轨迹并获取优化结果
    bool runOptimization(int graphUpdateTimes, gtsam::Values& currentEstimate);

    // 获取当前图优化计算中的估计位姿
    gtsam::Values getISAMCurrentEstimate();

    // 因子图是否已经开始创建
    bool isGraphMade() const;

    // 获取位姿图修改锁
    std::mutex& getMutex();

private:
    gtsam::NonlinearFactorGraph gtSAMgraph_;
    gtsam::Values initialEstimate_;
    gtsam::ISAM2* isam_;
    gtsam::Values isamCurrentEstimate_;
    bool gtSAMgraphMade_;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise_;
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise_;
    gtsam::noiseModel::Base::shared_ptr robustVisualLoopNoise_;
    gtsam::noiseModel::Base::shared_ptr robustGPSNoise_;

    std::mutex mtxPosegraph_;
};
