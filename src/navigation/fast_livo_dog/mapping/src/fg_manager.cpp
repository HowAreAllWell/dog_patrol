#include "mapping/fg_manager.h"

FactorGraphManager::FactorGraphManager() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
    gtSAMgraphMade_ = false;
}

FactorGraphManager::~FactorGraphManager() {
    delete isam_;
}

void FactorGraphManager::initNoises(double priorNoiseVar, double odomNoiseVarTrans, double odomNoiseVarRot, 
                                    double loopNoiseScore, double gpsXYNoiseVar, double gpsZNoiseVar) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar;
    priorNoise_ = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << odomNoiseVarTrans, odomNoiseVarTrans, odomNoiseVarTrans, odomNoiseVarRot, odomNoiseVarRot, odomNoiseVarRot;
    odomNoise_ = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6);

    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    );

    gtsam::Vector robustVisualNoiseVector6(6);
    robustVisualNoiseVector6 << loopNoiseScore * 10.0, loopNoiseScore * 10.0, loopNoiseScore * 10.0, 
                                loopNoiseScore * 10.0, loopNoiseScore * 10.0, loopNoiseScore * 10.0;
    robustVisualLoopNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustVisualNoiseVector6)
    );

    gtsam::Vector robustNoiseVector3(3);
    robustNoiseVector3 << gpsXYNoiseVar, gpsXYNoiseVar, gpsZNoiseVar;
    robustGPSNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3)
    );
}

bool FactorGraphManager::addPriorFactor(int index, const gtsam::Pose3& pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    if (gtSAMgraphMade_) return false;
    gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(index, pose, priorNoise_));
    initialEstimate_.insert(index, pose);
    gtSAMgraphMade_ = true;
    return true;
}

void FactorGraphManager::addOdomFactor(int prev_idx, int curr_idx, const gtsam::Pose3& relative_pose, const gtsam::Pose3& pose_to) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_idx, curr_idx, relative_pose, odomNoise_));
    initialEstimate_.insert(curr_idx, pose_to);
}

void FactorGraphManager::addGPSFactor(int index, const gtsam::Point3& gps_point) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::GPSFactor(index, gps_point, robustGPSNoise_));
}

void FactorGraphManager::addLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_idx, prev_idx, relative_pose, robustLoopNoise_));
}

void FactorGraphManager::addVisualLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_idx, prev_idx, relative_pose, robustVisualLoopNoise_));
}

bool FactorGraphManager::runOptimization(int graphUpdateTimes, gtsam::Values& currentEstimate) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    if (!gtSAMgraphMade_) return false;
    isam_->update(gtSAMgraph_, initialEstimate_);
    isam_->update();
    for (int i = graphUpdateTimes; i > 0; --i) {
        isam_->update();
    }
    gtSAMgraph_.resize(0);
    initialEstimate_.clear();
    isamCurrentEstimate_ = isam_->calculateEstimate();
    currentEstimate = isamCurrentEstimate_;
    return true;
}

gtsam::Values FactorGraphManager::getISAMCurrentEstimate() {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    return isamCurrentEstimate_;
}

bool FactorGraphManager::isGraphMade() const {
    return gtSAMgraphMade_;
}

std::mutex& FactorGraphManager::getMutex() {
    return mtxPosegraph_;
}
