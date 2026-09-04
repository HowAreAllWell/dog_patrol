#include "mapping/loop_detector.h"
#include <fstream>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

LoopDetector::LoopDetector(rclcpp::Node* node) : node_(node), kdtreeHistoryKeyPoses_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    double scDistThres = node_->get_parameter("sc_dist_thres").as_double();
    double scMaximumRadius = node_->get_parameter("sc_max_radius").as_double();
    scManager_.setSCdistThres(scDistThres);
    scManager_.setMaximumRadius(scMaximumRadius);

    historyKeyframeSearchRadius_ = node_->get_parameter("historyKeyframeSearchRadius").as_double();
    historyKeyframeSearchTimeDiff_ = node_->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    historyKeyframeSearchNum_ = node_->get_parameter("historyKeyframeSearchNum").as_int();
    loopFitnessScoreThreshold_ = node_->get_parameter("loopFitnessScoreThreshold").as_double();

    double historyCloudDownsample = 0.4;
    downSizeFilterICP_.setLeafSize(historyCloudDownsample, historyCloudDownsample, historyCloudDownsample);
}

void LoopDetector::addKeyframe(const pcl::PointCloud<PointTypeRGB>::Ptr& cloud, const Pose6D& odom_pose, double time, pcl::PointCloud<PointType>& cloud_ds) {
    std::lock_guard<std::mutex> lock(mKF_);
    keyframeLaserClouds_.push_back(cloud);
    keyframePoses_.push_back(odom_pose);
    keyframePosesUpdated_.push_back(odom_pose);
    keyframeTimes_.push_back(time);
    scManager_.makeAndSaveScancontextAndKeys(cloud_ds);
}

void LoopDetector::addVisualLoopCandidate(int prev_node_idx, int curr_node_idx) {
    std::lock_guard<std::mutex> lock(mBuf_);
    auto it = loopIndexContainer_.find(curr_node_idx);
    if (it == loopIndexContainer_.end()) {
        RCLCPP_DEBUG(node_->get_logger(),
                    "Visual loop candidate received in backend. History ID: %d, Current ID: %d.",
                    prev_node_idx, curr_node_idx);
        scLoopICPBuf_.push(std::make_pair(prev_node_idx, curr_node_idx));
    }
}

void LoopDetector::saveSCDatabase(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to open ScanContext database file for saving.");
        return;
    }

    std::lock_guard<std::mutex> lock(mKF_);
    int num_frames = keyframePosesUpdated_.size();
    int sc_size = scManager_.polarcontexts_.size();
    int save_num = std::min(num_frames, sc_size);

    ofs << save_num << "\n";
    for (int i = 0; i < save_num; ++i) {
        Pose6D p = keyframePosesUpdated_[i];
        ofs << i << " " << p.x << " " << p.y << " " << p.z << " " << p.roll << " " << p.pitch << " " << p.yaw
            << "\n";

        Eigen::MatrixXd sc = scManager_.polarcontexts_[i];
        ofs << sc.rows() << " " << sc.cols() << "\n";
        for (int r = 0; r < sc.rows(); ++r) {
            for (int c = 0; c < sc.cols(); ++c) {
                ofs << sc(r, c) << " ";
            }
            ofs << "\n";
        }
    }
    ofs.close();
    RCLCPP_INFO(node_->get_logger(),
                "ScanContext database saved to file (Path: %s, Frames count: %d).",
                filename.c_str(), save_num);
}

void LoopDetector::RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut) {
    cloudOut->points.resize(cloudIn->points.size());
    cloudOut->header = cloudIn->header;
    cloudOut->width = cloudIn->width;
    cloudOut->height = cloudIn->height;
    for (size_t i = 0; i < cloudIn->points.size(); ++i) {
        cloudOut->points[i].x = cloudIn->points[i].x;
        cloudOut->points[i].y = cloudIn->points[i].y;
        cloudOut->points[i].z = cloudIn->points[i].z;
        cloudOut->points[i].intensity = 0.0f;
    }
}

pcl::PointCloud<PointType>::Ptr LoopDetector::local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

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
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<PointTypeRGB>::Ptr LoopDetector::local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, const Pose6D& tf) {
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

void LoopDetector::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
    nearKeyframes->clear();
    int cloudSize;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        cloudSize = keyframeLaserClouds_.size();
    }
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;
        std::lock_guard<std::mutex> lock(mKF_);
        pcl::PointCloud<PointType>::Ptr cloudTempXYZI(new pcl::PointCloud<PointType>());
        RGB2XYZI(keyframeLaserClouds_[keyNear], cloudTempXYZI);
        *nearKeyframes += *local2global(cloudTempXYZI, keyframePosesUpdated_[keyNear]);
    }

    if (nearKeyframes->empty()) return;

    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP_.setInputCloud(nearKeyframes);
    downSizeFilterICP_.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

gtsam::Pose3 LoopDetector::doICPVirtualRelative(int loop_kf_idx, int curr_kf_idx,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocal,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopSubmapLocal,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocalRegisted) {
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    loopFindNearKeyframes(cureKeyframeCloud, curr_kf_idx, 0);
    loopFindNearKeyframes(targetKeyframeCloud, loop_kf_idx, historyKeyframeSearchNum_);

    if (cureKeyframeCloud->empty() || targetKeyframeCloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "ICP failed: Target or Current keyframe cloud is empty.");
        return gtsam::Pose3::identity();
    }

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    cureKeyframeCloudMsg.header.stamp = node_->now();
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    targetKeyframeCloudMsg.header.stamp = node_->now();
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    fast_gicp::FastVGICP<PointType, PointType> icp;
    icp.setResolution(1.0);
    icp.setNumThreads(4);
    icp.setMaxCorrespondenceDistance(150.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudRegMsg;
    pcl::toROSMsg(*unused_result, cureKeyframeCloudRegMsg);
    cureKeyframeCloudRegMsg.header.frame_id = "camera_init";
    cureKeyframeCloudRegMsg.header.stamp = node_->now();
    pubLoopScanLocalRegisted->publish(cureKeyframeCloudRegMsg);

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold_) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "Backend ICP verification failed. Current: %.4f, Threshold: %.4f.",
                     icp.getFitnessScore(), loopFitnessScoreThreshold_);
        return gtsam::Pose3::identity();
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR PGO constraint added between %d and %d. ICP Score: %.4f",
                    loop_kf_idx, curr_kf_idx, icp.getFitnessScore());
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    Pose6D curr_pose, loop_pose;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        curr_pose = keyframePosesUpdated_[curr_kf_idx];
        loop_pose = keyframePosesUpdated_[loop_kf_idx];
    }

    Eigen::Affine3f tWrong = pcl::getTransformation(curr_pose.x, curr_pose.y, curr_pose.z, curr_pose.roll, curr_pose.pitch, curr_pose.yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(double(loop_pose.roll), double(loop_pose.pitch), double(loop_pose.yaw)),
                                       gtsam::Point3(double(loop_pose.x), double(loop_pose.y), double(loop_pose.z)));

    return poseFrom.between(poseTo);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetector::vector2pc(const std::vector<Pose6D>& vectorPose6d) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : vectorPose6d) {
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

bool LoopDetector::detectLoopClosureDistance(int* loopKeyCur, int* loopKeyPre) {
    std::lock_guard<std::mutex> lock(mBuf_);
    auto it = loopIndexContainer_.find(*loopKeyCur);
    if (it != loopIndexContainer_.end()) return false;

    std::vector<Pose6D> poses_copy;
    std::vector<double> times_copy;
    {
        std::lock_guard<std::mutex> lock_kf(mKF_);
        poses_copy = keyframePoses_;
        times_copy = keyframeTimes_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(poses_copy);
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses_->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses_->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius_,
                                         pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (size_t i = 0; i < pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (std::abs(times_copy[id] - times_copy[*loopKeyCur]) > historyKeyframeSearchTimeDiff_) {
            *loopKeyPre = id;
            break;
        }
    }

    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre) return false;

    return true;
}

void LoopDetector::performSCLoopClosure() {
    int curr_node_idx;
    double curr_time;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (int(keyframePoses_.size()) < scManager_.NUM_EXCLUDE_RECENT) return;
        curr_node_idx = keyframePoses_.size() - 1;
        curr_time = keyframeTimes_[curr_node_idx];
    }

    auto detectResult = scManager_.detectLoopClosureID();
    int SCclosestHistoryFrameID = detectResult.first;

    if (SCclosestHistoryFrameID != -1) {
        const int prev_node_idx = SCclosestHistoryFrameID;
        double prev_time;
        {
            std::lock_guard<std::mutex> lock(mKF_);
            prev_time = keyframeTimes_[prev_node_idx];
        }

        double time_diff = std::abs(curr_time - prev_time);
        if (time_diff < historyKeyframeSearchTimeDiff_) {
            return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR ScanContext loop candidate proposed. History ID: %d, Current ID: %d. Running ICP verification...",
                    prev_node_idx, curr_node_idx);

        std::lock_guard<std::mutex> lock_buf(mBuf_);
        scLoopICPBuf_.push(std::make_pair(prev_node_idx, curr_node_idx));
    }
}

void LoopDetector::performRSLoopClosure() {
    int loopKeyCur;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (keyframePoses_.empty()) return;
        loopKeyCur = keyframePoses_.size() - 1;
    }
    int loopKeyPre = -1;

    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre)) {
        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR spatial distance loop candidate proposed. History ID: %d, Current ID: %d. Running ICP verification...",
                    loopKeyPre, loopKeyCur);
        std::lock_guard<std::mutex> lock_buf(mBuf_);
        scLoopICPBuf_.push(std::make_pair(loopKeyPre, loopKeyCur));
        loopIndexContainer_[loopKeyCur] = loopKeyPre;
    }
}

std::mutex& LoopDetector::getKFMutex() { return mKF_; }
std::mutex& LoopDetector::getBufMutex() { return mBuf_; }

std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& LoopDetector::getKeyframeLaserClouds() { return keyframeLaserClouds_; }
std::vector<Pose6D>& LoopDetector::getKeyframePoses() { return keyframePoses_; }
std::vector<Pose6D>& LoopDetector::getKeyframePosesUpdated() { return keyframePosesUpdated_; }
std::vector<double>& LoopDetector::getKeyframeTimes() { return keyframeTimes_; }
std::map<int, int>& LoopDetector::getLoopIndexContainer() { return loopIndexContainer_; }
std::queue<std::pair<int, int>>& LoopDetector::getScLoopICPBuf() { return scLoopICPBuf_; }
