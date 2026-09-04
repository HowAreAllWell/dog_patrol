#include "mapping/lidar_loop_node.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

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
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

using namespace gtsam;

LidarLoopNode::LidarLoopNode() : Node("lidar_loop_node") {
    initParameters();
    
    // 填充 MapPavingEngine 配置并实例化
    MapPavingEngine::Config paving_config;
    paving_config.save_directory = saveDirectory_;
    paving_config.T_body_to_baselink = T_body_to_baselink_;
    paving_config.pcd2pgm_map_resolution = pcd2pgm_map_resolution_;
    paving_config.pcd2pgm_thre_z_min = pcd2pgm_thre_z_min_;
    paving_config.pcd2pgm_thre_z_max = pcd2pgm_thre_z_max_;
    paving_config.pcd2pgm_thre_radius = pcd2pgm_thre_radius_;
    paving_config.pcd2pgm_thres_point_count = pcd2pgm_thres_point_count_;

    factorGraphManager_ = std::make_unique<FactorGraphManager>();
    loopDetector_ = std::make_unique<LoopDetector>(this);
    mapPavingEngine_ = std::make_unique<MapPavingEngine>(paving_config, this->get_logger());

    // 初始化噪声参数并传参
    factorGraphManager_->initNoises(
        PRIOR_NOISE_VAR_, ODOM_NOISE_VAR_TRANS_, ODOM_NOISE_VAR_ROT_,
        loopNoiseScore_, GPS_XY_NOISE_VAR_, GPS_Z_NOISE_VAR_
    );

    downSizeFilterScancontext_.setLeafSize(HISTORY_CLOUD_DOWNSAMPLE_, HISTORY_CLOUD_DOWNSAMPLE_,
                                           HISTORY_CLOUD_DOWNSAMPLE_);

    initPublishers();
    initSubscribers();
    initFileSave();

    // 启动多线程
    posegraphThread_ = std::thread(&LidarLoopNode::process_pg, this);
    lcDetectionThread_ = std::thread(&LidarLoopNode::process_lcd, this);
    icpThread_ = std::thread(&LidarLoopNode::process_icp, this);
    isamThread_ = std::thread(&LidarLoopNode::process_isam, this);

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
}

LidarLoopNode::~LidarLoopNode() {
    stopThreads_ = true;

    if (posegraphThread_.joinable()) posegraphThread_.join();
    if (lcDetectionThread_.joinable()) lcDetectionThread_.join();
    if (icpThread_.joinable()) icpThread_.join();
    if (isamThread_.joinable()) isamThread_.join();

    loopDetector_->saveSCDatabase(saveDirectory_ + "sc_database.txt");
    
    // 确保最近更新的关键帧索引覆盖所有已添加的帧，以防优化线程没来得及完成
    recentIdxUpdated_ = std::max(recentIdxUpdated_, int(loopDetector_->getKeyframePosesUpdated().size()) - 1);

    // 调用 MapPavingEngine 进行拼接、调平、过滤与文件保存
    mapPavingEngine_->buildAndSaveMaps(
        loopDetector_->getKeyframeLaserClouds(),
        loopDetector_->getKeyframePosesUpdated(),
        recentIdxUpdated_
    );

    if (pgTimeSaveStream_.is_open()) {
        pgTimeSaveStream_.close();
    }
}

void LidarLoopNode::RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut) {
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

void LidarLoopNode::initParameters() {
        this->declare_parameter<std::string>("save_directory", "./src/navigation/fast_livo_dog/data/");
    this->declare_parameter<double>("keyframe_meter_gap", 2.0);
    this->declare_parameter<double>("keyframe_deg_gap", 10.0);
    this->declare_parameter<double>("sc_dist_thres", 0.15);
    this->declare_parameter<double>("sc_max_radius", 80.0);
    this->declare_parameter<double>("historyKeyframeSearchRadius", 10.0);
    this->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
    this->declare_parameter<int>("historyKeyframeSearchNum", 25);
    this->declare_parameter<double>("loopNoiseScore", 0.5);
    this->declare_parameter<int>("graphUpdateTimes", 2);
    this->declare_parameter<double>("loopFitnessScoreThreshold", 0.20);
    this->declare_parameter<double>("speedFactor", 1.0);
    this->declare_parameter<double>("loopClosureFrequency", 2.0);
    this->declare_parameter<double>("graphUpdateFrequency", 1.0);
    this->declare_parameter<double>("vizmapFrequency", 0.5);
    this->declare_parameter<double>("vizPathFrequency", 10.0);
    this->declare_parameter<double>("mapviz_filter_size", 0.05);

    // pcd2pgm 栅格化参数
    this->declare_parameter<double>("pcd2pgm_map_resolution", 0.05);
    this->declare_parameter<double>("pcd2pgm_thre_z_min", -0.4);
    this->declare_parameter<double>("pcd2pgm_thre_z_max", 0.2);
    this->declare_parameter<double>("pcd2pgm_thre_radius", 0.8);
    this->declare_parameter<int>("pcd2pgm_thres_point_count", 10);

    saveDirectory_ = this->get_parameter("save_directory").as_string();
    keyframeMeterGap_ = this->get_parameter("keyframe_meter_gap").as_double();
    keyframeDegGap_ = this->get_parameter("keyframe_deg_gap").as_double();
    keyframeRadGap_ = deg2rad(keyframeDegGap_);
    scDistThres_ = this->get_parameter("sc_dist_thres").as_double();
    scMaximumRadius_ = this->get_parameter("sc_max_radius").as_double();
    historyKeyframeSearchRadius_ = this->get_parameter("historyKeyframeSearchRadius").as_double();
    historyKeyframeSearchTimeDiff_ = this->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    historyKeyframeSearchNum_ = this->get_parameter("historyKeyframeSearchNum").as_int();

    pcd2pgm_map_resolution_ = this->get_parameter("pcd2pgm_map_resolution").as_double();
    pcd2pgm_thre_z_min_ = this->get_parameter("pcd2pgm_thre_z_min").as_double();
    pcd2pgm_thre_z_max_ = this->get_parameter("pcd2pgm_thre_z_max").as_double();
    pcd2pgm_thre_radius_ = this->get_parameter("pcd2pgm_thre_radius").as_double();
    pcd2pgm_thres_point_count_ = this->get_parameter("pcd2pgm_thres_point_count").as_int();
    loopNoiseScore_ = this->get_parameter("loopNoiseScore").as_double();
    graphUpdateTimes_ = this->get_parameter("graphUpdateTimes").as_int();
    loopFitnessScoreThreshold_ = this->get_parameter("loopFitnessScoreThreshold").as_double();
    speedFactor_ = this->get_parameter("speedFactor").as_double();
    loopClosureFrequency_ = this->get_parameter("loopClosureFrequency").as_double() * speedFactor_;
    graphUpdateFrequency_ = this->get_parameter("graphUpdateFrequency").as_double() * speedFactor_;
    vizPathFrequency_ = this->get_parameter("vizPathFrequency").as_double() * speedFactor_;
    mapVizFilterSize_ = this->get_parameter("mapviz_filter_size").as_double();

    // 载入激光雷达和机体外参
    std::vector<double> ext_matrix = std::vector<double>(16, 0.0);
    if (!this->has_parameter("T_lidar_base")) {
        this->declare_parameter<std::vector<double>>("T_lidar_base", std::vector<double>(16, 0.0));
    }
    this->get_parameter("T_lidar_base", ext_matrix);
    if (ext_matrix.size() == 16 && ext_matrix[0] != 0.0) {
        T_body_to_baselink_ << ext_matrix[0], ext_matrix[1], ext_matrix[2], ext_matrix[3],
                               ext_matrix[4], ext_matrix[5], ext_matrix[6], ext_matrix[7],
                               ext_matrix[8], ext_matrix[9], ext_matrix[10], ext_matrix[11],
                               ext_matrix[12], ext_matrix[13], ext_matrix[14], ext_matrix[15];
    } else {
        T_body_to_baselink_ = Eigen::Matrix4f::Identity();
    }
    RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully.");
    RCLCPP_INFO(this->get_logger(), "  save_directory: %s", saveDirectory_.c_str());
    RCLCPP_INFO(this->get_logger(), "  keyframe_meter_gap: %.2f", keyframeMeterGap_);
    RCLCPP_INFO(this->get_logger(), "  loopClosureFrequency: %.2f", loopClosureFrequency_);
}

void LidarLoopNode::initFileSave() {
    pgKITTIformat_ = saveDirectory_ + "optimized_poses.txt";
    odomKITTIformat_ = saveDirectory_ + "odom_poses.txt";
    pgScansDirectory_ = saveDirectory_ + "Scans/";
    pgImageDirectory_ = saveDirectory_ + "Image/";

    std::string cmd1 = "mkdir -p " + saveDirectory_;
    std::string cmd2 = "rm -rf " + pgScansDirectory_ + " " + pgImageDirectory_;
    std::string cmd_clean_old = "rm -f " + saveDirectory_ + "*.pcd " + saveDirectory_ + "*.pgm " +
                                saveDirectory_ + "*.yaml " + saveDirectory_ + "*.txt " + saveDirectory_ + "*.bin";
    std::string cmd3 = "mkdir -p " + pgScansDirectory_ + " " + pgImageDirectory_;

    auto ret1 = system(cmd1.c_str());
    auto ret2 = system(cmd2.c_str());
    auto ret_clean = system(cmd_clean_old.c_str());
    auto ret3 = system(cmd3.c_str());
    (void)ret1;
    (void)ret2;
    (void)ret_clean;
    (void)ret3;

    pgTimeSaveStream_.open(saveDirectory_ + "times.txt", std::fstream::out);
    pgTimeSaveStream_.precision(std::numeric_limits<double>::max_digits10);
}

void LidarLoopNode::initPublishers() {
    pubOdomAftPGO_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
    pubPathAftPGO_ = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);

    // 自测三轨迹对比发布器
    pubPathOdom_ = this->create_publisher<nav_msgs::msg::Path>("/path_odom", 100);
    pubPathPGOArUco_ = this->create_publisher<nav_msgs::msg::Path>("/path_pgo_aruco", 100);

    pubLoopScanLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);
    pubLoopScanLocalRegisted_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local_registed", 100);

    pubLoopConstraintEdge_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);
    pubKeyFramesId_ = this->create_publisher<std_msgs::msg::Header>("/key_frames_ids", 10);

    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void LidarLoopNode::initSubscribers() {
    subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 100, std::bind(&LidarLoopNode::laserCloudFullResHandler, this, std::placeholders::_1));

    subLaserOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", 100, std::bind(&LidarLoopNode::laserOdometryHandler, this, std::placeholders::_1));

    subGPS_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 100, std::bind(&LidarLoopNode::gpsHandler, this, std::placeholders::_1));

    subVisualLoop_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/visual_loop_candidate", 100, std::bind(&LidarLoopNode::visualLoopHandler, this, std::placeholders::_1));

    // ArUco landmark 观测订阅 (latched QoS 匹配 visual_loop_node 侧的发布者)
    subArucoLandmarkObs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aruco_landmark_obs", rclcpp::QoS(10).transient_local(),
        std::bind(&LidarLoopNode::arucoLandmarkObsHandler, this, std::placeholders::_1));

    subVisualPGO_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/visual_pgo_constraint", 100,
        std::bind(&LidarLoopNode::visualPGOConstraintHandler, this, std::placeholders::_1));
}

void LidarLoopNode::laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg) {
    mBuf_.lock();
    odometryBuf_.push(msg);

    // 自测 1: 缓存原始前端 Odom 轨迹
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header = msg->header;
    pose_stamp.pose = msg->pose.pose;
    pathOdomMsg_.header = msg->header;
    pathOdomMsg_.header.frame_id = "camera_init";
    pathOdomMsg_.poses.push_back(pose_stamp);
    pubPathOdom_->publish(pathOdomMsg_);

    mBuf_.unlock();
}

void LidarLoopNode::laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mBuf_.lock();
    fullResBuf_.push(msg);
    mBuf_.unlock();
}

void LidarLoopNode::gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (useGPS_) {
        mBuf_.lock();
        gpsBuf_.push(msg);
        mBuf_.unlock();
    }
}

void LidarLoopNode::visualLoopHandler(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    int prev_node_idx = msg->data[0];
    int curr_node_idx = msg->data[1];
    loopDetector_->addVisualLoopCandidate(prev_node_idx, curr_node_idx);
}

void LidarLoopNode::visualPGOConstraintHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 8) return;
    std::lock_guard<std::mutex> lock(mVisualPGO_);
    visualPGOQueue_.push(*msg);
}

std::string LidarLoopNode::padZeros(int val, int num_digits) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

gtsam::Pose3 LidarLoopNode::Pose6DtoGTSAMPose3(const Pose6D& p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

Pose6D LidarLoopNode::getOdom(const nav_msgs::msg::Odometry::SharedPtr odom) {
    auto tx = odom->pose.pose.position.x;
    auto ty = odom->pose.pose.position.y;
    auto tz = odom->pose.pose.position.z;

    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                       odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Pose6D pose;
    pose.x = tx;
    pose.y = ty;
    pose.z = tz;
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
    pose.seq = 0;

    return pose;
}

Pose6D LidarLoopNode::diffTransformation(const Pose6D& p1, const Pose6D& p2) {
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(p1.x, p1.y, p1.z, p1.roll, p1.pitch, p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(p2.x, p2.y, p2.z, p2.roll, p2.pitch, p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    Pose6D dtf;
    dtf.x = double(std::abs(dx));
    dtf.y = double(std::abs(dy));
    dtf.z = double(std::abs(dz));
    dtf.roll = double(std::abs(droll));
    dtf.pitch = double(std::abs(dpitch));
    dtf.yaw = double(std::abs(dyaw));
    dtf.seq = 0;

    return dtf;
}

Eigen::Affine3f LidarLoopNode::Pose6dToAffine3f(const Pose6D& pose) {
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

void LidarLoopNode::saveOdometryVerticesKITTIformat(const std::string& filename) {
    std::fstream stream(filename.c_str(), std::fstream::out);
    auto& keyframePoses = loopDetector_->getKeyframePoses();
    loopDetector_->getKFMutex().lock();
    for (const auto& pose6d : keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
               << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
               << col3.z() << " " << t.z() << std::endl;
    }
    loopDetector_->getKFMutex().unlock();
}

void LidarLoopNode::saveOptimizedVerticesKITTIformat(const gtsam::Values& estimates, const std::string& filename) {
    std::fstream stream(filename.c_str(), std::fstream::out);
    for (const auto& key_value : estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
               << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
               << col3.z() << " " << t.z() << std::endl;
    }
}

// 接收 visual_loop_node 发来的 ArUco landmark 观测数据
void LidarLoopNode::arucoLandmarkObsHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 格式: [n_markers, (marker_id, kf_id, marker_size, T_lidar_marker[16]) x n]
    if (msg->data.size() < 1) return;
    int n = static_cast<int>(msg->data[0]);
    size_t idx = 1;
    std::lock_guard<std::mutex> lock(arucoLandmarkMutex_);
    for (int i = 0; i < n; ++i) {
        if (idx + 19 > msg->data.size()) break;
        int marker_id = static_cast<int>(msg->data[idx++]);
        ArucoLandmarkObs obs;
        obs.kf_id       = static_cast<int>(msg->data[idx++]);
        obs.marker_size  = msg->data[idx++];
        for (int j = 0; j < 16; ++j) obs.T_lidar_marker[j] = msg->data[idx++];
        arucoLandmarkObs_[marker_id] = obs;
    }
    RCLCPP_DEBUG(this->get_logger(), "ArUco: Received %d landmark observation(s).", n);
}

// 在 GTSAM 优化完成后，将 marker 的优化后世界坐标系位姿写出到 aruco_landmarks.yaml
void LidarLoopNode::saveArucoLandmarks() {
    std::lock_guard<std::mutex> lock(arucoLandmarkMutex_);
    if (arucoLandmarkObs_.empty()) return;

    auto& kfPoses = loopDetector_->getKeyframePosesUpdated();
    std::string filepath = saveDirectory_ + "aruco_landmarks.yaml";
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        RCLCPP_WARN(this->get_logger(), "ArUco: Cannot open %s for writing.", filepath.c_str());
        return;
    }

    ofs << "landmarks:" << std::endl;
    for (auto& [id, obs] : arucoLandmarkObs_) {
        int kf_id = obs.kf_id;
        if (kf_id < 0 || kf_id >= static_cast<int>(kfPoses.size())) {
            RCLCPP_WARN(this->get_logger(),
                "ArUco: KF %d for marker %d out of range (%zu KFs), skipping.",
                kf_id, id, kfPoses.size());
            continue;
        }

        // 读取当帧的优化后位姿 T_world_lidar (4x4 Eigen)
        const Pose6D& p = kfPoses[kf_id];
        Eigen::Matrix4d T_world_lidar = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd roll_angle( p.roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(p.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(  p.yaw,   Eigen::Vector3d::UnitZ());
        T_world_lidar.block<3,3>(0,0) = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
        T_world_lidar(0,3) = p.x;
        T_world_lidar(1,3) = p.y;
        T_world_lidar(2,3) = p.z;

        // 读取 T_lidar_marker (4x4)
        Eigen::Matrix4d T_lidar_marker;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                T_lidar_marker(r,c) = obs.T_lidar_marker[r*4+c];

        // T_world_marker = T_world_lidar * T_lidar_marker
        Eigen::Matrix4d T_world_marker = T_world_lidar * T_lidar_marker;

        ofs << "  - id: " << id << std::endl;
        ofs << "    marker_size: " << std::fixed << std::setprecision(3) << obs.marker_size << std::endl;
        ofs << "    pose:  # T_world_marker Row-Major 4x4" << std::endl;
        ofs << "      [";
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                if (r == 0 && c == 0) ofs << std::fixed << std::setprecision(6) << T_world_marker(r,c);
                else ofs << ", " << std::fixed << std::setprecision(6) << T_world_marker(r,c);
            }
        ofs << "]" << std::endl;
        RCLCPP_DEBUG(this->get_logger(),
            "ArUco: Saved landmark ID=%d (KF=%d) to aruco_landmarks.yaml", id, kf_id);
    }
    ofs.close();
    RCLCPP_DEBUG(this->get_logger(), "ArUco: aruco_landmarks.yaml saved to %s", filepath.c_str());
}

void LidarLoopNode::pubPath() {
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";

    auto& keyframePosesUpdated = loopDetector_->getKeyframePosesUpdated();
    auto& keyframeTimes = loopDetector_->getKeyframeTimes();

    loopDetector_->getKFMutex().lock();
    int num_poses = keyframePosesUpdated.size();
    for (int node_idx = 0; node_idx < recentIdxUpdated_ && node_idx < num_poses; node_idx++) {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.at(node_idx) * 1e9));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;

        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation.x = q.x();
        odomAftPGOthis.pose.pose.orientation.y = q.y();
        odomAftPGOthis.pose.pose.orientation.z = q.z();
        odomAftPGOthis.pose.pose.orientation.w = q.w();

        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    loopDetector_->getKFMutex().unlock();

    pubOdomAftPGO_->publish(odomAftPGO);
    pubPathAftPGO_->publish(pathAftPGO);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = odomAftPGO.header.stamp;
    transformStamped.header.frame_id = "camera_init";
    transformStamped.child_frame_id = "aft_pgo";
    transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;
    tfBroadcaster_->sendTransform(transformStamped);
}

void LidarLoopNode::visualizeLoopClosure() {
    auto& loopIndexContainer = loopDetector_->getLoopIndexContainer();
    auto& keyframePosesUpdated = loopDetector_->getKeyframePosesUpdated();
    auto& keyframeTimes = loopDetector_->getKeyframeTimes();

    loopDetector_->getBufMutex().lock();
    if (loopIndexContainer.empty()) {
        loopDetector_->getBufMutex().unlock();
        return;
    }

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = "camera_init";
    
    loopDetector_->getKFMutex().lock();
    markerNode.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.back() * 1e9));
    loopDetector_->getKFMutex().unlock();

    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = LOOP_NODE_MARKER_SCALE_;
    markerNode.scale.y = LOOP_NODE_MARKER_SCALE_;
    markerNode.scale.z = LOOP_NODE_MARKER_SCALE_;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;

    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    
    loopDetector_->getKFMutex().lock();
    markerEdge.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.back() * 1e9));
    loopDetector_->getKFMutex().unlock();

    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = LOOP_EDGE_MARKER_SCALE_;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    loopDetector_->getKFMutex().lock();
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = keyframePosesUpdated[key_cur].x;
        p.y = keyframePosesUpdated[key_cur].y;
        p.z = keyframePosesUpdated[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframePosesUpdated[key_pre].x;
        p.y = keyframePosesUpdated[key_pre].y;
        p.z = keyframePosesUpdated[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }
    loopDetector_->getKFMutex().unlock();
    loopDetector_->getBufMutex().unlock();

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge_->publish(markerArray);
}

void LidarLoopNode::updatePoses(const gtsam::Values& isamCurrentEstimate) {
    auto& keyframePosesUpdated = loopDetector_->getKeyframePosesUpdated();
    loopDetector_->getKFMutex().lock();
    int num_poses = keyframePosesUpdated.size();
    for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size()) && node_idx < num_poses; node_idx++) {
        Pose6D& p = keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    loopDetector_->getKFMutex().unlock();

    mtxRecentPose_.lock();
    if (!isamCurrentEstimate.empty()) {
        const gtsam::Pose3& lastOptimizedPose =
            isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size()) - 1);
        recentOptimizedX_ = lastOptimizedPose.translation().x();
        recentOptimizedY_ = lastOptimizedPose.translation().y();
    }
    recentIdxUpdated_ = int(keyframePosesUpdated.size()) - 1;
    mtxRecentPose_.unlock();
}

void LidarLoopNode::process_pg() {
    while (rclcpp::ok() && !stopThreads_) {
        while (!odometryBuf_.empty() && !fullResBuf_.empty()) {
            mBuf_.lock();
            while (!odometryBuf_.empty() && rclcpp::Time(odometryBuf_.front()->header.stamp).seconds() <
                                                 rclcpp::Time(fullResBuf_.front()->header.stamp).seconds())
                odometryBuf_.pop();

            if (odometryBuf_.empty()) {
                mBuf_.unlock();
                break;
            }

            timeLaserOdometry_ = rclcpp::Time(odometryBuf_.front()->header.stamp).seconds();
            timeLaser_ = rclcpp::Time(fullResBuf_.front()->header.stamp).seconds();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*fullResBuf_.front(), *tempCloudRGB);

            Pose6D pose_curr = getOdom(odometryBuf_.front());
            odometryBuf_.pop();

            Eigen::Affine3f t_body_to_world = Pose6dToAffine3f(pose_curr);
            Eigen::Affine3f t_world_to_body = t_body_to_world.inverse();
            pcl::transformPointCloud(*tempCloudRGB, *tempCloudRGB, t_world_to_body);

            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            RGB2XYZI(tempCloudRGB, thisKeyFrame);

            fullResBuf_.pop();

            double eps = 0.1;
            while (!gpsBuf_.empty()) {
                auto thisGPS = gpsBuf_.front();
                auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                if (std::abs(thisGPSTime - timeLaserOdometry_) < eps) {
                    currGPS_ = thisGPS;
                    hasGPSforThisKF_ = true;
                    break;
                } else {
                    hasGPSforThisKF_ = false;
                }
                gpsBuf_.pop();
            }
            mBuf_.unlock();

            odom_pose_prev_ = odom_pose_curr_;
            odom_pose_curr_ = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev_, odom_pose_curr_);

            double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
            translationAccumulated_ += delta_translation;
            rotaionAccumulated_ += (dtf.roll + dtf.pitch + dtf.yaw);

            if (translationAccumulated_ > keyframeMeterGap_ || rotaionAccumulated_ > keyframeRadGap_) {
                isNowKeyFrame_ = true;
                translationAccumulated_ = 0.0;
                rotaionAccumulated_ = 0.0;
            } else {
                isNowKeyFrame_ = false;
            }

            if (!isNowKeyFrame_) continue;

            if (!gpsOffsetInitialized_) {
                if (hasGPSforThisKF_) {
                    gpsAltitudeInitOffset_ = currGPS_->altitude;
                    gpsOffsetInitialized_ = true;
                }
            }

            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext_.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext_.filter(*thisKeyFrameDS);

            pcl::PointCloud<PointTypeRGB>::Ptr thisKeyFrameMapDSRGB(new pcl::PointCloud<PointTypeRGB>());
            pcl::VoxelGrid<PointTypeRGB> filterForMapRGB;
            filterForMapRGB.setLeafSize(mapVizFilterSize_, mapVizFilterSize_, mapVizFilterSize_);
            filterForMapRGB.setInputCloud(tempCloudRGB);
            filterForMapRGB.filter(*thisKeyFrameMapDSRGB);

            // 录入关键帧
            loopDetector_->addKeyframe(thisKeyFrameMapDSRGB, pose_curr, timeLaserOdometry_, *thisKeyFrameDS);

            std_msgs::msg::Header keyFrameHeader;
            keyFrameHeader.stamp = this->now();
            pubKeyFramesId_->publish(keyFrameHeader);

            auto& keyframePoses = loopDetector_->getKeyframePoses();
            const int prev_node_idx = keyframePoses.size() - 2;
            const int curr_node_idx = keyframePoses.size() - 1;

            if (!factorGraphManager_->isGraphMade()) {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                factorGraphManager_->addPriorFactor(init_node_idx, poseOrigin);
                nodesAddedToGraph_.store(init_node_idx + 1);
                RCLCPP_DEBUG(this->get_logger(), "Prior factor added to pose graph at node %d.",
                            init_node_idx);
            } else {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                factorGraphManager_->addOdomFactor(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), poseTo);

                if (hasGPSforThisKF_) {
                    double curr_altitude_offseted = currGPS_->altitude - gpsAltitudeInitOffset_;
                    mtxRecentPose_.lock();
                    gtsam::Point3 gpsConstraint(recentOptimizedX_, recentOptimizedY_, curr_altitude_offseted);
                    mtxRecentPose_.unlock();
                    factorGraphManager_->addGPSFactor(curr_node_idx, gpsConstraint);
                    RCLCPP_DEBUG(this->get_logger(),
                                "GPS prior factor added to pose graph at node %d.",
                                curr_node_idx);
                }

                if (curr_node_idx % 100 == 0) {
                    RCLCPP_DEBUG(this->get_logger(),
                                "Keyframe odometry factors successfully inserted (Last node: %d).",
                                curr_node_idx);
                }
                nodesAddedToGraph_.store(curr_node_idx + 1);
            }

            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory_ + curr_node_idx_str + ".pcd", *thisKeyFrame);
            pgTimeSaveStream_ << timeLaser_ << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void LidarLoopNode::process_lcd() {
    auto rate_duration = std::chrono::duration<double>(1.0 / loopClosureFrequency_);
    while (rclcpp::ok() && !stopThreads_) {
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
        loopDetector_->performSCLoopClosure();
        visualizeLoopClosure();
    }
}

void LidarLoopNode::process_icp() {
    while (rclcpp::ok() && !stopThreads_) {
        while (true) {
            std::pair<int, int> loop_idx_pair;
            bool has_data = false;
            
            loopDetector_->getBufMutex().lock();
            auto& queue = loopDetector_->getScLoopICPBuf();
            if (!queue.empty()) {
                if (queue.size() > MAX_ICP_QUEUE_SIZE_) {
                    RCLCPP_WARN(this->get_logger(),
                                "Heavy backlog in loop closure ICP queue (Queue size: %lu). Consider tuning loop closure parameters.",
                                queue.size());
                }
                loop_idx_pair = queue.front();
                
                // If visual loop candidate arrived before LiDAR keyframe is fully inserted into GTSAM, wait.
                int nodes_in_graph = nodesAddedToGraph_.load();
                if (loop_idx_pair.first >= nodes_in_graph || loop_idx_pair.second >= nodes_in_graph) {
                    loopDetector_->getBufMutex().unlock();
                    break;
                }
                
                queue.pop();
                has_data = true;
            }
            loopDetector_->getBufMutex().unlock();

            if (!has_data) break;

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose = loopDetector_->doICPVirtualRelative(
                prev_node_idx, curr_node_idx,
                pubLoopScanLocal_, pubLoopSubmapLocal_, pubLoopScanLocalRegisted_);

            if (!relative_pose.equals(gtsam::Pose3::identity())) {
                factorGraphManager_->addLoopFactor(curr_node_idx, prev_node_idx, relative_pose);
            }
        }

        // 处理纯视觉兜底约束
        mVisualPGO_.lock();
        auto& keyframePoses = loopDetector_->getKeyframePoses();
        int nodes_in_graph_v = nodesAddedToGraph_.load();
        while (!visualPGOQueue_.empty()) {
            auto msg = visualPGOQueue_.front();
            int prev_idx = (int)msg.data[0];
            int curr_idx = (int)msg.data[1];
            if (curr_idx >= nodes_in_graph_v || prev_idx >= nodes_in_graph_v) {
                break; // Wait until node is inserted
            }
            visualPGOQueue_.pop();

            // 提取前端里程计中的两帧相对平移距离，作为单目尺度标尺
            Pose6D p_prev = keyframePoses[prev_idx];
            Pose6D p_curr = keyframePoses[curr_idx];
            double dx = p_curr.x - p_prev.x;
            double dy = p_curr.y - p_prev.y;
            double dz = p_curr.z - p_prev.z;
            double scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (scale < 0.5) {
                RCLCPP_DEBUG(this->get_logger(), "Visual PGO rejected: odometry translation too small (%.2fm)", scale);
                continue;
            }

            // 构造相对位姿
            double t_x = msg.data[2] * scale;
            double t_y = msg.data[3] * scale;
            double t_z = msg.data[4] * scale;
            double r_x = msg.data[5];
            double r_y = msg.data[6];
            double r_z = msg.data[7];

            gtsam::Pose3 visual_pose(gtsam::Rot3::Rodrigues(r_x, r_y, r_z), gtsam::Point3(t_x, t_y, t_z));
            
            factorGraphManager_->addVisualLoopFactor(curr_idx, prev_idx, visual_pose);
            RCLCPP_INFO(this->get_logger(), "Visual PGO constraint added between %d and %d. Scale: %.2fm", prev_idx, curr_idx, scale);
        }
        mVisualPGO_.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void LidarLoopNode::process_isam() {
    auto rate_duration = std::chrono::duration<double>(1.0 / graphUpdateFrequency_);
    while (rclcpp::ok() && !stopThreads_) {
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
        if (factorGraphManager_->isGraphMade()) {
            // RCLCPP_INFO(this->get_logger(),
            //             "ISAM2 optimization triggered. Updating global trajectory.");
            
            gtsam::Values currentEstimate;
            factorGraphManager_->runOptimization(graphUpdateTimes_, currentEstimate);
            
            updatePoses(currentEstimate);
            pubPath();

            saveOptimizedVerticesKITTIformat(currentEstimate, pgKITTIformat_);
            saveOdometryVerticesKITTIformat(odomKITTIformat_);
            saveArucoLandmarks(); // Step 4: 建图过程中持续更新 ArUco landmark 局部标系位姿
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLoopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
