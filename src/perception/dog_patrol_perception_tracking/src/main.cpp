#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <rclcpp/rclcpp.hpp>

#include "vision_demo_host/modules/bearing_estimator.hpp"
#include "vision_demo_host/modules/camera_ingest.hpp"
#include "vision_demo_host/modules/det_filter.hpp"
#include "vision_demo_host/modules/identity_manager.hpp"
#include "vision_demo_host/modules/mot_tracker.hpp"
#include "vision_demo_host/modules/preprocess_infer.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"
#include "vision_demo_host/modules/runtime_monitor.hpp"
#include "vision_demo_host/modules/udp_json_adapter.hpp"
#include "vision_demo_host/modules/visualizer_recorder.hpp"

namespace {

const char *BoolStr(const bool value) { return value ? "true" : "false"; }

}  // namespace

class VisionDemoNode : public rclcpp::Node {
 public:
  VisionDemoNode()
      : Node("vision_demo_host_node"),
        infer_(vision_demo_host::PreprocessInfer::Config{}),
        det_filter_(vision_demo_host::DetFilter::Config{}),
        tracker_(vision_demo_host::MotTracker::Config{}),
        primary_manager_(vision_demo_host::PrimaryTargetManager::Config{}),
        identity_manager_(vision_demo_host::IdentityManager::Config{}),
        bearing_(vision_demo_host::BearingEstimator::Config{}),
        udp_("127.0.0.1", 5005),
        visualizer_(vision_demo_host::VisualizerRecorder::Config{}) {
    DeclareParameters();
    LoadConfigAndInitialize();

    const int timer_ms = this->get_parameter("runtime.tick_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms),
                                     std::bind(&VisionDemoNode::Tick, this));
    RCLCPP_INFO(get_logger(), "vision_demo_host_node started.");
  }

 private:
  void DeclareParameters() {
    this->declare_parameter<std::string>("camera.mvs_model", "MV-CU013-A0UC");
    this->declare_parameter<std::string>("camera.mvs_serial", "");
    this->declare_parameter<int>("camera.width", 1280);
    this->declare_parameter<int>("camera.height", 1024);
    this->declare_parameter<double>("camera.fps", 30.0);
    this->declare_parameter<int>("camera.timeout_ms", 1000);
    this->declare_parameter<std::string>(
        "camera.bayer_interpolation",
        vision_demo_host::CameraIngest::BayerInterpolationName(
            vision_demo_host::CameraIngest::kDefaultBayerInterpolation));
    this->declare_parameter<bool>("camera.bayer_smoothing",
                                  vision_demo_host::CameraIngest::kDefaultBayerSmoothing);

    this->declare_parameter<std::string>("detector.runtime_path", "");
    this->declare_parameter<double>("detector.raw_conf_threshold", 0.10);
    this->declare_parameter<int>("detector.input_width", 640);
    this->declare_parameter<int>("detector.input_height", 640);
    this->declare_parameter<double>("detector.person_conf_threshold", 0.10);
    this->declare_parameter<double>("detector.car_conf_threshold", 0.10);
    this->declare_parameter<bool>("detector.enable_fake_detection", false);

    this->declare_parameter<std::string>("tracker.config_path", "");
    this->declare_parameter<bool>("tracker.gmc_enabled", true);
    this->declare_parameter<bool>("tracker.reid_enabled", true);
    this->declare_parameter<double>("tracker.track_high_thresh", 0.5);
    this->declare_parameter<double>("tracker.track_low_thresh", 0.1);
    this->declare_parameter<double>("tracker.new_track_thresh", 0.7);
    this->declare_parameter<double>("tracker.match_thresh", 0.8);
    this->declare_parameter<int>("tracker.track_buffer", 30);
    this->declare_parameter<std::string>("tracker.gmc_method", "sparseOptFlow");
    this->declare_parameter<int>("tracker.gmc_downscale", 4);
    this->declare_parameter<bool>("tracker.with_reid", true);
    this->declare_parameter<std::string>("tracker.reid_backend", "light");
    this->declare_parameter<std::string>("tracker.reid_model_path", "");
    this->declare_parameter<int>("tracker.reid_input_width", 128);
    this->declare_parameter<int>("tracker.reid_input_height", 256);
    this->declare_parameter<int>("target.lost_threshold_frames", 180);
    this->declare_parameter<double>("target.min_person_area_px", 1000.0);
    this->declare_parameter<double>("target.max_center_jump_norm", 2.0);
    this->declare_parameter<double>("target.min_area_ratio", 0.25);
    this->declare_parameter<double>("target.max_area_ratio", 4.0);
    this->declare_parameter<int>("target.pending_recovery_frames", 3);

    this->declare_parameter<double>("bearing.camera_horizontal_fov_rad", 1.5708);
    this->declare_parameter<double>("bearing.camera_mount_x", 0.22646);
    this->declare_parameter<double>("bearing.camera_mount_y", 0.03);
    this->declare_parameter<double>("bearing.camera_mount_z", 0.20738);
    this->declare_parameter<double>("bearing.camera_mount_roll", 0.0);
    this->declare_parameter<double>("bearing.camera_mount_pitch", -1.57079);
    this->declare_parameter<double>("bearing.camera_mount_yaw", -3.14159);

    this->declare_parameter<int>("sid.feat_bank_size", 30);
    this->declare_parameter<double>("sid.recover_sim_thresh_strict", 0.85);
    this->declare_parameter<double>("sid.recover_sim_thresh_relaxed", 0.75);
    this->declare_parameter<int>("sid.recover_relaxed_max_missing_frames", 180);
    this->declare_parameter<int>("sid.occlusion_protect_frames", 30);
    this->declare_parameter<double>("sid.missing_assign_min_area_ratio", 0.40);
    this->declare_parameter<double>("sid.missing_assign_max_area_ratio", 4.00);
    this->declare_parameter<double>("sid.missing_assign_max_center_dist_norm", 2.50);
    this->declare_parameter<double>("sid.missing_assign_max_app_cost", 0.50);
    this->declare_parameter<double>("sid.overlap_iou_freeze", 0.10);
    this->declare_parameter<int>("sid.split_stable_frames", 3);
    this->declare_parameter<int>("sid.merge_hold_frames", 2);
    this->declare_parameter<double>("sid.app_w", 0.70);
    this->declare_parameter<double>("sid.geo_w", 0.20);
    this->declare_parameter<double>("sid.time_w", 0.10);
    this->declare_parameter<double>("sid.active_assign_max_cost", 0.55);
    this->declare_parameter<double>("sid.recovery_max_cost", 0.45);
    this->declare_parameter<double>("sid.raw_continuity_max_cost", 0.55);
    this->declare_parameter<double>("sid.min_assignment_margin", 0.08);
    this->declare_parameter<int>("sid.stable_frames_before_feature_update", 3);
    this->declare_parameter<bool>("sid.merged_requires_overlap", true);
    this->declare_parameter<bool>("sid.reid_enable", true);
    this->declare_parameter<std::string>("sid.reid_backend", "light");
    this->declare_parameter<std::string>("sid.reid_model_path", "");
    this->declare_parameter<int>("sid.reid_input_width", 128);
    this->declare_parameter<int>("sid.reid_input_height", 256);

    this->declare_parameter<std::string>("udp.ip", "127.0.0.1");
    this->declare_parameter<int>("udp.port", 5005);

    this->declare_parameter<bool>("visualization.enable", false);
    this->declare_parameter<bool>("recording.enable", false);
    this->declare_parameter<std::string>("recording.path", "/tmp/vision_demo_out.mp4");
    this->declare_parameter<double>("recording.fps", 15.0);

    this->declare_parameter<int>("runtime.tick_ms", 33);
  }

  void LoadConfigAndInitialize() {
    std::string error;

    vision_demo_host::CameraIngest::Config camera_cfg;
    camera_cfg.hik_mvs_model = this->get_parameter("camera.mvs_model").as_string();
    camera_cfg.hik_mvs_serial = this->get_parameter("camera.mvs_serial").as_string();
    camera_cfg.width = this->get_parameter("camera.width").as_int();
    camera_cfg.height = this->get_parameter("camera.height").as_int();
    camera_cfg.fps = this->get_parameter("camera.fps").as_double();
    camera_cfg.timeout_ms = this->get_parameter("camera.timeout_ms").as_int();
    if (!vision_demo_host::CameraIngest::ParseBayerInterpolation(
            this->get_parameter("camera.bayer_interpolation").as_string(),
            &camera_cfg.bayer_interpolation, &error)) {
      throw std::runtime_error(error);
    }
    camera_cfg.bayer_smoothing =
        this->get_parameter("camera.bayer_smoothing").as_bool();

    if (!camera_.Open(camera_cfg, &error)) {
      throw std::runtime_error("camera_ingest init failed: " + error);
    }

    vision_demo_host::PreprocessInfer::Config infer_cfg;
    infer_cfg.detector_runtime_path = this->get_parameter("detector.runtime_path").as_string();
    infer_cfg.raw_conf_threshold =
        static_cast<float>(this->get_parameter("detector.raw_conf_threshold").as_double());
    infer_cfg.input_width = this->get_parameter("detector.input_width").as_int();
    infer_cfg.input_height = this->get_parameter("detector.input_height").as_int();
    infer_cfg.enable_fake_detection = this->get_parameter("detector.enable_fake_detection").as_bool();
    infer_ = vision_demo_host::PreprocessInfer(infer_cfg);
    if (!infer_.Initialize(&error)) {
      throw std::runtime_error("preprocess_infer init failed: " + error);
    }

    vision_demo_host::DetFilter::Config filter_cfg;
    filter_cfg.person_conf_threshold =
        static_cast<float>(this->get_parameter("detector.person_conf_threshold").as_double());
    filter_cfg.car_conf_threshold =
        static_cast<float>(this->get_parameter("detector.car_conf_threshold").as_double());
    det_filter_ = vision_demo_host::DetFilter(filter_cfg);

    vision_demo_host::MotTracker::Config tracker_cfg;
    tracker_cfg.tracker_yaml_path = this->get_parameter("tracker.config_path").as_string();
    tracker_cfg.gmc_enabled = this->get_parameter("tracker.gmc_enabled").as_bool();
    const bool tracker_reid_enabled_param = this->get_parameter("tracker.reid_enabled").as_bool();
    tracker_cfg.reid_enabled = tracker_reid_enabled_param;
    tracker_cfg.track_high_thresh =
        static_cast<float>(this->get_parameter("tracker.track_high_thresh").as_double());
    tracker_cfg.track_low_thresh =
        static_cast<float>(this->get_parameter("tracker.track_low_thresh").as_double());
    tracker_cfg.new_track_thresh =
        static_cast<float>(this->get_parameter("tracker.new_track_thresh").as_double());
    tracker_cfg.match_thresh = static_cast<float>(this->get_parameter("tracker.match_thresh").as_double());
    tracker_cfg.track_buffer = this->get_parameter("tracker.track_buffer").as_int();
    tracker_cfg.gmc_method = this->get_parameter("tracker.gmc_method").as_string();
    tracker_cfg.gmc_downscale =
        std::max(1, static_cast<int>(this->get_parameter("tracker.gmc_downscale").as_int()));
    const bool tracker_with_reid_param = this->get_parameter("tracker.with_reid").as_bool();
    tracker_cfg.with_reid = tracker_with_reid_param;
    tracker_cfg.reid_backend = this->get_parameter("tracker.reid_backend").as_string();
    tracker_cfg.reid_model_path = this->get_parameter("tracker.reid_model_path").as_string();
    tracker_cfg.reid_input_width =
        std::max(16, static_cast<int>(this->get_parameter("tracker.reid_input_width").as_int()));
    tracker_cfg.reid_input_height =
        std::max(16, static_cast<int>(this->get_parameter("tracker.reid_input_height").as_int()));
    if (!tracker_reid_enabled_param || !tracker_with_reid_param) {
      RCLCPP_WARN(get_logger(), "reid is mandatory; override tracker.reid_enabled/with_reid to true");
    }
    if (infer_cfg.raw_conf_threshold > tracker_cfg.track_low_thresh) {
      RCLCPP_WARN(get_logger(),
                  "detector.raw_conf_threshold %.3f is above tracker.track_low_thresh %.3f; low-score recovery is pre-filtered before tracker",
                  infer_cfg.raw_conf_threshold, tracker_cfg.track_low_thresh);
    }
    if (filter_cfg.person_conf_threshold > tracker_cfg.track_low_thresh ||
        filter_cfg.car_conf_threshold > tracker_cfg.track_low_thresh) {
      RCLCPP_WARN(get_logger(),
                  "detector class thresholds person=%.3f car=%.3f exceed tracker.track_low_thresh %.3f; low-score recovery may be pre-filtered",
                  filter_cfg.person_conf_threshold, filter_cfg.car_conf_threshold, tracker_cfg.track_low_thresh);
    }
    tracker_cfg.reid_enabled = true;
    tracker_cfg.with_reid = true;
    tracker_ = vision_demo_host::MotTracker(tracker_cfg);
    if (!tracker_.Initialize(&error)) {
      throw std::runtime_error("mot_tracker init failed: " + error);
    }

    vision_demo_host::PrimaryTargetManager::Config target_cfg;
    target_cfg.lost_threshold_frames = this->get_parameter("target.lost_threshold_frames").as_int();
    target_cfg.min_person_area_px =
        static_cast<float>(this->get_parameter("target.min_person_area_px").as_double());
    target_cfg.max_center_jump_norm =
        std::max(0.0F, static_cast<float>(this->get_parameter("target.max_center_jump_norm").as_double()));
    target_cfg.min_area_ratio =
        std::max(0.0F, static_cast<float>(this->get_parameter("target.min_area_ratio").as_double()));
    target_cfg.max_area_ratio =
        std::max(target_cfg.min_area_ratio, static_cast<float>(this->get_parameter("target.max_area_ratio").as_double()));
    target_cfg.pending_recovery_frames =
        std::max(0, static_cast<int>(this->get_parameter("target.pending_recovery_frames").as_int()));
    primary_manager_ = vision_demo_host::PrimaryTargetManager(target_cfg);

    vision_demo_host::BearingEstimator::Config bearing_cfg;
    bearing_cfg.camera_horizontal_fov_rad =
        static_cast<float>(this->get_parameter("bearing.camera_horizontal_fov_rad").as_double());
    bearing_cfg.camera_mount_x =
        static_cast<float>(this->get_parameter("bearing.camera_mount_x").as_double());
    bearing_cfg.camera_mount_y =
        static_cast<float>(this->get_parameter("bearing.camera_mount_y").as_double());
    bearing_cfg.camera_mount_z =
        static_cast<float>(this->get_parameter("bearing.camera_mount_z").as_double());
    bearing_cfg.camera_mount_roll =
        static_cast<float>(this->get_parameter("bearing.camera_mount_roll").as_double());
    bearing_cfg.camera_mount_pitch =
        static_cast<float>(this->get_parameter("bearing.camera_mount_pitch").as_double());
    bearing_cfg.camera_mount_yaw =
        static_cast<float>(this->get_parameter("bearing.camera_mount_yaw").as_double());
    bearing_ = vision_demo_host::BearingEstimator(bearing_cfg);
    if (bearing_.UsingConfiguredMountRotation()) {
      RCLCPP_INFO(get_logger(), "%s", bearing_.ModeMessage().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", bearing_.ModeMessage().c_str());
    }

    vision_demo_host::IdentityManager::Config sid_cfg;
    sid_cfg.max_missing_frames = target_cfg.lost_threshold_frames;
    sid_cfg.feat_bank_size =
        std::max(1, static_cast<int>(this->get_parameter("sid.feat_bank_size").as_int()));
    sid_cfg.recover_sim_thresh_strict = std::clamp(
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_strict").as_double()), 0.0F, 1.0F);
    sid_cfg.recover_sim_thresh_relaxed = std::clamp(
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_relaxed").as_double()), 0.0F, 1.0F);
    sid_cfg.recover_relaxed_max_missing_frames =
        std::max(1, static_cast<int>(this->get_parameter("sid.recover_relaxed_max_missing_frames").as_int()));
    sid_cfg.occlusion_protect_frames =
        std::max(0, static_cast<int>(this->get_parameter("sid.occlusion_protect_frames").as_int()));
    sid_cfg.missing_assign_min_area_ratio = std::max(
        0.01F, static_cast<float>(this->get_parameter("sid.missing_assign_min_area_ratio").as_double()));
    sid_cfg.missing_assign_max_area_ratio = std::max(
        sid_cfg.missing_assign_min_area_ratio,
        static_cast<float>(this->get_parameter("sid.missing_assign_max_area_ratio").as_double()));
    sid_cfg.missing_assign_max_center_dist_norm = std::max(
        0.1F, static_cast<float>(this->get_parameter("sid.missing_assign_max_center_dist_norm").as_double()));
    sid_cfg.missing_assign_max_app_cost = std::clamp(
        static_cast<float>(this->get_parameter("sid.missing_assign_max_app_cost").as_double()), 0.0F, 1.0F);
    sid_cfg.overlap_iou_freeze =
        std::max(0.0F, static_cast<float>(this->get_parameter("sid.overlap_iou_freeze").as_double()));
    sid_cfg.split_stable_frames =
        std::max(1, static_cast<int>(this->get_parameter("sid.split_stable_frames").as_int()));
    sid_cfg.merge_hold_frames =
        std::max(1, static_cast<int>(this->get_parameter("sid.merge_hold_frames").as_int()));
    sid_cfg.app_w = std::max(0.0F, static_cast<float>(this->get_parameter("sid.app_w").as_double()));
    sid_cfg.geo_w = std::max(0.0F, static_cast<float>(this->get_parameter("sid.geo_w").as_double()));
    sid_cfg.time_w = std::max(0.0F, static_cast<float>(this->get_parameter("sid.time_w").as_double()));
    sid_cfg.active_assign_max_cost = std::clamp(
        static_cast<float>(this->get_parameter("sid.active_assign_max_cost").as_double()), 0.0F, 1.0F);
    sid_cfg.recovery_max_cost = std::clamp(
        static_cast<float>(this->get_parameter("sid.recovery_max_cost").as_double()), 0.0F, 1.0F);
    sid_cfg.raw_continuity_max_cost = std::clamp(
        static_cast<float>(this->get_parameter("sid.raw_continuity_max_cost").as_double()), 0.0F, 1.0F);
    sid_cfg.min_assignment_margin = std::max(
        0.0F, static_cast<float>(this->get_parameter("sid.min_assignment_margin").as_double()));
    sid_cfg.stable_frames_before_feature_update =
        std::max(1, static_cast<int>(this->get_parameter("sid.stable_frames_before_feature_update").as_int()));
    sid_cfg.merged_requires_overlap = this->get_parameter("sid.merged_requires_overlap").as_bool();
    const bool sid_reid_enable_param = this->get_parameter("sid.reid_enable").as_bool();
    if (!sid_reid_enable_param) {
      RCLCPP_WARN(get_logger(), "reid is mandatory; override sid.reid_enable to true");
    }
    sid_cfg.reid_enable = true;
    sid_cfg.reid_backend = this->get_parameter("sid.reid_backend").as_string();
    sid_cfg.reid_model_path = this->get_parameter("sid.reid_model_path").as_string();
    sid_cfg.reid_input_width =
        std::max(16, static_cast<int>(this->get_parameter("sid.reid_input_width").as_int()));
    sid_cfg.reid_input_height =
        std::max(16, static_cast<int>(this->get_parameter("sid.reid_input_height").as_int()));
    identity_manager_ = vision_demo_host::IdentityManager(sid_cfg);
    if (!identity_manager_.Initialize(&error)) {
      throw std::runtime_error("identity_manager init failed: " + error);
    }

    const std::string udp_ip = this->get_parameter("udp.ip").as_string();
    const int udp_port_raw = this->get_parameter("udp.port").as_int();
    if (udp_port_raw <= 0 || udp_port_raw > 65535) {
      throw std::runtime_error("udp.port invalid: " + std::to_string(udp_port_raw));
    }
    udp_ = vision_demo_host::UdpJsonAdapter(udp_ip, static_cast<uint16_t>(udp_port_raw));
    if (!udp_.Initialize(&error)) {
      throw std::runtime_error("udp_json_adapter init failed: " + error);
    }

    vision_demo_host::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      throw std::runtime_error("camera_ingest initial frame failed: " + error);
    }
    cv::Mat &frame = acquired_frame.bgr8;

    vision_demo_host::VisualizerRecorder::Config viz_cfg;
    viz_cfg.enable_visualization = this->get_parameter("visualization.enable").as_bool();
    viz_cfg.enable_recording = this->get_parameter("recording.enable").as_bool();
    viz_cfg.recording_path = this->get_parameter("recording.path").as_string();
    viz_cfg.recording_fps = this->get_parameter("recording.fps").as_double();
    viz_cfg.semantic_id_max_missing_frames = target_cfg.lost_threshold_frames;
    viz_cfg.sid_feat_bank_size = this->get_parameter("sid.feat_bank_size").as_int();
    viz_cfg.sid_recover_sim_thresh_strict =
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_strict").as_double());
    viz_cfg.sid_recover_sim_thresh_relaxed =
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_relaxed").as_double());
    viz_cfg.sid_recover_relaxed_max_missing_frames =
        this->get_parameter("sid.recover_relaxed_max_missing_frames").as_int();
    viz_cfg.sid_occlusion_protect_frames = this->get_parameter("sid.occlusion_protect_frames").as_int();
    viz_cfg.sid_overlap_iou_freeze =
        static_cast<float>(this->get_parameter("sid.overlap_iou_freeze").as_double());
    viz_cfg.sid_split_stable_frames = this->get_parameter("sid.split_stable_frames").as_int();
    viz_cfg.sid_merge_hold_frames = this->get_parameter("sid.merge_hold_frames").as_int();
    viz_cfg.sid_app_w = static_cast<float>(this->get_parameter("sid.app_w").as_double());
    viz_cfg.sid_geo_w = static_cast<float>(this->get_parameter("sid.geo_w").as_double());
    viz_cfg.sid_time_w = static_cast<float>(this->get_parameter("sid.time_w").as_double());
    viz_cfg.sid_active_assign_max_cost =
        static_cast<float>(this->get_parameter("sid.active_assign_max_cost").as_double());
    viz_cfg.sid_recovery_max_cost = static_cast<float>(this->get_parameter("sid.recovery_max_cost").as_double());
    viz_cfg.sid_raw_continuity_max_cost =
        static_cast<float>(this->get_parameter("sid.raw_continuity_max_cost").as_double());
    viz_cfg.sid_min_assignment_margin =
        static_cast<float>(this->get_parameter("sid.min_assignment_margin").as_double());
    viz_cfg.sid_stable_frames_before_feature_update =
        this->get_parameter("sid.stable_frames_before_feature_update").as_int();
    viz_cfg.sid_merged_requires_overlap = this->get_parameter("sid.merged_requires_overlap").as_bool();
    viz_cfg.sid_reid_enable = true;
    viz_cfg.sid_reid_backend = this->get_parameter("sid.reid_backend").as_string();
    viz_cfg.sid_reid_model_path = this->get_parameter("sid.reid_model_path").as_string();
    viz_cfg.sid_reid_input_width = this->get_parameter("sid.reid_input_width").as_int();
    viz_cfg.sid_reid_input_height = this->get_parameter("sid.reid_input_height").as_int();
    visualizer_ = vision_demo_host::VisualizerRecorder(viz_cfg);
    if (!visualizer_.Initialize(frame.size(), &error)) {
      throw std::runtime_error("visualizer_recorder init failed: " + error);
    }

    LogEffectiveConfig(camera_cfg, acquired_frame, infer_cfg, filter_cfg,
                       tracker_.EffectiveConfig(), target_cfg, bearing_cfg, sid_cfg, udp_ip,
                       udp_port_raw, viz_cfg);
  }

  void LogEffectiveConfig(const vision_demo_host::CameraIngest::Config &camera_cfg,
                          const vision_demo_host::CameraIngest::AcquiredFrame &acquired_frame,
                          const vision_demo_host::PreprocessInfer::Config &infer_cfg,
                          const vision_demo_host::DetFilter::Config &filter_cfg,
                          const vision_demo_host::MotTracker::Config &tracker_cfg,
                          const vision_demo_host::PrimaryTargetManager::Config &target_cfg,
                          const vision_demo_host::BearingEstimator::Config &bearing_cfg,
                          const vision_demo_host::IdentityManager::Config &identity_cfg,
                          const std::string &udp_ip,
                          const int udp_port,
                          const vision_demo_host::VisualizerRecorder::Config &viz_cfg) {
    RCLCPP_INFO(get_logger(),
                "startup_effective_config camera backend=hik_mvs requested_size=%dx%d requested_fps=%.2f timeout_ms=%d mvs_model=%s mvs_serial=%s bayer_interpolation=%s bayer_smoothing=%s conversion_target=BGR8",
                camera_cfg.width, camera_cfg.height, camera_cfg.fps, camera_cfg.timeout_ms,
                camera_cfg.hik_mvs_model.c_str(), camera_cfg.hik_mvs_serial.c_str(),
                vision_demo_host::CameraIngest::BayerInterpolationName(
                    camera_cfg.bayer_interpolation)
                    .c_str(),
                BoolStr(camera_cfg.bayer_smoothing));
    RCLCPP_INFO(
        get_logger(),
        "startup_camera_source pixel_type=%s pixel_type_value=0x%08x actual_size=%dx%d source_payload_bytes=%zu frame_number=%u source_timestamp_ns=%llu sdk_host_timestamp_raw=%lld device_timestamp_ticks=%llu",
        acquired_frame.source_pixel_type_name.c_str(), acquired_frame.source_pixel_type,
        acquired_frame.width, acquired_frame.height, acquired_frame.source_payload_bytes,
        acquired_frame.camera_frame_number,
        static_cast<unsigned long long>(acquired_frame.source_timestamp_ns),
        static_cast<long long>(acquired_frame.sdk_host_timestamp),
        static_cast<unsigned long long>(acquired_frame.device_timestamp_ticks));
    RCLCPP_INFO(get_logger(),
                "startup_effective_config detector runtime=%s raw_conf=%.3f input=%dx%d fake_detection=%s filter_person=%.3f filter_car=%.3f",
                infer_cfg.detector_runtime_path.c_str(), infer_cfg.raw_conf_threshold, infer_cfg.input_width,
                infer_cfg.input_height, BoolStr(infer_cfg.enable_fake_detection), filter_cfg.person_conf_threshold,
                filter_cfg.car_conf_threshold);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config tracker yaml=%s core_mode=%s high=%.3f low=%.3f new=%.3f match=%.3f buffer=%d gmc=%s gmc_method=%s gmc_downscale=%d reid=%s with_reid=%s reid_backend=%s reid_model=%s reid_input=%dx%d",
                tracker_cfg.tracker_yaml_path.c_str(), tracker_cfg.core_mode.c_str(), tracker_cfg.track_high_thresh,
                tracker_cfg.track_low_thresh, tracker_cfg.new_track_thresh, tracker_cfg.match_thresh,
                tracker_cfg.track_buffer, BoolStr(tracker_cfg.gmc_enabled), tracker_cfg.gmc_method.c_str(),
                tracker_cfg.gmc_downscale, BoolStr(tracker_cfg.reid_enabled), BoolStr(tracker_cfg.with_reid),
                tracker_cfg.reid_backend.c_str(), tracker_cfg.reid_model_path.c_str(), tracker_cfg.reid_input_width,
                tracker_cfg.reid_input_height);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config tracker_assoc confirm_hits=%d stage1_iou=%.3f stage2_iou=%.3f unconfirmed_iou=%.3f stage1_max=%.3f stage2_max=%.3f lost_recovery_max=%.3f unconfirmed_max=%.3f weights=%.3f/%.3f/%.3f app_gate=%.3f app_alpha=%.3f app_bins=%d/%d",
                tracker_cfg.confirm_hits, tracker_cfg.stage1_iou_min, tracker_cfg.stage2_iou_min,
                tracker_cfg.unconfirmed_iou_min, tracker_cfg.stage1_max_cost, tracker_cfg.stage2_max_cost,
                tracker_cfg.lost_recovery_max_cost, tracker_cfg.unconfirmed_max_cost, tracker_cfg.assoc_iou_weight,
                tracker_cfg.assoc_motion_weight, tracker_cfg.assoc_app_weight, tracker_cfg.appearance_gate,
                tracker_cfg.appearance_alpha, tracker_cfg.appearance_h_bins, tracker_cfg.appearance_s_bins);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config tracker_occlusion enter_min=%d base=%d extend_step=%d max=%d release_clear=%d shrink=%.3f suspect_area=%.3f neighbor_dist=%.3f overlap_iou=%.3f duplicate_iou=%.3f duplicate_center=%.3f motion_gate=%.3f low_score_app_gate=%s",
                tracker_cfg.occlusion_enter_min_frames, tracker_cfg.occlusion_base_frames,
                tracker_cfg.occlusion_extend_step, tracker_cfg.occlusion_max_frames,
                tracker_cfg.occlusion_release_clear_frames, tracker_cfg.occlusion_shrink_ratio,
                tracker_cfg.occlusion_suspect_area_ratio, tracker_cfg.occlusion_neighbor_center_dist_norm,
                tracker_cfg.occlusion_overlap_iou_min, tracker_cfg.duplicate_lost_iou,
                tracker_cfg.duplicate_lost_center_dist_norm, tracker_cfg.motion_gate_thresh,
                BoolStr(tracker_cfg.use_low_score_appearance_gate));
    RCLCPP_INFO(get_logger(),
                "startup_effective_config identity max_missing=%d feat_bank=%d recover_strict=%.3f recover_relaxed=%.3f relaxed_max_missing=%d occlusion_protect=%d missing_area=%.3f..%.3f missing_center=%.3f overlap_freeze=%.3f split_stable=%d merge_hold=%d weights=%.3f/%.3f/%.3f",
                identity_cfg.max_missing_frames, identity_cfg.feat_bank_size, identity_cfg.recover_sim_thresh_strict,
                identity_cfg.recover_sim_thresh_relaxed, identity_cfg.recover_relaxed_max_missing_frames,
                identity_cfg.occlusion_protect_frames, identity_cfg.missing_assign_min_area_ratio,
                identity_cfg.missing_assign_max_area_ratio, identity_cfg.missing_assign_max_center_dist_norm,
                identity_cfg.overlap_iou_freeze, identity_cfg.split_stable_frames, identity_cfg.merge_hold_frames,
                identity_cfg.app_w, identity_cfg.geo_w, identity_cfg.time_w);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config identity_assign active_max=%.3f recovery_max=%.3f raw_continuity_max=%.3f min_margin=%.3f stable_feature_frames=%d merged_requires_overlap=%s phase5_birth_manager=enabled reid=%s reid_backend=%s reid_model=%s reid_input=%dx%d",
                identity_cfg.active_assign_max_cost, identity_cfg.recovery_max_cost,
                identity_cfg.raw_continuity_max_cost, identity_cfg.min_assignment_margin,
                identity_cfg.stable_frames_before_feature_update, BoolStr(identity_cfg.merged_requires_overlap),
                BoolStr(identity_cfg.reid_enable),
                identity_cfg.reid_backend.c_str(), identity_cfg.reid_model_path.c_str(),
                identity_cfg.reid_input_width, identity_cfg.reid_input_height);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config primary lost_frames=%d min_area=%.1f max_center_jump=%.3f area_ratio=%.3f..%.3f pending_recovery=%d",
                target_cfg.lost_threshold_frames, target_cfg.min_person_area_px, target_cfg.max_center_jump_norm,
                target_cfg.min_area_ratio, target_cfg.max_area_ratio, target_cfg.pending_recovery_frames);
    RCLCPP_INFO(get_logger(),
                "startup_effective_config bearing hfov=%.5f mount_xyz=%.5f/%.5f/%.5f mount_rpy=%.5f/%.5f/%.5f udp=%s:%d visualization=%s recording=%s recording_path=%s recording_fps=%.2f",
                bearing_cfg.camera_horizontal_fov_rad, bearing_cfg.camera_mount_x, bearing_cfg.camera_mount_y,
                bearing_cfg.camera_mount_z, bearing_cfg.camera_mount_roll, bearing_cfg.camera_mount_pitch,
                bearing_cfg.camera_mount_yaw, udp_ip.c_str(), udp_port, BoolStr(viz_cfg.enable_visualization),
                BoolStr(viz_cfg.enable_recording), viz_cfg.recording_path.c_str(), viz_cfg.recording_fps);
  }

  void Tick() {
    std::string error;
    vision_demo_host::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "camera_ingest read failed: %s",
                           error.c_str());
      return;
    }
    cv::Mat &frame = acquired_frame.bgr8;

    auto detections = infer_.Infer(frame);
    const auto filtered = det_filter_.Filter(detections);
    auto tracks = tracker_.Update(filtered, frame);

    const auto primary_prev = primary_manager_.GetState();
    const auto identity_result = identity_manager_.Update(
        vision_demo_host::TrackletObservationsFromTracks(tracks), tracker_.LastTrackletHypotheses(), primary_prev,
        &frame);
    const auto primary = primary_manager_.Update(identity_result.identities);
    vision_demo_host::BearingOutput output{};
    if (primary.primary_track.has_value()) {
      output = bearing_.Estimate(primary.primary_track.value(), frame.cols, frame.rows);
    } else {
      output.u_norm = 0.5F;
      output.v_norm = 0.5F;
      output.bearing_base_rad = 0.0F;
      output.elevation_base_rad = 0.0F;
      output.bearing_confidence = 0.0F;
    }

    if (!udp_.Send(primary, output, &error, &identity_result)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "udp_json_adapter send failed: %s",
                           error.c_str());
    }

    visualizer_.Render(frame, tracks, primary, &identity_result, primary_manager_.LastDecisionReason(),
                       primary_manager_.LastRejectReason());

    if (monitor_.ShouldReport()) {
      const int primary_id = primary.primary_target_id;
      const auto camera_metrics = camera_.Metrics();
      RCLCPP_INFO(get_logger(),
                  "runtime_monitor fps=%.2f state=%s primary_id=%d raw_track_id=%d det=%zu filtered=%zu tracks=%zu",
                  monitor_.CurrentFps(), vision_demo_host::PrimaryStateToString(primary.state).c_str(),
                  primary_id, primary.raw_track_id, detections.size(), filtered.size(), tracks.size());
      RCLCPP_INFO(
          get_logger(),
          "camera_metrics frames=%llu acquisition_failures=%llu dropped_frames=%llu non_contiguous_frames=%llu camera_lost_packets=%llu acquisition_ms_p50_p95_p99=%.3f/%.3f/%.3f conversion_ms_p50_p95_p99=%.3f/%.3f/%.3f copy_ms_p50_p95_p99=%.3f/%.3f/%.3f samples=%zu",
          static_cast<unsigned long long>(camera_metrics.acquired_frames),
          static_cast<unsigned long long>(camera_metrics.acquisition_failures),
          static_cast<unsigned long long>(camera_metrics.dropped_frames),
          static_cast<unsigned long long>(camera_metrics.non_contiguous_frames),
          static_cast<unsigned long long>(camera_metrics.camera_lost_packets),
          camera_metrics.acquisition.p50_ms, camera_metrics.acquisition.p95_ms,
          camera_metrics.acquisition.p99_ms, camera_metrics.conversion.p50_ms,
          camera_metrics.conversion.p95_ms, camera_metrics.conversion.p99_ms,
          camera_metrics.copy.p50_ms, camera_metrics.copy.p95_ms,
          camera_metrics.copy.p99_ms, camera_metrics.acquisition.samples);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;

  vision_demo_host::CameraIngest camera_;
  vision_demo_host::PreprocessInfer infer_;
  vision_demo_host::DetFilter det_filter_;
  vision_demo_host::MotTracker tracker_;
  vision_demo_host::PrimaryTargetManager primary_manager_;
  vision_demo_host::IdentityManager identity_manager_;
  vision_demo_host::BearingEstimator bearing_;
  vision_demo_host::UdpJsonAdapter udp_;
  vision_demo_host::VisualizerRecorder visualizer_;
  vision_demo_host::RuntimeMonitor monitor_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VisionDemoNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_demo_host_node"), "Fatal init/runtime error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
