#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "vision_demo_host/modules/camera_ingest.hpp"
#include "vision_demo_host/modules/det_filter.hpp"
#include "vision_demo_host/modules/identity_manager.hpp"
#include "vision_demo_host/modules/mission_ros_adapter.hpp"
#include "vision_demo_host/modules/mot_tracker.hpp"
#include "vision_demo_host/modules/perception_readiness.hpp"
#include "vision_demo_host/modules/preprocess_infer.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"
#include "vision_demo_host/modules/runtime_monitor.hpp"
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
        identity_manager_(vision_demo_host::IdentityManager::Config{}) {
    RejectRetiredCameraParameterOverrides();
    DeclareParameters();
    InitializeMissionRosAdapter();
    LoadConfigAndInitialize();

    const int timer_ms = this->get_parameter("runtime.tick_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms),
                                     std::bind(&VisionDemoNode::Tick, this));
    RCLCPP_INFO(get_logger(), "vision_demo_host_node started.");
  }

  void ShutdownAndLog() {
    if (visualizer_ == nullptr) {
      return;
    }
    visualizer_->Shutdown();
    LogOverlayMetrics(visualizer_->Metrics(), "final");
  }

 private:
  void RejectRetiredCameraParameterOverrides() {
    const auto &overrides =
        this->get_node_parameters_interface()->get_parameter_overrides();
    const std::vector<std::string> retired_parameters{
        "camera.backend", "camera.rtsp_url", "camera.gstreamer_pipeline"};
    for (const auto &name : retired_parameters) {
      if (overrides.find(name) != overrides.end()) {
        throw std::invalid_argument(
            "Retired camera parameter override '" + name +
            "' is not supported; live input is Hik MVS-only. Remove the override.");
      }
    }
  }

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
    this->declare_parameter<double>("target.lost_event_timeout_sec", 0.5);
    this->declare_parameter<double>("target.reacquire_retention_sec", 6.0);
    this->declare_parameter<double>("target.handled_ignore_absence_sec", 30.0);

    this->declare_parameter<std::string>("mission.state_topic", "/mission/state");
    this->declare_parameter<std::string>("mission.event_topic", "/mission/event");
    this->declare_parameter<std::string>("mission.selected_target_bbox_topic",
                                         "/perception/selected_target_bbox");
    this->declare_parameter<std::string>("perception.camera_optical_frame_id",
                                         "hik_camera_optical_frame");
    this->declare_parameter<bool>("perception.authorization_placeholder_ready", false);
    this->declare_parameter<std::string>(
        "perception.authorization_placeholder_detail",
        "authorization capability has not been integrated");

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

    this->declare_parameter<bool>("visualization.enable", false);
    this->declare_parameter<int>("visualization.queue_capacity", 4);
    this->declare_parameter<bool>("recording.enable", false);
    this->declare_parameter<std::string>("recording.output_root", "data/diagnostics/live_overlays");
    this->declare_parameter<std::string>("recording.path",
                                         "data/diagnostics/live_overlays/vision_demo_overlay.mkv");
    this->declare_parameter<double>("recording.fps", 30.0);
    this->declare_parameter<bool>("runtime.inference_timing_metrics", true);

    this->declare_parameter<int>("runtime.tick_ms", 33);
  }

  void InitializeMissionRosAdapter() {
    const double lost_timeout_sec = this->get_parameter("target.lost_event_timeout_sec").as_double();
    const double reacquire_retention_sec =
        this->get_parameter("target.reacquire_retention_sec").as_double();
    if (!std::isfinite(lost_timeout_sec) || !std::isfinite(reacquire_retention_sec) ||
        lost_timeout_sec <= 0.0 || reacquire_retention_sec <= 0.0 ||
        lost_timeout_sec >= reacquire_retention_sec) {
      throw std::runtime_error(
          "target.lost_event_timeout_sec must be positive and shorter than "
          "target.reacquire_retention_sec");
    }

    vision_demo_host::MissionRosAdapter::Config mission_config;
    mission_config.mission_state_topic = this->get_parameter("mission.state_topic").as_string();
    mission_config.mission_event_topic = this->get_parameter("mission.event_topic").as_string();
    mission_config.target_bbox_topic =
        this->get_parameter("mission.selected_target_bbox_topic").as_string();
    mission_state_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    mission_config.mission_state_callback_group = mission_state_callback_group_;
    mission_config.authorization_placeholder_ready =
        this->get_parameter("perception.authorization_placeholder_ready").as_bool();
    mission_config.authorization_placeholder_detail =
        this->get_parameter("perception.authorization_placeholder_detail").as_string();
    mission_config.coordinator.lost_event_timeout = std::chrono::duration_cast<
        vision_demo_host::MissionCoordinator::Duration>(std::chrono::duration<double>(lost_timeout_sec));
    mission_config.coordinator.reacquire_retention = std::chrono::duration_cast<
        vision_demo_host::MissionCoordinator::Duration>(
        std::chrono::duration<double>(reacquire_retention_sec));
    camera_optical_frame_id_ = this->get_parameter("perception.camera_optical_frame_id").as_string();
    if (camera_optical_frame_id_.empty()) {
      throw std::runtime_error("perception.camera_optical_frame_id must not be empty");
    }
    mission_ros_adapter_ =
        std::make_unique<vision_demo_host::MissionRosAdapter>(*this, std::move(mission_config));
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
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {false, false, "camera input initialization failed: " + error});
      throw std::runtime_error("camera_ingest init failed: " + error);
    }

    vision_demo_host::PreprocessInfer::Config infer_cfg;
    infer_cfg.detector_runtime_path = this->get_parameter("detector.runtime_path").as_string();
    infer_cfg.raw_conf_threshold =
        static_cast<float>(this->get_parameter("detector.raw_conf_threshold").as_double());
    infer_cfg.input_width = this->get_parameter("detector.input_width").as_int();
    infer_cfg.input_height = this->get_parameter("detector.input_height").as_int();
    infer_cfg.enable_fake_detection = this->get_parameter("detector.enable_fake_detection").as_bool();
    infer_cfg.enable_timing_metrics = this->get_parameter("runtime.inference_timing_metrics").as_bool();
    infer_ = vision_demo_host::PreprocessInfer(infer_cfg);
    if (!infer_.Initialize(&error)) {
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {false, false, "detector initialization failed: " + error});
      throw std::runtime_error("preprocess_infer init failed: " + error);
    }
    mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus({true, false, {}});

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
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {true, false, "tracker initialization failed: " + error});
      throw std::runtime_error("mot_tracker init failed: " + error);
    }
    mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});

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
    const double handled_ignore_absence_sec =
        this->get_parameter("target.handled_ignore_absence_sec").as_double();
    if (!std::isfinite(handled_ignore_absence_sec) || handled_ignore_absence_sec <= 0.0) {
      throw std::runtime_error("target.handled_ignore_absence_sec must be positive");
    }
    target_cfg.handled_ignore_absence = std::chrono::duration_cast<
        vision_demo_host::PrimaryTargetManager::Duration>(
        std::chrono::duration<double>(handled_ignore_absence_sec));
    primary_manager_ = vision_demo_host::PrimaryTargetManager(target_cfg);

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

    vision_demo_host::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {true, true, "initial detection/tracking source frame failed: " + error});
      throw std::runtime_error("camera_ingest initial frame failed: " + error);
    }
    cv::Mat &frame = acquired_frame.bgr8;

    vision_demo_host::VisualizerRecorder::Config viz_cfg;
    viz_cfg.enable_preview = this->get_parameter("visualization.enable").as_bool();
    viz_cfg.enable_recording = this->get_parameter("recording.enable").as_bool();
    viz_cfg.recording_output_root = this->get_parameter("recording.output_root").as_string();
    viz_cfg.recording_path = this->get_parameter("recording.path").as_string();
    viz_cfg.recording_fps = this->get_parameter("recording.fps").as_double();
    const int visualization_queue_capacity = this->get_parameter("visualization.queue_capacity").as_int();
    viz_cfg.queue_capacity = visualization_queue_capacity > 0
                                 ? static_cast<std::size_t>(visualization_queue_capacity)
                                 : 0U;
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
    visualizer_ = std::make_unique<vision_demo_host::VisualizerRecorder>(viz_cfg);
    if (!visualizer_->Initialize(frame.size(), &error)) {
      throw std::runtime_error("visualizer_recorder init failed: " + error);
    }

    LogEffectiveConfig(camera_cfg, acquired_frame, infer_cfg, filter_cfg,
                       tracker_.EffectiveConfig(), target_cfg, sid_cfg, viz_cfg);
  }

  void LogEffectiveConfig(const vision_demo_host::CameraIngest::Config &camera_cfg,
                          const vision_demo_host::CameraIngest::AcquiredFrame &acquired_frame,
                          const vision_demo_host::PreprocessInfer::Config &infer_cfg,
                          const vision_demo_host::DetFilter::Config &filter_cfg,
                          const vision_demo_host::MotTracker::Config &tracker_cfg,
                          const vision_demo_host::PrimaryTargetManager::Config &target_cfg,
                          const vision_demo_host::IdentityManager::Config &identity_cfg,
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
                "startup_effective_config live_mode=%s preview=%s recording=%s recording_output_root=%s recording_path=%s recording_fps=%.2f overlay_queue_capacity=%zu",
                vision_demo_host::VisualizerRecorder::ModeName(viz_cfg).c_str(), BoolStr(viz_cfg.enable_preview),
                BoolStr(viz_cfg.enable_recording), viz_cfg.recording_output_root.c_str(),
                viz_cfg.recording_path.c_str(), viz_cfg.recording_fps, viz_cfg.queue_capacity);
  }

  void LogOverlayMetrics(const vision_demo_host::VisualizerRecorder::MetricsSnapshot &overlay_metrics,
                         const char *phase) {
    RCLCPP_INFO(
        get_logger(),
        "overlay_metrics phase=%s submitted=%llu enqueued=%llu queue_drops=%llu render_drops=%llu write_drops=%llu render_errors=%llu write_errors=%llu submitted_fps=%.2f rendered_fps=%.2f previewed_fps=%.2f written_fps=%.2f queue_wait_ms_p50_p95_p99=%.3f/%.3f/%.3f render_ms_p50_p95_p99=%.3f/%.3f/%.3f write_ms_p50_p95_p99=%.3f/%.3f/%.3f",
        phase, static_cast<unsigned long long>(overlay_metrics.submitted_frames),
        static_cast<unsigned long long>(overlay_metrics.enqueued_frames),
        static_cast<unsigned long long>(overlay_metrics.queue_dropped_frames),
        static_cast<unsigned long long>(overlay_metrics.render_dropped_frames),
        static_cast<unsigned long long>(overlay_metrics.write_dropped_frames),
        static_cast<unsigned long long>(overlay_metrics.render_errors),
        static_cast<unsigned long long>(overlay_metrics.write_errors), overlay_metrics.submitted_fps,
        overlay_metrics.rendered_fps, overlay_metrics.previewed_fps, overlay_metrics.written_fps,
        overlay_metrics.queue_wait.p50_ms, overlay_metrics.queue_wait.p95_ms,
        overlay_metrics.queue_wait.p99_ms, overlay_metrics.render.p50_ms,
        overlay_metrics.render.p95_ms, overlay_metrics.render.p99_ms,
        overlay_metrics.write.p50_ms, overlay_metrics.write.p95_ms, overlay_metrics.write.p99_ms);
    const std::string overlay_error = visualizer_->LastError();
    if (!overlay_error.empty()) {
      RCLCPP_WARN(get_logger(), "overlay worker last_error: %s", overlay_error.c_str());
    }
  }

  void ApplyMissionPrimaryLifecycle(
      const std::optional<vision_demo_host::MissionSnapshot> &mission,
      const std::optional<vision_demo_host::MissionSnapshot> &previous_mission) {
    if (!mission.has_value()) {
      return;
    }

    if (mission->phase == vision_demo_host::MissionPhase::kPatrol &&
        (!last_mission_for_primary_.has_value() ||
         last_mission_for_primary_->state_seq != mission->state_seq ||
         last_mission_for_primary_->phase != vision_demo_host::MissionPhase::kPatrol)) {
      int handled_semantic_id = -1;
      const auto preceding_mission = previous_mission.has_value() &&
                                           previous_mission->state_seq != mission->state_seq
                                       ? previous_mission
                                       : last_mission_for_primary_;
      if (preceding_mission.has_value() && preceding_mission->target_id > 0 &&
          (preceding_mission->phase == vision_demo_host::MissionPhase::kVerifyIdentity ||
           preceding_mission->phase == vision_demo_host::MissionPhase::kTrackIntruder)) {
        handled_semantic_id = preceding_mission->target_id;
      }
      primary_manager_.ResetForPatrolCycle(handled_semantic_id);
    }
    last_mission_for_primary_ = mission;
  }

  vision_demo_host::SourceFrameMetadata SourceMetadata(
      const vision_demo_host::CameraIngest::AcquiredFrame &frame) const {
    vision_demo_host::SourceFrameMetadata metadata;
    metadata.source_timestamp_ns = frame.source_timestamp_ns;
    metadata.camera_frame_number = frame.camera_frame_number;
    metadata.camera_frame_number_available = frame.camera_frame_number_available;
    metadata.image_width = frame.width;
    metadata.image_height = frame.height;
    metadata.optical_frame_id = camera_optical_frame_id_;
    return metadata;
  }

  vision_demo_host::MissionCoordinator::TimePoint MonotonicSourceTime() const {
    return vision_demo_host::MissionCoordinator::Clock::now();
  }

  void Tick() {
    // A MultiThreadedExecutor must not run two inference frames at once. ROS
    // mission callbacks only copy a validated snapshot under their own mutex;
    // the complete camera/detector/tracker/identity/coordinator chain stays
    // serialized here.
    std::lock_guard<std::mutex> pipeline_lock(pipeline_mutex_);
    mission_ros_adapter_->PublishReadiness();

    std::string error;
    vision_demo_host::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {true, true, "detection/tracking source frame failed: " + error});
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "camera_ingest read failed: %s",
                           error.c_str());
      return;
    }
    cv::Mat &frame = acquired_frame.bgr8;

    std::vector<vision_demo_host::Detection> detections;
    std::vector<vision_demo_host::Detection> filtered;
    std::vector<vision_demo_host::Track> tracks;
    try {
      detections = infer_.Infer(frame);
      filtered = det_filter_.Filter(detections);
      tracks = tracker_.Update(filtered, frame);
    } catch (const std::exception &exception) {
      mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus(
          {true, true, "detection/tracking frame processing failed: " + std::string(exception.what())});
      throw;
    }
    mission_ros_adapter_->detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});

    const auto source_time = MonotonicSourceTime();
    const auto mission = mission_ros_adapter_->CurrentMission();
    ApplyMissionPrimaryLifecycle(mission, mission_ros_adapter_->PreviousMission());
    const auto primary_prev = primary_manager_.GetState();
    auto identity_result = identity_manager_.Update(
        vision_demo_host::TrackletObservationsFromTracks(tracks), tracker_.LastTrackletHypotheses(), primary_prev,
        &frame);
    auto primary = mission.has_value() && mission->phase == vision_demo_host::MissionPhase::kPatrol
                       ? primary_manager_.UpdateForPatrol(identity_result.identities,
                                                          source_time)
                       : primary_manager_.Update(identity_result.identities);
    const auto source_metadata = SourceMetadata(acquired_frame);
    if (mission.has_value() && mission->phase == vision_demo_host::MissionPhase::kPatrol) {
      mission_ros_adapter_->PublishTargetConfirmed(mission.value(), primary, source_metadata);
    }
    mission_ros_adapter_->ProcessFrame(
        {vision_demo_host::MissionSnapshot{}, identity_result.identities, primary,
         source_time},
        source_metadata);

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
      const auto inference_metrics = infer_.Metrics();
      RCLCPP_INFO(
          get_logger(),
          "inference_metrics total_ms_p50_p95_p99=%.3f/%.3f/%.3f samples=%zu",
          inference_metrics.total.p50_ms, inference_metrics.total.p95_ms,
          inference_metrics.total.p99_ms, inference_metrics.total.samples);
      if (visualizer_ != nullptr) {
        LogOverlayMetrics(visualizer_->Metrics(), "runtime");
      }
    }

    if (visualizer_ != nullptr) {
      visualizer_->Submit(std::move(acquired_frame.bgr8), std::move(tracks), std::move(primary),
                          std::move(identity_result),
                          primary_manager_.LastDecisionReason(), primary_manager_.LastRejectReason());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;

  vision_demo_host::CameraIngest camera_;
  vision_demo_host::PreprocessInfer infer_;
  vision_demo_host::DetFilter det_filter_;
  vision_demo_host::MotTracker tracker_;
  std::unique_ptr<vision_demo_host::MissionRosAdapter> mission_ros_adapter_;
  rclcpp::CallbackGroup::SharedPtr mission_state_callback_group_;
  std::string camera_optical_frame_id_;
  std::optional<vision_demo_host::MissionSnapshot> last_mission_for_primary_;
  std::mutex pipeline_mutex_;
  vision_demo_host::PrimaryTargetManager primary_manager_;
  vision_demo_host::IdentityManager identity_manager_;
  std::unique_ptr<vision_demo_host::VisualizerRecorder> visualizer_;
  vision_demo_host::RuntimeMonitor monitor_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<VisionDemoNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
    executor.add_node(node);
    executor.spin();
    node->ShutdownAndLog();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_demo_host_node"), "Fatal init/runtime error: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
