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

#include "dog_patrol_perception_tracking/modules/camera_ingest.hpp"
#include "dog_patrol_perception_tracking/modules/det_filter.hpp"
#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"
#include "dog_patrol_perception_tracking/modules/mission_ros_adapter.hpp"
#include "dog_patrol_perception_tracking/modules/mot_tracker.hpp"
#include "dog_patrol_perception_tracking/modules/perception_config_materializer.hpp"
#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"
#include "dog_patrol_perception_tracking/modules/preprocess_infer.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"
#include "dog_patrol_perception_tracking/modules/runtime_monitor.hpp"
#include "dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp"
#include "dog_patrol_perception_tracking/modules/visualizer_recorder.hpp"

namespace {

const char *BoolStr(const bool value) { return value ? "true" : "false"; }

enum class RuntimeMode {
  kMission,
  kStandalone,
};

RuntimeMode ParseRuntimeMode(const std::string &value) {
  if (value == "mission") {
    return RuntimeMode::kMission;
  }
  if (value == "standalone") {
    return RuntimeMode::kStandalone;
  }
  throw std::runtime_error("runtime.mode must be 'mission' or 'standalone'");
}

struct RuntimeFrameOutput {
  dog_patrol_perception_tracking::PrimaryTargetResult primary;
  std::string primary_decision_reason;
  std::string primary_reject_reason;
};

class TrackingRuntime {
 public:
  virtual ~TrackingRuntime() = default;
  virtual const char *Name() const = 0;
  virtual bool FailFastOnInitializationError() const = 0;
  virtual dog_patrol_perception_tracking::PrimaryTargetResult CurrentPrimary() const = 0;
  virtual RuntimeFrameOutput ProcessFrame(
      const std::vector<dog_patrol_perception_tracking::IdentityObservation> &identities,
      const dog_patrol_perception_tracking::SourceFrameMetadata &source_metadata,
      const cv::Mat &frame) = 0;
};

class DetectionTrackingStatusSink {
 public:
  virtual ~DetectionTrackingStatusSink() = default;
  virtual void Report(
      dog_patrol_perception_tracking::DetectionTrackingReadiness::RuntimeStatus status) = 0;
  virtual void Publish() = 0;
};

class ObservationLifecycle {
 public:
  virtual ~ObservationLifecycle() = default;
  virtual void BeforeFrame() = 0;
  virtual bool Current() const = 0;
  virtual dog_patrol_perception_tracking::TargetImageRosAdapter::Metrics Metrics() const = 0;
};

class MissionTrackingRuntime final : public TrackingRuntime,
                                     public DetectionTrackingStatusSink {
 public:
  explicit MissionTrackingRuntime(
      std::unique_ptr<dog_patrol_perception_tracking::MissionRosAdapter> adapter)
      : adapter_(std::move(adapter)) {}

  const char *Name() const override { return "mission"; }
  bool FailFastOnInitializationError() const override { return false; }
  void Report(
      dog_patrol_perception_tracking::DetectionTrackingReadiness::RuntimeStatus status) override {
    adapter_->ReportDetectionTrackingRuntimeStatus(std::move(status));
  }
  void Publish() override { adapter_->PublishCapabilityStatus(); }
  dog_patrol_perception_tracking::PrimaryTargetResult CurrentPrimary() const override {
    return adapter_->CurrentPrimary();
  }
  RuntimeFrameOutput ProcessFrame(
      const std::vector<dog_patrol_perception_tracking::IdentityObservation> &identities,
      const dog_patrol_perception_tracking::SourceFrameMetadata &source_metadata,
      const cv::Mat &) override {
    auto frame = adapter_->ProcessFrame(
        identities, dog_patrol_perception_tracking::MissionCoordinator::Clock::now(),
        source_metadata);
    return {std::move(frame.primary), std::move(frame.primary_decision_reason),
            std::move(frame.primary_reject_reason)};
  }
 private:
  std::unique_ptr<dog_patrol_perception_tracking::MissionRosAdapter> adapter_;
};

class StandaloneTrackingRuntime final : public TrackingRuntime,
                                        public ObservationLifecycle {
 public:
  explicit StandaloneTrackingRuntime(
      dog_patrol_perception_tracking::PrimaryTargetManager::Config config,
      std::shared_ptr<dog_patrol_perception_tracking::TargetImageRosAdapter> image_adapter,
      dog_patrol_perception_tracking::PrimaryTargetObserver::CropConfig crop_config)
      : image_adapter_(std::move(image_adapter)),
        observer_(std::move(config), image_adapter_, crop_config) {}

  const char *Name() const override { return "standalone"; }
  bool FailFastOnInitializationError() const override { return true; }
  void BeforeFrame() override { observer_.InvalidateCurrentObservation(); }
  dog_patrol_perception_tracking::PrimaryTargetResult CurrentPrimary() const override {
    return observer_.CurrentPrimary();
  }
  RuntimeFrameOutput ProcessFrame(
      const std::vector<dog_patrol_perception_tracking::IdentityObservation> &identities,
      const dog_patrol_perception_tracking::SourceFrameMetadata &source_metadata,
      const cv::Mat &frame) override {
    auto output = observer_.Update(identities, source_metadata, frame);
    return {std::move(output.primary), std::move(output.primary_decision_reason),
            std::move(output.primary_reject_reason)};
  }
  bool Current() const override { return image_adapter_->HasCurrentObservation(); }
  dog_patrol_perception_tracking::TargetImageRosAdapter::Metrics Metrics() const override {
    return image_adapter_->GetMetrics();
  }

 private:
  std::shared_ptr<dog_patrol_perception_tracking::TargetImageRosAdapter> image_adapter_;
  dog_patrol_perception_tracking::PrimaryTargetObserver observer_;
};

}  // namespace

class PerceptionTrackingNode : public rclcpp::Node {
 public:
  PerceptionTrackingNode()
      : Node("dog_patrol_perception_tracking_node"),
        infer_(dog_patrol_perception_tracking::PreprocessInfer::Config{}),
        det_filter_(dog_patrol_perception_tracking::DetFilter::Config{}),
        tracker_(dog_patrol_perception_tracking::MotTracker::Config{}),
        identity_manager_(dog_patrol_perception_tracking::IdentityManager::Config{}) {
    RejectRetiredCameraParameterOverrides();
    DeclareParameters();
    const auto target_cfg = LoadPrimaryTargetConfig();
    InitializeRuntimeMode(target_cfg);
    try {
      LoadConfigAndInitialize(target_cfg);
      runtime_operational_ = true;
    } catch (const std::exception &exception) {
      ReportDetectionTrackingRuntimeStatus(
          {false, false, "tracking initialization failed: " + std::string(exception.what())});
      if (runtime_->FailFastOnInitializationError()) {
        RCLCPP_ERROR(get_logger(), "standalone tracking initialization failed: %s", exception.what());
        throw;
      }
      RCLCPP_ERROR(get_logger(),
                   "tracking initialization failed; node remains alive to report capability ERROR: %s",
                   exception.what());
    }

    const int timer_ms = this->get_parameter("runtime.tick_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms),
                                     std::bind(&PerceptionTrackingNode::Tick, this));
    if (runtime_operational_) {
      RCLCPP_INFO(get_logger(), "dog_patrol_perception_tracking_node started in %s mode.",
                  runtime_->Name());
    } else {
      RCLCPP_WARN(get_logger(),
                  "dog_patrol_perception_tracking_node started in capability-error reporting mode.");
    }
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
    const dog_patrol_perception_tracking::PerceptionConfigMaterializer::TrackerInput tracker_defaults;
    const dog_patrol_perception_tracking::PerceptionConfigMaterializer::IdentityInput identity_defaults;
    const dog_patrol_perception_tracking::PerceptionConfigMaterializer::VisualizerInput visualizer_defaults;

    this->declare_parameter<std::string>("camera.mvs_model", "MV-CU013-A0UC");
    this->declare_parameter<std::string>("camera.mvs_serial", "");
    this->declare_parameter<int>("camera.width", 1280);
    this->declare_parameter<int>("camera.height", 1024);
    this->declare_parameter<double>("camera.fps", 30.0);
    this->declare_parameter<int>("camera.timeout_ms", 1000);
    this->declare_parameter<std::string>(
        "camera.bayer_interpolation",
        dog_patrol_perception_tracking::CameraIngest::BayerInterpolationName(
            dog_patrol_perception_tracking::CameraIngest::kDefaultBayerInterpolation));
    this->declare_parameter<bool>("camera.bayer_smoothing",
                                  dog_patrol_perception_tracking::CameraIngest::kDefaultBayerSmoothing);

    this->declare_parameter<std::string>("detector.runtime_path", "");
    this->declare_parameter<double>("detector.raw_conf_threshold", 0.10);
    this->declare_parameter<int>("detector.input_width", 640);
    this->declare_parameter<int>("detector.input_height", 640);
    this->declare_parameter<double>("detector.person_conf_threshold", 0.10);
    this->declare_parameter<double>("detector.car_conf_threshold", 0.10);
    this->declare_parameter<bool>("detector.enable_fake_detection", false);

    this->declare_parameter<std::string>("tracker.config_path", tracker_defaults.config_path);
    this->declare_parameter<bool>("tracker.gmc_enabled", tracker_defaults.gmc_enabled);
    this->declare_parameter<bool>("tracker.reid_enabled", tracker_defaults.reid_enabled);
    this->declare_parameter<double>("tracker.track_high_thresh", tracker_defaults.track_high_thresh);
    this->declare_parameter<double>("tracker.track_low_thresh", tracker_defaults.track_low_thresh);
    this->declare_parameter<double>("tracker.new_track_thresh", tracker_defaults.new_track_thresh);
    this->declare_parameter<double>("tracker.match_thresh", tracker_defaults.match_thresh);
    this->declare_parameter<int>("tracker.track_buffer", tracker_defaults.track_buffer);
    this->declare_parameter<std::string>("tracker.gmc_method", tracker_defaults.gmc_method);
    this->declare_parameter<int>("tracker.gmc_downscale", tracker_defaults.gmc_downscale);
    this->declare_parameter<bool>("tracker.with_reid", tracker_defaults.with_reid);
    this->declare_parameter<std::string>("tracker.reid_backend", tracker_defaults.reid_backend);
    this->declare_parameter<std::string>("tracker.reid_model_path", tracker_defaults.reid_model_path);
    this->declare_parameter<int>("tracker.reid_input_width", tracker_defaults.reid_input_width);
    this->declare_parameter<int>("tracker.reid_input_height", tracker_defaults.reid_input_height);
    this->declare_parameter<int>("target.lost_threshold_frames", identity_defaults.target_lost_threshold_frames);
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
    this->declare_parameter<std::string>("perception.capability_status_topic",
                                         "/perception/capability_status");
    this->declare_parameter<std::string>("perception.camera_optical_frame_id",
                                         "hik_camera_optical_frame");
    this->declare_parameter<std::string>("target_image.topic",
                                         "/perception/tracked_target_image");
    this->declare_parameter<double>("target_image.crop_padding_ratio", 0.10);
    this->declare_parameter<double>("target_image.max_publish_hz", 10.0);
    this->declare_parameter<int>("target_image.queue_capacity", 2);

    this->declare_parameter<int>("sid.feat_bank_size", identity_defaults.feat_bank_size);
    this->declare_parameter<double>("sid.recover_sim_thresh_strict", identity_defaults.recover_sim_thresh_strict);
    this->declare_parameter<double>("sid.recover_sim_thresh_relaxed", identity_defaults.recover_sim_thresh_relaxed);
    this->declare_parameter<int>("sid.recover_relaxed_max_missing_frames",
                                 identity_defaults.recover_relaxed_max_missing_frames);
    this->declare_parameter<int>("sid.occlusion_protect_frames", identity_defaults.occlusion_protect_frames);
    this->declare_parameter<double>("sid.missing_assign_min_area_ratio",
                                    identity_defaults.missing_assign_min_area_ratio);
    this->declare_parameter<double>("sid.missing_assign_max_area_ratio",
                                    identity_defaults.missing_assign_max_area_ratio);
    this->declare_parameter<double>("sid.missing_assign_max_center_dist_norm",
                                    identity_defaults.missing_assign_max_center_dist_norm);
    this->declare_parameter<double>("sid.missing_assign_max_app_cost", identity_defaults.missing_assign_max_app_cost);
    this->declare_parameter<double>("sid.overlap_iou_freeze", identity_defaults.overlap_iou_freeze);
    this->declare_parameter<int>("sid.split_stable_frames", identity_defaults.split_stable_frames);
    this->declare_parameter<int>("sid.merge_hold_frames", identity_defaults.merge_hold_frames);
    this->declare_parameter<double>("sid.app_w", identity_defaults.app_w);
    this->declare_parameter<double>("sid.geo_w", identity_defaults.geo_w);
    this->declare_parameter<double>("sid.time_w", identity_defaults.time_w);
    this->declare_parameter<double>("sid.active_assign_max_cost", identity_defaults.active_assign_max_cost);
    this->declare_parameter<double>("sid.recovery_max_cost", identity_defaults.recovery_max_cost);
    this->declare_parameter<double>("sid.raw_continuity_max_cost", identity_defaults.raw_continuity_max_cost);
    this->declare_parameter<double>("sid.min_assignment_margin", identity_defaults.min_assignment_margin);
    this->declare_parameter<int>("sid.stable_frames_before_feature_update",
                                 identity_defaults.stable_frames_before_feature_update);
    this->declare_parameter<bool>("sid.merged_requires_overlap", identity_defaults.merged_requires_overlap);
    this->declare_parameter<bool>("sid.reid_enable", identity_defaults.reid_enable);
    this->declare_parameter<std::string>("sid.reid_backend", identity_defaults.reid_backend);
    this->declare_parameter<std::string>("sid.reid_model_path", identity_defaults.reid_model_path);
    this->declare_parameter<int>("sid.reid_input_width", identity_defaults.reid_input_width);
    this->declare_parameter<int>("sid.reid_input_height", identity_defaults.reid_input_height);

    this->declare_parameter<bool>("visualization.enable", visualizer_defaults.enable_preview);
    this->declare_parameter<int>("visualization.queue_capacity", visualizer_defaults.queue_capacity);
    this->declare_parameter<bool>("recording.enable", visualizer_defaults.enable_recording);
    this->declare_parameter<std::string>("recording.output_root", visualizer_defaults.recording_output_root);
    this->declare_parameter<std::string>("recording.path", visualizer_defaults.recording_path);
    this->declare_parameter<double>("recording.fps", visualizer_defaults.recording_fps);
    this->declare_parameter<bool>("runtime.inference_timing_metrics", true);

    this->declare_parameter<std::string>("runtime.mode", "mission");
    this->declare_parameter<int>("runtime.tick_ms", 33);
  }

  void InitializeRuntimeMode(
      const dog_patrol_perception_tracking::PrimaryTargetManager::Config &target_cfg) {
    camera_optical_frame_id_ = this->get_parameter("perception.camera_optical_frame_id").as_string();
    if (camera_optical_frame_id_.empty()) {
      throw std::runtime_error("perception.camera_optical_frame_id must not be empty");
    }

    const RuntimeMode mode = ParseRuntimeMode(this->get_parameter("runtime.mode").as_string());
    if (mode == RuntimeMode::kMission) {
      auto runtime = std::make_unique<MissionTrackingRuntime>(
          CreateMissionRosAdapter(target_cfg));
      detection_tracking_status_sink_ = runtime.get();
      runtime_ = std::move(runtime);
      return;
    }
    dog_patrol_perception_tracking::TargetImageRosAdapter::Config adapter_config;
    adapter_config.topic = this->get_parameter("target_image.topic").as_string();
    const auto queue_capacity = this->get_parameter("target_image.queue_capacity").as_int();
    if (queue_capacity <= 0) {
      throw std::runtime_error("target_image.queue_capacity must be positive");
    }
    adapter_config.queue_capacity = static_cast<std::size_t>(queue_capacity);
    adapter_config.max_publish_hz =
        this->get_parameter("target_image.max_publish_hz").as_double();
    auto image_adapter = std::make_shared<
        dog_patrol_perception_tracking::TargetImageRosAdapter>(*this, adapter_config);
    dog_patrol_perception_tracking::PrimaryTargetObserver::CropConfig crop_config;
    crop_config.padding_ratio = static_cast<float>(
        this->get_parameter("target_image.crop_padding_ratio").as_double());
    auto runtime = std::make_unique<StandaloneTrackingRuntime>(
        target_cfg, std::move(image_adapter), crop_config);
    observation_lifecycle_ = runtime.get();
    runtime_ = std::move(runtime);
  }

  dog_patrol_perception_tracking::PrimaryTargetManager::Config LoadPrimaryTargetConfig() {
    dog_patrol_perception_tracking::PrimaryTargetManager::Config target_cfg;
    target_cfg.lost_threshold_frames = this->get_parameter("target.lost_threshold_frames").as_int();
    target_cfg.min_person_area_px =
        static_cast<float>(this->get_parameter("target.min_person_area_px").as_double());
    target_cfg.max_center_jump_norm =
        std::max(0.0F, static_cast<float>(this->get_parameter("target.max_center_jump_norm").as_double()));
    target_cfg.min_area_ratio =
        std::max(0.0F, static_cast<float>(this->get_parameter("target.min_area_ratio").as_double()));
    target_cfg.max_area_ratio =
        std::max(target_cfg.min_area_ratio,
                 static_cast<float>(this->get_parameter("target.max_area_ratio").as_double()));
    target_cfg.pending_recovery_frames =
        std::max(0, static_cast<int>(this->get_parameter("target.pending_recovery_frames").as_int()));
    const double handled_ignore_absence_sec =
        this->get_parameter("target.handled_ignore_absence_sec").as_double();
    if (!std::isfinite(handled_ignore_absence_sec) || handled_ignore_absence_sec <= 0.0) {
      throw std::runtime_error("target.handled_ignore_absence_sec must be positive");
    }
    target_cfg.handled_ignore_absence = std::chrono::duration_cast<
        dog_patrol_perception_tracking::PrimaryTargetManager::Duration>(
        std::chrono::duration<double>(handled_ignore_absence_sec));
    return target_cfg;
  }

  std::unique_ptr<dog_patrol_perception_tracking::MissionRosAdapter>
  CreateMissionRosAdapter(
      const dog_patrol_perception_tracking::PrimaryTargetManager::Config &target_cfg) {
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

    dog_patrol_perception_tracking::MissionRosAdapter::Config mission_config;
    mission_config.mission_state_topic = this->get_parameter("mission.state_topic").as_string();
    mission_config.mission_event_topic = this->get_parameter("mission.event_topic").as_string();
    mission_config.target_bbox_topic =
        this->get_parameter("mission.selected_target_bbox_topic").as_string();
    mission_config.capability_status_topic =
        this->get_parameter("perception.capability_status_topic").as_string();
    mission_state_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    mission_config.mission_state_callback_group = mission_state_callback_group_;
    mission_config.primary = target_cfg;
    mission_config.coordinator.lost_event_timeout = std::chrono::duration_cast<
        dog_patrol_perception_tracking::MissionCoordinator::Duration>(std::chrono::duration<double>(lost_timeout_sec));
    mission_config.coordinator.reacquire_retention = std::chrono::duration_cast<
        dog_patrol_perception_tracking::MissionCoordinator::Duration>(
        std::chrono::duration<double>(reacquire_retention_sec));
    return std::make_unique<dog_patrol_perception_tracking::MissionRosAdapter>(
        *this, std::move(mission_config));
  }

  void ReportDetectionTrackingRuntimeStatus(
      dog_patrol_perception_tracking::DetectionTrackingReadiness::RuntimeStatus status) {
    if (detection_tracking_status_sink_ != nullptr) {
      detection_tracking_status_sink_->Report(std::move(status));
    }
  }

  void PublishCapabilityStatus() {
    if (detection_tracking_status_sink_ != nullptr) {
      detection_tracking_status_sink_->Publish();
    }
  }

  dog_patrol_perception_tracking::PrimaryTargetResult CurrentPrimary() const {
    return runtime_->CurrentPrimary();
  }

  void LoadConfigAndInitialize(const dog_patrol_perception_tracking::PrimaryTargetManager::Config &target_cfg) {
    std::string error;

    dog_patrol_perception_tracking::CameraIngest::Config camera_cfg;
    camera_cfg.hik_mvs_model = this->get_parameter("camera.mvs_model").as_string();
    camera_cfg.hik_mvs_serial = this->get_parameter("camera.mvs_serial").as_string();
    camera_cfg.width = this->get_parameter("camera.width").as_int();
    camera_cfg.height = this->get_parameter("camera.height").as_int();
    camera_cfg.fps = this->get_parameter("camera.fps").as_double();
    camera_cfg.timeout_ms = this->get_parameter("camera.timeout_ms").as_int();
    if (!dog_patrol_perception_tracking::CameraIngest::ParseBayerInterpolation(
            this->get_parameter("camera.bayer_interpolation").as_string(),
            &camera_cfg.bayer_interpolation, &error)) {
      throw std::runtime_error(error);
    }
    camera_cfg.bayer_smoothing =
        this->get_parameter("camera.bayer_smoothing").as_bool();

    if (!camera_.Open(camera_cfg, &error)) {
      ReportDetectionTrackingRuntimeStatus(
          {false, false, "camera input initialization failed: " + error});
      throw std::runtime_error("camera_ingest init failed: " + error);
    }

    dog_patrol_perception_tracking::PreprocessInfer::Config infer_cfg;
    infer_cfg.detector_runtime_path = this->get_parameter("detector.runtime_path").as_string();
    infer_cfg.raw_conf_threshold =
        static_cast<float>(this->get_parameter("detector.raw_conf_threshold").as_double());
    infer_cfg.input_width = this->get_parameter("detector.input_width").as_int();
    infer_cfg.input_height = this->get_parameter("detector.input_height").as_int();
    infer_cfg.enable_fake_detection = this->get_parameter("detector.enable_fake_detection").as_bool();
    infer_cfg.enable_timing_metrics = this->get_parameter("runtime.inference_timing_metrics").as_bool();
    infer_ = dog_patrol_perception_tracking::PreprocessInfer(infer_cfg);
    if (!infer_.Initialize(&error)) {
      ReportDetectionTrackingRuntimeStatus(
          {false, false, "detector initialization failed: " + error});
      throw std::runtime_error("preprocess_infer init failed: " + error);
    }
    ReportDetectionTrackingRuntimeStatus({true, false, {}});

    dog_patrol_perception_tracking::DetFilter::Config filter_cfg;
    filter_cfg.person_conf_threshold =
        static_cast<float>(this->get_parameter("detector.person_conf_threshold").as_double());
    filter_cfg.car_conf_threshold =
        static_cast<float>(this->get_parameter("detector.car_conf_threshold").as_double());
    det_filter_ = dog_patrol_perception_tracking::DetFilter(filter_cfg);

    dog_patrol_perception_tracking::PerceptionConfigMaterializer::TrackerInput tracker_input;
    tracker_input.config_path = this->get_parameter("tracker.config_path").as_string();
    tracker_input.gmc_enabled = this->get_parameter("tracker.gmc_enabled").as_bool();
    tracker_input.reid_enabled = this->get_parameter("tracker.reid_enabled").as_bool();
    tracker_input.track_high_thresh =
        static_cast<float>(this->get_parameter("tracker.track_high_thresh").as_double());
    tracker_input.track_low_thresh =
        static_cast<float>(this->get_parameter("tracker.track_low_thresh").as_double());
    tracker_input.new_track_thresh =
        static_cast<float>(this->get_parameter("tracker.new_track_thresh").as_double());
    tracker_input.match_thresh = static_cast<float>(this->get_parameter("tracker.match_thresh").as_double());
    tracker_input.track_buffer = this->get_parameter("tracker.track_buffer").as_int();
    tracker_input.gmc_method = this->get_parameter("tracker.gmc_method").as_string();
    tracker_input.gmc_downscale = static_cast<int>(this->get_parameter("tracker.gmc_downscale").as_int());
    tracker_input.with_reid = this->get_parameter("tracker.with_reid").as_bool();
    tracker_input.reid_backend = this->get_parameter("tracker.reid_backend").as_string();
    tracker_input.reid_model_path = this->get_parameter("tracker.reid_model_path").as_string();
    tracker_input.reid_input_width = static_cast<int>(this->get_parameter("tracker.reid_input_width").as_int());
    tracker_input.reid_input_height = static_cast<int>(this->get_parameter("tracker.reid_input_height").as_int());
    dog_patrol_perception_tracking::PerceptionConfigMaterializer::Diagnostics tracker_config_diagnostics;
    auto tracker_cfg = dog_patrol_perception_tracking::PerceptionConfigMaterializer::MaterializeTrackerConfig(
        tracker_input, &tracker_config_diagnostics);
    if (tracker_config_diagnostics.tracker_reid_forced) {
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
    tracker_ = dog_patrol_perception_tracking::MotTracker(tracker_cfg);
    if (!tracker_.Initialize(&error)) {
      ReportDetectionTrackingRuntimeStatus(
          {true, false, "tracker initialization failed: " + error});
      throw std::runtime_error("mot_tracker init failed: " + error);
    }
    ReportDetectionTrackingRuntimeStatus({true, true, {}});

    dog_patrol_perception_tracking::PerceptionConfigMaterializer::IdentityInput sid_input;
    sid_input.target_lost_threshold_frames = target_cfg.lost_threshold_frames;
    sid_input.feat_bank_size = static_cast<int>(this->get_parameter("sid.feat_bank_size").as_int());
    sid_input.recover_sim_thresh_strict =
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_strict").as_double());
    sid_input.recover_sim_thresh_relaxed =
        static_cast<float>(this->get_parameter("sid.recover_sim_thresh_relaxed").as_double());
    sid_input.recover_relaxed_max_missing_frames =
        static_cast<int>(this->get_parameter("sid.recover_relaxed_max_missing_frames").as_int());
    sid_input.occlusion_protect_frames = static_cast<int>(this->get_parameter("sid.occlusion_protect_frames").as_int());
    sid_input.missing_assign_min_area_ratio =
        static_cast<float>(this->get_parameter("sid.missing_assign_min_area_ratio").as_double());
    sid_input.missing_assign_max_area_ratio =
        static_cast<float>(this->get_parameter("sid.missing_assign_max_area_ratio").as_double());
    sid_input.missing_assign_max_center_dist_norm =
        static_cast<float>(this->get_parameter("sid.missing_assign_max_center_dist_norm").as_double());
    sid_input.missing_assign_max_app_cost =
        static_cast<float>(this->get_parameter("sid.missing_assign_max_app_cost").as_double());
    sid_input.overlap_iou_freeze =
        static_cast<float>(this->get_parameter("sid.overlap_iou_freeze").as_double());
    sid_input.split_stable_frames = static_cast<int>(this->get_parameter("sid.split_stable_frames").as_int());
    sid_input.merge_hold_frames = static_cast<int>(this->get_parameter("sid.merge_hold_frames").as_int());
    sid_input.app_w = static_cast<float>(this->get_parameter("sid.app_w").as_double());
    sid_input.geo_w = static_cast<float>(this->get_parameter("sid.geo_w").as_double());
    sid_input.time_w = static_cast<float>(this->get_parameter("sid.time_w").as_double());
    sid_input.active_assign_max_cost =
        static_cast<float>(this->get_parameter("sid.active_assign_max_cost").as_double());
    sid_input.recovery_max_cost = static_cast<float>(this->get_parameter("sid.recovery_max_cost").as_double());
    sid_input.raw_continuity_max_cost =
        static_cast<float>(this->get_parameter("sid.raw_continuity_max_cost").as_double());
    sid_input.min_assignment_margin =
        static_cast<float>(this->get_parameter("sid.min_assignment_margin").as_double());
    sid_input.stable_frames_before_feature_update =
        static_cast<int>(this->get_parameter("sid.stable_frames_before_feature_update").as_int());
    sid_input.merged_requires_overlap = this->get_parameter("sid.merged_requires_overlap").as_bool();
    sid_input.reid_enable = this->get_parameter("sid.reid_enable").as_bool();
    sid_input.reid_backend = this->get_parameter("sid.reid_backend").as_string();
    sid_input.reid_model_path = this->get_parameter("sid.reid_model_path").as_string();
    sid_input.reid_input_width = static_cast<int>(this->get_parameter("sid.reid_input_width").as_int());
    sid_input.reid_input_height = static_cast<int>(this->get_parameter("sid.reid_input_height").as_int());
    dog_patrol_perception_tracking::PerceptionConfigMaterializer::Diagnostics identity_config_diagnostics;
    const auto sid_cfg = dog_patrol_perception_tracking::PerceptionConfigMaterializer::MaterializeIdentityConfig(
        sid_input, &identity_config_diagnostics);
    if (identity_config_diagnostics.identity_reid_forced) {
      RCLCPP_WARN(get_logger(), "reid is mandatory; override sid.reid_enable to true");
    }
    identity_manager_ = dog_patrol_perception_tracking::IdentityManager(sid_cfg);
    if (!identity_manager_.Initialize(&error)) {
      throw std::runtime_error("identity_manager init failed: " + error);
    }

    dog_patrol_perception_tracking::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      ReportDetectionTrackingRuntimeStatus(
          {true, true, "initial detection/tracking source frame failed: " + error});
      throw std::runtime_error("camera_ingest initial frame failed: " + error);
    }
    cv::Mat &frame = acquired_frame.bgr8;

    dog_patrol_perception_tracking::PerceptionConfigMaterializer::VisualizerInput viz_input;
    viz_input.enable_preview = this->get_parameter("visualization.enable").as_bool();
    viz_input.enable_recording = this->get_parameter("recording.enable").as_bool();
    viz_input.recording_output_root = this->get_parameter("recording.output_root").as_string();
    viz_input.recording_path = this->get_parameter("recording.path").as_string();
    viz_input.recording_fps = this->get_parameter("recording.fps").as_double();
    viz_input.queue_capacity = static_cast<int>(this->get_parameter("visualization.queue_capacity").as_int());
    const auto viz_cfg =
        dog_patrol_perception_tracking::PerceptionConfigMaterializer::MaterializeVisualizerConfig(viz_input, sid_cfg);
    visualizer_ = std::make_unique<dog_patrol_perception_tracking::VisualizerRecorder>(viz_cfg);
    if (!visualizer_->Initialize(frame.size(), &error)) {
      throw std::runtime_error("visualizer_recorder init failed: " + error);
    }

    LogEffectiveConfig(camera_cfg, acquired_frame, infer_cfg, filter_cfg,
                       tracker_.EffectiveConfig(), target_cfg, sid_cfg, viz_cfg);
  }

  void LogEffectiveConfig(const dog_patrol_perception_tracking::CameraIngest::Config &camera_cfg,
                          const dog_patrol_perception_tracking::CameraIngest::AcquiredFrame &acquired_frame,
                          const dog_patrol_perception_tracking::PreprocessInfer::Config &infer_cfg,
                          const dog_patrol_perception_tracking::DetFilter::Config &filter_cfg,
                          const dog_patrol_perception_tracking::MotTracker::Config &tracker_cfg,
                          const dog_patrol_perception_tracking::PrimaryTargetManager::Config &target_cfg,
                          const dog_patrol_perception_tracking::IdentityManager::Config &identity_cfg,
                          const dog_patrol_perception_tracking::VisualizerRecorder::Config &viz_cfg) {
    RCLCPP_INFO(get_logger(),
                "startup_effective_config camera backend=hik_mvs requested_size=%dx%d requested_fps=%.2f timeout_ms=%d mvs_model=%s mvs_serial=%s bayer_interpolation=%s bayer_smoothing=%s conversion_target=BGR8",
                camera_cfg.width, camera_cfg.height, camera_cfg.fps, camera_cfg.timeout_ms,
                camera_cfg.hik_mvs_model.c_str(), camera_cfg.hik_mvs_serial.c_str(),
                dog_patrol_perception_tracking::CameraIngest::BayerInterpolationName(
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
                dog_patrol_perception_tracking::VisualizerRecorder::ModeName(viz_cfg).c_str(), BoolStr(viz_cfg.enable_preview),
                BoolStr(viz_cfg.enable_recording), viz_cfg.recording_output_root.c_str(),
                viz_cfg.recording_path.c_str(), viz_cfg.recording_fps, viz_cfg.queue_capacity);
  }

  void LogOverlayMetrics(const dog_patrol_perception_tracking::VisualizerRecorder::MetricsSnapshot &overlay_metrics,
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

  dog_patrol_perception_tracking::SourceFrameMetadata SourceMetadata(
      const dog_patrol_perception_tracking::CameraIngest::AcquiredFrame &frame) const {
    dog_patrol_perception_tracking::SourceFrameMetadata metadata;
    metadata.source_timestamp_ns = frame.source_timestamp_ns;
    metadata.camera_frame_number = frame.camera_frame_number;
    metadata.camera_frame_number_available = frame.camera_frame_number_available;
    metadata.image_width = frame.width;
    metadata.image_height = frame.height;
    metadata.optical_frame_id = camera_optical_frame_id_;
    return metadata;
  }

  void Tick() {
    // A MultiThreadedExecutor must not run two inference frames at once. ROS
    // mission callbacks only copy a validated snapshot under their own mutex;
    // the complete camera/detector/tracker/identity/coordinator chain stays
    // serialized here.
    std::lock_guard<std::mutex> pipeline_lock(pipeline_mutex_);
    if (observation_lifecycle_ != nullptr) {
      observation_lifecycle_->BeforeFrame();
    }
    PublishCapabilityStatus();
    if (!runtime_operational_) {
      return;
    }

    std::string error;
    dog_patrol_perception_tracking::CameraIngest::AcquiredFrame acquired_frame;
    if (!camera_.Read(&acquired_frame, &error)) {
      ReportDetectionTrackingRuntimeStatus(
          {true, true, "detection/tracking source frame failed: " + error});
      PublishCapabilityStatus();
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "camera_ingest read failed: %s",
                           error.c_str());
      return;
    }
    cv::Mat &frame = acquired_frame.bgr8;

    std::vector<dog_patrol_perception_tracking::Detection> detections;
    std::vector<dog_patrol_perception_tracking::Detection> filtered;
    std::vector<dog_patrol_perception_tracking::Track> tracks;
    try {
      detections = infer_.Infer(frame);
      filtered = det_filter_.Filter(detections);
      tracks = tracker_.Update(filtered, frame);
    } catch (const std::exception &exception) {
      ReportDetectionTrackingRuntimeStatus(
          {true, true, "detection/tracking frame processing failed: " + std::string(exception.what())});
      PublishCapabilityStatus();
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 2000,
                            "detection/tracking frame processing failed: %s", exception.what());
      return;
    }
    ReportDetectionTrackingRuntimeStatus({true, true, {}});

    const auto primary_prev = CurrentPrimary();
    auto identity_result = identity_manager_.Update(
        dog_patrol_perception_tracking::TrackletObservationsFromTracks(tracks), tracker_.LastTrackletHypotheses(), primary_prev,
        &frame);
    const auto source_metadata = SourceMetadata(acquired_frame);
    auto frame_output = runtime_->ProcessFrame(
        identity_result.identities, source_metadata, frame);
    auto &primary = frame_output.primary;

    if (monitor_.ShouldReport()) {
      const int primary_id = primary.primary_target_id;
      const auto camera_metrics = camera_.Metrics();
      RCLCPP_INFO(get_logger(),
                  "runtime_monitor mode=%s fps=%.2f state=%s primary_id=%d raw_track_id=%d observation_current=%s det=%zu filtered=%zu tracks=%zu",
                  runtime_->Name(),
                  monitor_.CurrentFps(), dog_patrol_perception_tracking::PrimaryStateToString(primary.state).c_str(),
                  primary_id, primary.raw_track_id,
                  BoolStr(observation_lifecycle_ != nullptr &&
                          observation_lifecycle_->Current()),
                  detections.size(), filtered.size(), tracks.size());
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
      if (observation_lifecycle_ != nullptr) {
        const auto image_metrics = observation_lifecycle_->Metrics();
        RCLCPP_INFO(
            get_logger(),
            "target_image_metrics submitted=%llu published=%llu queue_drops=%llu rate_limited=%llu",
            static_cast<unsigned long long>(image_metrics.submitted),
            static_cast<unsigned long long>(image_metrics.published),
            static_cast<unsigned long long>(image_metrics.queue_dropped),
            static_cast<unsigned long long>(image_metrics.rate_limited));
      }
      if (visualizer_ != nullptr) {
        LogOverlayMetrics(visualizer_->Metrics(), "runtime");
      }
    }

    if (visualizer_ != nullptr) {
      visualizer_->Submit(std::move(acquired_frame.bgr8), std::move(tracks), std::move(primary),
                          std::move(identity_result),
                          std::move(frame_output.primary_decision_reason),
                          std::move(frame_output.primary_reject_reason));
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;

  dog_patrol_perception_tracking::CameraIngest camera_;
  dog_patrol_perception_tracking::PreprocessInfer infer_;
  dog_patrol_perception_tracking::DetFilter det_filter_;
  dog_patrol_perception_tracking::MotTracker tracker_;
  std::unique_ptr<TrackingRuntime> runtime_;
  DetectionTrackingStatusSink *detection_tracking_status_sink_{nullptr};
  ObservationLifecycle *observation_lifecycle_{nullptr};
  rclcpp::CallbackGroup::SharedPtr mission_state_callback_group_;
  std::string camera_optical_frame_id_;
  std::mutex pipeline_mutex_;
  dog_patrol_perception_tracking::IdentityManager identity_manager_;
  std::unique_ptr<dog_patrol_perception_tracking::VisualizerRecorder> visualizer_;
  dog_patrol_perception_tracking::RuntimeMonitor monitor_;
  bool runtime_operational_{false};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<PerceptionTrackingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
    executor.add_node(node);
    executor.spin();
    node->ShutdownAndLog();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("dog_patrol_perception_tracking_node"), "Fatal init/runtime error: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
