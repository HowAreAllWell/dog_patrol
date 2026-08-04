#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"
#include "dog_patrol_perception_tracking/modules/mot_tracker.hpp"
#include "dog_patrol_perception_tracking/modules/visualizer_recorder.hpp"

namespace dog_patrol_perception_tracking {

class PerceptionConfigMaterializer {
 public:
  static constexpr const char *kDefaultReidBackend{"light"};
  static constexpr int kDefaultReidInputWidth{128};
  static constexpr int kDefaultReidInputHeight{256};
  static constexpr int kMinReidInputSize{16};

  struct TrackerInput {
    std::string config_path;
    bool gmc_enabled{MotTracker::Config::kDefaultGmcEnabled};
    bool reid_enabled{true};
    float track_high_thresh{0.5F};
    float track_low_thresh{0.1F};
    float new_track_thresh{0.7F};
    float match_thresh{0.8F};
    int track_buffer{30};
    std::string gmc_method{"sparseOptFlow"};
    int gmc_downscale{4};
    bool with_reid{true};
    std::string reid_backend{kDefaultReidBackend};
    std::string reid_model_path;
    int reid_input_width{kDefaultReidInputWidth};
    int reid_input_height{kDefaultReidInputHeight};
  };

  struct IdentityInput {
    int target_lost_threshold_frames{180};
    int feat_bank_size{30};
    float recover_sim_thresh_strict{0.85F};
    float recover_sim_thresh_relaxed{0.75F};
    int recover_relaxed_max_missing_frames{180};
    int occlusion_protect_frames{30};
    float missing_assign_min_area_ratio{0.40F};
    float missing_assign_max_area_ratio{4.00F};
    float missing_assign_max_center_dist_norm{2.50F};
    float missing_assign_max_app_cost{0.50F};
    float overlap_iou_freeze{0.10F};
    int split_stable_frames{3};
    int merge_hold_frames{2};
    float app_w{0.70F};
    float geo_w{0.20F};
    float time_w{0.10F};
    float active_assign_max_cost{0.55F};
    float recovery_max_cost{0.45F};
    float raw_continuity_max_cost{0.55F};
    float min_assignment_margin{0.08F};
    int stable_frames_before_feature_update{3};
    bool merged_requires_overlap{true};
    bool reid_enable{true};
    std::string reid_backend{kDefaultReidBackend};
    std::string reid_model_path;
    int reid_input_width{kDefaultReidInputWidth};
    int reid_input_height{kDefaultReidInputHeight};
  };

  struct VisualizerInput {
    bool enable_preview{false};
    bool enable_recording{false};
    std::string recording_output_root{"data/diagnostics/live_overlays"};
    std::string recording_path{"data/diagnostics/live_overlays/vision_demo_overlay.mkv"};
    std::vector<std::string> protected_source_dataset_roots{"data/captures"};
    double recording_fps{30.0};
    int queue_capacity{4};
  };

  struct Diagnostics {
    bool tracker_reid_forced{false};
    bool identity_reid_forced{false};
  };

  static MotTracker::Config MaterializeTrackerConfig(const TrackerInput &input,
                                                     Diagnostics *diagnostics = nullptr);
  static IdentityManager::Config MaterializeIdentityConfig(const IdentityInput &input,
                                                          Diagnostics *diagnostics = nullptr);
  static VisualizerRecorder::Config MaterializeVisualizerConfig(
      const VisualizerInput &input,
      const IdentityManager::Config &identity_config);
};

}  // namespace dog_patrol_perception_tracking
