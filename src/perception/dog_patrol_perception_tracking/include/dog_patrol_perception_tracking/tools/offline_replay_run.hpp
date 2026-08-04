#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace dog_patrol_perception_tracking::tools {

class OfflineReplayRun {
 public:
  struct DetectorConfig {
    std::string engine_path;
    float raw_conf_threshold{0.10F};
    float person_conf_threshold{0.10F};
    float car_conf_threshold{0.10F};
  };

  struct TrackerConfig {
    TrackerConfig();

    std::string config_path;
    bool gmc_enabled;
    std::string reid_backend;
    std::string reid_model_path;
    int reid_input_width;
    int reid_input_height;
  };

  struct IdentityConfig {
    IdentityConfig();

    int target_lost_threshold_frames;
    int feat_bank_size;
    float recover_sim_thresh_strict;
    float recover_sim_thresh_relaxed;
    int recover_relaxed_max_missing_frames;
    int occlusion_protect_frames;
    float missing_assign_min_area_ratio;
    float missing_assign_max_area_ratio;
    float missing_assign_max_center_dist_norm;
    float missing_assign_max_app_cost;
    float overlap_iou_freeze;
    int split_stable_frames;
    int merge_hold_frames;
    float app_w;
    float geo_w;
    float time_w;
    float active_assign_max_cost;
    float recovery_max_cost;
    float raw_continuity_max_cost;
    float min_assignment_margin;
    int stable_frames_before_feature_update;
    bool merged_requires_overlap;
    bool reid_enable;
    std::string reid_backend;
    std::string reid_model_path;
    int reid_input_width;
    int reid_input_height;
  };

  struct OutputConfig {
    bool save_frame_csv{true};
    bool save_sid_scores{true};
    bool save_tracks_csv{true};
    bool overlay_preview{false};
    bool overlay_record{false};
    bool short_dataset_dir_names{true};
    std::string overlay_video_name{"eval_overlay.mkv"};
  };

  struct Request {
    std::filesystem::path recordings_root{"data/captures"};
    std::filesystem::path results_root{"data/eval_results"};
    std::string run_name{"oe"};
    std::vector<std::string> datasets;
    std::filesystem::path explicit_video_path;
    DetectorConfig detector;
    TrackerConfig tracker;
    IdentityConfig identity;
    OutputConfig output;
  };

  struct Result {
    int exit_code{1};
    bool all_ok{false};
    std::filesystem::path run_dir;
  };

  static Result Run(const Request &request);
};

}  // namespace dog_patrol_perception_tracking::tools
