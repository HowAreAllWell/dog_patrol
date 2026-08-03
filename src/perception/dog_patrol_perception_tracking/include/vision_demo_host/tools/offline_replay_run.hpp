#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace vision_demo_host::tools {

class OfflineReplayRun {
 public:
  struct DetectorConfig {
    std::string engine_path{
        "/path/to/my_workplace/vision_demo_ws/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine"};
    float raw_conf_threshold{0.10F};
    float person_conf_threshold{0.10F};
    float car_conf_threshold{0.10F};
  };

  struct TrackerConfig {
    std::string config_path{"/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/config/bot_sort.yaml"};
    bool gmc_enabled{false};
    std::string reid_backend{"light"};
    std::string reid_model_path{};
    int reid_input_width{128};
    int reid_input_height{256};
  };

  struct IdentityConfig {
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
    std::string reid_backend{"light"};
    std::string reid_model_path{};
    int reid_input_width{128};
    int reid_input_height{256};
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

}  // namespace vision_demo_host::tools
