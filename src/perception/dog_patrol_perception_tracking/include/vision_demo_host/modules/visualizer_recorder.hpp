#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class VisualizerRecorder {
 public:
  struct Config {
    bool enable_visualization{false};
    bool enable_recording{false};
    std::string recording_path{"/tmp/vision_demo_out.mp4"};
    double recording_fps{15.0};
    int semantic_id_max_missing_frames{180};
    int sid_feat_bank_size{30};
    float sid_recover_sim_thresh_strict{0.85F};
    float sid_overlap_iou_freeze{0.10F};
    int sid_occlusion_protect_frames{30};
    int sid_split_stable_frames{3};
    int sid_merge_hold_frames{2};
    float sid_app_w{0.70F};
    float sid_geo_w{0.20F};
    float sid_time_w{0.10F};
    float sid_active_assign_max_cost{0.55F};
    float sid_recovery_max_cost{0.45F};
    float sid_raw_continuity_max_cost{0.55F};
    float sid_min_assignment_margin{0.08F};
    int sid_stable_frames_before_feature_update{3};
    bool sid_merged_requires_overlap{true};
    bool sid_reid_enable{true};
    std::string sid_reid_backend{"light"};
    std::string sid_reid_model_path{};
    float sid_recover_sim_thresh_relaxed{0.75F};
    int sid_recover_relaxed_max_missing_frames{180};
    int sid_reid_input_width{128};
    int sid_reid_input_height{256};
  };

  explicit VisualizerRecorder(Config config);

  bool Initialize(const cv::Size &frame_size, std::string *error);
  void Render(const cv::Mat &frame, const std::vector<Track> &tracks,
              const PrimaryTargetResult &primary,
              const IdentityManagerResult *identity_result,
              const std::string &primary_decision_reason = {},
              const std::string &primary_reject_reason = {});

 private:
  Config config_;
  cv::VideoWriter writer_;
};

}  // namespace vision_demo_host
