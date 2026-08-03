#include "vision_demo_host/modules/perception_config_materializer.hpp"

#include <algorithm>

namespace vision_demo_host {

MotTracker::Config PerceptionConfigMaterializer::MaterializeTrackerConfig(
    const TrackerInput &input,
    Diagnostics *diagnostics) {
  MotTracker::Config config;
  config.tracker_yaml_path = input.config_path;
  config.gmc_enabled = input.gmc_enabled;
  config.reid_enabled = true;
  config.track_high_thresh = input.track_high_thresh;
  config.track_low_thresh = input.track_low_thresh;
  config.new_track_thresh = input.new_track_thresh;
  config.match_thresh = input.match_thresh;
  config.track_buffer = input.track_buffer;
  config.gmc_method = input.gmc_method;
  config.gmc_downscale = std::max(1, input.gmc_downscale);
  config.with_reid = true;
  config.reid_backend = input.reid_backend;
  config.reid_model_path = input.reid_model_path;
  config.reid_input_width = std::max(kMinReidInputSize, input.reid_input_width);
  config.reid_input_height = std::max(kMinReidInputSize, input.reid_input_height);

  if (diagnostics != nullptr) {
    diagnostics->tracker_reid_forced = !input.reid_enabled || !input.with_reid;
  }
  return config;
}

IdentityManager::Config PerceptionConfigMaterializer::MaterializeIdentityConfig(
    const IdentityInput &input,
    Diagnostics *diagnostics) {
  IdentityManager::Config config;
  config.max_missing_frames = std::max(1, input.target_lost_threshold_frames);
  config.feat_bank_size = std::max(1, input.feat_bank_size);
  config.recover_sim_thresh_strict = std::clamp(input.recover_sim_thresh_strict, 0.0F, 1.0F);
  config.recover_sim_thresh_relaxed = std::clamp(input.recover_sim_thresh_relaxed, 0.0F, 1.0F);
  config.recover_relaxed_max_missing_frames = std::max(1, input.recover_relaxed_max_missing_frames);
  config.occlusion_protect_frames = std::max(0, input.occlusion_protect_frames);
  config.missing_assign_min_area_ratio = std::max(0.01F, input.missing_assign_min_area_ratio);
  config.missing_assign_max_area_ratio =
      std::max(config.missing_assign_min_area_ratio, input.missing_assign_max_area_ratio);
  config.missing_assign_max_center_dist_norm = std::max(0.1F, input.missing_assign_max_center_dist_norm);
  config.missing_assign_max_app_cost = std::clamp(input.missing_assign_max_app_cost, 0.0F, 1.0F);
  config.overlap_iou_freeze = std::max(0.0F, input.overlap_iou_freeze);
  config.split_stable_frames = std::max(1, input.split_stable_frames);
  config.merge_hold_frames = std::max(1, input.merge_hold_frames);
  config.app_w = std::max(0.0F, input.app_w);
  config.geo_w = std::max(0.0F, input.geo_w);
  config.time_w = std::max(0.0F, input.time_w);
  config.active_assign_max_cost = std::clamp(input.active_assign_max_cost, 0.0F, 1.0F);
  config.recovery_max_cost = std::clamp(input.recovery_max_cost, 0.0F, 1.0F);
  config.raw_continuity_max_cost = std::clamp(input.raw_continuity_max_cost, 0.0F, 1.0F);
  config.min_assignment_margin = std::max(0.0F, input.min_assignment_margin);
  config.stable_frames_before_feature_update = std::max(1, input.stable_frames_before_feature_update);
  config.merged_requires_overlap = input.merged_requires_overlap;
  config.reid_enable = true;
  config.reid_backend = input.reid_backend;
  config.reid_model_path = input.reid_model_path;
  config.reid_input_width = std::max(kMinReidInputSize, input.reid_input_width);
  config.reid_input_height = std::max(kMinReidInputSize, input.reid_input_height);

  if (diagnostics != nullptr) {
    diagnostics->identity_reid_forced = !input.reid_enable;
  }
  return config;
}

VisualizerRecorder::Config PerceptionConfigMaterializer::MaterializeVisualizerConfig(
    const VisualizerInput &input,
    const IdentityManager::Config &identity_config) {
  VisualizerRecorder::Config config;
  config.enable_preview = input.enable_preview;
  config.enable_recording = input.enable_recording;
  config.recording_output_root = input.recording_output_root;
  config.recording_path = input.recording_path;
  config.protected_source_dataset_roots = input.protected_source_dataset_roots;
  config.recording_fps = input.recording_fps;
  config.queue_capacity = input.queue_capacity > 0 ? static_cast<std::size_t>(input.queue_capacity) : 0U;
  config.semantic_id_max_missing_frames = identity_config.max_missing_frames;
  config.sid_feat_bank_size = identity_config.feat_bank_size;
  config.sid_recover_sim_thresh_strict = identity_config.recover_sim_thresh_strict;
  config.sid_recover_sim_thresh_relaxed = identity_config.recover_sim_thresh_relaxed;
  config.sid_recover_relaxed_max_missing_frames = identity_config.recover_relaxed_max_missing_frames;
  config.sid_occlusion_protect_frames = identity_config.occlusion_protect_frames;
  config.sid_overlap_iou_freeze = identity_config.overlap_iou_freeze;
  config.sid_split_stable_frames = identity_config.split_stable_frames;
  config.sid_merge_hold_frames = identity_config.merge_hold_frames;
  config.sid_app_w = identity_config.app_w;
  config.sid_geo_w = identity_config.geo_w;
  config.sid_time_w = identity_config.time_w;
  config.sid_active_assign_max_cost = identity_config.active_assign_max_cost;
  config.sid_recovery_max_cost = identity_config.recovery_max_cost;
  config.sid_raw_continuity_max_cost = identity_config.raw_continuity_max_cost;
  config.sid_min_assignment_margin = identity_config.min_assignment_margin;
  config.sid_stable_frames_before_feature_update = identity_config.stable_frames_before_feature_update;
  config.sid_merged_requires_overlap = identity_config.merged_requires_overlap;
  config.sid_reid_enable = identity_config.reid_enable;
  config.sid_reid_backend = identity_config.reid_backend;
  config.sid_reid_model_path = identity_config.reid_model_path;
  config.sid_reid_input_width = identity_config.reid_input_width;
  config.sid_reid_input_height = identity_config.reid_input_height;
  return config;
}

}  // namespace vision_demo_host
