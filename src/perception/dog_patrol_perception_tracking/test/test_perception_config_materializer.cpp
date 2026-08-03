#include <gtest/gtest.h>

#include <string>

#include "vision_demo_host/modules/perception_config_materializer.hpp"

namespace {

using vision_demo_host::PerceptionConfigMaterializer;

}  // namespace

TEST(PerceptionConfigMaterializerTest, TrackerMaterializationForcesReidAndClampsDimensions) {
  PerceptionConfigMaterializer::TrackerInput input;
  input.config_path = "/tmp/tracker.yaml";
  input.gmc_enabled = true;
  input.reid_enabled = false;
  input.with_reid = false;
  input.gmc_downscale = 0;
  input.reid_backend = "osnet_onnx";
  input.reid_model_path = "/tmp/reid.onnx";
  input.reid_input_width = 8;
  input.reid_input_height = -1;

  PerceptionConfigMaterializer::Diagnostics diagnostics;
  const auto config = PerceptionConfigMaterializer::MaterializeTrackerConfig(input, &diagnostics);

  EXPECT_EQ(config.tracker_yaml_path, "/tmp/tracker.yaml");
  EXPECT_TRUE(config.gmc_enabled);
  EXPECT_EQ(config.gmc_downscale, 1);
  EXPECT_TRUE(config.reid_enabled);
  EXPECT_TRUE(config.with_reid);
  EXPECT_EQ(config.reid_backend, "osnet_onnx");
  EXPECT_EQ(config.reid_model_path, "/tmp/reid.onnx");
  EXPECT_EQ(config.reid_input_width, PerceptionConfigMaterializer::kMinReidInputSize);
  EXPECT_EQ(config.reid_input_height, PerceptionConfigMaterializer::kMinReidInputSize);
  EXPECT_TRUE(diagnostics.tracker_reid_forced);
}

TEST(PerceptionConfigMaterializerTest, IdentityMaterializationOwnsValidationPolicy) {
  PerceptionConfigMaterializer::IdentityInput input;
  input.target_lost_threshold_frames = 0;
  input.feat_bank_size = 0;
  input.recover_sim_thresh_strict = 2.0F;
  input.recover_sim_thresh_relaxed = -1.0F;
  input.recover_relaxed_max_missing_frames = -4;
  input.occlusion_protect_frames = -3;
  input.missing_assign_min_area_ratio = -2.0F;
  input.missing_assign_max_area_ratio = 0.001F;
  input.missing_assign_max_center_dist_norm = -1.0F;
  input.missing_assign_max_app_cost = 3.0F;
  input.overlap_iou_freeze = -1.0F;
  input.split_stable_frames = 0;
  input.merge_hold_frames = 0;
  input.app_w = -1.0F;
  input.geo_w = -2.0F;
  input.time_w = -3.0F;
  input.active_assign_max_cost = 9.0F;
  input.recovery_max_cost = -1.0F;
  input.raw_continuity_max_cost = 9.0F;
  input.min_assignment_margin = -1.0F;
  input.stable_frames_before_feature_update = -1;
  input.reid_enable = false;
  input.reid_backend = "osnet_onnx";
  input.reid_model_path = "/tmp/sid.onnx";
  input.reid_input_width = 4;
  input.reid_input_height = 5;

  PerceptionConfigMaterializer::Diagnostics diagnostics;
  const auto config = PerceptionConfigMaterializer::MaterializeIdentityConfig(input, &diagnostics);

  EXPECT_EQ(config.max_missing_frames, 1);
  EXPECT_EQ(config.feat_bank_size, 1);
  EXPECT_FLOAT_EQ(config.recover_sim_thresh_strict, 1.0F);
  EXPECT_FLOAT_EQ(config.recover_sim_thresh_relaxed, 0.0F);
  EXPECT_EQ(config.recover_relaxed_max_missing_frames, 1);
  EXPECT_EQ(config.occlusion_protect_frames, 0);
  EXPECT_FLOAT_EQ(config.missing_assign_min_area_ratio, 0.01F);
  EXPECT_FLOAT_EQ(config.missing_assign_max_area_ratio, 0.01F);
  EXPECT_FLOAT_EQ(config.missing_assign_max_center_dist_norm, 0.1F);
  EXPECT_FLOAT_EQ(config.missing_assign_max_app_cost, 1.0F);
  EXPECT_FLOAT_EQ(config.overlap_iou_freeze, 0.0F);
  EXPECT_EQ(config.split_stable_frames, 1);
  EXPECT_EQ(config.merge_hold_frames, 1);
  EXPECT_FLOAT_EQ(config.app_w, 0.0F);
  EXPECT_FLOAT_EQ(config.geo_w, 0.0F);
  EXPECT_FLOAT_EQ(config.time_w, 0.0F);
  EXPECT_FLOAT_EQ(config.active_assign_max_cost, 1.0F);
  EXPECT_FLOAT_EQ(config.recovery_max_cost, 0.0F);
  EXPECT_FLOAT_EQ(config.raw_continuity_max_cost, 1.0F);
  EXPECT_FLOAT_EQ(config.min_assignment_margin, 0.0F);
  EXPECT_EQ(config.stable_frames_before_feature_update, 1);
  EXPECT_TRUE(config.reid_enable);
  EXPECT_EQ(config.reid_backend, "osnet_onnx");
  EXPECT_EQ(config.reid_model_path, "/tmp/sid.onnx");
  EXPECT_EQ(config.reid_input_width, PerceptionConfigMaterializer::kMinReidInputSize);
  EXPECT_EQ(config.reid_input_height, PerceptionConfigMaterializer::kMinReidInputSize);
  EXPECT_TRUE(diagnostics.identity_reid_forced);
}

TEST(PerceptionConfigMaterializerTest, VisualizerMirrorsSanitizedIdentityConfig) {
  PerceptionConfigMaterializer::IdentityInput identity_input;
  identity_input.target_lost_threshold_frames = -10;
  identity_input.feat_bank_size = -2;
  identity_input.reid_enable = false;
  identity_input.reid_input_width = 1;
  identity_input.reid_input_height = 2;
  const auto identity_config = PerceptionConfigMaterializer::MaterializeIdentityConfig(identity_input);

  PerceptionConfigMaterializer::VisualizerInput visualizer_input;
  visualizer_input.enable_preview = true;
  visualizer_input.enable_recording = true;
  visualizer_input.queue_capacity = -4;
  visualizer_input.recording_output_root = "/tmp/out";
  visualizer_input.recording_path = "/tmp/out/overlay.mkv";

  const auto config =
      PerceptionConfigMaterializer::MaterializeVisualizerConfig(visualizer_input, identity_config);

  EXPECT_TRUE(config.enable_preview);
  EXPECT_TRUE(config.enable_recording);
  EXPECT_EQ(config.queue_capacity, 0U);
  EXPECT_EQ(config.recording_output_root, "/tmp/out");
  EXPECT_EQ(config.recording_path, "/tmp/out/overlay.mkv");
  EXPECT_EQ(config.semantic_id_max_missing_frames, 1);
  EXPECT_EQ(config.sid_feat_bank_size, 1);
  EXPECT_TRUE(config.sid_reid_enable);
  EXPECT_EQ(config.sid_reid_input_width, PerceptionConfigMaterializer::kMinReidInputSize);
  EXPECT_EQ(config.sid_reid_input_height, PerceptionConfigMaterializer::kMinReidInputSize);
}
