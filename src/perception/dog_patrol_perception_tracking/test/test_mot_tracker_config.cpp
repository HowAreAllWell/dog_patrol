#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision_demo_host/modules/det_filter.hpp"
#include "vision_demo_host/modules/mot_tracker.hpp"

namespace {

std::filesystem::path WriteTrackerConfig(const std::string &name, const std::string &body) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream ofs(path);
  ofs << body;
  return path;
}

vision_demo_host::MotTracker::Config BaseTrackerConfig(const std::filesystem::path &path) {
  vision_demo_host::MotTracker::Config cfg;
  cfg.tracker_yaml_path = path.string();
  cfg.gmc_enabled = false;
  cfg.reid_backend = "light";
  cfg.reid_model_path.clear();
  return cfg;
}

vision_demo_host::Detection PersonDet(const float confidence, const cv::Rect2f &bbox) {
  vision_demo_host::Detection det;
  det.class_id = vision_demo_host::ClassId::kPerson;
  det.confidence = confidence;
  det.bbox = bbox;
  return det;
}

}  // namespace

TEST(MotTrackerConfigTest, GmcDownscaleAndStageCostConfigParse) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_config_parse.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "gmc_downscale: 7\n"
      "stage1_max_cost: 0.42\n"
      "stage2_max_cost: 0.33\n"
      "lost_recovery_max_cost: 0.22\n"
      "unconfirmed_max_cost: 0.11\n"
      "use_low_score_appearance_gate: false\n"
      "duplicate_lost_iou: 0.61\n"
      "duplicate_lost_center_dist_norm: 1.25\n");

  vision_demo_host::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const auto &cfg = tracker.EffectiveConfig();
  EXPECT_EQ(cfg.gmc_downscale, 7);
  EXPECT_FLOAT_EQ(cfg.stage1_max_cost, 0.42F);
  EXPECT_FLOAT_EQ(cfg.stage2_max_cost, 0.33F);
  EXPECT_FLOAT_EQ(cfg.lost_recovery_max_cost, 0.22F);
  EXPECT_FLOAT_EQ(cfg.unconfirmed_max_cost, 0.11F);
  EXPECT_FALSE(cfg.use_low_score_appearance_gate);
  EXPECT_FLOAT_EQ(cfg.duplicate_lost_iou, 0.61F);
  EXPECT_FLOAT_EQ(cfg.duplicate_lost_center_dist_norm, 1.25F);
}

TEST(MotTrackerConfigTest, StageMaxCostRejectsHighCostSelectedMatch) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_stage_cost.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n"
      "stage1_iou_min: 0.01\n"
      "stage1_max_cost: 0.01\n"
      "stage2_max_cost: 0.80\n"
      "lost_recovery_max_cost: 0.80\n"
      "unconfirmed_max_cost: 0.80\n"
      "assoc_iou_weight: 1.0\n"
      "assoc_motion_weight: 0.0\n"
      "assoc_app_weight: 0.0\n");

  vision_demo_host::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(160, 160, CV_8UC3, cv::Scalar(0, 0, 0));
  auto first = tracker.Update({PersonDet(0.90F, cv::Rect2f(10, 10, 50, 50))}, frame);
  ASSERT_EQ(first.size(), 1U);
  ASSERT_TRUE(first.front().is_confirmed);

  auto second = tracker.Update({PersonDet(0.90F, cv::Rect2f(12, 10, 50, 50))}, frame);
  EXPECT_TRUE(second.empty());
}

TEST(MotTrackerConfigTest, LowScoreDetectionCanEnterStage2) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_low_score.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n"
      "stage1_iou_min: 0.20\n"
      "stage2_iou_min: 0.10\n"
      "stage1_max_cost: 0.80\n"
      "stage2_max_cost: 0.80\n"
      "lost_recovery_max_cost: 0.80\n"
      "unconfirmed_max_cost: 0.80\n"
      "use_low_score_appearance_gate: true\n");

  vision_demo_host::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(160, 160, CV_8UC3, cv::Scalar(32, 64, 96));
  auto first = tracker.Update({PersonDet(0.90F, cv::Rect2f(10, 10, 50, 50))}, frame);
  ASSERT_EQ(first.size(), 1U);

  auto second = tracker.Update({PersonDet(0.20F, cv::Rect2f(12, 10, 50, 50))}, frame);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second.front().id, first.front().id);
  EXPECT_TRUE(second.front().low_score_update);
  EXPECT_TRUE(second.front().association.low_score_detection);
  EXPECT_EQ(second.front().association.stage, "stage2_confirmed_low");
}

TEST(DetFilterTest, DefaultsPreserveLowScoreDetections) {
  vision_demo_host::DetFilter filter(vision_demo_host::DetFilter::Config{});

  const std::vector<vision_demo_host::Detection> input{
      PersonDet(0.09F, cv::Rect2f(0, 0, 10, 10)),
      PersonDet(0.10F, cv::Rect2f(0, 0, 10, 10)),
      vision_demo_host::Detection{vision_demo_host::ClassId::kCar, 0.10F, cv::Rect2f(0, 0, 10, 10)},
      vision_demo_host::Detection{vision_demo_host::ClassId::kUnknown, 0.99F, cv::Rect2f(0, 0, 10, 10)},
  };

  const auto filtered = filter.Filter(input);
  ASSERT_EQ(filtered.size(), 2U);
  EXPECT_EQ(filtered[0].class_id, vision_demo_host::ClassId::kPerson);
  EXPECT_FLOAT_EQ(filtered[0].confidence, 0.10F);
  EXPECT_EQ(filtered[1].class_id, vision_demo_host::ClassId::kCar);
  EXPECT_FLOAT_EQ(filtered[1].confidence, 0.10F);
}
