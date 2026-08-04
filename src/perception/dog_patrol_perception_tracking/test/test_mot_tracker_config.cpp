#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/det_filter.hpp"
#include "dog_patrol_perception_tracking/modules/mot_tracker.hpp"

namespace {

std::filesystem::path WriteTrackerConfig(const std::string &name, const std::string &body) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream ofs(path);
  ofs << body;
  return path;
}

dog_patrol_perception_tracking::MotTracker::Config BaseTrackerConfig(const std::filesystem::path &path) {
  dog_patrol_perception_tracking::MotTracker::Config cfg;
  cfg.tracker_yaml_path = path.string();
  cfg.gmc_enabled = false;
  cfg.reid_backend = "light";
  cfg.reid_model_path.clear();
  return cfg;
}

dog_patrol_perception_tracking::Detection PersonDet(const float confidence, const cv::Rect2f &bbox) {
  dog_patrol_perception_tracking::Detection det;
  det.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  det.confidence = confidence;
  det.bbox = bbox;
  return det;
}

}  // namespace

TEST(MotTrackerConfigTest, DefaultConfigKeepsGmcDisabled) {
  const dog_patrol_perception_tracking::MotTracker::Config cfg;
  EXPECT_FALSE(cfg.gmc_enabled);
}

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

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
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

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
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

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
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

TEST(MotTrackerConfigTest, FinalTracksAreMirroredAsTrackedHypotheses) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_hypotheses.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(180, 220, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto tracks = tracker.Update(
      {PersonDet(0.90F, cv::Rect2f(20, 30, 50, 80)), PersonDet(0.85F, cv::Rect2f(130, 35, 45, 75))}, frame);
  ASSERT_EQ(tracks.size(), 2U);

  const auto &hypotheses = tracker.LastTrackletHypotheses();
  ASSERT_EQ(hypotheses.size(), tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    EXPECT_EQ(hypotheses[i].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kTracked);
    EXPECT_EQ(hypotheses[i].raw_track_id, tracks[i].id);
    EXPECT_EQ(hypotheses[i].class_id, tracks[i].class_id);
    EXPECT_FLOAT_EQ(hypotheses[i].confidence, tracks[i].confidence);
    EXPECT_EQ(hypotheses[i].bbox, tracks[i].bbox);
    EXPECT_EQ(hypotheses[i].candidate_reason, "final_track_output");
    EXPECT_FALSE(hypotheses[i].related_raw_track_id.has_value());
    EXPECT_EQ(hypotheses[i].association.stage, tracks[i].association.stage);
  }
}

TEST(MotTrackerConfigTest, SuppressedNewTrackDuplicateIsRecordedAsHypothesis) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_suppressed_candidate.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(220, 220, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto first = tracker.Update({PersonDet(0.90F, cv::Rect2f(40, 30, 60, 110))}, frame);
  ASSERT_EQ(first.size(), 1U);
  const int raw_id = first.front().id;

  const auto second = tracker.Update(
      {PersonDet(0.91F, cv::Rect2f(42, 30, 60, 110)), PersonDet(0.82F, cv::Rect2f(48, 35, 58, 105))}, frame);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second.front().id, raw_id);

  const auto &hypotheses = tracker.LastTrackletHypotheses();
  ASSERT_EQ(hypotheses.size(), 2U);
  EXPECT_EQ(hypotheses[0].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kTracked);
  EXPECT_EQ(hypotheses[0].raw_track_id, raw_id);
  EXPECT_EQ(hypotheses[1].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kSuppressedDuplicateCandidate);
  EXPECT_EQ(hypotheses[1].raw_track_id, -1);
  EXPECT_EQ(hypotheses[1].class_id, dog_patrol_perception_tracking::ClassId::kPerson);
  EXPECT_FLOAT_EQ(hypotheses[1].confidence, 0.82F);
  EXPECT_EQ(hypotheses[1].bbox, cv::Rect2f(48, 35, 58, 105));
  EXPECT_EQ(hypotheses[1].candidate_reason, "new_track_suppressed_duplicate_tracked");
  ASSERT_TRUE(hypotheses[1].related_raw_track_id.has_value());
  EXPECT_EQ(*hypotheses[1].related_raw_track_id, raw_id);
}

TEST(MotTrackerConfigTest, SuppressedNewTrackNearLostTrackIsRecordedAsHypothesis) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_suppressed_lost_candidate.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n"
      "stage1_iou_min: 0.20\n"
      "stage1_max_cost: 0.01\n"
      "lost_recovery_max_cost: 0.01\n"
      "assoc_iou_weight: 1.0\n"
      "assoc_motion_weight: 0.0\n"
      "assoc_app_weight: 0.0\n"
      "duplicate_lost_iou: 0.50\n"
      "duplicate_lost_center_dist_norm: 1.0\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(220, 220, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto first = tracker.Update({PersonDet(0.90F, cv::Rect2f(40, 30, 60, 110))}, frame);
  ASSERT_EQ(first.size(), 1U);
  const int raw_id = first.front().id;

  const auto lost_frame = tracker.Update({}, frame);
  ASSERT_TRUE(lost_frame.empty());

  const auto suppressed = tracker.Update({PersonDet(0.92F, cv::Rect2f(42, 30, 60, 110))}, frame);
  EXPECT_TRUE(suppressed.empty());

  const auto &hypotheses = tracker.LastTrackletHypotheses();
  ASSERT_EQ(hypotheses.size(), 1U);
  EXPECT_EQ(hypotheses[0].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kSuppressedDuplicateCandidate);
  EXPECT_EQ(hypotheses[0].candidate_reason, "new_track_suppressed_duplicate_lost");
  ASSERT_TRUE(hypotheses[0].related_raw_track_id.has_value());
  EXPECT_EQ(*hypotheses[0].related_raw_track_id, raw_id);
}

TEST(MotTrackerConfigTest, AgedLostTrackDoesNotSuppressSeparatedHighScoreDetectionByCenterOnly) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_lost_center_suppress.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "confirm_hits: 1\n"
      "track_buffer: 30\n"
      "stage1_iou_min: 0.20\n"
      "stage1_max_cost: 0.80\n"
      "lost_recovery_max_cost: 0.60\n"
      "assoc_iou_weight: 1.0\n"
      "assoc_motion_weight: 0.0\n"
      "assoc_app_weight: 0.0\n"
      "duplicate_lost_iou: 0.50\n"
      "duplicate_lost_center_dist_norm: 1.0\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(240, 240, CV_8UC3, cv::Scalar(32, 64, 96));
  const auto first = tracker.Update(
      {PersonDet(0.90F, cv::Rect2f(90, 30, 40, 100)), PersonDet(0.90F, cv::Rect2f(10, 30, 40, 100))},
      frame);
  ASSERT_EQ(first.size(), 2U);

  for (int i = 0; i < 8; ++i) {
    const auto only_other = tracker.Update({PersonDet(0.90F, cv::Rect2f(10, 30, 40, 100))}, frame);
    ASSERT_EQ(only_other.size(), 1U);
  }

  const auto separated = tracker.Update(
      {PersonDet(0.90F, cv::Rect2f(10, 30, 40, 100)), PersonDet(0.90F, cv::Rect2f(40, 10, 140, 140))},
      frame);
  ASSERT_EQ(separated.size(), 2U);
  bool saw_new_track = false;
  for (const auto &track : separated) {
    if (track.association.stage == "new_track_high") {
      saw_new_track = true;
    }
  }
  EXPECT_TRUE(saw_new_track);
}

TEST(MotTrackerConfigTest, SeparatedHighScoreDetectionSpawnsDuringSplit) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_split_spawn.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "core_mode: old_minimal\n"
      "confirm_hits: 1\n"
      "stage1_iou_min: 0.20\n"
      "stage1_max_cost: 0.95\n"
      "unconfirmed_max_cost: 0.95\n"
      "assoc_iou_weight: 1.0\n"
      "assoc_motion_weight: 0.0\n"
      "assoc_app_weight: 0.0\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(1024, 1280, CV_8UC3, cv::Scalar(32, 64, 96));
  const auto merged = tracker.Update({PersonDet(0.90F, cv::Rect2f(585.0F, 212.0F, 274.0F, 553.0F))}, frame);
  ASSERT_EQ(merged.size(), 1U);

  tracker.Update({PersonDet(0.84F, cv::Rect2f(638.5F, 245.75F, 161.0F, 499.25F))}, frame);

  const auto split = tracker.Update(
      {PersonDet(0.85F, cv::Rect2f(640.0F, 248.5F, 147.5F, 468.5F)),
       PersonDet(0.88F, cv::Rect2f(704.0F, 207.25F, 191.0F, 552.75F))},
      frame);
  ASSERT_EQ(split.size(), 2U);
  bool saw_different_track = false;
  for (const auto &track : split) {
    if (track.id != merged.front().id) {
      saw_different_track = true;
    }
  }
  EXPECT_TRUE(saw_different_track);
}

TEST(MotTrackerConfigTest, DuplicateOverlappedOutputsAreHiddenDuringOcclusion) {
  const auto yaml = WriteTrackerConfig(
      "vision_demo_tracker_duplicate_output.yaml",
      "track_high_thresh: 0.50\n"
      "track_low_thresh: 0.10\n"
      "new_track_thresh: 0.70\n"
      "core_mode: old_minimal\n"
      "confirm_hits: 1\n"
      "stage1_iou_min: 0.01\n"
      "stage1_max_cost: 0.95\n"
      "assoc_iou_weight: 1.0\n"
      "assoc_motion_weight: 0.0\n"
      "assoc_app_weight: 0.0\n");

  dog_patrol_perception_tracking::MotTracker tracker(BaseTrackerConfig(yaml));
  std::string error;
  ASSERT_TRUE(tracker.Initialize(&error)) << error;

  const cv::Mat frame(400, 400, CV_8UC3, cv::Scalar(32, 64, 96));
  const auto separated = tracker.Update(
      {PersonDet(0.90F, cv::Rect2f(20, 40, 140, 220)), PersonDet(0.86F, cv::Rect2f(180, 42, 115, 218))},
      frame);
  ASSERT_EQ(separated.size(), 2U);

  const auto closing = tracker.Update(
      {PersonDet(0.91F, cv::Rect2f(60, 40, 140, 220)), PersonDet(0.84F, cv::Rect2f(140, 42, 115, 218))},
      frame);
  ASSERT_EQ(closing.size(), 2U);

  const auto overlapped = tracker.Update(
      {PersonDet(0.92F, cv::Rect2f(90, 40, 140, 220)), PersonDet(0.80F, cv::Rect2f(115, 42, 115, 218))},
      frame);
  ASSERT_EQ(overlapped.size(), 1U);
  EXPECT_FLOAT_EQ(overlapped.front().confidence, 0.92F);

  const auto &hypotheses = tracker.LastTrackletHypotheses();
  ASSERT_EQ(hypotheses.size(), 2U);
  EXPECT_EQ(hypotheses[0].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kTracked);
  EXPECT_EQ(hypotheses[0].raw_track_id, overlapped.front().id);
  EXPECT_FALSE(hypotheses[0].related_raw_track_id.has_value());

  EXPECT_EQ(hypotheses[1].status, dog_patrol_perception_tracking::TrackletHypothesisStatus::kSuppressedDuplicateCandidate);
  EXPECT_NE(hypotheses[1].raw_track_id, overlapped.front().id);
  EXPECT_GT(hypotheses[1].raw_track_id, 0);
  EXPECT_EQ(hypotheses[1].class_id, dog_patrol_perception_tracking::ClassId::kPerson);
  EXPECT_FLOAT_EQ(hypotheses[1].confidence, 0.80F);
  EXPECT_EQ(hypotheses[1].bbox, cv::Rect2f(115, 42, 115, 218));
  EXPECT_EQ(hypotheses[1].candidate_reason, "duplicate_output_hidden");
  ASSERT_TRUE(hypotheses[1].related_raw_track_id.has_value());
  EXPECT_EQ(*hypotheses[1].related_raw_track_id, overlapped.front().id);
}

TEST(DetFilterTest, DefaultsPreserveLowScoreDetections) {
  dog_patrol_perception_tracking::DetFilter filter(dog_patrol_perception_tracking::DetFilter::Config{});

  const std::vector<dog_patrol_perception_tracking::Detection> input{
      PersonDet(0.09F, cv::Rect2f(0, 0, 10, 10)),
      PersonDet(0.10F, cv::Rect2f(0, 0, 10, 10)),
      dog_patrol_perception_tracking::Detection{dog_patrol_perception_tracking::ClassId::kCar, 0.10F, cv::Rect2f(0, 0, 10, 10)},
      dog_patrol_perception_tracking::Detection{dog_patrol_perception_tracking::ClassId::kUnknown, 0.99F, cv::Rect2f(0, 0, 10, 10)},
  };

  const auto filtered = filter.Filter(input);
  ASSERT_EQ(filtered.size(), 2U);
  EXPECT_EQ(filtered[0].class_id, dog_patrol_perception_tracking::ClassId::kPerson);
  EXPECT_FLOAT_EQ(filtered[0].confidence, 0.10F);
  EXPECT_EQ(filtered[1].class_id, dog_patrol_perception_tracking::ClassId::kCar);
  EXPECT_FLOAT_EQ(filtered[1].confidence, 0.10F);
}
