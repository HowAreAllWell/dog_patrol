#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "mot_tracker_observability.hpp"

namespace {

class RecordingObservabilityWriter final : public dog_patrol_perception_tracking::MotTrackerObservabilityWriter {
 public:
  void BeginFrame(const int frame_id) override { begin_frames.push_back(frame_id); }

  void WriteTracks(const int frame_id,
                   const std::vector<dog_patrol_perception_tracking::MotTrackerTrackObservation> &tracks) override {
    track_frames.emplace_back(frame_id, tracks.size());
  }

  void WriteDetections(
      const int frame_id,
      const std::vector<dog_patrol_perception_tracking::MotTrackerDetectionObservation> &detections) override {
    detection_frames.emplace_back(frame_id, detections.size());
  }

  void WriteGmc(const int frame_id, const dog_patrol_perception_tracking::MotTrackerGmcObservation &gmc) override {
    gmc_frames.emplace_back(frame_id, gmc.ok);
  }

  void WritePair(const int frame_id, const dog_patrol_perception_tracking::MotTrackerPairObservation &pair) override {
    pair_frames.emplace_back(frame_id, pair.stage_name);
  }

  std::vector<int> begin_frames;
  std::vector<std::pair<int, std::size_t>> track_frames;
  std::vector<std::pair<int, std::size_t>> detection_frames;
  std::vector<std::pair<int, bool>> gmc_frames;
  std::vector<std::pair<int, std::string>> pair_frames;
};

std::vector<std::string> ReadLines(const std::filesystem::path &path) {
  std::ifstream ifs(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::filesystem::path UniqueTempDir(const std::string &name) {
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() / (name + "_" + std::to_string(tick));
}

cv::Mat StateWithFirstValue(const float value) {
  cv::Mat state = cv::Mat::zeros(8, 1, CV_32F);
  state.at<float>(0, 0) = value;
  return state;
}

cv::Mat CovWithDiag(const float value) {
  cv::Mat cov = cv::Mat::eye(8, 8, CV_32F);
  cov *= value;
  return cov;
}

}  // namespace

TEST(MotTrackerObservabilityTest, MemoryWriterOnlySeesEnabledFrames) {
  dog_patrol_perception_tracking::MotTrackerObservabilityConfig config;
  config.enabled = true;
  config.frame_start = 2;
  config.frame_end = 3;

  auto writer = std::make_unique<RecordingObservabilityWriter>();
  auto *recording = writer.get();
  dog_patrol_perception_tracking::MotTrackerObservability observability(config, std::move(writer));

  observability.BeginFrame(1);
  observability.WriteGmc(1, dog_patrol_perception_tracking::MotTrackerGmcObservation{true, cv::Mat::eye(2, 3, CV_32F)});
  observability.BeginFrame(2);
  observability.WriteGmc(2, dog_patrol_perception_tracking::MotTrackerGmcObservation{true, cv::Mat::eye(2, 3, CV_32F)});
  observability.BeginFrame(3);
  observability.WriteDetections(
      3, {dog_patrol_perception_tracking::MotTrackerDetectionObservation{"high", 0, 0, dog_patrol_perception_tracking::ClassId::kPerson,
                                                           0.9F, cv::Rect2f(1, 2, 3, 4)}});
  observability.WritePair(4, dog_patrol_perception_tracking::MotTrackerPairObservation{});

  EXPECT_EQ(recording->begin_frames, std::vector<int>({1, 2, 3}));
  ASSERT_EQ(recording->gmc_frames.size(), 1U);
  EXPECT_EQ(recording->gmc_frames.front().first, 2);
  EXPECT_TRUE(recording->gmc_frames.front().second);
  ASSERT_EQ(recording->detection_frames.size(), 1U);
  EXPECT_EQ(recording->detection_frames.front().first, 3);
  EXPECT_EQ(recording->pair_frames.size(), 0U);
}

TEST(MotTrackerObservabilityTest, CsvWriterPreservesDiagnosticFiles) {
  const auto output_dir = UniqueTempDir("vision_demo_mot_tracker_observability");
  std::filesystem::remove_all(output_dir);

  dog_patrol_perception_tracking::MotTrackerObservabilityConfig config;
  config.enabled = true;
  config.output_dir = output_dir;
  config.frame_start = 10;
  config.frame_end = 10;
  auto observability = dog_patrol_perception_tracking::MotTrackerObservability::CreateCsv(config);

  observability->BeginFrame(9);
  observability->WriteGmc(9, dog_patrol_perception_tracking::MotTrackerGmcObservation{true, cv::Mat::eye(2, 3, CV_32F)});
  ASSERT_TRUE(std::filesystem::exists(output_dir / "gmc.csv"));

  observability->BeginFrame(10);
  cv::Mat warp = cv::Mat::eye(2, 3, CV_32F);
  warp.at<float>(0, 2) = 2.0F;
  warp.at<float>(1, 2) = 3.0F;
  observability->WriteGmc(10, dog_patrol_perception_tracking::MotTrackerGmcObservation{true, warp});
  observability->WriteDetections(
      10, {dog_patrol_perception_tracking::MotTrackerDetectionObservation{"high", 0, 0, dog_patrol_perception_tracking::ClassId::kPerson,
                                                            0.9F, cv::Rect2f(1, 2, 3, 4)}});

  dog_patrol_perception_tracking::MotTrackerTrackObservation track;
  track.track_idx = 2;
  track.track_id = 7;
  track.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  track.is_confirmed = true;
  track.hits = 3;
  track.age = 4;
  track.time_since_update = 0;
  track.predicted_bbox = cv::Rect2f(10, 20, 30, 40);
  track.bbox = cv::Rect2f(11, 21, 31, 41);
  track.pre_gmc_pred_bbox = cv::Rect2f(9, 19, 29, 39);
  track.pre_gmc_bbox = cv::Rect2f(8, 18, 28, 38);
  track.post_gmc_pred_bbox = track.predicted_bbox;
  track.post_gmc_bbox = track.bbox;
  track.pre_state_post = StateWithFirstValue(1.0F);
  track.post_predict_state_pre = StateWithFirstValue(2.0F);
  track.post_predict_state_post = StateWithFirstValue(3.0F);
  track.post_gmc_state_pre = StateWithFirstValue(4.0F);
  track.post_gmc_state_post = StateWithFirstValue(5.0F);
  track.pre_error_cov_post = CovWithDiag(1.0F);
  track.post_predict_error_cov_pre = CovWithDiag(2.0F);
  track.post_predict_error_cov_post = CovWithDiag(3.0F);
  track.post_gmc_error_cov_pre = CovWithDiag(4.0F);
  track.post_gmc_error_cov_post = CovWithDiag(5.0F);
  observability->WriteTracks(10, {track});

  dog_patrol_perception_tracking::MotTrackerPairObservation pair;
  pair.stage_name = "stage1";
  pair.track_idx = 2;
  pair.track_id = 7;
  pair.det_local_idx = 0;
  pair.det_src_idx = 0;
  pair.selected = true;
  pair.terms.iou = 0.25F;
  pair.terms.fused_cost = 0.5F;
  pair.terms.eligible = true;
  pair.pre_gmc_pred_bbox = track.pre_gmc_pred_bbox;
  pair.post_gmc_pred_bbox = track.post_gmc_pred_bbox;
  pair.pre_state_post = StateWithFirstValue(6.0F);
  pair.post_predict_state_pre = StateWithFirstValue(7.0F);
  pair.post_gmc_state_pre = StateWithFirstValue(8.0F);
  observability->WritePair(10, pair);
  observability.reset();

  const auto gmc_lines = ReadLines(output_dir / "gmc.csv");
  ASSERT_EQ(gmc_lines.size(), 2U);
  EXPECT_EQ(gmc_lines[0], "frame,gmc_ok,warp00,warp01,warp02,warp10,warp11,warp12");
  EXPECT_EQ(gmc_lines[1], "10,1,1,0,2,0,1,3");

  const auto detection_lines = ReadLines(output_dir / "detections.csv");
  ASSERT_EQ(detection_lines.size(), 2U);
  EXPECT_EQ(detection_lines[0], "frame,level,det_local_idx,det_src_idx,class,score,x,y,w,h");
  EXPECT_EQ(detection_lines[1], "10,high,0,0,0,0.9,1,2,3,4");

  const auto track_lines = ReadLines(output_dir / "tracks.csv");
  ASSERT_EQ(track_lines.size(), 2U);
  EXPECT_NE(track_lines[0].find("frame,track_idx,track_id"), std::string::npos);
  EXPECT_NE(track_lines[1].find("10,2,7,0,0,1,3,4,0,0,10,20,30,40"), std::string::npos);

  const auto pair_lines = ReadLines(output_dir / "pairs.csv");
  ASSERT_EQ(pair_lines.size(), 2U);
  EXPECT_NE(pair_lines[0].find("frame,stage,track_idx"), std::string::npos);
  EXPECT_NE(pair_lines[1].find("10,stage1,2,7,0,0,0,0.25"), std::string::npos);

  std::filesystem::remove_all(output_dir);
}
