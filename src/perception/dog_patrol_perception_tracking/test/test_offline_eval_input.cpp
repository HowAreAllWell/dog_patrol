#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "dog_patrol_perception_tracking/tools/offline_eval_input.hpp"

namespace {

class OfflineEvalInputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() / "vision_demo_offline_eval_input_test";
    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);
  }

  void TearDown() override { std::filesystem::remove_all(root_); }

  void WriteFile(const std::filesystem::path &path, const std::string &contents) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    ASSERT_TRUE(out) << path;
    out << contents;
    ASSERT_TRUE(out) << path;
  }

  void WriteCompletedFfv1Take(const std::filesystem::path &take_dir, const std::size_t written_frames = 2U) {
    WriteFile(take_dir / "video.mkv", "test video placeholder");
    WriteFile(take_dir / "metadata.json",
              "{\n"
              "  \"state\": \"complete\",\n"
              "  \"codec\": \"FFV1\",\n"
              "  \"container\": \"MKV\",\n"
              "  \"counts\": {\n"
              "    \"written_frames\": " +
                  std::to_string(written_frames) + "\n"
                                                     "  }\n"
                                                     "}\n");
    WriteFile(take_dir / "frame_timestamps.csv",
              "capture_index,source_timestamp_ns,sdk_host_timestamp,camera_frame_number,"
              "camera_frame_number_available,device_timestamp_ticks,source_pixel_type,"
              "source_pixel_type_name,width,height,source_payload_bytes,camera_lost_packets\n"
              "0,100,1,10,true,20,17301514,\"BayerGB8\",1280,1024,1310720,0\n"
              "2,200,2,12,true,40,17301514,\"BayerGB8\",1280,1024,1310720,0\n");
  }

  std::filesystem::path root_;
};

TEST_F(OfflineEvalInputTest, DiscoversCompletedFfv1TakeAndValidatesTimestampSidecar) {
  const auto take_dir = root_ / "capture" / "take_001";
  WriteCompletedFfv1Take(take_dir);

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});

  ASSERT_TRUE(discovery.ok) << discovery.error;
  EXPECT_EQ(discovery.input.video_path, take_dir / "video.mkv");
  EXPECT_EQ(discovery.input.source_kind, dog_patrol_perception_tracking::tools::OfflineEvalSourceKind::kFfv1Capture);
  ASSERT_TRUE(discovery.input.capture.has_value());
  EXPECT_EQ(discovery.input.capture->written_frames, 2U);
  EXPECT_EQ(discovery.input.timestamp_validation.rows, 2U);
  EXPECT_TRUE(discovery.input.timestamp_validation.ok) << discovery.input.timestamp_validation.error;
}

TEST_F(OfflineEvalInputTest, MakesCompletedCaptureMetadataMismatchExplicit) {
  const auto take_dir = root_ / "capture" / "take_001";
  WriteCompletedFfv1Take(take_dir, 3U);

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});

  EXPECT_FALSE(discovery.ok);
  EXPECT_NE(discovery.error.find("written_frames"), std::string::npos);
}

TEST_F(OfflineEvalInputTest, ExplicitVideoPathWinsOverDatasetDefault) {
  const auto dataset_dir = root_ / "legacy";
  WriteFile(dataset_dir / "video.mp4", "historical H264 placeholder");
  const auto explicit_video = root_ / "external" / "clip.mkv";
  WriteFile(explicit_video, "explicit video placeholder");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({dataset_dir, explicit_video});

  ASSERT_TRUE(discovery.ok) << discovery.error;
  EXPECT_EQ(discovery.input.video_path, explicit_video);
  EXPECT_EQ(discovery.input.source_kind, dog_patrol_perception_tracking::tools::OfflineEvalSourceKind::kExplicitVideo);
}

TEST_F(OfflineEvalInputTest, ExplicitHistoricalMp4UsesMigrationSourceKind) {
  const auto historical_video = root_ / "orin_hik_h264_MOT" / "03" / "video.mp4";
  WriteFile(historical_video, "historical H264 placeholder");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({{}, historical_video});

  ASSERT_TRUE(discovery.ok) << discovery.error;
  EXPECT_EQ(discovery.input.source_kind,
            dog_patrol_perception_tracking::tools::OfflineEvalSourceKind::kHistoricalH264);
}

TEST_F(OfflineEvalInputTest, RejectsMp4OutsideHistoricalMigrationDataset) {
  const auto unrelated_mp4 = root_ / "external" / "clip.mp4";
  WriteFile(unrelated_mp4, "unrelated MP4 placeholder");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({{}, unrelated_mp4});

  EXPECT_FALSE(discovery.ok);
  EXPECT_NE(discovery.error.find("orin_hik_h264_MOT migration data"), std::string::npos);
}

TEST_F(OfflineEvalInputTest, RejectsUnsupportedExplicitVideoExtension) {
  const auto unsupported_video = root_ / "external" / "clip.avi";
  WriteFile(unsupported_video, "unsupported video placeholder");

  const auto discovery =
      dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({{}, unsupported_video});

  EXPECT_FALSE(discovery.ok);
  EXPECT_NE(discovery.error.find("must be FFV1/MKV or historical MP4"),
            std::string::npos);
}

TEST_F(OfflineEvalInputTest, RetainsHistoricalMp4FallbackForMigrationRegression) {
  const auto dataset_dir = root_ / "orin_hik_h264_MOT" / "01";
  WriteFile(dataset_dir / "video.mp4", "historical H264 placeholder");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({dataset_dir, {}});

  ASSERT_TRUE(discovery.ok) << discovery.error;
  EXPECT_EQ(discovery.input.video_path, dataset_dir / "video.mp4");
  EXPECT_EQ(discovery.input.source_kind, dog_patrol_perception_tracking::tools::OfflineEvalSourceKind::kHistoricalH264);
}

TEST_F(OfflineEvalInputTest, RetainsHistoricalMp4WhenLegacyMetadataIsNotACaptureSidecar) {
  const auto dataset_dir = root_ / "orin_hik_h264_MOT" / "01";
  WriteFile(dataset_dir / "video.mp4", "historical H264 placeholder");
  WriteFile(dataset_dir / "metadata.json", "{\"legacy_recording\": true, \"frame_count\": 42}\n");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({dataset_dir, {}});

  ASSERT_TRUE(discovery.ok) << discovery.error;
  EXPECT_EQ(discovery.input.source_kind, dog_patrol_perception_tracking::tools::OfflineEvalSourceKind::kHistoricalH264);
  EXPECT_FALSE(discovery.input.capture.has_value());
}

TEST_F(OfflineEvalInputTest, ReplayFrameCountMustMatchCompletedCaptureMetadata) {
  const auto take_dir = root_ / "capture" / "take_001";
  WriteCompletedFfv1Take(take_dir);
  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});
  ASSERT_TRUE(discovery.ok) << discovery.error;

  const auto replay = dog_patrol_perception_tracking::tools::ValidateOfflineEvalReplay(discovery.input, 1U);

  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.error.find("decoded frame count"), std::string::npos);
}

TEST_F(OfflineEvalInputTest, OverlayArtifactsCannotBePlacedInsideSourceDataset) {
  const auto take_dir = root_ / "capture" / "take_001";
  WriteCompletedFfv1Take(take_dir);
  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});
  ASSERT_TRUE(discovery.ok) << discovery.error;

  const auto plan = dog_patrol_perception_tracking::tools::PlanOfflineEvalOverlayArtifacts(
      discovery.input, take_dir / "eval_result", false, "eval_overlay.mkv");

  EXPECT_FALSE(plan.ok);
  EXPECT_NE(plan.error.find("source dataset"), std::string::npos);
}

TEST_F(OfflineEvalInputTest, OverlayArtifactsCannotReachSourceDatasetThroughSymlink) {
  const auto take_dir = root_ / "capture" / "take_001";
  const auto results_link = root_ / "results_link";
  WriteCompletedFfv1Take(take_dir);

  std::error_code filesystem_error;
  std::filesystem::create_directory_symlink(take_dir, results_link, filesystem_error);
  if (filesystem_error) {
    GTEST_SKIP() << "Cannot create temporary symlink: " << filesystem_error.message();
  }

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});
  ASSERT_TRUE(discovery.ok) << discovery.error;

  const auto plan = dog_patrol_perception_tracking::tools::PlanOfflineEvalOverlayArtifacts(
      discovery.input, results_link / "issue82_run" / "s01", true, "eval_overlay.mkv");

  EXPECT_FALSE(plan.ok);
  EXPECT_NE(plan.error.find("source dataset"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(take_dir / "issue82_run"));
}

TEST_F(OfflineEvalInputTest, SourceBoundaryStillRejectsArtifactsForAnInvalidCaptureTake) {
  const auto take_dir = root_ / "capture" / "incomplete_take";
  WriteFile(take_dir / "video.mkv", "incomplete capture placeholder");
  WriteFile(take_dir / "metadata.json", "{\"state\": \"incomplete\"}\n");

  const auto discovery = dog_patrol_perception_tracking::tools::DiscoverOfflineEvalInput({take_dir, {}});
  EXPECT_FALSE(discovery.ok);

  dog_patrol_perception_tracking::tools::OfflineEvalInput source_boundary;
  source_boundary.dataset_directory = take_dir;
  const auto plan = dog_patrol_perception_tracking::tools::PlanOfflineEvalOverlayArtifacts(
      source_boundary, take_dir / "eval_result", false, "eval_overlay.mkv");

  EXPECT_FALSE(plan.ok);
  EXPECT_NE(plan.error.find("source dataset"), std::string::npos);
}

TEST(OfflineEvalInputModesTest, SupportsAllFourIndependentOverlayCombinations) {
  using dog_patrol_perception_tracking::tools::OfflineEvalOverlayMode;
  EXPECT_EQ(dog_patrol_perception_tracking::tools::OfflineEvalOverlayModeFor(false, false), OfflineEvalOverlayMode::kHeadless);
  EXPECT_EQ(dog_patrol_perception_tracking::tools::OfflineEvalOverlayModeFor(true, false), OfflineEvalOverlayMode::kPreviewOnly);
  EXPECT_EQ(dog_patrol_perception_tracking::tools::OfflineEvalOverlayModeFor(false, true), OfflineEvalOverlayMode::kRecordOnly);
  EXPECT_EQ(dog_patrol_perception_tracking::tools::OfflineEvalOverlayModeFor(true, true), OfflineEvalOverlayMode::kPreviewAndRecord);
}

}  // namespace
