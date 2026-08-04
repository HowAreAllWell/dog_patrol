#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "dog_patrol_perception_tracking/modules/perception_config_materializer.hpp"
#include "dog_patrol_perception_tracking/tools/offline_replay_run.hpp"

namespace {

class OfflineReplayRunTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() / "vision_demo_offline_replay_run_test";
    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);
  }

  void TearDown() override { std::filesystem::remove_all(root_); }

  std::string ReadText(const std::filesystem::path &path) {
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
  }

  std::filesystem::path root_;
};

TEST_F(OfflineReplayRunTest, WritesRunArtifactsWhenInputValidationFailsBeforeInference) {
  dog_patrol_perception_tracking::tools::OfflineReplayRun::Request request;
  request.recordings_root = root_ / "captures";
  request.results_root = root_ / "results";
  request.run_name = "artifact";
  request.datasets = {"missing_take"};

  const auto result = dog_patrol_perception_tracking::tools::OfflineReplayRun::Run(request);

  EXPECT_EQ(result.exit_code, 1);
  EXPECT_FALSE(result.all_ok);
  ASSERT_FALSE(result.run_dir.empty());
  EXPECT_TRUE(std::filesystem::exists(result.run_dir / "dataset_dir_map.csv"));
  EXPECT_TRUE(std::filesystem::exists(result.run_dir / "global_summary.md"));

  const auto dataset_result_dir = result.run_dir / "s01";
  EXPECT_TRUE(std::filesystem::exists(dataset_result_dir / "summary.json"));
  EXPECT_TRUE(std::filesystem::exists(dataset_result_dir / "summary.md"));
  EXPECT_TRUE(std::filesystem::exists(dataset_result_dir / "identity_metrics.json"));
  EXPECT_TRUE(std::filesystem::exists(dataset_result_dir / "identity_metrics.md"));

  const std::string summary = ReadText(dataset_result_dir / "summary.json");
  EXPECT_NE(summary.find("\"dataset_name\": \"missing_take\""), std::string::npos);
  EXPECT_NE(summary.find("\"ok\": false"), std::string::npos);
  EXPECT_NE(summary.find("No replay video found"), std::string::npos);
  EXPECT_NE(ReadText(result.run_dir / "dataset_dir_map.csv").find("s01,missing_take"), std::string::npos);
}

TEST_F(OfflineReplayRunTest, RejectsResultDirectoryInsideSourceBeforeCreatingRun) {
  const auto source_dir = root_ / "captures" / "take_001";
  std::filesystem::create_directories(source_dir);

  dog_patrol_perception_tracking::tools::OfflineReplayRun::Request request;
  request.recordings_root = root_ / "captures";
  request.results_root = source_dir / "results";
  request.run_name = "preflight";
  request.datasets = {"take_001"};

  const auto result = dog_patrol_perception_tracking::tools::OfflineReplayRun::Run(request);

  EXPECT_EQ(result.exit_code, 2);
  EXPECT_FALSE(result.all_ok);
  EXPECT_FALSE(std::filesystem::exists(result.run_dir));
}

TEST_F(OfflineReplayRunTest, RequestDefaultsUsePerceptionMaterializerInputs) {
  const dog_patrol_perception_tracking::tools::OfflineReplayRun::Request request;
  const dog_patrol_perception_tracking::PerceptionConfigMaterializer::TrackerInput tracker_defaults;
  const dog_patrol_perception_tracking::PerceptionConfigMaterializer::IdentityInput identity_defaults;

  EXPECT_EQ(request.tracker.gmc_enabled, tracker_defaults.gmc_enabled);
  EXPECT_EQ(request.tracker.reid_backend, tracker_defaults.reid_backend);
  EXPECT_EQ(request.tracker.reid_input_width, tracker_defaults.reid_input_width);
  EXPECT_EQ(request.tracker.reid_input_height, tracker_defaults.reid_input_height);

  EXPECT_EQ(request.identity.target_lost_threshold_frames,
            identity_defaults.target_lost_threshold_frames);
  EXPECT_EQ(request.identity.feat_bank_size, identity_defaults.feat_bank_size);
  EXPECT_FLOAT_EQ(request.identity.recover_sim_thresh_strict,
                  identity_defaults.recover_sim_thresh_strict);
  EXPECT_FLOAT_EQ(request.identity.recover_sim_thresh_relaxed,
                  identity_defaults.recover_sim_thresh_relaxed);
  EXPECT_EQ(request.identity.reid_enable, identity_defaults.reid_enable);
  EXPECT_EQ(request.identity.reid_backend, identity_defaults.reid_backend);
  EXPECT_EQ(request.identity.reid_input_width, identity_defaults.reid_input_width);
  EXPECT_EQ(request.identity.reid_input_height, identity_defaults.reid_input_height);
}

}  // namespace
