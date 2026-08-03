#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "vision_demo_host/tools/offline_replay_run.hpp"

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
  vision_demo_host::tools::OfflineReplayRun::Request request;
  request.recordings_root = root_ / "captures";
  request.results_root = root_ / "results";
  request.run_name = "artifact";
  request.datasets = {"missing_take"};

  const auto result = vision_demo_host::tools::OfflineReplayRun::Run(request);

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

  vision_demo_host::tools::OfflineReplayRun::Request request;
  request.recordings_root = root_ / "captures";
  request.results_root = source_dir / "results";
  request.run_name = "preflight";
  request.datasets = {"take_001"};

  const auto result = vision_demo_host::tools::OfflineReplayRun::Run(request);

  EXPECT_EQ(result.exit_code, 2);
  EXPECT_FALSE(result.all_ok);
  EXPECT_FALSE(std::filesystem::exists(result.run_dir));
}

}  // namespace
