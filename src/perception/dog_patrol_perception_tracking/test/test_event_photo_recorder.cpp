#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include "dog_patrol_perception_tracking/modules/event_photo_recorder.hpp"

namespace {

using dog_patrol_perception_tracking::EventPhotoRecorder;
using dog_patrol_perception_tracking::MissionEventAction;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::PerceptionMissionEvent;
using dog_patrol_perception_tracking::SourceFrameMetadata;

constexpr std::uint64_t kTime = 1710000000123000000ULL;
constexpr std::uint64_t kMs = 1000000ULL;
const cv::Scalar kBlue{240, 20, 10};
const cv::Scalar kGreen{10, 230, 20};
const cv::Scalar kRed{20, 10, 240};

MissionSnapshot Mission(MissionPhase phase, std::uint32_t seq, int target = 7) {
  return MissionSnapshot{seq, phase, target, 0};
}

SourceFrameMetadata Metadata(std::uint64_t stamp) {
  SourceFrameMetadata metadata;
  metadata.source_timestamp_ns = stamp;
  metadata.image_width = 96;
  metadata.image_height = 64;
  metadata.optical_frame_id = "camera";
  return metadata;
}

std::string LocalSecond(std::uint64_t stamp) {
  const auto seconds = static_cast<std::time_t>(stamp / 1000000000ULL);
  std::tm local{};
  localtime_r(&seconds, &local);
  char text[32]{};
  std::strftime(text, sizeof(text), "%Y%m%d_%H%M%S", &local);
  return text;
}

std::string ReadText(const std::filesystem::path &path) {
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

class PhotoRecorderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string pattern = "/tmp/dog-patrol-photos-XXXXXX";
    char *created = ::mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    root_ = created;
    config_.root = root_.string();
  }

  void TearDown() override {
    std::error_code ignored;
    std::filesystem::remove_all(root_, ignored);
  }

  std::unique_ptr<EventPhotoRecorder> Recorder() {
    return std::make_unique<EventPhotoRecorder>(config_, [this](const auto &error) {
      std::lock_guard<std::mutex> lock(errors_mutex_);
      errors_.push_back(error);
    });
  }

  void Frame(EventPhotoRecorder &recorder, MissionSnapshot mission, std::uint64_t stamp,
             const cv::Scalar &color, int trusted = 7,
             std::vector<MissionEventAction> events = {}, std::uint64_t mission_stamp = 0) {
    cv::Mat image(64, 96, CV_8UC3, color);
    recorder.ObserveFrame(mission, trusted, events, Metadata(stamp), image, mission_stamp);
    image.setTo(cv::Scalar(0, 0, 0));  // Emulate a later in-place overlay/render pass.
  }

  std::vector<std::filesystem::path> Files(const std::string &directory,
                                          const std::string &round_tag = "",
                                          const std::string &extension = ".jpg") {
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::is_directory(root_ / directory)) {
      return files;
    }
    for (const auto &entry : std::filesystem::directory_iterator(root_ / directory)) {
      const auto name = entry.path().filename().string();
      if (entry.path().extension() == extension &&
          (round_tag.empty() || name.find(round_tag + ".") != std::string::npos ||
           name.find(round_tag + "_") != std::string::npos)) {
        files.push_back(entry.path());
      }
    }
    std::sort(files.begin(), files.end());
    return files;
  }

  void ExpectColor(const std::filesystem::path &path, const cv::Scalar &color) {
    const auto image = cv::imread(path.string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(image.empty());
    EXPECT_EQ(image.size(), cv::Size(96, 64));
    const auto mean = cv::mean(image);
    for (int channel = 0; channel < 3; ++channel) {
      EXPECT_NEAR(mean[channel], color[channel], 3.0);
    }
  }

  EventPhotoRecorder::Config config_;
  std::filesystem::path root_;
  std::mutex errors_mutex_;
  std::vector<std::string> errors_;
};

TEST_F(PhotoRecorderTest, SavesOnlyPublishedDetectionOnceAndCopiesFullSourceFrame) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kPatrol, 10, 0);
  const MissionEventAction confirmed{PerceptionMissionEvent::kTargetConfirmed, 7, 10};
  Frame(*recorder, mission, kTime, kGreen);  // No published event: no detection photo.
  Frame(*recorder, mission, kTime + 100 * kMs, kBlue, 7, {confirmed});
  Frame(*recorder, mission, kTime + 200 * kMs, kRed, 7, {confirmed});
  recorder->Flush();

  const auto files = Files("tracking/event");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
  EXPECT_EQ(files.front().filename().string(), LocalSecond(kTime + 100 * kMs) + "_t7.jpg");
  for (const char *directory : {"tracking/event", "tracking/lost", "face", "voice", "reid"}) {
    EXPECT_TRUE(std::filesystem::is_directory(root_ / directory));
  }
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round1"));
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round2"));
  EXPECT_EQ(recorder->GetMetrics().written, 1U);
}

TEST_F(PhotoRecorderTest, LostPhotoUsesLastTrustedVisibleFrameAndOriginalTimestamp) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  Frame(*recorder, mission, kTime + 3000 * kMs, kGreen, 0);
  Frame(*recorder, mission, kTime + 6000 * kMs, kRed, 0,
        {{PerceptionMissionEvent::kTargetLost, 7, 20}});
  recorder->Flush();

  const auto files = Files("tracking/lost");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
  EXPECT_EQ(files.front().filename().string(), LocalSecond(kTime) + "_t7.jpg");
}

TEST_F(PhotoRecorderTest, VoiceRoundsUseStartForFailureAndLatestPrecedingSceneForSuccess) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, mission, kTime + 200 * kMs, kGreen, 0);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 250 * kMs);
  Frame(*recorder, mission, kTime + 300 * kMs, kRed, 0);
  Frame(*recorder, mission, kTime + 500 * kMs, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 400 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 350 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 350 * kMs);
  recorder->Flush();

  const auto failed = Files("voice/failed", "_r1");
  const auto passed = Files("voice/passed", "_r2");
  ASSERT_EQ(failed.size(), 1U);
  ASSERT_EQ(passed.size(), 1U);
  ExpectColor(failed.front(), kBlue);
  ExpectColor(passed.front(), kRed);
  EXPECT_EQ(failed.front().filename().string(), LocalSecond(kTime) + "_t7_r1.jpg");
  EXPECT_EQ(passed.front().filename().string(), LocalSecond(kTime + 300 * kMs) + "_t7_r2.jpg");
  EXPECT_TRUE(std::filesystem::is_directory(root_ / "voice/passed"));
  EXPECT_TRUE(std::filesystem::is_directory(root_ / "voice/failed"));
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round1"));
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round2"));
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round3"));
  EXPECT_EQ(recorder->GetMetrics().written, 2U);
}

TEST_F(PhotoRecorderTest, PendingWaitBeforeFirstVerifyFrameUsesEarlierSameLifecycleScene) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kApproachTarget, 19), kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 150 * kMs);
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime + 200 * kMs,
        kGreen, 7, {}, kTime + 100 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 250 * kMs);
  recorder->Flush();

  const auto files = Files("voice/failed", "_r1");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 0U);
  EXPECT_EQ(recorder->GetMetrics().pending_dropped, 0U);
}

TEST_F(PhotoRecorderTest, DelayedResultUsesRealMissionCloseTimeAndCannotUseNewSessionFrames) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 150 * kMs);
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime + 300 * kMs, kGreen);
  // Processing sees RECOVER on a frame acquired before the actual transition.
  Frame(*recorder, Mission(MissionPhase::kRecoverPatrol, 21), kTime + 400 * kMs,
        kBlue, 0, {}, kTime + 500 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 450 * kMs);
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 30), kTime + 1000 * kMs, kRed);
  recorder->RecordVoice(30, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 1050 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 1100 * kMs);
  recorder->RecordVoice(30, 7, 1, EventPhotoRecorder::PASSED, kTime + 1100 * kMs);
  recorder->Flush();

  const auto files = Files("voice/passed", "_r1");
  ASSERT_EQ(files.size(), 2U);
  ExpectColor(files[0], kBlue);
  ExpectColor(files[1], kRed);
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 1U);
}

TEST_F(PhotoRecorderTest, ResultArrivingBeforeWaitIsResolvedWithoutMixingRounds) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::NOT_PASSED, kTime + 300 * kMs);
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime + 100 * kMs, kGreen);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 150 * kMs);
  recorder->Flush();
  const auto files = Files("voice/failed", "_r2");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kGreen);
}

TEST_F(PhotoRecorderTest, MissingOrStaleSceneIsReportedInsteadOfBorrowingAnotherTarget) {
  config_.max_frame_age_ns = 100 * kMs;
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 500 * kMs);
  Frame(*recorder, mission, kTime + 600 * kMs, kGreen);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 700 * kMs);
  recorder->RecordVoice(20, 99, 1, EventPhotoRecorder::PASSED, kTime + 700 * kMs);
  recorder->Flush();
  EXPECT_TRUE(Files("voice/passed", "_r1").empty());
  EXPECT_TRUE(Files("voice/failed", "_r1").empty());
  EXPECT_GE(recorder->GetMetrics().missing_frame, 1U);
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 1U);
  EXPECT_FALSE(errors_.empty());
}

TEST_F(PhotoRecorderTest, NewTargetLifecycleCannotReusePreviousLastVisiblePhoto) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kTrackIntruder, 20), kTime, kBlue);
  Frame(*recorder, Mission(MissionPhase::kRecoverPatrol, 21), kTime + 100 * kMs, kRed, 0);
  Frame(*recorder, Mission(MissionPhase::kConfirmTarget, 30), kTime + 200 * kMs,
        kGreen, 0, {{PerceptionMissionEvent::kTargetLost, 7, 30}});
  recorder->Flush();
  EXPECT_TRUE(Files("tracking/lost").empty());
  EXPECT_EQ(recorder->GetMetrics().missing_frame, 1U);
}

TEST_F(PhotoRecorderTest, DetectionFrameRemainsLastVisibleWhenTargetImmediatelyDisappears) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kPatrol, 10, 0), kTime, kBlue, 7,
        {{PerceptionMissionEvent::kTargetConfirmed, 7, 10}});
  Frame(*recorder, Mission(MissionPhase::kPatrol, 10, 0), kTime + 100 * kMs, kRed, 0);
  Frame(*recorder, Mission(MissionPhase::kConfirmTarget, 11), kTime + 200 * kMs, kGreen, 0);
  Frame(*recorder, Mission(MissionPhase::kConfirmTarget, 11), kTime + 6200 * kMs,
        kRed, 0, {{PerceptionMissionEvent::kTargetLost, 7, 11}});
  recorder->Flush();
  const auto files = Files("tracking/lost");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
}

TEST_F(PhotoRecorderTest, ClosedSessionAcceptsDelayedWaitAndResultWithinItsTimeWindow) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime + 300 * kMs, kGreen);
  Frame(*recorder, Mission(MissionPhase::kRecoverPatrol, 21), kTime + 600 * kMs,
        kRed, 0, {}, kTime + 500 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 400 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 700 * kMs);
  recorder->Flush();
  const auto files = Files("voice/failed", "_r1");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 1U);
}

TEST_F(PhotoRecorderTest, WaitsForDelayedProcessingToSelectNearestPrecedingFrame) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, mission, kTime + 80 * kMs, kGreen);
  Frame(*recorder, mission, kTime + 120 * kMs, kRed);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 150 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 160 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 300 * kMs);
  Frame(*recorder, mission, kTime + 200 * kMs, kGreen);
  Frame(*recorder, mission, kTime + 250 * kMs, kBlue);
  Frame(*recorder, mission, kTime + 350 * kMs, kRed);
  recorder->Flush();
  const auto failed = Files("voice/failed", "_r1");
  const auto passed = Files("voice/passed", "_r2");
  ASSERT_EQ(failed.size(), 1U);
  ASSERT_EQ(passed.size(), 1U);
  ExpectColor(failed.front(), kGreen);
  ExpectColor(passed.front(), kBlue);
}

TEST_F(PhotoRecorderTest, FramesCapturedBeforePassButProcessedAfterCloseKeepOldSession) {
  auto recorder = Recorder();
  const auto verify = Mission(MissionPhase::kVerifyIdentity, 20);
  const auto recover = Mission(MissionPhase::kRecoverPatrol, 21);
  Frame(*recorder, verify, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, verify, kTime + 300 * kMs, kGreen);
  Frame(*recorder, recover, kTime + 400 * kMs, kRed, 0, {}, kTime + 500 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 450 * kMs);
  Frame(*recorder, recover, kTime + 440 * kMs, kBlue, 0, {}, kTime + 500 * kMs);
  Frame(*recorder, recover, kTime + 470 * kMs, kRed, 0, {}, kTime + 500 * kMs);
  Frame(*recorder, recover, kTime + 600 * kMs, kGreen, 0, {}, kTime + 500 * kMs);
  recorder->Flush();
  const auto files = Files("voice/passed", "_r1");
  ASSERT_EQ(files.size(), 1U);
  ExpectColor(files.front(), kBlue);
  EXPECT_EQ(files.front().filename().string(), LocalSecond(kTime + 440 * kMs) + "_t7_r1.jpg");
}

TEST_F(PhotoRecorderTest, OldCapturedSceneBelongsToClosedTargetEvenWhenNewTargetIsActive) {
  auto recorder = Recorder();
  const auto first = Mission(MissionPhase::kVerifyIdentity, 20, 7);
  const auto next = Mission(MissionPhase::kVerifyIdentity, 30, 8);
  Frame(*recorder, first, kTime, kBlue);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, first, kTime + 300 * kMs, kGreen);
  Frame(*recorder, Mission(MissionPhase::kRecoverPatrol, 21), kTime + 350 * kMs,
        kBlue, 0, {}, kTime + 500 * kMs);
  // Target B is now active, but this source frame still belongs to A's window.
  Frame(*recorder, next, kTime + 400 * kMs, kRed, 8, {}, kTime + 600 * kMs);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 450 * kMs);
  recorder->RecordVoice(30, 8, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 605 * kMs);
  // This gap frame belongs to neither A's closed window nor B's new lifecycle.
  Frame(*recorder, next, kTime + 550 * kMs, kBlue, 8, {}, kTime + 600 * kMs);
  Frame(*recorder, next, kTime + 610 * kMs, kGreen, 8, {}, kTime + 600 * kMs);
  recorder->RecordVoice(30, 8, 1, EventPhotoRecorder::NOT_PASSED, kTime + 620 * kMs);
  recorder->RecordVoice(30, 8, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 650 * kMs);
  Frame(*recorder, next, kTime + 700 * kMs, kBlue, 8, {}, kTime + 600 * kMs);
  recorder->RecordVoice(30, 8, 2, EventPhotoRecorder::PASSED, kTime + 680 * kMs);
  recorder->Flush();

  const auto results = Files("voice/passed", "_r2");
  ASSERT_EQ(results.size(), 2U);
  ExpectColor(results[0], kRed);
  ExpectColor(results[1], kGreen);
  EXPECT_EQ(results[0].filename().string(), LocalSecond(kTime) + "_t7_r2.jpg");
  EXPECT_EQ(results[1].filename().string(), LocalSecond(kTime) + "_t8_r2.jpg");
  EXPECT_TRUE(Files("voice/failed", "_r1").empty());
  EXPECT_GE(recorder->GetMetrics().missing_frame, 1U);
}

TEST_F(PhotoRecorderTest, MoreCompletedRoundsThanHistoryCapacityRemainRecordable) {
  config_.frame_history_capacity = 2;
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  for (std::uint32_t round = 1; round <= 20; ++round) {
    const auto stamp = kTime + round * 1000 * kMs;
    Frame(*recorder, mission, stamp, kBlue);
    recorder->RecordVoice(20, 7, round, EventPhotoRecorder::WAIT_STARTED, stamp);
    recorder->RecordVoice(20, 7, round, EventPhotoRecorder::NOT_PASSED, stamp + 100 * kMs);
    recorder->Flush();
    const auto files = Files("voice/failed", "_r" + std::to_string(round));
    ASSERT_EQ(files.size(), 1U);
    ExpectColor(files.front(), kBlue);
    EXPECT_EQ(files.front().filename().string(),
              LocalSecond(stamp) + "_t7_r" + std::to_string(round) + ".jpg");
    EXPECT_FALSE(std::filesystem::exists(root_ / ("voice/round" + std::to_string(round))));
  }
  EXPECT_EQ(recorder->GetMetrics().written, 20U);
  EXPECT_EQ(Files("voice/failed", "_r1").size(), 1U);
  EXPECT_EQ(Files("voice/failed", "_r2").size(), 1U);
  EXPECT_FALSE(std::filesystem::exists(root_ / "voice/round21"));
  EXPECT_EQ(recorder->GetMetrics().pending_dropped, 0U);
  EXPECT_EQ(recorder->GetMetrics().missing_frame, 0U);
}

TEST_F(PhotoRecorderTest, HeartbeatRecoveryCreatesNewGenerationForSameStateAndRound) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, mission, kTime + 200 * kMs, kGreen);
  cv::Mat unavailable(64, 96, CV_8UC3, kRed);
  recorder->ObserveFrame(std::nullopt, 0, {}, Metadata(kTime + 300 * kMs), unavailable);
  Frame(*recorder, mission, kTime + 500 * kMs, kRed, 7, {}, kTime);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 550 * kMs);
  Frame(*recorder, mission, kTime + 600 * kMs, kGreen, 7, {}, kTime);
  // The first generation's genuinely old result may still arrive now.
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 250 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 650 * kMs);
  recorder->Flush();
  const auto files = Files("voice/failed", "_r1");
  ASSERT_EQ(files.size(), 2U);
  ExpectColor(files[0], kBlue);
  ExpectColor(files[1], kRed);
  EXPECT_EQ(files[0].filename().string(), LocalSecond(kTime) + "_t7_r1.jpg");
  EXPECT_EQ(files[1].filename().string(), LocalSecond(kTime) + "_t7_r1_1.jpg");
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 0U);
}

TEST_F(PhotoRecorderTest, RepeatedSecondAcrossInstancesAddsSuffixWithoutOverwriting) {
  const auto mission = Mission(MissionPhase::kPatrol, 10, 0);
  const std::vector<MissionEventAction> events{{PerceptionMissionEvent::kTargetConfirmed, 7, 10}};
  {
    auto first = Recorder();
    Frame(*first, mission, kTime, kBlue, 7, events);
    // Destruction drains the queued write even without an explicit Flush().
  }
  {
    auto second = Recorder();
    Frame(*second, mission, kTime + 400 * kMs, kRed, 7, events);
  }
  const auto files = Files("tracking/event");
  ASSERT_EQ(files.size(), 2U);
  ExpectColor(files[0], kBlue);
  ExpectColor(files[1], kRed);
  EXPECT_EQ(files[0].filename().string(), LocalSecond(kTime) + "_t7.jpg");
  EXPECT_EQ(files[1].filename().string(), LocalSecond(kTime) + "_t7_1.jpg");
}

TEST_F(PhotoRecorderTest, DiskFailureIsVisibleAndDoesNotThrowOnPerceptionThread) {
  const auto blocked = root_ / "ordinary-file";
  std::ofstream(blocked) << "not a directory";
  config_.root = blocked.string();
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kPatrol, 10, 0), kTime, kBlue, 7,
        {{PerceptionMissionEvent::kTargetConfirmed, 7, 10}});
  recorder->Flush();
  EXPECT_EQ(recorder->GetMetrics().write_errors, 1U);
  EXPECT_EQ(recorder->GetMetrics().written, 0U);
  EXPECT_FALSE(errors_.empty());
}

TEST_F(PhotoRecorderTest, PendingEventsAndWriteQueueAreBoundedAndReportDrops) {
  config_.frame_history_capacity = 2;
  config_.write_queue_capacity = 1;
  auto recorder = Recorder();
  for (std::uint32_t seq = 100; seq < 103; ++seq) {
    recorder->RecordVoice(seq, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime);
  }
  // Batch two independent photo actions while the producer owns the mutex;
  // the worker cannot drain the one-slot queue before the second enqueue.
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue, 7,
        {{PerceptionMissionEvent::kTargetConfirmed, 7, 20},
         {PerceptionMissionEvent::kTargetLost, 7, 20}});
  recorder->Flush();
  EXPECT_EQ(recorder->GetMetrics().queue_dropped, 1U);
  EXPECT_EQ(recorder->GetMetrics().pending_dropped, 3U);
  EXPECT_EQ(recorder->GetMetrics().written, 1U);
  EXPECT_GE(errors_.size(), 3U);
}

TEST_F(PhotoRecorderTest, InvalidConfigurationIsRejected) {
  config_.frame_history_capacity = 0;
  EXPECT_THROW(Recorder(), std::invalid_argument);
}

TEST_F(PhotoRecorderTest, VoiceConfidenceUsesDecisionSnapshotAndSharesPhotoBasename) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, mission, kTime + 200 * kMs, kGreen);
  Frame(*recorder, mission, kTime + 400 * kMs, kRed);
  EventPhotoRecorder::VoiceConfidence confidence{2, 75, {1, 4}};
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 300 * kMs, confidence);
  confidence.matching_channels.clear();  // The queued job owns its snapshot.
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 300 * kMs,
                        EventPhotoRecorder::VoiceConfidence{1, 50, {0}});
  recorder->Flush();
  const auto photos = Files("voice/passed");
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(photos.size(), 1U);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(photos[0].stem(), records[0].stem());
  ExpectColor(photos[0], kGreen);
  const auto json = ReadText(records[0]);
  EXPECT_NE(json.find("\"state_seq\": 20"), std::string::npos);
  EXPECT_NE(json.find("\"target_id\": 7"), std::string::npos);
  EXPECT_NE(json.find("\"round_number\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"vote_count\": 2"), std::string::npos);
  EXPECT_NE(json.find("\"matching_channels\": [1, 4]"), std::string::npos);
  EXPECT_NE(json.find("\"confidence_percent\": 75"), std::string::npos);
  EXPECT_NE(json.find("\"image_file\": \"" + photos[0].filename().string() + "\""), std::string::npos);
  EXPECT_NE(json.find("\"image_stamp_ns\": " + std::to_string(kTime + 200 * kMs)), std::string::npos);
  EXPECT_NE(json.find("\"result_stamp_ns\": " + std::to_string(kTime + 300 * kMs)), std::string::npos);
  EXPECT_TRUE(Files("voice/passed", "", ".tmp").empty());
}

TEST_F(PhotoRecorderTest, FailedZeroAndLaterPassedRoundKeepSeparateMetadata) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
  Frame(*recorder, mission, kTime + 200 * kMs, kGreen);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::NOT_PASSED, kTime + 250 * kMs,
                        EventPhotoRecorder::VoiceConfidence{0, 0, {}});
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::WAIT_STARTED, kTime + 260 * kMs);
  Frame(*recorder, mission, kTime + 300 * kMs, kRed);
  recorder->RecordVoice(20, 7, 2, EventPhotoRecorder::PASSED, kTime + 300 * kMs,
                        EventPhotoRecorder::VoiceConfidence{3, 90, {0, 2, 5}});
  recorder->Flush();
  const auto failed = Files("voice/failed");
  ASSERT_EQ(failed.size(), 1U);
  ExpectColor(failed[0], kBlue);
  const auto failed_json = Files("voice/failed", "", ".json");
  const auto passed_json = Files("voice/passed", "", ".json");
  ASSERT_EQ(failed_json.size(), 1U);
  ASSERT_EQ(passed_json.size(), 1U);
  EXPECT_NE(ReadText(failed_json[0]).find("\"confidence_valid\": true"), std::string::npos);
  EXPECT_NE(ReadText(failed_json[0]).find("\"confidence_percent\": 0"), std::string::npos);
  EXPECT_NE(ReadText(passed_json[0]).find("\"confidence_percent\": 90"), std::string::npos);
  EXPECT_NE(ReadText(passed_json[0]).find("\"round_number\": 2"), std::string::npos);
}

TEST_F(PhotoRecorderTest, MissingSceneRetainsConfidenceWithNullPhoto) {
  auto recorder = Recorder();
  const auto mission = Mission(MissionPhase::kVerifyIdentity, 20);
  Frame(*recorder, mission, kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 1500 * kMs);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 1700 * kMs,
                        EventPhotoRecorder::VoiceConfidence{1, 50, {0}});
  recorder->Flush();
  EXPECT_TRUE(Files("voice/passed").empty());
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(records.size(), 1U);
  const auto json = ReadText(records[0]);
  EXPECT_NE(json.find("\"confidence_percent\": 50"), std::string::npos);
  EXPECT_NE(json.find("\"image_file\": null"), std::string::npos);
  EXPECT_NE(json.find("\"image_stamp_ns\": null"), std::string::npos);
  EXPECT_NE(json.find("\"photo_status\": \"missing_frame\""), std::string::npos);
}

TEST_F(PhotoRecorderTest, DisabledPhotosStillPersistConfidenceAndAvoidJsonOnlyCollisions) {
  config_.photos_enabled = false;
  for (int instance = 0; instance < 2; ++instance) {
    auto recorder = Recorder();
    Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
    recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime + 100 * kMs);
    recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime + 200 * kMs,
                          EventPhotoRecorder::VoiceConfidence{4, 100, {0, 1, 2, 3}});
  }
  EXPECT_TRUE(Files("voice/passed").empty());
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(records.size(), 2U);
  EXPECT_EQ(records[1].filename().string(), LocalSecond(kTime) + "_t7_r1_1.json");
  for (const auto &record : records) {
    const auto json = ReadText(record);
    EXPECT_NE(json.find("\"confidence_percent\": 100"), std::string::npos);
    EXPECT_NE(json.find("\"image_file\": null"), std::string::npos);
    EXPECT_NE(json.find("\"photo_status\": \"disabled\""), std::string::npos);
  }
}

TEST_F(PhotoRecorderTest, InvalidScoresAndDuplicateChannelsCannotConsumeTheRound) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime);
  for (const auto &invalid : std::vector<EventPhotoRecorder::VoiceConfidence>{
           {1, 25, {0}}, {2, 75, {0, 0}}, {1, 50, {6}}, {0, 0, {}}}) {
    recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime, invalid);
  }
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime,
                        EventPhotoRecorder::VoiceConfidence{6, 100, {0, 1, 2, 3, 4, 5}});
  recorder->Flush();
  EXPECT_EQ(recorder->GetMetrics().invalid_events, 4U);
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(records.size(), 1U);
  EXPECT_NE(ReadText(records[0]).find("\"vote_count\": 6"), std::string::npos);
}

TEST_F(PhotoRecorderTest, LegacyMissingMeasurementIsNullRatherThanMeasuredZero) {
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime);
  recorder->Flush();
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(records.size(), 1U);
  const auto json = ReadText(records[0]);
  EXPECT_NE(json.find("\"confidence_valid\": false"), std::string::npos);
  EXPECT_NE(json.find("\"vote_count\": null"), std::string::npos);
  EXPECT_NE(json.find("\"confidence_percent\": null"), std::string::npos);
}

TEST_F(PhotoRecorderTest, StaleJsonTemporaryFileDoesNotOverwriteHistoryOrLoseTheNewResult) {
  std::filesystem::create_directories(root_ / "voice/passed");
  const auto stale = root_ / "voice/passed" / (LocalSecond(kTime) + "_t7_r1.json.tmp");
  std::ofstream(stale) << "incomplete earlier record";
  auto recorder = Recorder();
  Frame(*recorder, Mission(MissionPhase::kVerifyIdentity, 20), kTime, kBlue);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::WAIT_STARTED, kTime);
  recorder->RecordVoice(20, 7, 1, EventPhotoRecorder::PASSED, kTime,
                        EventPhotoRecorder::VoiceConfidence{1, 50, {0}});
  recorder->Flush();
  const auto photos = Files("voice/passed");
  const auto records = Files("voice/passed", "", ".json");
  ASSERT_EQ(photos.size(), 1U);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(photos[0].filename().string(), LocalSecond(kTime) + "_t7_r1_1.jpg");
  EXPECT_EQ(photos[0].stem(), records[0].stem());
  EXPECT_EQ(ReadText(stale), "incomplete earlier record");
  EXPECT_EQ(recorder->GetMetrics().write_errors, 0U);
}

}  // namespace
