#include "vision_demo_host/modules/visualizer_recorder.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "vision_demo_host/modules/primary_recovery_debug.hpp"

namespace vision_demo_host {
namespace {

class BlockingOverlayArtifactWriter final : public OverlayArtifactWriter {
 public:
  struct State {
    std::mutex mutex;
    std::condition_variable changed;
    bool write_entered{false};
    bool allow_write{false};
    std::vector<cv::Mat> canvases;
  };

  explicit BlockingOverlayArtifactWriter(std::shared_ptr<State> state) : state_(std::move(state)) {}

  bool Open(const cv::Size &, double, std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const cv::Mat &canvas, std::string *error) override {
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->write_entered = true;
    state_->changed.notify_all();
    state_->changed.wait(lock, [this] { return state_->allow_write; });
    state_->canvases.push_back(canvas.clone());
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Close(std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<State> state_;
};

class BlockingOverlayArtifactWriterFactory final : public OverlayArtifactWriterFactory {
 public:
  explicit BlockingOverlayArtifactWriterFactory(std::shared_ptr<BlockingOverlayArtifactWriter::State> state)
      : state_(std::move(state)) {}

  std::unique_ptr<OverlayArtifactWriter> Create(const std::filesystem::path &, double) override {
    return std::make_unique<BlockingOverlayArtifactWriter>(state_);
  }

 private:
  std::shared_ptr<BlockingOverlayArtifactWriter::State> state_;
};

IdentityManagerResult VisibleIdentity() {
  IdentityManagerResult result;
  IdentityObservation identity;
  identity.semantic_id = 7;
  identity.state = IdentityState::kActive;
  identity.supporting_raw_track_id = 3;
  result.identities.push_back(identity);
  return result;
}

Track VisibleTrack() {
  Track track;
  track.id = 3;
  track.bbox = cv::Rect2f(1.0F, 1.0F, 4.0F, 4.0F);
  return track;
}

PrimaryTargetResult LockedPrimary(const Track &track) {
  PrimaryTargetResult primary;
  primary.state = PrimaryState::kLocked;
  primary.primary_track = track;
  primary.primary_target_id = 7;
  primary.raw_track_id = track.id;
  return primary;
}

TEST(VisualizerRecorderTest, FourLiveModesAreIndependentlyNamedAndValidated) {
  VisualizerRecorder::Config config;
  std::string error;

  EXPECT_EQ(VisualizerRecorder::ModeName(config), "inference_only");
  EXPECT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;

  config.enable_preview = true;
  EXPECT_EQ(VisualizerRecorder::ModeName(config), "preview");
  EXPECT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;

  config.enable_preview = false;
  config.enable_recording = true;
  config.recording_output_root = "/tmp";
  config.recording_path = "/tmp/issue83_overlay.mkv";
  EXPECT_EQ(VisualizerRecorder::ModeName(config), "record");
  EXPECT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;

  config.enable_preview = true;
  EXPECT_EQ(VisualizerRecorder::ModeName(config), "preview_record");
  EXPECT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;
}

TEST(VisualizerRecorderTest, ActiveRecordingRequiresFfv1MkvPathAndBoundedQueue) {
  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.recording_path = "/tmp/issue83_overlay.mp4";
  std::string error;
  EXPECT_FALSE(VisualizerRecorder::ValidateConfig(config, &error));
  EXPECT_NE(error.find(".mkv"), std::string::npos);

  config.recording_path = "/tmp/issue83_overlay.mkv";
  config.queue_capacity = 0;
  EXPECT_FALSE(VisualizerRecorder::ValidateConfig(config, &error));
  EXPECT_NE(error.find("queue_capacity"), std::string::npos);
}

TEST(VisualizerRecorderTest, ActiveRecordingRejectsPathsOutsideDiagnosticResultRoot) {
  const auto temp_root = std::filesystem::temp_directory_path() / "issue83_output_root_contract";
  const auto diagnostic_root = temp_root / "diagnostics";
  const auto clean_source_path = temp_root / "captures" / "source_take.mkv";

  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.recording_output_root = diagnostic_root.string();
  config.recording_path = clean_source_path.string();
  std::string error;
  EXPECT_FALSE(VisualizerRecorder::ValidateConfig(config, &error));
  EXPECT_NE(error.find("recording.output_root"), std::string::npos);

  config.recording_path = (diagnostic_root / "nested" / "overlay.mkv").string();
  EXPECT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;
}

TEST(VisualizerRecorderTest, ActiveRecordingRejectsDiagnosticRootSymlinkEscape) {
  const auto temp_root = std::filesystem::temp_directory_path() / "issue83_output_root_symlink";
  const auto diagnostic_root = temp_root / "diagnostics";
  const auto clean_source_root = temp_root / "captures";
  const auto escape_link = diagnostic_root / "escape";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(diagnostic_root);
  std::filesystem::create_directories(clean_source_root);
  std::error_code filesystem_error;
  std::filesystem::create_directory_symlink(clean_source_root, escape_link, filesystem_error);
  if (filesystem_error) {
    std::filesystem::remove_all(temp_root);
    GTEST_SKIP() << "Cannot create temporary symlink: " << filesystem_error.message();
  }

  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.recording_output_root = diagnostic_root.string();
  config.protected_source_dataset_roots = {clean_source_root.string()};
  const auto escaped_child = clean_source_root / "must_not_be_created";
  config.recording_path = (escape_link / "must_not_be_created" / "overlay.mkv").string();
  std::string error;
  ASSERT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;

  VisualizerRecorder recorder(config);
  EXPECT_FALSE(recorder.Initialize(cv::Size(8, 8), &error));
  EXPECT_NE(error.find("recording.output_root"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(escaped_child));
  recorder.Shutdown();
  std::filesystem::remove_all(temp_root);
}

TEST(VisualizerRecorderTest, ActiveRecordingRejectsProtectedCaptureRootAndRootSymlink) {
  const auto temp_root = std::filesystem::temp_directory_path() / "issue83_protected_capture_root";
  const auto clean_source_root = temp_root / "captures";
  const auto symlinked_output_root = temp_root / "diagnostics_link";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(clean_source_root);

  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.protected_source_dataset_roots = {clean_source_root.string()};
  config.recording_output_root = clean_source_root.string();
  config.recording_path = (clean_source_root / "overlay.mkv").string();
  std::string error;
  EXPECT_FALSE(VisualizerRecorder::ValidateConfig(config, &error));
  EXPECT_NE(error.find("protected clean source"), std::string::npos);

  std::error_code filesystem_error;
  std::filesystem::create_directory_symlink(clean_source_root, symlinked_output_root, filesystem_error);
  if (filesystem_error) {
    std::filesystem::remove_all(temp_root);
    GTEST_SKIP() << "Cannot create temporary symlink: " << filesystem_error.message();
  }
  config.recording_output_root = symlinked_output_root.string();
  config.recording_path = (symlinked_output_root / "overlay.mkv").string();
  ASSERT_TRUE(VisualizerRecorder::ValidateConfig(config, &error)) << error;
  VisualizerRecorder recorder(config);
  EXPECT_FALSE(recorder.Initialize(cv::Size(8, 8), &error));
  EXPECT_NE(error.find("protected clean source"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(clean_source_root / "overlay.mkv"));
  recorder.Shutdown();
  std::filesystem::remove_all(temp_root);
}

TEST(VisualizerRecorderTest, PreviewLifecycleFailsClearlyWithoutAnInteractiveDisplay) {
  const char *previous_display = std::getenv("DISPLAY");
  const std::string saved_display = previous_display == nullptr ? "" : previous_display;
  unsetenv("DISPLAY");

  VisualizerRecorder::Config config;
  config.enable_preview = true;
  VisualizerRecorder recorder(config);
  std::string error;
  EXPECT_FALSE(recorder.Initialize(cv::Size(8, 8), &error));
  EXPECT_NE(error.find("DISPLAY"), std::string::npos);
  recorder.Shutdown();

  if (previous_display == nullptr) {
    unsetenv("DISPLAY");
  } else {
    setenv("DISPLAY", saved_display.c_str(), 1);
  }
}

TEST(VisualizerRecorderTest, FullQueueDropsNewestOverlayWithoutBlockingInferenceSubmit) {
  auto state = std::make_shared<BlockingOverlayArtifactWriter::State>();
  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.recording_output_root = "/tmp";
  config.recording_path = "/tmp/issue83_overlay.mkv";
  config.queue_capacity = 1;
  VisualizerRecorder recorder(
      config, std::make_unique<BlockingOverlayArtifactWriterFactory>(state));
  std::string error;
  ASSERT_TRUE(recorder.Initialize(cv::Size(8, 8), &error)) << error;

  const cv::Mat first(8, 8, CV_8UC3, cv::Scalar(3, 5, 7));
  const auto identity = VisibleIdentity();
  recorder.Submit(first, {VisibleTrack()}, PrimaryTargetResult{}, identity);
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->changed.wait_for(lock, std::chrono::seconds(1), [&state] {
      return state->write_entered;
    }));
  }

  auto submit = std::async(std::launch::async, [&recorder, &identity] {
    const cv::Mat second(8, 8, CV_8UC3, cv::Scalar(11, 13, 17));
    recorder.Submit(second, {VisibleTrack()}, PrimaryTargetResult{}, identity);
    const cv::Mat third(8, 8, CV_8UC3, cv::Scalar(19, 23, 29));
    recorder.Submit(third, {VisibleTrack()}, PrimaryTargetResult{}, identity);
  });
  EXPECT_EQ(submit.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->allow_write = true;
  }
  state->changed.notify_all();
  submit.get();
  recorder.Shutdown();

  const auto metrics = recorder.Metrics();
  EXPECT_EQ(metrics.submitted_frames, 3U);
  EXPECT_EQ(metrics.enqueued_frames, 2U);
  EXPECT_EQ(metrics.dropped_frames(), 1U);
  EXPECT_EQ(metrics.written_frames, 2U);
  EXPECT_EQ(metrics.write_errors, 0U);
}

TEST(VisualizerRecorderTest, DefaultWriterPersistsTrackingIdentityAndPrimaryOverlay) {
  const auto output_root = std::filesystem::temp_directory_path() / "issue83_overlay_writer_test_results";
  const auto output_path = output_root / "overlay.mkv";
  std::filesystem::remove_all(output_root);
  VisualizerRecorder::Config config;
  config.enable_recording = true;
  config.recording_output_root = output_root.string();
  config.recording_path = output_path.string();
  config.queue_capacity = 2;
  VisualizerRecorder recorder(config);
  std::string error;
  ASSERT_TRUE(recorder.Initialize(cv::Size(160, 100), &error)) << error;

  const cv::Mat frame(100, 160, CV_8UC3, cv::Scalar(3, 5, 7));
  Track track = VisibleTrack();
  track.bbox = cv::Rect2f(24.0F, 40.0F, 72.0F, 32.0F);
  const PrimaryTargetResult primary = LockedPrimary(track);
  IdentityManagerResult identity_result = VisibleIdentity();
  identity_result.primary_semantic_id = 7;
  identity_result.identities.front().primary = true;
  recorder.Submit(frame, {track}, primary, identity_result);
  recorder.Shutdown();

  const auto metrics = recorder.Metrics();
  ASSERT_EQ(metrics.written_frames, 1U) << recorder.LastError();
  EXPECT_TRUE(std::filesystem::exists(output_path));
  cv::VideoCapture capture(output_path.string());
  cv::Mat decoded;
  ASSERT_TRUE(capture.read(decoded));
  EXPECT_EQ(decoded.size(), frame.size());
  EXPECT_EQ(frame.at<cv::Vec3b>(0, 0), cv::Vec3b(3, 5, 7));

  cv::Mat expected = frame.clone();
  const cv::Scalar primary_color(0, 0, 255);
  cv::rectangle(expected, track.bbox, primary_color, 2);
  cv::putText(expected, "id=7 ACTIVE raw=3", CompactOverlayTrackLabelPoint(expected.size(), track.bbox),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, primary_color, 2);
  cv::putText(expected, "LOCKED id=7 raw=3", cv::Point(20, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 2);
  EXPECT_EQ(decoded.at<cv::Vec3b>(40, 24), cv::Vec3b(0, 0, 255));
  EXPECT_EQ(cv::norm(decoded, expected, cv::NORM_INF), 0.0)
      << "FFV1 artifact must contain the deterministic track/identity/primary canvas";
  std::filesystem::remove_all(output_root);
}

}  // namespace
}  // namespace vision_demo_host
