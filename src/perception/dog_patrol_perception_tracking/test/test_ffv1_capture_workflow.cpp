#include "vision_demo_host/modules/ffv1_capture_workflow.hpp"
#include "vision_demo_host/modules/ffv1_capture_artifact_writer.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace vision_demo_host {
namespace {

struct CapturedArtifacts {
  std::vector<CaptureTakeDescriptor> descriptors;
  std::vector<CaptureFrame> frames;
  std::vector<CaptureTakeSummary> summaries;
};

class InMemoryCaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit InMemoryCaptureArtifactWriter(std::shared_ptr<CapturedArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  bool Begin(const CaptureTakeDescriptor &descriptor,
             const CaptureFrameContract &,
             std::string *error) override {
    artifacts_->descriptors.push_back(descriptor);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const CaptureFrame &frame, std::string *error) override {
    artifacts_->frames.push_back(frame);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Finish(const CaptureTakeSummary &summary, std::string *error) override {
    artifacts_->summaries.push_back(summary);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<CapturedArtifacts> artifacts_;
};

class InMemoryCaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  explicit InMemoryCaptureArtifactWriterFactory(std::shared_ptr<CapturedArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  std::unique_ptr<CaptureArtifactWriter> Create() override {
    return std::make_unique<InMemoryCaptureArtifactWriter>(artifacts_);
  }

 private:
  std::shared_ptr<CapturedArtifacts> artifacts_;
};

struct BlockingBeginArtifacts {
  std::mutex mutex;
  std::condition_variable changed;
  bool begin_entered{false};
  bool allow_begin{false};
};

class BlockingBeginCaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit BlockingBeginCaptureArtifactWriter(std::shared_ptr<BlockingBeginArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  bool Begin(const CaptureTakeDescriptor &, const CaptureFrameContract &,
             std::string *error) override {
    std::unique_lock<std::mutex> lock(artifacts_->mutex);
    artifacts_->begin_entered = true;
    artifacts_->changed.notify_all();
    artifacts_->changed.wait(lock, [this] { return artifacts_->allow_begin; });
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const CaptureFrame &, std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Finish(const CaptureTakeSummary &, std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<BlockingBeginArtifacts> artifacts_;
};

class BlockingBeginCaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  explicit BlockingBeginCaptureArtifactWriterFactory(std::shared_ptr<BlockingBeginArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  std::unique_ptr<CaptureArtifactWriter> Create() override {
    return std::make_unique<BlockingBeginCaptureArtifactWriter>(artifacts_);
  }

 private:
  std::shared_ptr<BlockingBeginArtifacts> artifacts_;
};

struct FailingWriteArtifacts {
  std::size_t writes{0};
  std::vector<CaptureTakeSummary> summaries;
};

class FailingWriteCaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit FailingWriteCaptureArtifactWriter(std::shared_ptr<FailingWriteArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  bool Begin(const CaptureTakeDescriptor &, const CaptureFrameContract &,
             std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const CaptureFrame &, std::string *error) override {
    ++artifacts_->writes;
    if (error != nullptr) {
      *error = "simulated FFV1 write failure";
    }
    return false;
  }

  bool Finish(const CaptureTakeSummary &summary, std::string *error) override {
    artifacts_->summaries.push_back(summary);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<FailingWriteArtifacts> artifacts_;
};

class FailingWriteCaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  explicit FailingWriteCaptureArtifactWriterFactory(std::shared_ptr<FailingWriteArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  std::unique_ptr<CaptureArtifactWriter> Create() override {
    return std::make_unique<FailingWriteCaptureArtifactWriter>(artifacts_);
  }

 private:
  std::shared_ptr<FailingWriteArtifacts> artifacts_;
};

struct BlockingWriteArtifacts {
  std::mutex mutex;
  std::condition_variable changed;
  bool write_entered{false};
  bool allow_write{false};
  std::vector<CaptureTakeSummary> summaries;
};

class BlockingWriteCaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit BlockingWriteCaptureArtifactWriter(std::shared_ptr<BlockingWriteArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  bool Begin(const CaptureTakeDescriptor &, const CaptureFrameContract &,
             std::string *error) override {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Write(const CaptureFrame &, std::string *error) override {
    std::unique_lock<std::mutex> lock(artifacts_->mutex);
    artifacts_->write_entered = true;
    artifacts_->changed.notify_all();
    artifacts_->changed.wait(lock, [this] { return artifacts_->allow_write; });
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool Finish(const CaptureTakeSummary &summary, std::string *error) override {
    std::lock_guard<std::mutex> lock(artifacts_->mutex);
    artifacts_->summaries.push_back(summary);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<BlockingWriteArtifacts> artifacts_;
};

class BlockingWriteCaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  explicit BlockingWriteCaptureArtifactWriterFactory(std::shared_ptr<BlockingWriteArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  std::unique_ptr<CaptureArtifactWriter> Create() override {
    return std::make_unique<BlockingWriteCaptureArtifactWriter>(artifacts_);
  }

 private:
  std::shared_ptr<BlockingWriteArtifacts> artifacts_;
};

struct FailingBeginArtifacts {
  std::size_t begin_attempts{0};
  std::vector<CaptureTakeSummary> summaries;
};

class FailingBeginCaptureArtifactWriter final : public CaptureArtifactWriter {
 public:
  explicit FailingBeginCaptureArtifactWriter(std::shared_ptr<FailingBeginArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  bool Begin(const CaptureTakeDescriptor &, const CaptureFrameContract &,
             std::string *error) override {
    ++artifacts_->begin_attempts;
    if (error != nullptr) {
      *error = "simulated FFV1 initialization failure";
    }
    return false;
  }

  bool Write(const CaptureFrame &, std::string *error) override {
    if (error != nullptr) {
      *error = "Write must not run after initialization failure";
    }
    return false;
  }

  bool Finish(const CaptureTakeSummary &summary, std::string *error) override {
    artifacts_->summaries.push_back(summary);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  std::shared_ptr<FailingBeginArtifacts> artifacts_;
};

class FailingBeginCaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  explicit FailingBeginCaptureArtifactWriterFactory(std::shared_ptr<FailingBeginArtifacts> artifacts)
      : artifacts_(std::move(artifacts)) {}

  std::unique_ptr<CaptureArtifactWriter> Create() override {
    return std::make_unique<FailingBeginCaptureArtifactWriter>(artifacts_);
  }

 private:
  std::shared_ptr<FailingBeginArtifacts> artifacts_;
};

CameraIngest::AcquiredFrame MakeFrame(const std::uint32_t camera_frame_number,
                                      const std::uint64_t source_timestamp_ns) {
  CameraIngest::AcquiredFrame frame;
  frame.bgr8 = cv::Mat(2, 3, CV_8UC3, cv::Scalar(7, 11, 13));
  frame.source_timestamp_ns = source_timestamp_ns;
  frame.camera_frame_number = camera_frame_number;
  frame.camera_frame_number_available = true;
  frame.source_pixel_type = 0x0108000AU;
  frame.source_pixel_type_name = "BayerGB8";
  frame.width = frame.bgr8.cols;
  frame.height = frame.bgr8.rows;
  frame.source_payload_bytes = frame.bgr8.total() * frame.bgr8.elemSize();
  return frame;
}

TEST(Ffv1CaptureWorkflowTest, StartFrameStopFinalizesOneTakeWithObservedCountsAndContract) {
  auto artifacts = std::make_shared<CapturedArtifacts>();
  Ffv1CaptureWorkflow::Config config;
  config.take_name_prefix = "capture";
  config.queue_capacity = 2;
  config.bayer_interpolation = CameraIngest::BayerInterpolation::kBalanced;
  Ffv1CaptureWorkflow workflow(
      config, std::make_unique<InMemoryCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  EXPECT_EQ(workflow.Snapshot().state, CaptureState::kRecording);

  workflow.Submit(MakeFrame(41U, 1'500U));

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error)) << error;
  EXPECT_EQ(workflow.Snapshot().state, CaptureState::kStandby);
  ASSERT_EQ(artifacts->descriptors.size(), 1U);
  EXPECT_EQ(artifacts->descriptors.front().name, "capture_001");
  ASSERT_EQ(artifacts->frames.size(), 1U);
  EXPECT_EQ(artifacts->frames.front().capture_index, 0U);
  EXPECT_EQ(artifacts->frames.front().source.camera_frame_number, 41U);
  ASSERT_EQ(artifacts->summaries.size(), 1U);
  const CaptureTakeSummary &summary = artifacts->summaries.front();
  EXPECT_TRUE(summary.complete);
  EXPECT_EQ(summary.captured_frames, 1U);
  EXPECT_EQ(summary.written_frames, 1U);
  EXPECT_EQ(summary.dropped_frames, 0U);
  EXPECT_EQ(summary.write_errors, 0U);
  EXPECT_EQ(summary.frame_contract.width, 3);
  EXPECT_EQ(summary.frame_contract.height, 2);
  EXPECT_EQ(summary.frame_contract.source_pixel_type_name, "BayerGB8");
  EXPECT_EQ(summary.frame_contract.bayer_interpolation, "balanced");
}

TEST(Ffv1CaptureWorkflowTest, DefaultConfigurationRecordsBalancedBayerWithoutSmoothing) {
  auto artifacts = std::make_shared<CapturedArtifacts>();
  Ffv1CaptureWorkflow workflow(
      Ffv1CaptureWorkflow::Config{},
      std::make_unique<InMemoryCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(41U, 1'500U));
  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error)) << error;

  ASSERT_EQ(artifacts->summaries.size(), 1U);
  const CaptureFrameContract &contract = artifacts->summaries.front().frame_contract;
  EXPECT_EQ(contract.bayer_interpolation, "balanced");
  EXPECT_FALSE(contract.bayer_smoothing);
}

TEST(Ffv1CaptureWorkflowTest, AcquisitionDoesNotBlockWhileTheWriterInitializes) {
  auto artifacts = std::make_shared<BlockingBeginArtifacts>();
  Ffv1CaptureWorkflow::Config config;
  config.queue_capacity = 2;
  Ffv1CaptureWorkflow workflow(
      config, std::make_unique<BlockingBeginCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(1U, 1'100U));
  {
    std::unique_lock<std::mutex> lock(artifacts->mutex);
    ASSERT_TRUE(artifacts->changed.wait_for(lock, std::chrono::seconds(1), [&artifacts] {
      return artifacts->begin_entered;
    }));
  }

  auto submit_second_frame = std::async(std::launch::async, [&workflow] {
    workflow.Submit(MakeFrame(2U, 1'200U));
  });
  EXPECT_EQ(submit_second_frame.wait_for(std::chrono::milliseconds(100)),
            std::future_status::ready);

  {
    std::lock_guard<std::mutex> lock(artifacts->mutex);
    artifacts->allow_begin = true;
  }
  artifacts->changed.notify_all();
  submit_second_frame.get();
  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error)) << error;
}

TEST(Ffv1CaptureWorkflowTest, MarkerAndQuitFinalizeSeparateTakes) {
  auto artifacts = std::make_shared<CapturedArtifacts>();
  Ffv1CaptureWorkflow workflow(
      Ffv1CaptureWorkflow::Config{},
      std::make_unique<InMemoryCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(1U, 1'100U));
  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kAddMarker, 1'150U, &error)) << error;
  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStop, 1'200U, &error)) << error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 2'000U, &error)) << error;
  workflow.Submit(MakeFrame(2U, 2'100U));
  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kQuit, 2'200U, &error)) << error;

  const CaptureSnapshot snapshot = workflow.Snapshot();
  EXPECT_EQ(snapshot.state, CaptureState::kStopped);
  ASSERT_EQ(snapshot.completed_takes.size(), 2U);
  EXPECT_EQ(snapshot.completed_takes[0].descriptor.name, "take_001");
  EXPECT_TRUE(snapshot.completed_takes[0].complete);
  ASSERT_EQ(snapshot.completed_takes[0].markers.size(), 1U);
  EXPECT_EQ(snapshot.completed_takes[0].markers[0].note, "manual_marker");
  EXPECT_EQ(snapshot.completed_takes[1].descriptor.name, "take_002");
  EXPECT_TRUE(snapshot.completed_takes[1].complete);
}

TEST(Ffv1CaptureWorkflowTest, WriterFailureIsExplicitAndDropsRemainingQueuedFrames) {
  auto artifacts = std::make_shared<FailingWriteArtifacts>();
  Ffv1CaptureWorkflow::Config config;
  config.queue_capacity = 2;
  Ffv1CaptureWorkflow workflow(
      config, std::make_unique<FailingWriteCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(1U, 1'100U));
  workflow.Submit(MakeFrame(2U, 1'200U));
  EXPECT_FALSE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error));
  EXPECT_NE(error.find("simulated FFV1 write failure"), std::string::npos);
  EXPECT_EQ(artifacts->writes, 1U);
  ASSERT_EQ(artifacts->summaries.size(), 1U);
  EXPECT_FALSE(artifacts->summaries[0].complete);
  EXPECT_EQ(artifacts->summaries[0].captured_frames, 2U);
  EXPECT_EQ(artifacts->summaries[0].written_frames, 0U);
  EXPECT_EQ(artifacts->summaries[0].dropped_frames, 2U);
  EXPECT_EQ(artifacts->summaries[0].write_errors, 1U);
}

TEST(Ffv1CaptureWorkflowTest, FullBoundedQueueDropsNewestCapturedFrame) {
  auto artifacts = std::make_shared<BlockingWriteArtifacts>();
  Ffv1CaptureWorkflow::Config config;
  config.queue_capacity = 1;
  Ffv1CaptureWorkflow workflow(
      config, std::make_unique<BlockingWriteCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(1U, 1'100U));
  {
    std::unique_lock<std::mutex> lock(artifacts->mutex);
    ASSERT_TRUE(artifacts->changed.wait_for(lock, std::chrono::seconds(1), [&artifacts] {
      return artifacts->write_entered;
    }));
  }
  workflow.Submit(MakeFrame(2U, 1'200U));
  workflow.Submit(MakeFrame(3U, 1'300U));
  {
    std::lock_guard<std::mutex> lock(artifacts->mutex);
    artifacts->allow_write = true;
  }
  artifacts->changed.notify_all();

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error)) << error;
  std::lock_guard<std::mutex> lock(artifacts->mutex);
  ASSERT_EQ(artifacts->summaries.size(), 1U);
  EXPECT_EQ(artifacts->summaries[0].captured_frames, 3U);
  EXPECT_EQ(artifacts->summaries[0].written_frames, 2U);
  EXPECT_EQ(artifacts->summaries[0].dropped_frames, 1U);
}

TEST(Ffv1CaptureWorkflowTest, InitializationFailureFinalizesAnIncompleteTakeWithTheFirstError) {
  auto artifacts = std::make_shared<FailingBeginArtifacts>();
  Ffv1CaptureWorkflow workflow(
      Ffv1CaptureWorkflow::Config{},
      std::make_unique<FailingBeginCaptureArtifactWriterFactory>(artifacts));
  std::string error;

  ASSERT_TRUE(workflow.HandleControl(CaptureControl::kStart, 1'000U, &error)) << error;
  workflow.Submit(MakeFrame(1U, 1'100U));
  EXPECT_FALSE(workflow.HandleControl(CaptureControl::kStop, 2'000U, &error));
  EXPECT_NE(error.find("simulated FFV1 initialization failure"), std::string::npos);
  EXPECT_EQ(artifacts->begin_attempts, 1U);
  ASSERT_EQ(artifacts->summaries.size(), 1U);
  EXPECT_FALSE(artifacts->summaries[0].complete);
  EXPECT_FALSE(artifacts->summaries[0].writer_opened);
  EXPECT_EQ(artifacts->summaries[0].last_write_error,
            "simulated FFV1 initialization failure");
}

TEST(Ffv1CaptureArtifactWriterTest, PersistsLosslessBgr8FrameAndTakeMetadata) {
  const auto output_dir = std::filesystem::temp_directory_path() /
                          "vision_demo_ffv1_capture_writer_test";
  std::filesystem::remove_all(output_dir);
  Ffv1CaptureArtifactWriterFactory::Config config;
  config.session_directory = output_dir;
  config.requested_fps = 30.0;
  config.mvs_model = "MV-CU013-A0UC";
  Ffv1CaptureArtifactWriterFactory factory(config);
  auto writer = factory.Create();
  CaptureTakeDescriptor descriptor;
  descriptor.sequence = 1U;
  descriptor.name = "capture_001";
  descriptor.started_wall_time_ns = 1'000U;
  CaptureFrameContract contract;
  contract.source_pixel_type = 0x0108000AU;
  contract.source_pixel_type_name = "BayerGB8";
  contract.width = 4;
  contract.height = 2;
  contract.source_payload_bytes = 24U;
  contract.bayer_interpolation = "balanced";
  std::string error;
  if (!writer->Begin(descriptor, contract, &error)) {
    GTEST_SKIP() << "This OpenCV build has no usable FFV1 writer: " << error;
  }

  CaptureFrame frame;
  frame.capture_index = 0U;
  frame.source = MakeFrame(41U, 1'500U);
  frame.source.bgr8 = cv::Mat(2, 4, CV_8UC3, cv::Scalar(7, 11, 13));
  frame.source.width = frame.source.bgr8.cols;
  frame.source.height = frame.source.bgr8.rows;
  frame.source.source_payload_bytes = frame.source.bgr8.total() * frame.source.bgr8.elemSize();
  ASSERT_TRUE(writer->Write(frame, &error)) << error;
  CaptureTakeSummary summary;
  summary.descriptor = descriptor;
  summary.frame_contract = contract;
  summary.complete = true;
  summary.writer_opened = true;
  summary.finished_wall_time_ns = 2'000U;
  summary.captured_frames = 1U;
  summary.written_frames = 1U;
  CaptureMarker marker;
  marker.index = 0U;
  marker.wall_time_ns = 1'750U;
  marker.elapsed_seconds = 0.00000075;
  marker.note = "manual_marker";
  summary.markers.push_back(marker);
  ASSERT_TRUE(writer->Finish(summary, &error)) << error;

  const auto take_dir = output_dir / descriptor.name;
  EXPECT_TRUE(std::filesystem::exists(take_dir / "video.mkv"));
  EXPECT_TRUE(std::filesystem::exists(take_dir / "frame_timestamps.csv"));
  EXPECT_TRUE(std::filesystem::exists(take_dir / "markers.csv"));
  std::ifstream metadata(take_dir / "metadata.json");
  const std::string metadata_text((std::istreambuf_iterator<char>(metadata)),
                                  std::istreambuf_iterator<char>());
  EXPECT_NE(metadata_text.find("\"state\": \"complete\""), std::string::npos);
  EXPECT_NE(metadata_text.find("\"codec\": \"FFV1\""), std::string::npos);
  EXPECT_NE(metadata_text.find("\"backend\": \"native_ffmpeg\""), std::string::npos);
  EXPECT_NE(metadata_text.find("\"thread_type\": \"slice\""), std::string::npos);
  EXPECT_NE(metadata_text.find("\"captured_frames\": 1"), std::string::npos);
  EXPECT_NE(metadata_text.find("\"source_pixel_type_name\": \"BayerGB8\""),
            std::string::npos);

  cv::VideoCapture video((take_dir / "video.mkv").string());
  cv::Mat decoded;
  ASSERT_TRUE(video.read(decoded));
  EXPECT_EQ(decoded.size(), frame.source.bgr8.size());
  EXPECT_EQ(cv::norm(decoded, frame.source.bgr8, cv::NORM_INF), 0.0);
  std::filesystem::remove_all(output_dir);
}

TEST(Ffv1CaptureArtifactWriterTest, PersistsIncompleteMetadataWhenFfv1NeverOpened) {
  const auto output_dir = std::filesystem::temp_directory_path() /
                          "vision_demo_ffv1_capture_incomplete_test";
  std::filesystem::remove_all(output_dir);
  Ffv1CaptureArtifactWriterFactory::Config config;
  config.session_directory = output_dir;
  config.requested_fps = 30.0;
  Ffv1CaptureArtifactWriterFactory factory(config);
  auto writer = factory.Create();
  CaptureTakeSummary summary;
  summary.descriptor.sequence = 1U;
  summary.descriptor.name = "take_001";
  summary.descriptor.started_wall_time_ns = 1'000U;
  summary.finished_wall_time_ns = 1'000'001'000U;
  summary.complete = false;
  summary.last_write_error = "mandatory FFV1 writer unavailable";

  std::string error;
  ASSERT_TRUE(writer->Finish(summary, &error)) << error;
  const auto take_dir = output_dir / summary.descriptor.name;
  EXPECT_FALSE(std::filesystem::exists(take_dir / "video.mkv"));
  EXPECT_TRUE(std::filesystem::exists(take_dir / "frame_timestamps.csv"));
  EXPECT_TRUE(std::filesystem::exists(take_dir / "markers.csv"));
  std::ifstream metadata(take_dir / "metadata.json");
  const std::string metadata_text((std::istreambuf_iterator<char>(metadata)),
                                  std::istreambuf_iterator<char>());
  EXPECT_NE(metadata_text.find("\"state\": \"incomplete\""), std::string::npos);
  EXPECT_NE(metadata_text.find("\"writer_opened\": false"), std::string::npos);
  EXPECT_NE(metadata_text.find("mandatory FFV1 writer unavailable"), std::string::npos);
  EXPECT_NE(metadata_text.find("\"stream_fps_is_nominal\": true"), std::string::npos);
  EXPECT_NE(metadata_text.find("\"written_fps\": 0"), std::string::npos);
  std::filesystem::remove_all(output_dir);
}

}  // namespace
}  // namespace vision_demo_host
