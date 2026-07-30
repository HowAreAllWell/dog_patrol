#include "vision_demo_host/modules/ffv1_capture_workflow.hpp"

#include <condition_variable>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace vision_demo_host {
namespace {

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

std::string TakeName(const std::string &prefix, const unsigned int sequence) {
  std::ostringstream name;
  name << prefix << "_" << std::setfill('0') << std::setw(3) << sequence;
  return name.str();
}

}  // namespace

struct Ffv1CaptureWorkflow::ActiveTake {
  mutable std::mutex mutex;
  std::condition_variable wake_writer;
  std::deque<CaptureFrame> queue;
  std::unique_ptr<CaptureArtifactWriter> writer;
  CaptureTakeSummary summary;
  CameraIngest::AcquiredFrame previous_frame;
  bool has_previous_frame{false};
  bool accept_frames{true};
  bool writer_failed{false};
  bool stop_requested{false};
  std::thread writer_thread;
};

Ffv1CaptureWorkflow::Ffv1CaptureWorkflow(
    Config config, std::unique_ptr<CaptureArtifactWriterFactory> writer_factory)
    : config_(std::move(config)), writer_factory_(std::move(writer_factory)) {}

Ffv1CaptureWorkflow::~Ffv1CaptureWorkflow() { Interrupt(0); }

bool Ffv1CaptureWorkflow::HandleControl(const CaptureControl control,
                                        const std::uint64_t wall_time_ns,
                                        std::string *error) {
  switch (control) {
    case CaptureControl::kStart:
      return Start(wall_time_ns, error);
    case CaptureControl::kStop:
      return Finalize(true, false, wall_time_ns, error);
    case CaptureControl::kAddMarker: {
      std::shared_ptr<ActiveTake> take;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != CaptureState::kRecording || !active_take_) {
          return Fail(error, "marker requires an active recording take");
        }
        take = active_take_;
      }
      std::lock_guard<std::mutex> take_lock(take->mutex);
      CaptureMarker marker;
      marker.index = static_cast<unsigned int>(take->summary.markers.size());
      marker.wall_time_ns = wall_time_ns;
      marker.elapsed_seconds =
          wall_time_ns >= take->summary.descriptor.started_wall_time_ns
              ? static_cast<double>(wall_time_ns - take->summary.descriptor.started_wall_time_ns) /
                    1'000'000'000.0
              : 0.0;
      marker.note = "manual_marker";
      take->summary.markers.push_back(std::move(marker));
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    case CaptureControl::kTogglePreviewInfo: {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == CaptureState::kStopped) {
        return Fail(error, "preview information cannot be toggled after quit");
      }
      preview_info_enabled_ = !preview_info_enabled_;
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    case CaptureControl::kQuit:
      return Finalize(true, true, wall_time_ns, error);
  }
  return Fail(error, "unsupported capture control");
}

void Ffv1CaptureWorkflow::Submit(const CameraIngest::AcquiredFrame &frame) {
  std::shared_ptr<ActiveTake> take;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CaptureState::kRecording || !active_take_) {
      return;
    }
    take = active_take_;
  }

  std::lock_guard<std::mutex> take_lock(take->mutex);
  if (!take->accept_frames) {
    return;
  }
  CaptureFrame captured;
  captured.capture_index = take->summary.captured_frames++;
  if (take->has_previous_frame) {
    take->summary.camera_frame_gaps += CountCameraFrameGap(take->previous_frame, frame);
  }
  take->previous_frame = frame;
  take->previous_frame.bgr8.release();
  take->has_previous_frame = true;
  if (take->writer_failed) {
    ++take->summary.dropped_frames;
    return;
  }
  if (take->queue.size() >= config_.queue_capacity) {
    ++take->summary.dropped_frames;
    return;
  }
  captured.source = frame;
  captured.source.bgr8 = frame.bgr8.clone();
  take->queue.push_back(std::move(captured));
  take->wake_writer.notify_one();
}

void Ffv1CaptureWorkflow::Interrupt(const std::uint64_t wall_time_ns) {
  std::string ignored_error;
  Finalize(false, true, wall_time_ns, &ignored_error);
}

CaptureSnapshot Ffv1CaptureWorkflow::Snapshot() const {
  CaptureSnapshot snapshot;
  std::shared_ptr<ActiveTake> take;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot.state = state_;
    snapshot.preview_info_enabled = preview_info_enabled_;
    snapshot.completed_takes = completed_takes_;
    take = active_take_;
  }
  if (take) {
    std::lock_guard<std::mutex> take_lock(take->mutex);
    snapshot.active_take_name = take->summary.descriptor.name;
    snapshot.active_take = take->summary;
  }
  return snapshot;
}

CaptureFrameContract Ffv1CaptureWorkflow::ContractFrom(
    const CameraIngest::AcquiredFrame &frame, const Config &config) {
  CaptureFrameContract contract;
  contract.source_pixel_type = frame.source_pixel_type;
  contract.source_pixel_type_name = frame.source_pixel_type_name.empty()
                                        ? CameraIngest::PixelTypeName(frame.source_pixel_type)
                                        : frame.source_pixel_type_name;
  contract.width = frame.width > 0 ? frame.width : frame.bgr8.cols;
  contract.height = frame.height > 0 ? frame.height : frame.bgr8.rows;
  contract.source_payload_bytes = frame.source_payload_bytes;
  contract.bayer_interpolation = CameraIngest::BayerInterpolationName(config.bayer_interpolation);
  contract.bayer_smoothing = config.bayer_smoothing;
  return contract;
}

std::uint64_t Ffv1CaptureWorkflow::CountCameraFrameGap(
    const CameraIngest::AcquiredFrame &previous, const CameraIngest::AcquiredFrame &current) {
  if (!previous.camera_frame_number_available || !current.camera_frame_number_available) {
    return 0;
  }
  const std::uint32_t expected = previous.camera_frame_number + 1U;
  if (current.camera_frame_number <= expected) {
    return 0;
  }
  return static_cast<std::uint64_t>(current.camera_frame_number - expected);
}

bool Ffv1CaptureWorkflow::Start(const std::uint64_t wall_time_ns, std::string *error) {
  std::shared_ptr<ActiveTake> take = std::make_shared<ActiveTake>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CaptureState::kStopped) {
      return Fail(error, "capture workflow has already quit");
    }
    if (state_ != CaptureState::kStandby) {
      return Fail(error, "start requires STANDBY state");
    }
    if (writer_factory_ == nullptr) {
      return Fail(error, "capture artifact writer factory is unavailable");
    }
    if (config_.queue_capacity == 0U) {
      return Fail(error, "capture queue capacity must be positive");
    }
    take->writer = writer_factory_->Create();
    if (take->writer == nullptr) {
      return Fail(error, "capture artifact writer factory returned null");
    }
    take->summary.descriptor.sequence = next_take_sequence_++;
    take->summary.descriptor.name = TakeName(config_.take_name_prefix,
                                             take->summary.descriptor.sequence);
    take->summary.descriptor.started_wall_time_ns = wall_time_ns;
    active_take_ = take;
    state_ = CaptureState::kRecording;
  }
  take->writer_thread = std::thread(&Ffv1CaptureWorkflow::RunWriter, take, config_);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool Ffv1CaptureWorkflow::Finalize(const bool complete, const bool stop_after_finalize,
                                   const std::uint64_t wall_time_ns, std::string *error) {
  std::shared_ptr<ActiveTake> take;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CaptureState::kStopped) {
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    if (state_ == CaptureState::kStandby) {
      state_ = stop_after_finalize ? CaptureState::kStopped : CaptureState::kStandby;
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    if (state_ != CaptureState::kRecording || !active_take_) {
      return Fail(error, "capture take is already finalizing");
    }
    state_ = CaptureState::kFinalizing;
    take = active_take_;
  }

  {
    std::lock_guard<std::mutex> take_lock(take->mutex);
    take->accept_frames = false;
    take->stop_requested = true;
    take->summary.finished_wall_time_ns = wall_time_ns;
  }
  take->wake_writer.notify_one();
  if (take->writer_thread.joinable()) {
    take->writer_thread.join();
  }

  CaptureTakeSummary summary;
  {
    std::lock_guard<std::mutex> take_lock(take->mutex);
    take->summary.complete = complete && take->summary.writer_opened &&
                             take->summary.write_errors == 0U;
    summary = take->summary;
  }
  std::string finish_error;
  if (summary.writer_opened && !take->writer->Finish(summary, &finish_error)) {
    std::lock_guard<std::mutex> take_lock(take->mutex);
    ++take->summary.write_errors;
    take->summary.complete = false;
    take->summary.last_write_error = finish_error;
    summary = take->summary;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    completed_takes_.push_back(summary);
    active_take_.reset();
    state_ = stop_after_finalize ? CaptureState::kStopped : CaptureState::kStandby;
  }
  if (!summary.last_write_error.empty()) {
    return Fail(error, summary.last_write_error);
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

void Ffv1CaptureWorkflow::RunWriter(const std::shared_ptr<ActiveTake> &take,
                                    const Config &config) {
  while (true) {
    CaptureFrame frame;
    {
      std::unique_lock<std::mutex> lock(take->mutex);
      take->wake_writer.wait(lock, [&take] {
        return !take->queue.empty() || take->stop_requested;
      });
      if (take->queue.empty() && take->stop_requested) {
        return;
      }
      frame = std::move(take->queue.front());
      take->queue.pop_front();
    }

    std::string write_error;
    bool needs_begin = false;
    CaptureTakeDescriptor descriptor;
    CaptureFrameContract contract;
    bool writer_failed = false;
    {
      std::lock_guard<std::mutex> lock(take->mutex);
      writer_failed = take->writer_failed;
      if (!writer_failed && !take->summary.writer_opened) {
        take->summary.frame_contract = ContractFrom(frame.source, config);
        descriptor = take->summary.descriptor;
        contract = take->summary.frame_contract;
        needs_begin = true;
      }
    }
    if (writer_failed) {
      std::lock_guard<std::mutex> lock(take->mutex);
      ++take->summary.dropped_frames;
      continue;
    }
    bool write_ok = true;
    if (needs_begin) {
      write_ok = take->writer->Begin(descriptor, contract, &write_error);
      if (write_ok) {
        std::lock_guard<std::mutex> lock(take->mutex);
        take->summary.writer_opened = true;
      }
    }
    if (write_ok) {
      write_ok = take->writer->Write(frame, &write_error);
    }
    if (write_ok) {
      std::lock_guard<std::mutex> lock(take->mutex);
      ++take->summary.written_frames;
    } else {
      std::lock_guard<std::mutex> lock(take->mutex);
      ++take->summary.write_errors;
      ++take->summary.dropped_frames;
      take->summary.last_write_error = write_error.empty() ? "capture artifact write failed"
                                                            : write_error;
      take->writer_failed = true;
    }
  }
}

}  // namespace vision_demo_host
