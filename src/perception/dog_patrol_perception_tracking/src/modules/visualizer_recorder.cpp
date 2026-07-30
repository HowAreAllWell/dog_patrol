#include "vision_demo_host/modules/visualizer_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <sstream>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_demo_host/modules/ffv1_mkv_writer.hpp"
#include "vision_demo_host/modules/primary_recovery_debug.hpp"

namespace vision_demo_host {
namespace {

constexpr char kPreviewWindowName[] = "vision_demo_host";

bool Fail(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool ResolveAbsolutePath(const std::filesystem::path &path, const char *name,
                         std::filesystem::path *resolved, std::string *error) {
  std::error_code filesystem_error;
  *resolved = std::filesystem::absolute(path, filesystem_error).lexically_normal();
  if (filesystem_error) {
    return Fail(error, std::string("Unable to resolve ") + name + ": " + filesystem_error.message());
  }
  return true;
}

bool IsAtOrBelow(const std::filesystem::path &candidate, const std::filesystem::path &root) {
  auto candidate_part = candidate.begin();
  for (auto root_part = root.begin(); root_part != root.end(); ++root_part, ++candidate_part) {
    if (candidate_part == candidate.end() || *candidate_part != *root_part) {
      return false;
    }
  }
  return true;
}

bool IsPathWithinRoot(const std::filesystem::path &candidate, const std::filesystem::path &root,
                      std::string *error) {
  std::filesystem::path absolute_candidate;
  std::filesystem::path absolute_root;
  if (!ResolveAbsolutePath(candidate, "recording.path", &absolute_candidate, error) ||
      !ResolveAbsolutePath(root, "recording.output_root", &absolute_root, error)) {
    return false;
  }
  if (!IsAtOrBelow(absolute_candidate, absolute_root)) {
    return Fail(error,
                "recording.path must remain under recording.output_root so overlay results cannot overlap clean source datasets");
  }
  return true;
}

bool IsOutputRootSeparateFromProtectedSources(
    const std::filesystem::path &output_root,
    const std::vector<std::string> &protected_source_dataset_roots,
    const bool canonicalize, std::string *error) {
  std::filesystem::path resolved_output_root;
  std::error_code filesystem_error;
  if (canonicalize) {
    resolved_output_root = std::filesystem::weakly_canonical(output_root, filesystem_error);
    if (filesystem_error) {
      return Fail(error, "Unable to canonicalize recording.output_root: " + filesystem_error.message());
    }
  } else if (!ResolveAbsolutePath(output_root, "recording.output_root", &resolved_output_root, error)) {
    return false;
  }

  for (const std::string &source_root : protected_source_dataset_roots) {
    if (source_root.empty()) {
      return Fail(error, "protected source dataset root must not be empty");
    }
    std::filesystem::path resolved_source_root;
    if (canonicalize) {
      resolved_source_root = std::filesystem::weakly_canonical(source_root, filesystem_error);
      if (filesystem_error) {
        return Fail(error, "Unable to canonicalize protected source dataset root: " +
                               filesystem_error.message());
      }
    } else if (!ResolveAbsolutePath(source_root, "protected source dataset root", &resolved_source_root,
                                    error)) {
      return false;
    }
    if (IsAtOrBelow(resolved_output_root, resolved_source_root) ||
        IsAtOrBelow(resolved_source_root, resolved_output_root)) {
      return Fail(error,
                  "recording.output_root must not overlap a protected clean source dataset root");
    }
  }
  return true;
}

bool IsCanonicalPathWithinRoot(const std::filesystem::path &candidate,
                               const std::filesystem::path &root, std::string *error) {
  std::error_code filesystem_error;
  const std::filesystem::path canonical_candidate =
      std::filesystem::weakly_canonical(candidate, filesystem_error);
  if (filesystem_error) {
    return Fail(error, "Unable to canonicalize recording.path: " + filesystem_error.message());
  }
  const std::filesystem::path canonical_root = std::filesystem::weakly_canonical(root, filesystem_error);
  if (filesystem_error) {
    return Fail(error, "Unable to canonicalize recording.output_root: " + filesystem_error.message());
  }
  return IsPathWithinRoot(canonical_candidate, canonical_root, error);
}

const IdentityObservation *FindIdentityByRawTrack(const IdentityManagerResult &result, const int raw_track_id) {
  for (const auto &identity : result.identities) {
    if (identity.supporting_raw_track_id.has_value() && *identity.supporting_raw_track_id == raw_track_id) {
      return &identity;
    }
  }
  return nullptr;
}

class Ffv1OverlayArtifactWriter final : public OverlayArtifactWriter {
 public:
  explicit Ffv1OverlayArtifactWriter(std::filesystem::path output_path)
      : output_path_(std::move(output_path)) {}

  bool Open(const cv::Size &frame_size, const double fps, std::string *error) override {
    Ffv1MkvWriter::Config config;
    config.output_path = output_path_;
    config.width = frame_size.width;
    config.height = frame_size.height;
    config.fps = fps;
    writer_ = std::make_unique<Ffv1MkvWriter>(std::move(config));
    if (!writer_->Open(error)) {
      writer_.reset();
      return false;
    }
    return true;
  }

  bool Write(const cv::Mat &canvas, std::string *error) override {
    return writer_ != nullptr && writer_->Write(canvas, error);
  }

  bool Close(std::string *error) override {
    if (writer_ == nullptr) {
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }
    const bool closed = writer_->Close(error);
    writer_.reset();
    return closed;
  }

 private:
  std::filesystem::path output_path_;
  std::unique_ptr<Ffv1MkvWriter> writer_;
};

class Ffv1OverlayArtifactWriterFactory final : public OverlayArtifactWriterFactory {
 public:
  std::unique_ptr<OverlayArtifactWriter> Create(const std::filesystem::path &output_path,
                                                 const double) override {
    return std::make_unique<Ffv1OverlayArtifactWriter>(output_path);
  }
};

}  // namespace

VisualizerRecorder::VisualizerRecorder(
    Config config, std::unique_ptr<OverlayArtifactWriterFactory> artifact_writer_factory)
    : config_(std::move(config)), artifact_writer_factory_(std::move(artifact_writer_factory)) {
  if (artifact_writer_factory_ == nullptr) {
    artifact_writer_factory_ = std::make_unique<Ffv1OverlayArtifactWriterFactory>();
  }
}

VisualizerRecorder::~VisualizerRecorder() { Shutdown(); }

bool VisualizerRecorder::ValidateConfig(const Config &config, std::string *error) {
  if (config.queue_capacity == 0U) {
    return Fail(error, "visualization.queue_capacity must be positive");
  }
  if (!config.enable_recording) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  if (std::filesystem::path(config.recording_path).extension() != ".mkv") {
    return Fail(error, "recording.path must end in .mkv because active overlay recording is FFV1/MKV");
  }
  if (config.recording_output_root.empty()) {
    return Fail(error, "recording.output_root must be set for diagnostic overlay recording");
  }
  if (!IsOutputRootSeparateFromProtectedSources(config.recording_output_root,
                                                config.protected_source_dataset_roots, false, error)) {
    return false;
  }
  if (!IsPathWithinRoot(config.recording_path, config.recording_output_root, error)) {
    return false;
  }
  if (!std::isfinite(config.recording_fps) || config.recording_fps <= 0.0) {
    return Fail(error, "recording.fps must be finite and positive");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::string VisualizerRecorder::ModeName(const Config &config) {
  if (config.enable_preview && config.enable_recording) {
    return "preview_record";
  }
  if (config.enable_preview) {
    return "preview";
  }
  if (config.enable_recording) {
    return "record";
  }
  return "inference_only";
}

bool VisualizerRecorder::Initialize(const cv::Size &frame_size, std::string *error) {
  if (initialized_) {
    return Fail(error, "visualizer recorder is already initialized");
  }
  if (!ValidateConfig(config_, error)) {
    return false;
  }
  if (frame_size.width <= 0 || frame_size.height <= 0) {
    return Fail(error, "visualizer recorder requires positive frame dimensions");
  }
  if (config_.enable_preview) {
    const char *display = std::getenv("DISPLAY");
    if (display == nullptr || std::string(display).empty()) {
      return Fail(error, "overlay preview requires an interactive DISPLAY; use inference-only or record-only headless");
    }
  }
  if (config_.enable_recording) {
    const std::filesystem::path output_root(config_.recording_output_root);
    const std::filesystem::path parent = std::filesystem::path(config_.recording_path).parent_path();
    std::error_code filesystem_error;
    std::filesystem::create_directories(output_root, filesystem_error);
    if (filesystem_error) {
      return Fail(error, "Failed to create diagnostic overlay output root: " + filesystem_error.message());
    }
    if (!IsOutputRootSeparateFromProtectedSources(
            output_root, config_.protected_source_dataset_roots, true, error)) {
      return false;
    }
    // Resolve the requested artifact before creating its parent. In particular,
    // this prevents a lexical child of the diagnostic root from following a
    // symlink into a clean source dataset merely because its final subdirectory
    // does not exist yet.
    if (!IsCanonicalPathWithinRoot(config_.recording_path, output_root, error)) {
      return false;
    }
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, filesystem_error);
      if (filesystem_error) {
        return Fail(error, "Failed to create diagnostic overlay output directory: " + filesystem_error.message());
      }
    }
    if (!IsCanonicalPathWithinRoot(config_.recording_path, output_root, error)) {
      return false;
    }
  }

  frame_size_ = frame_size;
  initialized_ = true;
  started_at_ = std::chrono::steady_clock::now();
  if (config_.enable_preview || config_.enable_recording) {
    worker_ = std::thread(&VisualizerRecorder::WorkerLoop, this);
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

void VisualizerRecorder::Submit(cv::Mat frame, std::vector<Track> tracks, PrimaryTargetResult primary,
                                IdentityManagerResult identity_result,
                                std::string primary_decision_reason,
                                std::string primary_reject_reason) {
  if (!initialized_ || (!config_.enable_preview && !config_.enable_recording)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++metrics_.submitted_frames;
  }
  if (frame.empty() || frame.type() != CV_8UC3 || frame.size() != frame_size_) {
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++metrics_.render_dropped_frames;
    }
    AddError("overlay submit frame violates the BGR8 frame contract");
    return;
  }

  bool enqueued = false;
  {
    // This mutex protects only the O(1) deque move. The worker releases it before
    // canvas rendering, GUI calls, encoder work, and filesystem I/O, so inference
    // can never wait for those stages.
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!stopping_ && queue_.size() < config_.queue_capacity) {
      Job job;
      job.frame = std::move(frame);
      job.tracks = std::move(tracks);
      job.primary = std::move(primary);
      job.identity_result = std::move(identity_result);
      job.primary_decision_reason = std::move(primary_decision_reason);
      job.primary_reject_reason = std::move(primary_reject_reason);
      job.enqueued_at = std::chrono::steady_clock::now();
      queue_.push_back(std::move(job));
      enqueued = true;
    }
  }
  {
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    if (enqueued) {
      ++metrics_.enqueued_frames;
    } else {
      ++metrics_.queue_dropped_frames;
    }
  }
  if (enqueued) {
    queue_changed_.notify_one();
  }
}

void VisualizerRecorder::Shutdown() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = true;
  }
  queue_changed_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

VisualizerRecorder::MetricsSnapshot VisualizerRecorder::Metrics() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  MetricsSnapshot snapshot = metrics_;
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
  if (elapsed > 0.0) {
    snapshot.submitted_fps = static_cast<double>(snapshot.submitted_frames) / elapsed;
    snapshot.rendered_fps = static_cast<double>(snapshot.rendered_frames) / elapsed;
    snapshot.previewed_fps = static_cast<double>(snapshot.previewed_frames) / elapsed;
    snapshot.written_fps = static_cast<double>(snapshot.written_frames) / elapsed;
  }
  snapshot.queue_wait = Summarize(queue_wait_ms_);
  snapshot.render = Summarize(render_ms_);
  snapshot.write = Summarize(write_ms_);
  return snapshot;
}

std::string VisualizerRecorder::LastError() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  return last_error_;
}

cv::Mat VisualizerRecorder::BuildOverlayCanvas(const Job &job) {
  // The worker owns rendering. Cloning here keeps the BGR8 inference/source frame
  // immutable even when callers retain a shared OpenCV view of it.
  cv::Mat canvas = job.frame.clone();
  const int primary_semantic_id = job.primary.primary_target_id;
  for (const auto &track : job.tracks) {
    const auto *identity = FindIdentityByRawTrack(job.identity_result, track.id);
    if (identity == nullptr || identity->semantic_id < 0) {
      continue;
    }
    const int semantic_id = identity->semantic_id;
    const bool is_primary = (primary_semantic_id > 0 && semantic_id == primary_semantic_id);
    const cv::Scalar color = is_primary ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::rectangle(canvas, track.bbox, color, 2);
    std::ostringstream label;
    label << "id=" << semantic_id << " " << IdentityStateToString(identity->state) << " raw=" << track.id;
    cv::putText(canvas, label.str(), CompactOverlayTrackLabelPoint(canvas.size(), track.bbox),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
  }
  const std::string primary_line = BuildPrimaryOverlayLine(
      job.primary, job.identity_result, job.primary_decision_reason, job.primary_reject_reason);
  cv::putText(canvas, primary_line, cv::Point(20, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 2);
  return canvas;
}

VisualizerRecorder::PercentileSummary VisualizerRecorder::Summarize(const std::vector<double> &samples) {
  PercentileSummary result;
  result.samples = samples.size();
  if (samples.empty()) {
    return result;
  }
  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](const double fraction) {
    const auto rank = static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sorted.size())));
    return sorted[std::max<std::size_t>(1U, rank) - 1U];
  };
  result.p50_ms = percentile(0.50);
  result.p95_ms = percentile(0.95);
  result.p99_ms = percentile(0.99);
  return result;
}

void VisualizerRecorder::Observe(std::vector<double> *samples, const double milliseconds) {
  constexpr std::size_t kMaxSamples = 2048U;
  if (!std::isfinite(milliseconds) || milliseconds < 0.0 || samples == nullptr) {
    return;
  }
  if (samples->size() == kMaxSamples) {
    samples->erase(samples->begin());
  }
  samples->push_back(milliseconds);
}

void VisualizerRecorder::AddError(const std::string &error) {
  if (error.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  last_error_ = error;
}

void VisualizerRecorder::WorkerLoop() {
  std::unique_ptr<OverlayArtifactWriter> artifact_writer;
  bool writer_failed = false;
  bool preview_failed = false;
  bool preview_window_open = false;

  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_changed_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty() && stopping_) {
        break;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
    }

    const auto dequeued_at = std::chrono::steady_clock::now();
    const auto render_started_at = dequeued_at;
    cv::Mat canvas;
    try {
      canvas = BuildOverlayCanvas(job);
    } catch (const cv::Exception &exception) {
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.render_errors;
        ++metrics_.render_dropped_frames;
      }
      AddError("overlay canvas render failed: " + std::string(exception.what()));
      continue;
    }
    const auto rendered_at = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++metrics_.rendered_frames;
      Observe(&queue_wait_ms_, std::chrono::duration<double, std::milli>(dequeued_at - job.enqueued_at).count());
      Observe(&render_ms_, std::chrono::duration<double, std::milli>(rendered_at - render_started_at).count());
    }

    if (config_.enable_preview) {
      if (preview_failed) {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.render_dropped_frames;
      } else {
        try {
          if (!preview_window_open) {
            cv::namedWindow(kPreviewWindowName, cv::WINDOW_NORMAL);
            preview_window_open = true;
          }
          cv::imshow(kPreviewWindowName, canvas);
          cv::waitKey(1);
          std::lock_guard<std::mutex> lock(metrics_mutex_);
          ++metrics_.previewed_frames;
        } catch (const cv::Exception &exception) {
          preview_failed = true;
          {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            ++metrics_.render_errors;
            ++metrics_.render_dropped_frames;
          }
          AddError("overlay preview failed: " + std::string(exception.what()));
        }
      }
    }

    if (config_.enable_recording) {
      if (writer_failed) {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.write_dropped_frames;
        continue;
      }
      std::string error;
      if (artifact_writer == nullptr) {
        artifact_writer = artifact_writer_factory_->Create(config_.recording_path, config_.recording_fps);
        if (artifact_writer == nullptr || !artifact_writer->Open(canvas.size(), config_.recording_fps, &error)) {
          writer_failed = true;
          {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            ++metrics_.write_errors;
            ++metrics_.write_dropped_frames;
          }
          AddError(error.empty() ? "Failed to create active FFV1 overlay writer" : error);
          continue;
        }
      }
      const auto write_started_at = std::chrono::steady_clock::now();
      if (!artifact_writer->Write(canvas, &error)) {
        writer_failed = true;
        {
          std::lock_guard<std::mutex> lock(metrics_mutex_);
          ++metrics_.write_errors;
          ++metrics_.write_dropped_frames;
        }
        AddError(error.empty() ? "Failed to write active FFV1 overlay frame" : error);
        continue;
      }
      const auto wrote_at = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.written_frames;
        Observe(&write_ms_, std::chrono::duration<double, std::milli>(wrote_at - write_started_at).count());
      }
    }
  }

  if (artifact_writer != nullptr) {
    std::string error;
    if (!artifact_writer->Close(&error)) {
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.write_errors;
      }
      AddError(error.empty() ? "Failed to finalize active FFV1 overlay writer" : error);
    }
  }
  if (preview_window_open) {
    try {
      cv::destroyWindow(kPreviewWindowName);
    } catch (const cv::Exception &exception) {
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++metrics_.render_errors;
      }
      AddError("overlay preview cleanup failed: " + std::string(exception.what()));
    }
  }
}

}  // namespace vision_demo_host
