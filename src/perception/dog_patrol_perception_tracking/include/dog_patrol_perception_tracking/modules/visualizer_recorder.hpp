#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

// The worker-only boundary for diagnostic overlay recording. Implementations must
// accept the already-rendered canvas and must never run on the inference thread.
class OverlayArtifactWriter {
 public:
  virtual ~OverlayArtifactWriter() = default;

  virtual bool Open(const cv::Size &frame_size, double fps, std::string *error) = 0;
  virtual bool Write(const cv::Mat &canvas, std::string *error) = 0;
  virtual bool Close(std::string *error) = 0;
};

class OverlayArtifactWriterFactory {
 public:
  virtual ~OverlayArtifactWriterFactory() = default;

  virtual std::unique_ptr<OverlayArtifactWriter> Create(const std::filesystem::path &output_path,
                                                         double fps) = 0;
};

class VisualizerRecorder {
 public:
  struct Config {
    bool enable_preview{false};
    bool enable_recording{false};
    // Result artifacts are constrained to this root. A live camera has no source
    // directory to compare against, so this is the explicit trust boundary that
    // keeps diagnostic overlays out of clean capture datasets.
    std::string recording_output_root{"data/diagnostics/live_overlays"};
    std::string recording_path{"data/diagnostics/live_overlays/vision_demo_overlay.mkv"};
    // The active live camera has no input directory; project capture roots are
    // consequently declared as protected paths rather than inferred per frame.
    std::vector<std::string> protected_source_dataset_roots{"data/captures"};
    double recording_fps{30.0};
    std::size_t queue_capacity{4};
    int semantic_id_max_missing_frames{180};
    int sid_feat_bank_size{30};
    float sid_recover_sim_thresh_strict{0.85F};
    float sid_overlap_iou_freeze{0.10F};
    int sid_occlusion_protect_frames{30};
    int sid_split_stable_frames{3};
    int sid_merge_hold_frames{2};
    float sid_app_w{0.70F};
    float sid_geo_w{0.20F};
    float sid_time_w{0.10F};
    float sid_active_assign_max_cost{0.55F};
    float sid_recovery_max_cost{0.45F};
    float sid_raw_continuity_max_cost{0.55F};
    float sid_min_assignment_margin{0.08F};
    int sid_stable_frames_before_feature_update{3};
    bool sid_merged_requires_overlap{true};
    bool sid_reid_enable{true};
    std::string sid_reid_backend{"light"};
    std::string sid_reid_model_path{};
    float sid_recover_sim_thresh_relaxed{0.75F};
    int sid_recover_relaxed_max_missing_frames{180};
    int sid_reid_input_width{128};
    int sid_reid_input_height{256};
  };

  struct PercentileSummary {
    std::size_t samples{0};
    double p50_ms{0.0};
    double p95_ms{0.0};
    double p99_ms{0.0};
  };

  struct MetricsSnapshot {
    std::uint64_t submitted_frames{0};
    std::uint64_t enqueued_frames{0};
    std::uint64_t queue_dropped_frames{0};
    std::uint64_t render_dropped_frames{0};
    std::uint64_t write_dropped_frames{0};
    std::uint64_t rendered_frames{0};
    std::uint64_t previewed_frames{0};
    std::uint64_t written_frames{0};
    std::uint64_t render_errors{0};
    std::uint64_t write_errors{0};
    double submitted_fps{0.0};
    double rendered_fps{0.0};
    double previewed_fps{0.0};
    double written_fps{0.0};
    PercentileSummary queue_wait;
    PercentileSummary render;
    PercentileSummary write;

    std::uint64_t dropped_frames() const {
      return queue_dropped_frames + render_dropped_frames + write_dropped_frames;
    }
  };

  explicit VisualizerRecorder(
      Config config,
      std::unique_ptr<OverlayArtifactWriterFactory> artifact_writer_factory = nullptr);
  ~VisualizerRecorder();

  VisualizerRecorder(const VisualizerRecorder &) = delete;
  VisualizerRecorder &operator=(const VisualizerRecorder &) = delete;

  static bool ValidateConfig(const Config &config, std::string *error);
  static std::string ModeName(const Config &config);

  bool Initialize(const cv::Size &frame_size, std::string *error);

  // Transfers the caller's BGR8 ownership into a bounded, non-blocking output queue.
  // Pass a moved CameraIngest::AcquiredFrame::bgr8 to avoid a second frame copy.
  void Submit(cv::Mat frame, std::vector<Track> tracks, PrimaryTargetResult primary,
              IdentityManagerResult identity_result, std::string primary_decision_reason = {},
              std::string primary_reject_reason = {});
  void Shutdown();

  MetricsSnapshot Metrics() const;
  std::string LastError() const;

 private:
  struct Job {
    cv::Mat frame;
    std::vector<Track> tracks;
    PrimaryTargetResult primary;
    IdentityManagerResult identity_result;
    std::string primary_decision_reason;
    std::string primary_reject_reason;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  static cv::Mat BuildOverlayCanvas(const Job &job);
  static PercentileSummary Summarize(const std::vector<double> &samples);
  static void Observe(std::vector<double> *samples, double milliseconds);
  void AddError(const std::string &error);
  void WorkerLoop();

  Config config_;
  std::unique_ptr<OverlayArtifactWriterFactory> artifact_writer_factory_;
  cv::Size frame_size_;
  bool initialized_{false};
  bool stopping_{false};
  std::thread worker_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_changed_;
  std::deque<Job> queue_;

  mutable std::mutex metrics_mutex_;
  MetricsSnapshot metrics_;
  std::vector<double> queue_wait_ms_;
  std::vector<double> render_ms_;
  std::vector<double> write_ms_;
  std::chrono::steady_clock::time_point started_at_{};
  std::string last_error_;
};

}  // namespace dog_patrol_perception_tracking
