#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace vision_demo_host {

class CameraIngest {
 public:
  enum class BayerInterpolation : unsigned int {
    kFast = 0,
    kBalanced = 1,
    kOptimal = 2,
    kOptimalPlus = 3,
  };

  struct Config {
    std::string hik_mvs_model;
    std::string hik_mvs_serial;
    int width{1280};
    int height{1024};
    double fps{30.0};
    int timeout_ms{1000};
    BayerInterpolation bayer_interpolation{BayerInterpolation::kOptimal};
    bool bayer_smoothing{false};
  };

  struct AcquiredFrame {
    cv::Mat bgr8;
    std::uint64_t source_timestamp_ns{0};
    std::int64_t sdk_host_timestamp{0};
    std::uint32_t camera_frame_number{0};
    bool camera_frame_number_available{false};
    std::uint64_t device_timestamp_ticks{0};
    std::uint32_t source_pixel_type{0};
    std::string source_pixel_type_name;
    int width{0};
    int height{0};
    std::size_t source_payload_bytes{0};
    std::uint32_t camera_lost_packets{0};
    double acquisition_ms{0.0};
    double conversion_ms{0.0};
    double copy_ms{0.0};
  };

  struct SourceFrameMetadata {
    std::uint64_t source_timestamp_ns{0};
    std::int64_t sdk_host_timestamp{0};
    std::uint32_t camera_frame_number{0};
    std::uint64_t device_timestamp_ticks{0};
    std::uint32_t source_pixel_type{0};
    int width{0};
    int height{0};
    std::size_t source_payload_bytes{0};
    std::uint32_t camera_lost_packets{0};
  };

  struct PercentileSummary {
    std::size_t samples{0};
    double p50_ms{0.0};
    double p95_ms{0.0};
    double p99_ms{0.0};
  };

  class StageTiming {
   public:
    void ObserveMilliseconds(double milliseconds);
    PercentileSummary Summary() const;
    void Clear();

   private:
    static constexpr std::size_t kMaxSamples = 2048;
    std::vector<double> samples_;
  };

  class FrameContinuity {
   public:
    std::uint64_t Observe(std::uint32_t frame_number);
    std::uint64_t DroppedFrames() const;
    std::uint64_t NonContiguousFrames() const;
    void Reset();

   private:
    bool has_previous_{false};
    std::uint32_t previous_{0};
    std::uint64_t dropped_frames_{0};
    std::uint64_t non_contiguous_frames_{0};
  };

  struct MetricsSnapshot {
    std::uint64_t acquired_frames{0};
    std::uint64_t acquisition_failures{0};
    std::uint64_t dropped_frames{0};
    std::uint64_t non_contiguous_frames{0};
    std::uint64_t camera_lost_packets{0};
    PercentileSummary acquisition;
    PercentileSummary conversion;
    PercentileSummary copy;
  };

  CameraIngest();
  ~CameraIngest();

  static bool ValidateConfig(const Config &config, std::string *error);
  static bool ParseBayerInterpolation(const std::string &value,
                                      BayerInterpolation *interpolation,
                                      std::string *error);
  static std::string BayerInterpolationName(BayerInterpolation interpolation);
  static std::string PixelTypeName(std::uint32_t pixel_type);
  static void ApplySourceFrameMetadata(const SourceFrameMetadata &metadata,
                                       AcquiredFrame *frame);

  bool Open(const Config &config, std::string *error);
  bool Read(AcquiredFrame *frame, std::string *error);
  MetricsSnapshot Metrics() const;
  void Close();

 private:
  struct Impl;
  Config config_{};
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_demo_host
