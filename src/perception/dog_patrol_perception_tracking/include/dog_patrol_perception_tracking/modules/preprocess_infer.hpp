#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class PreprocessInfer {
 public:
  struct Config {
    std::string detector_runtime_path;
    float raw_conf_threshold{0.25F};
    int input_width{640};
    int input_height{640};
    bool enable_fake_detection{false};
    bool enable_timing_metrics{false};
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

  struct MetricsSnapshot {
    PercentileSummary resize;
    PercentileSummary border;
    PercentileSummary channel_swap;
    PercentileSummary normalize;
    PercentileSummary layout;
    PercentileSummary h2d;
    PercentileSummary tensor_rt;
    PercentileSummary d2h;
    PercentileSummary parser;
    PercentileSummary total;
  };

  explicit PreprocessInfer(Config config);
  ~PreprocessInfer();

  PreprocessInfer(const PreprocessInfer &) = delete;
  PreprocessInfer &operator=(const PreprocessInfer &) = delete;
  PreprocessInfer(PreprocessInfer &&) noexcept;
  PreprocessInfer &operator=(PreprocessInfer &&) noexcept;

  bool Initialize(std::string *error);
  std::vector<Detection> Infer(const cv::Mat &frame);
  MetricsSnapshot Metrics() const;
  void ResetMetrics();

 private:
  struct Impl;
  Config config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_patrol_perception_tracking
