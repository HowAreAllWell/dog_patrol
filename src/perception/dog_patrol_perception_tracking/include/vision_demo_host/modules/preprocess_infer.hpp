#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class PreprocessInfer {
 public:
  struct Config {
    std::string detector_runtime_path;
    float raw_conf_threshold{0.25F};
    int input_width{640};
    int input_height{640};
    bool enable_fake_detection{false};
  };

  explicit PreprocessInfer(Config config);
  ~PreprocessInfer();

  PreprocessInfer(const PreprocessInfer &) = delete;
  PreprocessInfer &operator=(const PreprocessInfer &) = delete;
  PreprocessInfer(PreprocessInfer &&) noexcept;
  PreprocessInfer &operator=(PreprocessInfer &&) noexcept;

  bool Initialize(std::string *error);
  std::vector<Detection> Infer(const cv::Mat &frame);

 private:
  struct Impl;
  Config config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_demo_host
