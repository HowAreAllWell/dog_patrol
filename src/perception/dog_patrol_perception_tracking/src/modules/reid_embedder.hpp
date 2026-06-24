#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace vision_demo_host {

class ReIdEmbedder {
 public:
  struct Config {
    std::string backend{"light"};
    std::string model_path{};
    int input_width{128};
    int input_height{256};
    std::string light_profile{"identity"};
    int light_h_bins{16};
    int light_s_bins{8};
  };

  ReIdEmbedder();
  explicit ReIdEmbedder(Config config);

  bool Initialize(std::string *error);
  bool IsEnabled() const;
  bool IsReady() const { return ready_; }
  bool UsesOnnx() const;
  cv::Mat Extract(const cv::Mat &frame, const cv::Rect2f &bbox) const;
  std::vector<float> ExtractVector(const cv::Mat &frame, const cv::Rect2f &bbox) const;

  static std::string NormalizeBackend(std::string backend);
  static bool BackendUsesOnnx(const std::string &backend);

 private:
  Config config_;
  cv::dnn::Net net_;
  bool ready_{false};
};

}  // namespace vision_demo_host
