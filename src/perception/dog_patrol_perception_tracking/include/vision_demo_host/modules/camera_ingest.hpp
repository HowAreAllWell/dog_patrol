#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace vision_demo_host {

class CameraIngest {
 public:
  enum class Backend {
    kGstreamer,
    kHikMvs,
  };

  struct Config {
    Backend backend{Backend::kGstreamer};
    std::string gstreamer_pipeline;
    std::string hik_mvs_model;
    std::string hik_mvs_serial;
    int width{1280};
    int height{1024};
    double fps{30.0};
    int timeout_ms{1000};
  };

  CameraIngest();
  ~CameraIngest();

  bool Open(const Config &config, std::string *error);
  bool Read(cv::Mat *frame, std::string *error);
  void Close();

 private:
  struct Impl;
  Config config_{};
  cv::VideoCapture capture_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_demo_host
