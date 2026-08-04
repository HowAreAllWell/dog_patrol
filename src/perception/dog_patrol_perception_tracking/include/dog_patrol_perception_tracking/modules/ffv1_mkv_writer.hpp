#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

namespace dog_patrol_perception_tracking {

// Owns the native FFmpeg FFV1/MKV encoding lifecycle for a fixed BGR8 frame contract.
// Capture and live-overlay artifacts share this writer so their active lossless path
// has one codec/threading implementation.
class Ffv1MkvWriter {
 public:
  struct Config {
    std::filesystem::path output_path;
    int width{0};
    int height{0};
    double fps{0.0};
  };

  struct EncoderInfo {
    int thread_count{0};
    int slice_count{0};
    std::string pixel_format{"bgr0"};
  };

  explicit Ffv1MkvWriter(Config config);
  ~Ffv1MkvWriter();

  Ffv1MkvWriter(const Ffv1MkvWriter &) = delete;
  Ffv1MkvWriter &operator=(const Ffv1MkvWriter &) = delete;

  bool Open(std::string *error);
  bool Write(const cv::Mat &bgr8, std::string *error);
  bool Close(std::string *error);

  const EncoderInfo &Info() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_patrol_perception_tracking
