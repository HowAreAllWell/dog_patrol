#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vision_demo_host {

class AppearanceFeatureService {
 public:
  struct Config {
    std::string backend{"light"};
    std::string model_path{};
    int input_width{128};
    int input_height{256};
    int light_h_bins{16};
    int light_s_bins{8};
  };

  enum class Profile {
    kTracker,
    kIdentity,
  };

  AppearanceFeatureService();
  AppearanceFeatureService(Config config, Profile profile);
  ~AppearanceFeatureService();

  AppearanceFeatureService(const AppearanceFeatureService &other);
  AppearanceFeatureService &operator=(const AppearanceFeatureService &other);
  AppearanceFeatureService(AppearanceFeatureService &&) noexcept;
  AppearanceFeatureService &operator=(AppearanceFeatureService &&) noexcept;

  bool Initialize(std::string *error);
  bool UsesOnnx() const;
  bool IsReady() const;

  cv::Mat ExtractTrackerFeature(const cv::Mat &frame, const cv::Rect2f &bbox) const;
  std::vector<float> ExtractIdentityFeature(const cv::Mat &frame, const cv::Rect2f &bbox) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_demo_host
