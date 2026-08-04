#include "dog_patrol_perception_tracking/modules/appearance_feature_service.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "reid_embedder.hpp"

namespace dog_patrol_perception_tracking {
namespace {

const char *LightProfileName(const AppearanceFeatureService::Profile profile) {
  switch (profile) {
    case AppearanceFeatureService::Profile::kTracker:
      return "light_tracker";
    case AppearanceFeatureService::Profile::kIdentity:
    default:
      return "light_identity";
  }
}

}  // namespace

class AppearanceFeatureService::Impl {
 public:
  Impl() : Impl(Config{}, Profile::kIdentity) {}

  Impl(Config config, const Profile profile)
      : config(std::move(config)), profile(profile) {
    this->config.input_width = std::max(16, this->config.input_width);
    this->config.input_height = std::max(16, this->config.input_height);
    this->config.light_h_bins = std::max(4, this->config.light_h_bins);
    this->config.light_s_bins = std::max(4, this->config.light_s_bins);
    backend = ReIdEmbedder(ReIdEmbedder::Config{this->config.backend,
                                                this->config.model_path,
                                                this->config.input_width,
                                                this->config.input_height,
                                                LightProfileName(profile),
                                                this->config.light_h_bins,
                                                this->config.light_s_bins});
  }

  Config config;
  Profile profile{Profile::kIdentity};
  ReIdEmbedder backend;
};

AppearanceFeatureService::AppearanceFeatureService()
    : AppearanceFeatureService(Config{}, Profile::kIdentity) {}

AppearanceFeatureService::AppearanceFeatureService(Config config, const Profile profile)
    : impl_(std::make_unique<Impl>(std::move(config), profile)) {
}

AppearanceFeatureService::~AppearanceFeatureService() = default;

AppearanceFeatureService::AppearanceFeatureService(const AppearanceFeatureService &other)
    : impl_(std::make_unique<Impl>(*other.impl_)) {}

AppearanceFeatureService &AppearanceFeatureService::operator=(const AppearanceFeatureService &other) {
  if (this != &other) {
    impl_ = std::make_unique<Impl>(*other.impl_);
  }
  return *this;
}

AppearanceFeatureService::AppearanceFeatureService(AppearanceFeatureService &&) noexcept = default;

AppearanceFeatureService &AppearanceFeatureService::operator=(AppearanceFeatureService &&) noexcept = default;

bool AppearanceFeatureService::Initialize(std::string *error) { return impl_->backend.Initialize(error); }

bool AppearanceFeatureService::UsesOnnx() const { return impl_->backend.UsesOnnx(); }

bool AppearanceFeatureService::IsReady() const { return impl_->backend.IsReady(); }

cv::Mat AppearanceFeatureService::ExtractTrackerFeature(const cv::Mat &frame, const cv::Rect2f &bbox) const {
  return impl_->backend.Extract(frame, bbox);
}

std::vector<float> AppearanceFeatureService::ExtractIdentityFeature(const cv::Mat &frame,
                                                                    const cv::Rect2f &bbox) const {
  return impl_->backend.ExtractVector(frame, bbox);
}

}  // namespace dog_patrol_perception_tracking
