#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dog_patrol_perception_tracking {

PrimaryTargetObserver::PrimaryTargetObserver() : PrimaryTargetObserver(PrimaryTargetManager::Config{}) {}

PrimaryTargetObserver::PrimaryTargetObserver(PrimaryTargetManager::Config config)
    : PrimaryTargetObserver(std::move(config), nullptr) {}

PrimaryTargetObserver::PrimaryTargetObserver(
    PrimaryTargetManager::Config config,
    std::shared_ptr<PrimaryTargetObservationSink> sink)
    : PrimaryTargetObserver(std::move(config), std::move(sink), CropConfig{}) {}

PrimaryTargetObserver::PrimaryTargetObserver(
    PrimaryTargetManager::Config config,
    std::shared_ptr<PrimaryTargetObservationSink> sink,
    CropConfig crop_config)
    : primary_manager_(std::move(config)), sink_(std::move(sink)),
      crop_config_(crop_config) {
  if (!std::isfinite(crop_config_.padding_ratio) || crop_config_.padding_ratio < 0.0F) {
    throw std::invalid_argument("crop padding_ratio must be finite and non-negative");
  }
}

void LatestPrimaryTargetObservation::Consume(
    std::optional<PrimaryTargetObservation> observation) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = std::move(observation);
}

std::optional<PrimaryTargetObservation> LatestPrimaryTargetObservation::Current() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_;
}

PrimaryTargetObserver::Output PrimaryTargetObserver::Update(
    const std::vector<IdentityObservation> &identities,
    const SourceFrameMetadata &source,
    const cv::Mat &source_image) {
  Output output;
  output.primary = primary_manager_.Update(identities);
  output.primary_decision_reason = primary_manager_.LastDecisionReason();
  output.primary_reject_reason = primary_manager_.LastRejectReason();
  output.observation = BuildObservation(output.primary, source, source_image, crop_config_);
  if (sink_ != nullptr) {
    sink_->Consume(output.observation);
  }
  return output;
}

PrimaryTargetResult PrimaryTargetObserver::CurrentPrimary() const {
  return primary_manager_.GetState();
}

void PrimaryTargetObserver::InvalidateCurrentObservation() {
  if (sink_ != nullptr) {
    sink_->Consume(std::nullopt);
  }
}

std::optional<PrimaryTargetObservation> PrimaryTargetObserver::BuildObservation(
    const PrimaryTargetResult &primary,
    const SourceFrameMetadata &source,
    const cv::Mat &source_image,
    const CropConfig &crop_config) {
  if (!IsTrustedCurrentPrimary(primary) || source_image.empty() ||
      source.source_timestamp_ns == 0U || source.image_width != source_image.cols ||
      source.image_height != source_image.rows || source.optical_frame_id.empty()) {
    return std::nullopt;
  }

  const Track &track = primary.primary_track.value();
  if (!std::isfinite(track.bbox.x) || !std::isfinite(track.bbox.y) ||
      !std::isfinite(track.bbox.width) || !std::isfinite(track.bbox.height) ||
      track.bbox.width <= 0.0F || track.bbox.height <= 0.0F) {
    return std::nullopt;
  }

  const double pad_x = static_cast<double>(track.bbox.width) * crop_config.padding_ratio;
  const double pad_y = static_cast<double>(track.bbox.height) * crop_config.padding_ratio;
  const double x_start = static_cast<double>(track.bbox.x) - pad_x;
  const double y_start = static_cast<double>(track.bbox.y) - pad_y;
  const double x_end = static_cast<double>(track.bbox.x) + track.bbox.width + pad_x;
  const double y_end = static_cast<double>(track.bbox.y) + track.bbox.height + pad_y;
  if (!std::isfinite(x_start) || !std::isfinite(y_start) ||
      !std::isfinite(x_end) || !std::isfinite(y_end)) {
    return std::nullopt;
  }

  const auto clamp_x = [width = source_image.cols](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(width));
  };
  const auto clamp_y = [height = source_image.rows](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(height));
  };
  const int x_min = static_cast<int>(clamp_x(std::floor(x_start)));
  const int y_min = static_cast<int>(clamp_y(std::floor(y_start)));
  const int x_max = static_cast<int>(clamp_x(std::ceil(x_end)));
  const int y_max = static_cast<int>(clamp_y(std::ceil(y_end)));
  if (x_max <= x_min || y_max <= y_min) {
    return std::nullopt;
  }

  PrimaryTargetObservation observation;
  observation.target_id = primary.primary_target_id;
  observation.source = source;
  observation.bbox = cv::Rect{x_min, y_min, x_max - x_min, y_max - y_min};
  observation.confidence = track.confidence;
  observation.target_image = source_image(observation.bbox).clone();
  return observation;
}

}  // namespace dog_patrol_perception_tracking
