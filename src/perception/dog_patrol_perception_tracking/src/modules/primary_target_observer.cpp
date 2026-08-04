#include "dog_patrol_perception_tracking/modules/primary_target_observer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dog_patrol_perception_tracking {

PrimaryTargetObserver::PrimaryTargetObserver() : PrimaryTargetObserver(PrimaryTargetManager::Config{}) {}

PrimaryTargetObserver::PrimaryTargetObserver(PrimaryTargetManager::Config config)
    : primary_manager_(std::move(config)) {}

PrimaryTargetObserver::Output PrimaryTargetObserver::Update(
    const std::vector<IdentityObservation> &identities,
    const SourceFrameMetadata &source,
    const cv::Mat &source_image) {
  Output output;
  output.primary = primary_manager_.Update(identities);
  output.primary_decision_reason = primary_manager_.LastDecisionReason();
  output.primary_reject_reason = primary_manager_.LastRejectReason();
  output.observation = BuildObservation(output.primary, source, source_image);
  return output;
}

PrimaryTargetResult PrimaryTargetObserver::CurrentPrimary() const {
  return primary_manager_.GetState();
}

std::optional<PrimaryTargetObservation> PrimaryTargetObserver::BuildObservation(
    const PrimaryTargetResult &primary,
    const SourceFrameMetadata &source,
    const cv::Mat &source_image) {
  if (primary.state != PrimaryState::kLocked || primary.primary_target_id <= 0 ||
      !primary.primary_track.has_value() || source_image.empty() ||
      source.source_timestamp_ns == 0U || source.image_width != source_image.cols ||
      source.image_height != source_image.rows || source.optical_frame_id.empty()) {
    return std::nullopt;
  }

  const Track &track = primary.primary_track.value();
  if (!track.authoritative || track.id != primary.raw_track_id || !track.is_confirmed ||
      track.class_id != ClassId::kPerson || track.occlusion_suspect ||
      track.low_score_update || track.association.low_score_detection ||
      track.just_recovered || track.association.recovered_from_lost ||
      (!track.association.stage.empty() && !track.association.passed_final_cost_gate) ||
      !std::isfinite(track.confidence) ||
      track.confidence < 0.0F || track.confidence > 1.0F ||
      !std::isfinite(track.bbox.x) || !std::isfinite(track.bbox.y) ||
      !std::isfinite(track.bbox.width) || !std::isfinite(track.bbox.height) ||
      track.bbox.width <= 0.0F || track.bbox.height <= 0.0F) {
    return std::nullopt;
  }

  const double x_end = static_cast<double>(track.bbox.x) + track.bbox.width;
  const double y_end = static_cast<double>(track.bbox.y) + track.bbox.height;
  if (!std::isfinite(x_end) || !std::isfinite(y_end)) {
    return std::nullopt;
  }

  const auto clamp_x = [width = source_image.cols](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(width));
  };
  const auto clamp_y = [height = source_image.rows](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(height));
  };
  const int x_min = static_cast<int>(clamp_x(std::floor(track.bbox.x)));
  const int y_min = static_cast<int>(clamp_y(std::floor(track.bbox.y)));
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
