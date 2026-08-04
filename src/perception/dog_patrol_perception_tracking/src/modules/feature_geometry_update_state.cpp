#include "dog_patrol_perception_tracking/modules/feature_geometry_update_state.hpp"

#include <algorithm>

namespace dog_patrol_perception_tracking {
namespace {

cv::Point2f BBoxCenter(const cv::Rect2f &bbox) {
  return cv::Point2f(bbox.x + bbox.width * 0.5F, bbox.y + bbox.height * 0.5F);
}

}  // namespace

void FeatureGeometryUpdateState::Apply(const FeatureUpdatePolicy::Decision &decision,
                                       const Observation &observation, const Config &config, State *state) {
  if (state == nullptr) {
    return;
  }

  if (decision.geometry_update_allowed) {
    const cv::Point2f new_center = BBoxCenter(observation.bbox);
    if (state->has_reliable_geometry && state->last_reliable_frame >= 0) {
      const int dt = std::max(1, observation.frame_index - state->last_reliable_frame);
      const cv::Point2f measured_velocity =
          (new_center - state->reliable_center) * (1.0F / static_cast<float>(dt));
      state->reliable_velocity = 0.65F * state->reliable_velocity + 0.35F * measured_velocity;
    }
    state->reliable_bbox = observation.bbox;
    state->reliable_center = new_center;
    state->last_reliable_frame = observation.frame_index;
    state->has_reliable_geometry = true;
    state->stable_update_frames += 1;
  } else {
    state->stable_update_frames = 0;
  }

  const bool stable_enough =
      state->feature_bank.empty() ||
      state->stable_update_frames >= std::max(1, config.stable_frames_before_feature_update);
  if (!decision.feature_update_allowed || !decision.geometry_update_allowed || !stable_enough ||
      observation.feature.empty()) {
    return;
  }

  state->feature_bank.push_back(observation.feature);
  const std::size_t max_size = static_cast<std::size_t>(std::max(1, config.feature_bank_max_size));
  while (state->feature_bank.size() > max_size) {
    state->feature_bank.pop_front();
  }
}

}  // namespace dog_patrol_perception_tracking
