#include "dog_patrol_perception_tracking/modules/det_filter.hpp"

namespace dog_patrol_perception_tracking {

DetFilter::DetFilter(Config config) : config_(config) {}

std::vector<Detection> DetFilter::Filter(const std::vector<Detection> &detections) const {
  std::vector<Detection> filtered;
  filtered.reserve(detections.size());

  for (const auto &det : detections) {
    if (det.class_id == ClassId::kPerson && det.confidence >= config_.person_conf_threshold) {
      filtered.push_back(det);
    } else if (det.class_id == ClassId::kCar && det.confidence >= config_.car_conf_threshold) {
      filtered.push_back(det);
    }
  }

  return filtered;
}

}  // namespace dog_patrol_perception_tracking
