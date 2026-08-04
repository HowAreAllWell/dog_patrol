#pragma once

#include <vector>

#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class DetFilter {
 public:
  struct Config {
    float person_conf_threshold{0.10F};
    float car_conf_threshold{0.10F};
  };

  explicit DetFilter(Config config);

  std::vector<Detection> Filter(const std::vector<Detection> &detections) const;

 private:
  Config config_;
};

}  // namespace dog_patrol_perception_tracking
