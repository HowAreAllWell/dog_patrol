#pragma once

#include <vector>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

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

}  // namespace vision_demo_host
