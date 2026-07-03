#pragma once

#include <deque>
#include <vector>

#include <opencv2/core/types.hpp>

#include "vision_demo_host/modules/feature_update_policy.hpp"

namespace vision_demo_host {

class FeatureGeometryUpdateState {
 public:
  struct State {
    cv::Rect2f reliable_bbox;
    cv::Point2f reliable_center{0.0F, 0.0F};
    cv::Point2f reliable_velocity{0.0F, 0.0F};
    int last_reliable_frame{-1};
    bool has_reliable_geometry{false};
    int stable_update_frames{0};
    std::deque<std::vector<float>> feature_bank;
  };

  struct Observation {
    cv::Rect2f bbox;
    int frame_index{0};
    std::vector<float> feature;
  };

  struct Config {
    int feature_bank_max_size{30};
    int stable_frames_before_feature_update{3};
  };

  static void Apply(const FeatureUpdatePolicy::Decision &decision, const Observation &observation,
                    const Config &config, State *state);
};

}  // namespace vision_demo_host
