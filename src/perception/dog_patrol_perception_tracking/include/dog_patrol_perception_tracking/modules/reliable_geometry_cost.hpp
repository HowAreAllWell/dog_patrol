#pragma once

#include <opencv2/core/types.hpp>

namespace dog_patrol_perception_tracking {

class ReliableGeometryCost {
 public:
  struct State {
    cv::Rect2f latest_bbox;
    cv::Point2f latest_center{0.0F, 0.0F};
    cv::Rect2f reliable_bbox;
    cv::Point2f reliable_center{0.0F, 0.0F};
    cv::Point2f reliable_velocity{0.0F, 0.0F};
    int missing_frames{0};
    bool has_reliable_geometry{false};
  };

  struct MissingGateConfig {
    float min_area_ratio{0.25F};
    float max_area_ratio{4.0F};
    float max_center_dist_norm{1.2F};
    float max_app_cost{0.65F};
    float active_max_cost{0.75F};
  };

  static cv::Rect2f ReferenceBBox(const State &state);
  static cv::Point2f PredictedCenter(const State &state);
  static float GeometryCost(const cv::Rect2f &bbox, const State &state);
  static bool PassesMissingIdentityGate(const cv::Rect2f &bbox, const State &state, float app_cost,
                                        float geo_cost, const MissingGateConfig &config);
  static bool PassesShortMissingAppearanceGate(const State &state, float app_cost, float geo_cost,
                                               float max_app_cost);
};

}  // namespace dog_patrol_perception_tracking
