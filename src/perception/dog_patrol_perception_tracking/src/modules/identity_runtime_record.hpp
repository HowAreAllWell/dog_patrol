#pragma once

#include <opencv2/core/types.hpp>

#include "dog_patrol_perception_tracking/modules/feature_geometry_update_state.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

struct IdentityRuntimeRecord {
  int semantic_id{-1};
  ClassId class_id{ClassId::kUnknown};
  cv::Rect2f last_bbox;
  cv::Point2f last_center{0.0F, 0.0F};
  FeatureGeometryUpdateState::State feature_geometry;
  float last_assignment_cost{1.0F};
  float last_assignment_margin{0.0F};
  int last_seen_frame{-1};
  int missing_frames{0};
  bool seen_this_frame{false};
  int occlusion_protect_remaining{0};
  int supporting_raw_track_id{-1};
  float confidence{0.0F};
};

}  // namespace dog_patrol_perception_tracking
