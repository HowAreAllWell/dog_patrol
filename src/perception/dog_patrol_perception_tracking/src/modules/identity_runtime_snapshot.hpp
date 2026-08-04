#pragma once

#include <opencv2/core/types.hpp>

#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

struct IdentityRuntimeSnapshot {
  int semantic_id{-1};
  ClassId class_id{ClassId::kUnknown};
  cv::Rect2f bbox;
  cv::Rect2f reliable_bbox;
  bool has_reliable_geometry{false};
  int missing_frames{0};
  int occlusion_protect_remaining{0};
  bool seen_this_frame{false};
  int supporting_raw_track_id{-1};
  float confidence{0.0F};
};

}  // namespace dog_patrol_perception_tracking
