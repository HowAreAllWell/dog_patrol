#pragma once

#include <cstdint>
#include <string>

namespace dog_patrol_perception_tracking {

// ROS-independent metadata for the camera frame that produced a perception
// result. Mission transports may project it into message headers, while
// standalone consumers can use it directly.
struct SourceFrameMetadata {
  std::uint64_t source_timestamp_ns{0};
  std::uint32_t camera_frame_number{0};
  bool camera_frame_number_available{false};
  int image_width{0};
  int image_height{0};
  std::string optical_frame_id;
};

}  // namespace dog_patrol_perception_tracking
