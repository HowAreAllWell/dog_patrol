#pragma once

#include <chrono>
#include <cstdint>

namespace dog_patrol_perception_tracking {

class RuntimeMonitor {
 public:
  RuntimeMonitor();
  bool ShouldReport();
  double CurrentFps() const;

 private:
  std::chrono::steady_clock::time_point last_report_time_;
  std::chrono::steady_clock::time_point start_window_time_;
  std::uint64_t frame_count_window_{0};
  double current_fps_{0.0};
};

}  // namespace dog_patrol_perception_tracking
