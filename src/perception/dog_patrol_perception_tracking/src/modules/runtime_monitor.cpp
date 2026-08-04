#include "dog_patrol_perception_tracking/modules/runtime_monitor.hpp"

namespace dog_patrol_perception_tracking {

RuntimeMonitor::RuntimeMonitor() {
  start_window_time_ = std::chrono::steady_clock::now();
  last_report_time_ = start_window_time_;
}

bool RuntimeMonitor::ShouldReport() {
  frame_count_window_++;
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_window_time_);

  if (elapsed.count() > 0) {
    current_fps_ = static_cast<double>(frame_count_window_) / (static_cast<double>(elapsed.count()) / 1000.0);
  }

  const auto since_report =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time_).count();
  if (since_report >= 1000) {
    last_report_time_ = now;
    start_window_time_ = now;
    frame_count_window_ = 0;
    return true;
  }

  return false;
}

double RuntimeMonitor::CurrentFps() const { return current_fps_; }

}  // namespace dog_patrol_perception_tracking
