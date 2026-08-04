#pragma once

#include <cstdint>
#include <string>

namespace dog_patrol_perception_tracking {

// A capability is either operational, still initializing, or has reported a
// failure that requires its owning runtime to recover before perception is ready.
enum class PerceptionReadiness : std::uint8_t {
  kNotReady = 0,
  kReady = 1,
  kFailure = 2,
};

struct PerceptionReadinessContribution {
  std::string capability;
  PerceptionReadiness readiness{PerceptionReadiness::kNotReady};
  std::string detail;
};

// Maps the real detector/tracker initialization and runtime state to the
// perception-internal capability contract. Aggregation belongs to the
// orchestrator, not to tracking.
class DetectionTrackingReadiness {
 public:
  static constexpr const char *kCapability = "detection_tracking";

  struct RuntimeStatus {
    bool detector_initialized{false};
    bool tracker_initialized{false};
    std::string failure_detail;
  };

  // Called from the vision node's existing detector/tracker initialization and
  // frame-processing path. It does not publish ROS messages or consume policy.
  void ReportRuntimeStatus(RuntimeStatus status);
  PerceptionReadinessContribution Contribution() const;

 private:
  RuntimeStatus status_;
};

}  // namespace dog_patrol_perception_tracking
