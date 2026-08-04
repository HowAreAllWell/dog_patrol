#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"

#include <utility>

namespace dog_patrol_perception_tracking {
void DetectionTrackingReadiness::ReportRuntimeStatus(RuntimeStatus status) {
  status_ = std::move(status);
}

PerceptionReadinessContribution DetectionTrackingReadiness::Contribution() const {
  if (!status_.failure_detail.empty()) {
    return {kCapability, PerceptionReadiness::kFailure, status_.failure_detail};
  }
  if (!status_.detector_initialized) {
    return {kCapability, PerceptionReadiness::kNotReady, "detector initialization pending"};
  }
  if (!status_.tracker_initialized) {
    return {kCapability, PerceptionReadiness::kNotReady, "tracker initialization pending"};
  }
  return {kCapability, PerceptionReadiness::kReady, "detector and tracker runtime ready"};
}

}  // namespace dog_patrol_perception_tracking
