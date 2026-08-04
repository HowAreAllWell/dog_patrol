#include "identity_runtime_lifecycle_coordinator.hpp"

namespace dog_patrol_perception_tracking {

void IdentityRuntimeLifecycleCoordinator::ApplyEndFrameAging(const EndFrameInput &input) {
  if (input.identity_store == nullptr) {
    return;
  }
  input.identity_store->AgeOneFrame();
}

}  // namespace dog_patrol_perception_tracking
