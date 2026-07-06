#include "identity_runtime_lifecycle_coordinator.hpp"

namespace vision_demo_host {

void IdentityRuntimeLifecycleCoordinator::ApplyEndFrameAging(const EndFrameInput &input) {
  if (input.identity_store == nullptr) {
    return;
  }
  input.identity_store->AgeOneFrame();
}

}  // namespace vision_demo_host
