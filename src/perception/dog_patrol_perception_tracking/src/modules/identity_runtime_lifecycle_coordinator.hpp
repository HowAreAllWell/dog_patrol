#pragma once

#include "identity_runtime_store.hpp"

namespace dog_patrol_perception_tracking {

class IdentityRuntimeLifecycleCoordinator {
 public:
  struct EndFrameInput {
    IdentityRuntimeStore *identity_store{nullptr};
  };

  static void ApplyEndFrameAging(const EndFrameInput &input);
};

}  // namespace dog_patrol_perception_tracking
