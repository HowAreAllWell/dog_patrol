#pragma once

#include "identity_runtime_store.hpp"

namespace vision_demo_host {

class IdentityRuntimeLifecycleCoordinator {
 public:
  struct EndFrameInput {
    IdentityRuntimeStore *identity_store{nullptr};
  };

  static void ApplyEndFrameAging(const EndFrameInput &input);
};

}  // namespace vision_demo_host
