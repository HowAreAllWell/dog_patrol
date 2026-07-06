#pragma once

#include <string>

namespace vision_demo_host {

enum class IdentityLifecycleMode {
  kNormal,
  kMerged,
  kSplitRecovery,
  kNormalResumed,
};

std::string IdentityLifecycleModeToString(IdentityLifecycleMode mode);

}  // namespace vision_demo_host
