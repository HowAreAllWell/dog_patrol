#pragma once

#include <string>

namespace dog_patrol_perception_tracking {

enum class IdentityLifecycleMode {
  kNormal,
  kMerged,
  kSplitRecovery,
  kNormalResumed,
};

std::string IdentityLifecycleModeToString(IdentityLifecycleMode mode);

}  // namespace dog_patrol_perception_tracking
