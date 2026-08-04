#pragma once

#include <unordered_map>
#include <vector>

#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"
#include "identity_runtime_snapshot.hpp"

namespace dog_patrol_perception_tracking {

class IdentityObservationProjection {
 public:
  enum class TargetLifecycle {
    kVisibleIdentity,
    kOccludedIdentity,
    kMergedGroup,
    kSplitCandidate,
    kNewBirthCandidate,
    kLostIdentity,
  };

  struct Input {
    std::vector<IdentityRuntimeSnapshot> snapshots;
    std::unordered_map<int, TrackletObservation> observations_by_raw_track_id;
    std::vector<IdentityManager::ScoreDebugRow> debug_rows;
    IdentityManager::Mode mode{IdentityManager::Mode::kNormal};
    int primary_semantic_id{-1};
    bool feature_update_frozen{false};
    int max_missing_frames{0};
  };

  static IdentityManagerResult Build(const Input &input);
  static TargetLifecycle ProjectTargetLifecycle(
      const IdentityObservation &identity, IdentityManager::Mode mode,
      const std::vector<IdentityManager::Phase3ShadowDebugRow> &phase3_rows = {});
};

std::string TargetLifecycleToString(IdentityObservationProjection::TargetLifecycle lifecycle);

}  // namespace dog_patrol_perception_tracking
