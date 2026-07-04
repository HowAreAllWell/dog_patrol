#pragma once

#include <unordered_map>
#include <vector>

#include "vision_demo_host/modules/identity_manager.hpp"
#include "legacy_identity_snapshot.hpp"

namespace vision_demo_host {

class IdentityObservationProjection {
 public:
  struct Input {
    std::vector<LegacyIdentitySnapshot> snapshots;
    std::unordered_map<int, TrackletObservation> observations_by_raw_track_id;
    std::vector<IdentityManager::ScoreDebugRow> debug_rows;
    IdentityManager::Mode mode{IdentityManager::Mode::kNormal};
    int primary_semantic_id{-1};
    bool feature_update_frozen{false};
    int max_missing_frames{0};
  };

  static IdentityManagerResult Build(const Input &input);
};

}  // namespace vision_demo_host
