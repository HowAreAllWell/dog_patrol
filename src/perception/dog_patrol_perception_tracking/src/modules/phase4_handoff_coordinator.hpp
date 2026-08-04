#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "occlusion_group_shadow_lifecycle.hpp"
#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"

namespace dog_patrol_perception_tracking {

class Phase4HandoffCoordinator {
 public:
  using ApplyPairwiseAssignmentFn = std::function<bool(int, int, int, int)>;
  using ApplyMergedSingleBlobHandoffFn = std::function<bool(int, int, int)>;
  using ApplyMergedSideRecoveryFn = std::function<bool(int, int, int, int)>;
  using ApplyMergedSplitHandoffFn = std::function<bool(int, int, int, int)>;
  using RefreshOutputsFn = std::function<void()>;

  struct PairwiseInput {
    bool enabled{false};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
    std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
    int *next_event_idx{nullptr};
  };

  struct SingleBlobInput {
    bool enabled{false};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
    std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
    int *next_event_idx{nullptr};
  };

  struct SideRecoveryInput {
    bool enabled{false};
    int current_frame_idx{-1};
    const std::vector<TrackletHypothesis> *shadow_hypotheses{nullptr};
    const std::unordered_map<int, TrackletObservation> *observations_by_raw_track_id{nullptr};
    std::vector<IdentityManager::ScoreDebugRow> *score_rows{nullptr};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
    std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
    OcclusionGroupShadowLifecycle::State *occlusion_state{nullptr};
    int *next_event_idx{nullptr};
  };

  struct MergedSplitInput {
    bool enabled{false};
    int current_frame_idx{-1};
    int phase4_continuity_raw{-1};
    int phase4_continuity_sid{-1};
    const std::unordered_map<int, int> *prev_raw_to_semantic{nullptr};
    const std::unordered_map<int, TrackletObservation> *observations_by_raw_track_id{nullptr};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
    std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
    OcclusionGroupShadowLifecycle::State *occlusion_state{nullptr};
    int *next_event_idx{nullptr};
  };

  static bool ApplyPairwiseAssignment(const PairwiseInput &input,
                                      const ApplyPairwiseAssignmentFn &apply_pairwise_assignment,
                                      const RefreshOutputsFn &refresh_outputs);
  static bool ApplyMergedSingleBlobHandoff(const SingleBlobInput &input,
                                           const ApplyMergedSingleBlobHandoffFn &apply_single_blob_handoff,
                                           const RefreshOutputsFn &refresh_outputs);
  static bool ApplyMergedSideRecovery(const SideRecoveryInput &input,
                                      const ApplyMergedSideRecoveryFn &apply_side_recovery,
                                      const RefreshOutputsFn &refresh_outputs);
  static bool ApplyMergedSplitHandoff(const MergedSplitInput &input,
                                      const ApplyMergedSplitHandoffFn &apply_merged_split_handoff,
                                      const RefreshOutputsFn &refresh_outputs);
};

}  // namespace dog_patrol_perception_tracking
