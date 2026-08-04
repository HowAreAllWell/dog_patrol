#pragma once

#include <map>
#include <set>
#include <string>

#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"

namespace dog_patrol_perception_tracking {

class OcclusionGroupShadowLifecycle {
 public:
  struct MergedGroupShadowState {
    int group_id{-1};
    std::set<int> semantic_ids;
    int carrier_semantic_id{-1};
    int carrier_raw_track_id{-1};
    int age_frames{0};
    int last_update_frame{-1};
    bool active{false};
  };

  struct SplitCandidateShadowState {
    int group_id{-1};
    int candidate_raw_track_id{-1};
    int candidate_semantic_id{-1};
    cv::Rect2f bbox;
    float confidence{0.0F};
    std::string reason;
    int related_raw_track_id{-1};
    std::string hypothesis_status;
    int stable_frames{0};
    int last_update_frame{-1};
    bool seen_this_frame{false};
  };

  struct State {
    MergedGroupShadowState merged_group;
    MergedGroupShadowState recent_merged_group;
    bool has_recent_merged_group{false};
    std::map<int, SplitCandidateShadowState> split_candidates_by_raw_id;
    int next_merged_group_id{1};
  };

  struct ShadowRowsContext {
    int current_frame_idx{0};
    int *next_event_idx{nullptr};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
  };

  struct SyncGroupModeInput {
    IdentityManager::Mode mode{IdentityManager::Mode::kNormal};
    std::set<int> person_semantic_ids;
    int carrier_semantic_id{-1};
    int carrier_raw_track_id{-1};
    int phase4_continuity_raw{-1};
  };

  struct ObserveSplitCandidateInput {
    TrackletHypothesis hypothesis;
    int related_raw_track_id{-1};
    int candidate_semantic_id{-1};
  };

  static void Reset(State *state);
  static const MergedGroupShadowState *RecoveryGroup(const State &state, int current_frame_idx);
  static void SyncGroupMode(State *state, const SyncGroupModeInput &input, const ShadowRowsContext &context);
  static void MarkSplitCandidatesUnseen(State *state);
  static void ObserveSplitCandidate(State *state, const ObserveSplitCandidateInput &input,
                                    const ShadowRowsContext &context);
  static void EndMissingSplitCandidates(State *state, const ShadowRowsContext &context);
  static void AppendSplitCandidateRow(const State &state, const SplitCandidateShadowState &candidate,
                                      const std::string &event_type, const ShadowRowsContext &context,
                                      const std::string &reason_override = {});
  static void AppendGroupRow(const MergedGroupShadowState &group, const std::string &semantic_ids,
                             const std::string &event_type, const std::string &reason,
                             const ShadowRowsContext &context);
};

}  // namespace dog_patrol_perception_tracking
