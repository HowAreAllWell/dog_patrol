#include "occlusion_group_shadow_lifecycle.hpp"

#include <sstream>
#include <utility>

namespace dog_patrol_perception_tracking {
namespace {

std::string JoinSemanticIds(const std::set<int> &semantic_ids) {
  std::ostringstream oss;
  bool first = true;
  for (const int semantic_id : semantic_ids) {
    if (!first) {
      oss << "|";
    }
    first = false;
    oss << semantic_id;
  }
  return oss.str();
}

std::string TrackletHypothesisStatusToDebugString(const TrackletHypothesisStatus status) {
  switch (status) {
    case TrackletHypothesisStatus::kTracked:
      return "tracked";
    case TrackletHypothesisStatus::kTentative:
      return "tentative";
    case TrackletHypothesisStatus::kLostPrediction:
      return "lost_prediction";
    case TrackletHypothesisStatus::kSuppressedDuplicateCandidate:
      return "suppressed_duplicate_candidate";
    case TrackletHypothesisStatus::kSplitCandidate:
      return "split_candidate";
    case TrackletHypothesisStatus::kLowQualityCandidate:
      return "low_quality_candidate";
    default:
      return "unknown";
  }
}

void AppendRow(IdentityManager::Phase3ShadowDebugRow row,
               const OcclusionGroupShadowLifecycle::ShadowRowsContext &context) {
  row.frame_idx = context.current_frame_idx;
  row.event_idx = (*context.next_event_idx)++;
  context.phase3_rows->push_back(std::move(row));
}

}  // namespace

void OcclusionGroupShadowLifecycle::Reset(State *state) { *state = State{}; }

const OcclusionGroupShadowLifecycle::MergedGroupShadowState *OcclusionGroupShadowLifecycle::RecoveryGroup(
    const State &state, const int current_frame_idx) {
  if (state.merged_group.active) {
    return &state.merged_group;
  }
  if (state.has_recent_merged_group &&
      current_frame_idx - state.recent_merged_group.last_update_frame <= 2) {
    return &state.recent_merged_group;
  }
  return nullptr;
}

void OcclusionGroupShadowLifecycle::AppendGroupRow(const MergedGroupShadowState &group, const std::string &semantic_ids,
                                                   const std::string &event_type, const std::string &reason,
                                                   const ShadowRowsContext &context) {
  IdentityManager::Phase3ShadowDebugRow row;
  row.event_type = event_type;
  row.group_id = group.group_id;
  row.semantic_ids = semantic_ids;
  row.carrier_semantic_id = group.carrier_semantic_id;
  row.carrier_raw_track_id = group.carrier_raw_track_id;
  row.reason = reason;
  row.group_age_frames = group.age_frames;
  row.group_last_update_frame = group.last_update_frame;
  AppendRow(std::move(row), context);
}

void OcclusionGroupShadowLifecycle::AppendSplitCandidateRow(const State &state,
                                                            const SplitCandidateShadowState &candidate,
                                                            const std::string &event_type,
                                                            const ShadowRowsContext &context,
                                                            const std::string &reason_override) {
  IdentityManager::Phase3ShadowDebugRow row;
  row.event_type = event_type;
  row.group_id = candidate.group_id;
  row.semantic_ids = JoinSemanticIds(state.merged_group.semantic_ids);
  row.carrier_semantic_id = state.merged_group.carrier_semantic_id;
  row.carrier_raw_track_id = state.merged_group.carrier_raw_track_id;
  row.candidate_raw_track_id = candidate.candidate_raw_track_id;
  row.candidate_semantic_id = candidate.candidate_semantic_id;
  row.candidate_bbox = candidate.bbox;
  row.candidate_confidence = candidate.confidence;
  row.reason = reason_override.empty() ? candidate.reason : reason_override;
  row.related_raw_track_id = candidate.related_raw_track_id;
  row.hypothesis_status = candidate.hypothesis_status;
  row.candidate_stable_frames = candidate.stable_frames;
  row.group_age_frames = state.merged_group.age_frames;
  row.group_last_update_frame = state.merged_group.last_update_frame;
  AppendRow(std::move(row), context);
}

void OcclusionGroupShadowLifecycle::SyncGroupMode(State *state, const SyncGroupModeInput &input,
                                                  const ShadowRowsContext &context) {
  const bool legacy_group_mode =
      input.mode == IdentityManager::Mode::kMerged || input.mode == IdentityManager::Mode::kSplitRecovery;
  if (legacy_group_mode) {
    if (!state->merged_group.active) {
      state->merged_group = MergedGroupShadowState{};
      state->merged_group.group_id = state->next_merged_group_id++;
      state->merged_group.active = true;
      state->merged_group.age_frames = 1;
      state->merged_group.semantic_ids = input.person_semantic_ids;
      state->merged_group.carrier_semantic_id = input.carrier_semantic_id;
      state->merged_group.carrier_raw_track_id = input.carrier_raw_track_id;
      state->merged_group.last_update_frame = context.current_frame_idx;
      AppendGroupRow(state->merged_group, JoinSemanticIds(state->merged_group.semantic_ids), "merged_group_enter",
                     input.mode == IdentityManager::Mode::kMerged ? "legacy_mode_merged_enter"
                                                                  : "legacy_mode_split_recovery_enter",
                     context);
      return;
    }

    state->merged_group.age_frames += 1;
    state->merged_group.semantic_ids.insert(input.person_semantic_ids.begin(), input.person_semantic_ids.end());
    if (input.carrier_semantic_id > 0 && input.phase4_continuity_raw <= 0) {
      state->merged_group.carrier_semantic_id = input.carrier_semantic_id;
      state->merged_group.carrier_raw_track_id = input.carrier_raw_track_id;
    }
    state->merged_group.last_update_frame = context.current_frame_idx;
    AppendGroupRow(state->merged_group, JoinSemanticIds(state->merged_group.semantic_ids), "merged_group_update",
                   input.mode == IdentityManager::Mode::kMerged ? "legacy_mode_merged_hold"
                                                                : "legacy_mode_split_recovery_hold",
                   context);
    return;
  }

  if (!state->merged_group.active) {
    return;
  }

  state->merged_group.last_update_frame = context.current_frame_idx;
  for (auto &entry : state->split_candidates_by_raw_id) {
    AppendSplitCandidateRow(*state, entry.second, "split_candidate_end", context, "group_end");
  }
  state->split_candidates_by_raw_id.clear();
  AppendGroupRow(state->merged_group, JoinSemanticIds(state->merged_group.semantic_ids), "merged_group_end",
                 input.mode == IdentityManager::Mode::kNormalResumed ? "legacy_mode_normal_resumed"
                                                                     : "legacy_mode_normal",
                 context);
  state->recent_merged_group = state->merged_group;
  state->has_recent_merged_group = true;
  state->merged_group = MergedGroupShadowState{};
}

void OcclusionGroupShadowLifecycle::MarkSplitCandidatesUnseen(State *state) {
  for (auto &entry : state->split_candidates_by_raw_id) {
    entry.second.seen_this_frame = false;
  }
}

void OcclusionGroupShadowLifecycle::ObserveSplitCandidate(State *state, const ObserveSplitCandidateInput &input,
                                                          const ShadowRowsContext &context) {
  auto &candidate = state->split_candidates_by_raw_id[input.hypothesis.raw_track_id];
  const bool first_seen = candidate.stable_frames == 0 || candidate.group_id != state->merged_group.group_id;
  candidate.group_id = state->merged_group.group_id;
  candidate.candidate_raw_track_id = input.hypothesis.raw_track_id;
  candidate.candidate_semantic_id = input.candidate_semantic_id;
  candidate.bbox = input.hypothesis.bbox;
  candidate.confidence = input.hypothesis.confidence;
  candidate.reason = input.hypothesis.candidate_reason;
  candidate.related_raw_track_id = input.related_raw_track_id;
  candidate.hypothesis_status = TrackletHypothesisStatusToDebugString(input.hypothesis.status);
  candidate.stable_frames = first_seen ? 1 : candidate.stable_frames + 1;
  candidate.last_update_frame = context.current_frame_idx;
  candidate.seen_this_frame = true;
  AppendSplitCandidateRow(*state, candidate, first_seen ? "split_candidate_enter" : "split_candidate_update",
                          context);
}

void OcclusionGroupShadowLifecycle::EndMissingSplitCandidates(State *state, const ShadowRowsContext &context) {
  for (auto it = state->split_candidates_by_raw_id.begin(); it != state->split_candidates_by_raw_id.end();) {
    if (it->second.seen_this_frame) {
      ++it;
      continue;
    }
    AppendSplitCandidateRow(*state, it->second, "split_candidate_end", context, "candidate_missing");
    it = state->split_candidates_by_raw_id.erase(it);
  }
}

}  // namespace dog_patrol_perception_tracking
