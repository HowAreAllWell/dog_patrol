#include "phase4_handoff_coordinator.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace vision_demo_host {
namespace {

std::vector<std::pair<int, int>> ParsePairwisePairs(const std::string &pairs) {
  std::vector<std::pair<int, int>> out;
  std::stringstream ss(pairs);
  std::string token;
  while (std::getline(ss, token, '|')) {
    const auto sep = token.find("->");
    if (sep == std::string::npos) {
      return {};
    }
    try {
      const int raw_id = std::stoi(token.substr(0, sep));
      const int semantic_id = std::stoi(token.substr(sep + 2));
      out.emplace_back(raw_id, semantic_id);
    } catch (...) {
      return {};
    }
  }
  return out;
}

bool LooksLikeMergedSplitHandoff(const cv::Rect2f &carrier_bbox, const cv::Rect2f &candidate_bbox) {
  const float carrier_area = std::max(1.0F, carrier_bbox.area());
  const float candidate_area = std::max(1.0F, candidate_bbox.area());
  const float carrier_aspect = carrier_bbox.width / std::max(1.0F, carrier_bbox.height);
  const bool candidate_is_taller_and_higher =
      candidate_bbox.height >= carrier_bbox.height * 1.08F &&
      candidate_bbox.y + carrier_bbox.height * 0.05F < carrier_bbox.y;
  return carrier_aspect <= 0.40F &&
         (carrier_area <= 0.85F * candidate_area || candidate_is_taller_and_higher);
}

bool LooksLikeMergedSideRecovery(const cv::Rect2f &carrier_bbox, const cv::Rect2f &candidate_bbox,
                                 const float candidate_confidence) {
  const float candidate_height = std::max(1.0F, candidate_bbox.height);
  const float candidate_aspect = candidate_bbox.width / candidate_height;
  if (candidate_confidence < 0.45F || candidate_height < 300.0F || candidate_aspect < 0.25F ||
      candidate_aspect > 0.70F) {
    return false;
  }

  const float vertical_overlap =
      std::max(0.0F, std::min(candidate_bbox.y + candidate_bbox.height, carrier_bbox.y + carrier_bbox.height) -
                         std::max(candidate_bbox.y, carrier_bbox.y));
  if (vertical_overlap / candidate_height < 0.45F) {
    return false;
  }

  const float carrier_left = carrier_bbox.x;
  const float carrier_right = carrier_bbox.x + carrier_bbox.width;
  const float candidate_left = candidate_bbox.x;
  const float candidate_right = candidate_bbox.x + candidate_bbox.width;
  const bool touches_carrier_side =
      (candidate_left < carrier_left && candidate_right > carrier_left) ||
      (candidate_left < carrier_right && candidate_right > carrier_right);
  const float side_gap = std::min(std::abs(candidate_right - carrier_left),
                                  std::abs(candidate_left - carrier_right));
  const bool close_to_carrier_side = side_gap <= std::max(25.0F, carrier_bbox.width * 0.20F);
  return touches_carrier_side || close_to_carrier_side;
}

std::pair<int, int> SideRecoveryCarrier(const OcclusionGroupShadowLifecycle::MergedGroupShadowState &group,
                                        const std::vector<IdentityManager::ScoreDebugRow> &score_rows,
                                        const int candidate_raw_track_id) {
  int related_raw_track_id = group.carrier_raw_track_id;
  int related_semantic_id = group.carrier_semantic_id;
  for (const auto &row : score_rows) {
    if (row.raw_track_id == candidate_raw_track_id || !row.accepted ||
        group.semantic_ids.count(row.semantic_id) == 0) {
      continue;
    }
    if (row.stage == "raw_continuity" || row.stage == "assign_candidate" ||
        row.stage == "phase4_merged_split_handoff") {
      related_raw_track_id = row.raw_track_id;
      related_semantic_id = row.semantic_id;
      break;
    }
  }
  return {related_raw_track_id, related_semantic_id};
}

void AppendPairwiseRows(const IdentityManager::Phase3ShadowDebugRow &matrix_row,
                        const std::vector<std::pair<int, int>> &pairs,
                        std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows,
                        int *next_event_idx) {
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    IdentityManager::Phase3ShadowDebugRow row = matrix_row;
    row.event_idx = (*next_event_idx)++;
    row.event_type = "phase4_pairwise_assignment";
    row.reason = "pairwise_appearance_override";
    row.candidate_raw_track_id = pairs[i].first;
    row.candidate_semantic_id = pairs[i].second;
    row.related_raw_track_id = pairs[pairs.size() - 1 - i].first;
    phase3_rows->push_back(std::move(row));
  }
}

void AppendSingleBlobRow(const IdentityManager::Phase3ShadowDebugRow &decision_row,
                         std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows,
                         int *next_event_idx) {
  IdentityManager::Phase3ShadowDebugRow row = decision_row;
  row.event_idx = (*next_event_idx)++;
  row.event_type = "phase4_merged_single_blob_handoff";
  row.reason = "merged_single_blob_handoff";
  phase3_rows->push_back(std::move(row));
}

void AppendSideRecoveryRow(const int current_frame_idx,
                           const OcclusionGroupShadowLifecycle::State &occlusion_state,
                           const OcclusionGroupShadowLifecycle::MergedGroupShadowState &group,
                           const TrackletHypothesis &hypothesis,
                           const int candidate_semantic_id,
                           const int related_raw_track_id,
                           const int related_semantic_id,
                           const std::string &reason,
                           std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows,
                           int *next_event_idx) {
  const auto candidate_it = occlusion_state.split_candidates_by_raw_id.find(hypothesis.raw_track_id);
  IdentityManager::Phase3ShadowDebugRow row;
  row.frame_idx = current_frame_idx;
  row.event_idx = (*next_event_idx)++;
  row.event_type = "phase4_merged_side_recovery";
  row.group_id = group.group_id;
  row.semantic_ids = "";
  bool first = true;
  for (const int semantic_id : group.semantic_ids) {
    if (!first) {
      row.semantic_ids += "|";
    }
    first = false;
    row.semantic_ids += std::to_string(semantic_id);
  }
  row.carrier_semantic_id = related_semantic_id;
  row.carrier_raw_track_id = related_raw_track_id;
  row.candidate_raw_track_id = hypothesis.raw_track_id;
  row.candidate_semantic_id = candidate_semantic_id;
  row.candidate_bbox = hypothesis.bbox;
  row.candidate_confidence = hypothesis.confidence;
  row.reason = reason;
  row.related_raw_track_id = related_raw_track_id;
  row.hypothesis_status = hypothesis.status == TrackletHypothesisStatus::kTracked ? "tracked" : "unknown";
  row.candidate_stable_frames =
      candidate_it == occlusion_state.split_candidates_by_raw_id.end() ? 1 : candidate_it->second.stable_frames;
  row.group_age_frames = group.age_frames;
  row.group_last_update_frame = group.last_update_frame;
  phase3_rows->push_back(std::move(row));
}

}  // namespace

bool Phase4HandoffCoordinator::ApplyPairwiseAssignment(const PairwiseInput &input,
                                                       const ApplyPairwiseAssignmentFn &apply_pairwise_assignment,
                                                       const RefreshOutputsFn &refresh_outputs) {
  if (!input.enabled || input.phase3_rows == nullptr || input.raw_to_semantic_id == nullptr ||
      input.next_event_idx == nullptr) {
    return false;
  }

  for (const auto &row : *input.phase3_rows) {
    if (row.event_type != "pairwise_assignment_matrix" || !row.pairwise_appearance_override) {
      continue;
    }
    const auto alternate_pairs = ParsePairwisePairs(row.pairwise_alternate_pairs);
    if (alternate_pairs.size() != 2U || alternate_pairs[0].first <= 0 || alternate_pairs[0].second <= 0 ||
        alternate_pairs[1].first <= 0 || alternate_pairs[1].second <= 0 ||
        alternate_pairs[0].first == alternate_pairs[1].first ||
        alternate_pairs[0].second == alternate_pairs[1].second) {
      continue;
    }
    const bool applied = apply_pairwise_assignment(
        alternate_pairs[0].first, alternate_pairs[0].second,
        alternate_pairs[1].first, alternate_pairs[1].second);
    if (!applied) {
      continue;
    }
    (*input.raw_to_semantic_id)[alternate_pairs[0].first] = alternate_pairs[0].second;
    (*input.raw_to_semantic_id)[alternate_pairs[1].first] = alternate_pairs[1].second;
    const IdentityManager::Phase3ShadowDebugRow matrix_row = row;
    refresh_outputs();
    AppendPairwiseRows(matrix_row, alternate_pairs, input.phase3_rows, input.next_event_idx);
    return true;
  }

  return false;
}

bool Phase4HandoffCoordinator::ApplyMergedSingleBlobHandoff(
    const SingleBlobInput &input,
    const ApplyMergedSingleBlobHandoffFn &apply_single_blob_handoff,
    const RefreshOutputsFn &refresh_outputs) {
  if (!input.enabled || input.phase3_rows == nullptr || input.raw_to_semantic_id == nullptr ||
      input.next_event_idx == nullptr) {
    return false;
  }

  const IdentityManager::Phase3ShadowDebugRow *continuity_decision = nullptr;
  for (const auto &row : *input.phase3_rows) {
    if (row.event_type == "single_blob_handoff_decision" &&
        row.reason == "single_blob_continuity_kept" &&
        row.carrier_raw_track_id > 0 &&
        row.carrier_semantic_id > 0 &&
        row.candidate_semantic_id == row.carrier_semantic_id) {
      continuity_decision = &row;
    }
  }

  int accepted_decision_index = -1;
  for (int i = 0; i < static_cast<int>(input.phase3_rows->size()); ++i) {
    const auto &row = (*input.phase3_rows)[static_cast<std::size_t>(i)];
    if (row.event_type != "single_blob_handoff_decision" ||
        (row.reason != "single_blob_handoff_accepted" &&
         row.reason != "single_blob_handoff_eligible") ||
        row.carrier_raw_track_id <= 0 || row.carrier_semantic_id <= 0 ||
        row.candidate_semantic_id <= 0 ||
        row.carrier_semantic_id == row.candidate_semantic_id) {
      continue;
    }
    if (row.reason == "single_blob_handoff_eligible" && continuity_decision != nullptr) {
      const bool handoff_margin_ok =
          row.decision_geo_cost <= 0.75F &&
          row.decision_final_score <= continuity_decision->decision_final_score + 0.04F &&
          row.decision_app_cost + 0.025F <= continuity_decision->decision_app_cost;
      if (!handoff_margin_ok) {
        continue;
      }
    }
    accepted_decision_index = i;
    break;
  }
  if (accepted_decision_index < 0) {
    return false;
  }

  auto &accepted_decision = (*input.phase3_rows)[static_cast<std::size_t>(accepted_decision_index)];
  const bool applied = apply_single_blob_handoff(
      accepted_decision.carrier_raw_track_id,
      accepted_decision.carrier_semantic_id,
      accepted_decision.candidate_semantic_id);
  if (!applied) {
    return false;
  }

  accepted_decision.reason = "single_blob_handoff_accepted";
  accepted_decision.decision_selected = true;
  accepted_decision.decision_accepted = true;
  (*input.raw_to_semantic_id)[accepted_decision.carrier_raw_track_id] = accepted_decision.candidate_semantic_id;
  refresh_outputs();
  AppendSingleBlobRow(accepted_decision, input.phase3_rows, input.next_event_idx);
  return true;
}

bool Phase4HandoffCoordinator::ApplyMergedSideRecovery(const SideRecoveryInput &input,
                                                       const ApplyMergedSideRecoveryFn &apply_side_recovery,
                                                       const RefreshOutputsFn &refresh_outputs) {
  if (!input.enabled || input.shadow_hypotheses == nullptr || input.observations_by_raw_track_id == nullptr ||
      input.score_rows == nullptr || input.phase3_rows == nullptr || input.raw_to_semantic_id == nullptr ||
      input.occlusion_state == nullptr || input.next_event_idx == nullptr) {
    return false;
  }

  const OcclusionGroupShadowLifecycle::MergedGroupShadowState *group =
      OcclusionGroupShadowLifecycle::RecoveryGroup(*input.occlusion_state, input.current_frame_idx);
  if (group == nullptr || group->group_id <= 0 || group->semantic_ids.size() < 2) {
    return false;
  }

  for (const auto &hypothesis : *input.shadow_hypotheses) {
    if (hypothesis.class_id != ClassId::kPerson || hypothesis.raw_track_id <= 0 ||
        hypothesis.status != TrackletHypothesisStatus::kTracked ||
        hypothesis.raw_track_id == group->carrier_raw_track_id ||
        input.raw_to_semantic_id->find(hypothesis.raw_track_id) != input.raw_to_semantic_id->end()) {
      continue;
    }

    const auto [related_raw_track_id, related_semantic_id] =
        SideRecoveryCarrier(*group, *input.score_rows, hypothesis.raw_track_id);
    if (related_raw_track_id <= 0 || related_semantic_id <= 0 ||
        group->semantic_ids.count(related_semantic_id) == 0) {
      continue;
    }
    const auto carrier_obs_it = input.observations_by_raw_track_id->find(related_raw_track_id);
    if (carrier_obs_it == input.observations_by_raw_track_id->end() ||
        !LooksLikeMergedSideRecovery(carrier_obs_it->second.bbox, hypothesis.bbox, hypothesis.confidence)) {
      continue;
    }

    int candidate_semantic_id = -1;
    for (const int semantic_id : group->semantic_ids) {
      if (semantic_id != related_semantic_id) {
        candidate_semantic_id = semantic_id;
        break;
      }
    }
    if (candidate_semantic_id <= 0) {
      continue;
    }

    const bool applied = apply_side_recovery(
        related_raw_track_id, related_semantic_id, hypothesis.raw_track_id, candidate_semantic_id);
    if (!applied) {
      continue;
    }
    (*input.raw_to_semantic_id)[hypothesis.raw_track_id] = candidate_semantic_id;
    refresh_outputs();
    AppendSideRecoveryRow(input.current_frame_idx, *input.occlusion_state, *group, hypothesis,
                          candidate_semantic_id, related_raw_track_id, related_semantic_id,
                          "merged_side_recovery", input.phase3_rows, input.next_event_idx);
    return true;
  }

  return false;
}

bool Phase4HandoffCoordinator::ApplyMergedSplitHandoff(
    const MergedSplitInput &input,
    const ApplyMergedSplitHandoffFn &apply_merged_split_handoff,
    const RefreshOutputsFn &refresh_outputs) {
  if (!input.enabled || input.prev_raw_to_semantic == nullptr || input.observations_by_raw_track_id == nullptr ||
      input.phase3_rows == nullptr || input.raw_to_semantic_id == nullptr ||
      input.occlusion_state == nullptr || input.next_event_idx == nullptr) {
    return false;
  }

  for (auto &entry : input.occlusion_state->split_candidates_by_raw_id) {
    auto &candidate = entry.second;
    if (!candidate.seen_this_frame || input.phase4_continuity_sid <= 0) {
      continue;
    }
    if (input.prev_raw_to_semantic->find(candidate.candidate_raw_track_id) != input.prev_raw_to_semantic->end()) {
      continue;
    }
    int exposed_partial_sid = -1;
    for (const int semantic_id : input.occlusion_state->merged_group.semantic_ids) {
      if (semantic_id != input.phase4_continuity_sid) {
        exposed_partial_sid = semantic_id;
        break;
      }
    }
    if (input.phase4_continuity_raw <= 0 || input.phase4_continuity_sid <= 0 || exposed_partial_sid <= 0 ||
        input.phase4_continuity_raw == candidate.candidate_raw_track_id ||
        exposed_partial_sid == input.phase4_continuity_sid) {
      continue;
    }
    const auto carrier_obs_it = input.observations_by_raw_track_id->find(input.phase4_continuity_raw);
    if (carrier_obs_it == input.observations_by_raw_track_id->end() ||
        !LooksLikeMergedSplitHandoff(carrier_obs_it->second.bbox, candidate.bbox)) {
      continue;
    }
    const bool applied = apply_merged_split_handoff(
        input.phase4_continuity_raw, exposed_partial_sid,
        candidate.candidate_raw_track_id, input.phase4_continuity_sid);
    if (!applied) {
      continue;
    }

    (*input.raw_to_semantic_id)[input.phase4_continuity_raw] = exposed_partial_sid;
    (*input.raw_to_semantic_id)[candidate.candidate_raw_track_id] = input.phase4_continuity_sid;
    candidate.candidate_semantic_id = input.phase4_continuity_sid;
    input.occlusion_state->merged_group.carrier_raw_track_id = input.phase4_continuity_raw;
    input.occlusion_state->merged_group.carrier_semantic_id = exposed_partial_sid;
    refresh_outputs();
    OcclusionGroupShadowLifecycle::ShadowRowsContext context;
    context.current_frame_idx = input.current_frame_idx;
    context.next_event_idx = input.next_event_idx;
    context.phase3_rows = input.phase3_rows;
    OcclusionGroupShadowLifecycle::AppendSplitCandidateRow(
        *input.occlusion_state, candidate, "phase4_merged_split_handoff", context, "merged_split_handoff");
    return true;
  }

  return false;
}

}  // namespace vision_demo_host
