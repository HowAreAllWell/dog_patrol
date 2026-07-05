#include "phase5_birth_coordinator.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace vision_demo_host {
namespace {

bool RequiresStability(const TrackletObservation &observation) {
  const float height = std::max(1.0F, observation.bbox.height);
  const float aspect = observation.bbox.width / height;
  return observation.bbox.area() < 20000.0F && height < 300.0F &&
         aspect >= 0.25F && aspect <= 0.60F;
}

std::string HiddenStatus(const std::string &reason) {
  if (reason == "ambiguous_recovery_pending") {
    return "pending_recovery";
  }
  if (reason == "duplicate_split_hidden") {
    return "hidden_duplicate_split";
  }
  if (reason == "skinny_partial_hidden" || reason == "wide_fragment_hidden") {
    return "hidden_partial_fragment";
  }
  return "hidden";
}

void AppendRow(const IdentityManager::ScoreDebugRow &score,
               const TrackletObservation *observation,
               const std::string &event_type,
               const std::string &reason,
               const std::string &status,
               const int stable_frames,
               const int current_frame_idx,
               int *event_idx,
               std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows) {
  IdentityManager::Phase3ShadowDebugRow row;
  row.frame_idx = current_frame_idx;
  row.event_idx = (*event_idx)++;
  row.event_type = event_type;
  row.candidate_raw_track_id = score.raw_track_id;
  row.candidate_semantic_id = score.semantic_id;
  if (observation != nullptr) {
    row.candidate_bbox = observation->bbox;
    row.candidate_confidence = observation->confidence;
  }
  row.reason = reason;
  row.hypothesis_status = status;
  row.candidate_stable_frames = stable_frames;
  row.decision_app_cost = score.app_cost;
  row.decision_geo_cost = score.geo_cost;
  row.decision_time_cost = score.time_cost;
  row.decision_final_score = score.final_score;
  row.decision_margin = score.margin;
  row.decision_selected = score.selected;
  row.decision_accepted = score.accepted;
  phase3_rows->push_back(std::move(row));
}

}  // namespace

void Phase5BirthCoordinator::ApplyAcceptedBirths(const ApplyInput &input,
                                                 const ApplyAcceptedAllocationFn &apply_accepted_allocation,
                                                 const RefreshScoreRowsFn &refresh_score_rows) {
  if (!input.enabled || input.score_rows == nullptr || input.shadow_by_raw_track_id == nullptr ||
      input.observations_by_raw_track_id == nullptr || input.raw_to_semantic_id == nullptr) {
    return;
  }

  std::vector<int> allocation_raw_ids;
  for (auto &row : *input.score_rows) {
    if (row.stage != "phase5_birth_candidate" || row.raw_track_id <= 0 || row.accepted) {
      continue;
    }
    row.selected = true;

    if (row.reject_reason != "phase5_birth_manager_pending") {
      input.shadow_by_raw_track_id->erase(row.raw_track_id);
      continue;
    }

    const auto obs_it = input.observations_by_raw_track_id->find(row.raw_track_id);
    if (obs_it != input.observations_by_raw_track_id->end() && RequiresStability(obs_it->second)) {
      auto &candidate = (*input.shadow_by_raw_track_id)[row.raw_track_id];
      if (candidate.last_update_frame != input.current_frame_idx - 1) {
        candidate.stable_frames = 0;
      }
      candidate.stable_frames += 1;
      candidate.bbox = obs_it->second.bbox;
      candidate.confidence = obs_it->second.confidence;
      candidate.last_update_frame = input.current_frame_idx;
      candidate.requires_stability = true;
      if (candidate.stable_frames < 2) {
        row.reject_reason = "small_new_person_pending";
        continue;
      }
    }

    allocation_raw_ids.push_back(row.raw_track_id);
  }

  for (const int raw_track_id : allocation_raw_ids) {
    apply_accepted_allocation(raw_track_id);
    input.shadow_by_raw_track_id->erase(raw_track_id);
  }
  if (allocation_raw_ids.empty()) {
    return;
  }

  refresh_score_rows();
  for (const auto &row : *input.score_rows) {
    if (row.stage == "phase5_new_semantic" && row.accepted &&
        row.raw_track_id > 0 && row.semantic_id > 0) {
      (*input.raw_to_semantic_id)[row.raw_track_id] = row.semantic_id;
    }
  }
}

void Phase5BirthCoordinator::AppendShadowLifecycleRows(const ShadowRowsInput &input) {
  if (input.observations == nullptr || input.observations_by_raw_track_id == nullptr ||
      input.raw_to_semantic_id == nullptr || input.score_rows == nullptr ||
      input.shadow_by_raw_track_id == nullptr || input.phase3_rows == nullptr ||
      input.next_event_idx == nullptr) {
    return;
  }

  std::set<int> scored_new_birth_raw_ids;
  for (const auto &score : *input.score_rows) {
    if (score.stage != "birth_candidate" && score.stage != "new_semantic" &&
        score.stage != "phase5_new_semantic" && score.stage != "phase5_birth_candidate") {
      continue;
    }
    if (score.raw_track_id <= 0) {
      continue;
    }
    scored_new_birth_raw_ids.insert(score.raw_track_id);
    const auto obs_it = input.observations_by_raw_track_id->find(score.raw_track_id);
    const TrackletObservation *observation =
        obs_it == input.observations_by_raw_track_id->end() ? nullptr : &obs_it->second;
    const auto pending_it = input.shadow_by_raw_track_id->find(score.raw_track_id);
    const int stable_frames =
        pending_it == input.shadow_by_raw_track_id->end()
            ? 0
            : pending_it->second.stable_frames +
                  (pending_it->second.last_update_frame == input.current_frame_idx ? 0 : 1);

    if (score.stage == "phase5_birth_candidate" &&
        score.reject_reason == "phase5_birth_manager_pending") {
      continue;
    }
    if (score.stage == "phase5_birth_candidate" &&
        score.reject_reason == "small_new_person_pending") {
      AppendRow(score, observation, "new_birth_candidate_pending",
                "small_new_person_pending", "pending_stability", stable_frames,
                input.current_frame_idx, input.next_event_idx, input.phase3_rows);
      continue;
    }
    if (score.stage == "birth_candidate") {
      AppendRow(score, observation, "new_birth_candidate_hidden",
                score.reject_reason.empty() ? "birth_candidate_hidden" : score.reject_reason,
                HiddenStatus(score.reject_reason), 0, input.current_frame_idx,
                input.next_event_idx, input.phase3_rows);
      input.shadow_by_raw_track_id->erase(score.raw_track_id);
      continue;
    }
    if (score.stage == "phase5_birth_candidate") {
      AppendRow(score, observation, "new_birth_candidate_hidden",
                score.reject_reason.empty() ? "phase5_birth_candidate_hidden" : score.reject_reason,
                HiddenStatus(score.reject_reason), 0, input.current_frame_idx,
                input.next_event_idx, input.phase3_rows);
      input.shadow_by_raw_track_id->erase(score.raw_track_id);
      continue;
    }
    if ((score.stage == "new_semantic" || score.stage == "phase5_new_semantic") && score.accepted) {
      const bool promoted_after_stability =
          pending_it != input.shadow_by_raw_track_id->end() &&
          pending_it->second.requires_stability;
      AppendRow(score, observation, "new_birth_candidate_allocated",
                promoted_after_stability ? "small_stable_new_person_promoted" : "phase5_birth_manager_allocated",
                "allocated", stable_frames, input.current_frame_idx,
                input.next_event_idx, input.phase3_rows);
      input.shadow_by_raw_track_id->erase(score.raw_track_id);
    }
  }

  for (const auto &obs : *input.observations) {
    if (obs.class_id != ClassId::kPerson || obs.raw_track_id <= 0 ||
        input.raw_to_semantic_id->find(obs.raw_track_id) != input.raw_to_semantic_id->end() ||
        scored_new_birth_raw_ids.count(obs.raw_track_id) > 0) {
      continue;
    }

    auto &candidate = (*input.shadow_by_raw_track_id)[obs.raw_track_id];
    if (candidate.last_update_frame != input.current_frame_idx - 1) {
      candidate.stable_frames = 0;
    }
    candidate.stable_frames += 1;
    candidate.bbox = obs.bbox;
    candidate.confidence = obs.confidence;
    candidate.last_update_frame = input.current_frame_idx;
    candidate.requires_stability = true;

    IdentityManager::ScoreDebugRow pending_score;
    pending_score.raw_track_id = obs.raw_track_id;
    pending_score.semantic_id = -1;
    AppendRow(pending_score, &obs, "new_birth_candidate_pending",
              "small_new_person_pending", "pending_stability", candidate.stable_frames,
              input.current_frame_idx, input.next_event_idx, input.phase3_rows);
  }

  for (auto it = input.shadow_by_raw_track_id->begin();
       it != input.shadow_by_raw_track_id->end();) {
    if (it->second.last_update_frame == input.current_frame_idx) {
      ++it;
      continue;
    }
    it = input.shadow_by_raw_track_id->erase(it);
  }
}

}  // namespace vision_demo_host
