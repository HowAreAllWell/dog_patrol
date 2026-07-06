#include "inactive_recovery_input_collector.hpp"

namespace vision_demo_host {

InactiveRecoveryInputCollector::Result InactiveRecoveryInputCollector::Collect(const Input &input) {
  Result result;
  if (input.tracks == nullptr || input.person_track_indices == nullptr || input.person_features == nullptr ||
      input.assigned_track_to_sid == nullptr || input.inactive_semantic_ids == nullptr) {
    return result;
  }

  result.recovery_track_indices.reserve(input.person_track_indices->size());
  result.selected_features.reserve(input.person_track_indices->size());
  result.solver_tracks.reserve(input.person_track_indices->size());
  for (std::size_t person_row = 0; person_row < input.person_track_indices->size(); ++person_row) {
    const int track_idx = (*input.person_track_indices)[person_row];
    if (track_idx < 0 || track_idx >= static_cast<int>(input.tracks->size())) {
      continue;
    }
    if (input.assigned_track_to_sid->find(track_idx) != input.assigned_track_to_sid->end()) {
      continue;
    }
    const Track &track = (*input.tracks)[static_cast<std::size_t>(track_idx)];
    if (track.occlusion_suspect) {
      continue;
    }

    result.recovery_track_indices.push_back(track_idx);
    const std::vector<float> feature =
        person_row < input.person_features->size() ? (*input.person_features)[person_row] : std::vector<float>{};
    result.selected_features.push_back(feature);

    InactiveRecoverySolver::TrackInput solver_track;
    solver_track.track_idx = track_idx;
    solver_track.raw_track_id = track.id;
    result.solver_tracks.push_back(std::move(solver_track));
  }

  result.free_semantic_ids.reserve(input.inactive_semantic_ids->size());
  result.solver_candidates.reserve(input.inactive_semantic_ids->size());
  for (const int semantic_id : *input.inactive_semantic_ids) {
    if (input.semantic_id_used && input.semantic_id_used(semantic_id)) {
      continue;
    }
    result.free_semantic_ids.push_back(semantic_id);

    InactiveRecoverySolver::CandidateInput solver_candidate;
    solver_candidate.semantic_id = semantic_id;
    result.solver_candidates.push_back(std::move(solver_candidate));
  }

  if (!input.find_identity || !input.can_recover_identity || !input.score_evidence) {
    return result;
  }

  result.solver_scores.reserve(result.recovery_track_indices.size() * result.free_semantic_ids.size());
  for (std::size_t track_row = 0; track_row < result.recovery_track_indices.size(); ++track_row) {
    const int track_idx = result.recovery_track_indices[track_row];
    const Track &track = (*input.tracks)[static_cast<std::size_t>(track_idx)];
    const std::vector<float> &feature = result.selected_features[track_row];
    for (std::size_t candidate_col = 0; candidate_col < result.free_semantic_ids.size(); ++candidate_col) {
      const int semantic_id = result.free_semantic_ids[candidate_col];
      const IdentityRuntimeRecord *identity = input.find_identity(semantic_id);
      if (identity == nullptr) {
        continue;
      }
      if (!input.can_recover_identity(*identity)) {
        continue;
      }

      const ScoreEvidence evidence = input.score_evidence(track, *identity, feature);
      InactiveRecoverySolver::CandidateScore score;
      score.track_row = static_cast<int>(track_row);
      score.candidate_col = static_cast<int>(candidate_col);
      score.app_cost = evidence.app_cost;
      score.geo_cost = evidence.geo_cost;
      score.similarity = evidence.similarity;
      score.recover_threshold = evidence.recover_threshold;
      score.passes_missing_identity_gate = evidence.passes_missing_identity_gate;
      result.solver_scores.push_back(std::move(score));
    }
  }

  return result;
}

}  // namespace vision_demo_host
