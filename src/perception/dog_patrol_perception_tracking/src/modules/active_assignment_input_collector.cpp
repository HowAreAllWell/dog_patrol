#include "active_assignment_input_collector.hpp"

namespace vision_demo_host {

ActiveAssignmentInputCollector::Result ActiveAssignmentInputCollector::Collect(const Input &input) {
  Result result;
  if (input.tracks == nullptr || input.person_track_indices == nullptr || input.person_features == nullptr ||
      input.assigned_track_to_sid == nullptr || input.active_semantic_ids == nullptr) {
    return result;
  }

  result.unassigned_track_indices.reserve(input.person_track_indices->size());
  result.selected_features.reserve(input.person_track_indices->size());
  result.builder_tracks.reserve(input.person_track_indices->size());
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

    result.unassigned_track_indices.push_back(track_idx);
    const std::vector<float> feature =
        person_row < input.person_features->size() ? (*input.person_features)[person_row] : std::vector<float>{};
    result.selected_features.push_back(feature);

    AssignmentCandidateBuilder::ActiveTrackInput builder_track;
    builder_track.track_idx = track_idx;
    builder_track.raw_track_id = track.id;
    builder_track.association = track.association;
    result.builder_tracks.push_back(std::move(builder_track));
  }

  result.free_semantic_ids.reserve(input.active_semantic_ids->size());
  result.builder_candidates.reserve(input.active_semantic_ids->size());
  for (const int semantic_id : *input.active_semantic_ids) {
    if (input.semantic_id_used && input.semantic_id_used(semantic_id)) {
      continue;
    }
    result.free_semantic_ids.push_back(semantic_id);

    AssignmentCandidateBuilder::ActiveCandidateInput builder_candidate;
    builder_candidate.semantic_id = semantic_id;
    const IdentityRuntimeRecord *identity = input.find_identity ? input.find_identity(semantic_id) : nullptr;
    builder_candidate.missing_frames = identity == nullptr ? 0 : identity->missing_frames;
    result.builder_candidates.push_back(std::move(builder_candidate));
  }

  if (!input.score_evidence || !input.find_identity) {
    return result;
  }

  result.builder_scores.reserve(result.unassigned_track_indices.size() * result.free_semantic_ids.size());
  for (std::size_t track_row = 0; track_row < result.unassigned_track_indices.size(); ++track_row) {
    const int track_idx = result.unassigned_track_indices[track_row];
    const Track &track = (*input.tracks)[static_cast<std::size_t>(track_idx)];
    const std::vector<float> &feature = result.selected_features[track_row];
    for (std::size_t candidate_col = 0; candidate_col < result.free_semantic_ids.size(); ++candidate_col) {
      const int semantic_id = result.free_semantic_ids[candidate_col];
      const IdentityRuntimeRecord *identity = input.find_identity(semantic_id);
      if (identity == nullptr) {
        continue;
      }

      const ScoreEvidence evidence = input.score_evidence(track, *identity, feature);
      AssignmentCandidateBuilder::CandidateScore score;
      score.track_row = static_cast<int>(track_row);
      score.candidate_col = static_cast<int>(candidate_col);
      score.app_cost = evidence.app_cost;
      score.geo_cost = evidence.geo_cost;
      score.time_cost = evidence.time_cost;
      score.final_score = evidence.final_score;
      score.passes_missing_identity_gate = evidence.passes_missing_identity_gate;
      score.passes_missing_appearance_gate = evidence.passes_missing_appearance_gate;
      result.builder_scores.push_back(std::move(score));
    }
  }

  return result;
}

}  // namespace vision_demo_host
