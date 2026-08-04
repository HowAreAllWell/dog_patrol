#include "assignment_candidate_builder.hpp"

#include <algorithm>

namespace dog_patrol_perception_tracking {

AssignmentCandidateBuilder::DebugRow AssignmentCandidateBuilder::MakeAssignCandidateRow(
    const int track_idx, const int raw_track_id, const int semantic_id, const float app_cost, const float geo_cost,
    const float time_cost, const float final_score, const std::string &reject_reason) {
  DebugRow row;
  row.track_idx = track_idx;
  row.raw_track_id = raw_track_id;
  row.semantic_id = semantic_id;
  row.app_cost = app_cost;
  row.geo_cost = geo_cost;
  row.time_cost = time_cost;
  row.final_score = final_score;
  row.stage = "assign_candidate";
  row.reject_reason = reject_reason;
  return row;
}

AssignmentCandidateBuilder::ActiveBuildResult AssignmentCandidateBuilder::BuildActiveAssignments(
    const std::vector<ActiveTrackInput> &tracks, const std::vector<ActiveCandidateInput> &candidates,
    const std::vector<CandidateScore> &scores) {
  ActiveBuildResult result;
  result.solver_tracks.reserve(tracks.size());
  result.solver_candidates.reserve(candidates.size());
  result.cost_matrix.assign(tracks.size(), std::vector<float>(candidates.size(), 1.0F));
  result.appearance_cost_matrix.assign(tracks.size(), std::vector<float>(candidates.size(), 1.0F));

  for (const auto &track : tracks) {
    ActiveAssignmentSolver::TrackInput solver_track;
    solver_track.track_idx = track.track_idx;
    solver_track.raw_track_id = track.raw_track_id;
    solver_track.association = track.association;
    result.solver_tracks.push_back(std::move(solver_track));
  }
  for (const auto &candidate : candidates) {
    ActiveAssignmentSolver::CandidateInput solver_candidate;
    solver_candidate.semantic_id = candidate.semantic_id;
    solver_candidate.missing_frames = candidate.missing_frames;
    result.solver_candidates.push_back(std::move(solver_candidate));
  }

  result.debug_rows.reserve(scores.size());
  for (const auto &score : scores) {
    if (score.track_row < 0 || score.candidate_col < 0 ||
        score.track_row >= static_cast<int>(tracks.size()) ||
        score.candidate_col >= static_cast<int>(candidates.size())) {
      continue;
    }
    const auto &track = tracks[static_cast<std::size_t>(score.track_row)];
    const auto &candidate = candidates[static_cast<std::size_t>(score.candidate_col)];
    std::string reject_reason;
    float matrix_cost = score.final_score;
    if (!score.passes_missing_identity_gate) {
      reject_reason = "missing_identity_gate_reject";
      matrix_cost = ActiveAssignmentSolver::kBigCost;
    } else if (!score.passes_missing_appearance_gate) {
      reject_reason = "missing_appearance_gate_reject";
      matrix_cost = ActiveAssignmentSolver::kBigCost;
    }
    result.cost_matrix[static_cast<std::size_t>(score.track_row)][static_cast<std::size_t>(score.candidate_col)] =
        matrix_cost;
    result.appearance_cost_matrix[static_cast<std::size_t>(score.track_row)]
                                 [static_cast<std::size_t>(score.candidate_col)] = score.app_cost;
    result.debug_rows.push_back(MakeAssignCandidateRow(track.track_idx, track.raw_track_id, candidate.semantic_id,
                                                       score.app_cost, score.geo_cost, score.time_cost,
                                                       score.final_score, reject_reason));
  }

  return result;
}

void AssignmentCandidateBuilder::ApplyActiveSolverResults(
    const std::vector<ActiveAssignmentSolver::Assignment> &assignments, const bool phase4_pairwise_override_pending,
    std::vector<DebugRow> *rows) {
  if (rows == nullptr) {
    return;
  }
  for (const auto &assignment : assignments) {
    for (auto &row : *rows) {
      if (row.track_idx != assignment.track_idx || row.semantic_id != assignment.semantic_id ||
          row.stage != "assign_candidate") {
        continue;
      }
      row.selected = true;
      row.margin = assignment.margin;
      row.accepted = phase4_pairwise_override_pending ? false : assignment.accepted;
      row.reject_reason =
          phase4_pairwise_override_pending ? "phase4_pairwise_assignment_pending" : assignment.reject_reason;
    }
  }
}

std::vector<AssignmentCandidateBuilder::DebugRow> AssignmentCandidateBuilder::BuildInactiveRecoveryRows(
    const std::vector<InactiveRecoverySolver::CandidateDecision> &decisions) {
  std::vector<DebugRow> rows;
  rows.reserve(decisions.size());
  for (const auto &decision : decisions) {
    DebugRow row;
    row.track_idx = decision.track_idx;
    row.raw_track_id = decision.raw_track_id;
    row.semantic_id = decision.semantic_id;
    row.app_cost = decision.app_cost;
    row.geo_cost = decision.geo_cost;
    row.final_score = decision.similarity;
    row.stage = "inactive_recover_candidate";
    row.accepted = decision.accepted;
    row.reject_reason = decision.reject_reason;
    rows.push_back(std::move(row));
  }
  return rows;
}

void AssignmentCandidateBuilder::ApplyInactiveRecoveryAssignments(
    const std::vector<InactiveRecoverySolver::Assignment> &assignments, std::vector<DebugRow> *rows) {
  if (rows == nullptr) {
    return;
  }
  for (const auto &assignment : assignments) {
    for (auto &row : *rows) {
      if (row.track_idx != assignment.track_idx || row.semantic_id != assignment.semantic_id ||
          row.stage != "inactive_recover_candidate") {
        continue;
      }
      row.selected = true;
      row.margin = assignment.margin;
      row.accepted = true;
    }
  }
}

}  // namespace dog_patrol_perception_tracking
