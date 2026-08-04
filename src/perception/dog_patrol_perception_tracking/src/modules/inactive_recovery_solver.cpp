#include "inactive_recovery_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "active_assignment_solver.hpp"

namespace dog_patrol_perception_tracking {
namespace {

float RecoveryMaxCost(const InactiveRecoverySolver::Config &config) {
  return std::clamp(config.recovery_max_cost, 0.0F, 1.0F);
}

InactiveRecoverySolver::CandidateDecision BuildDecision(
    const InactiveRecoverySolver::TrackInput &track,
    const InactiveRecoverySolver::CandidateInput &candidate,
    const InactiveRecoverySolver::CandidateScore &score,
    const InactiveRecoverySolver::Config &config) {
  InactiveRecoverySolver::CandidateDecision decision;
  decision.track_row = score.track_row;
  decision.candidate_col = score.candidate_col;
  decision.track_idx = track.track_idx;
  decision.raw_track_id = track.raw_track_id;
  decision.semantic_id = candidate.semantic_id;
  decision.app_cost = score.app_cost;
  decision.geo_cost = score.geo_cost;
  decision.similarity = score.similarity;
  decision.recovery_cost = 1.0F - score.similarity;

  if (!score.passes_missing_identity_gate) {
    decision.reject_reason = "missing_identity_gate_reject";
    return decision;
  }
  decision.accepted =
      score.similarity >= std::clamp(score.recover_threshold, 0.0F, 1.0F) &&
      decision.recovery_cost <= RecoveryMaxCost(config);
  if (score.similarity < std::clamp(score.recover_threshold, 0.0F, 1.0F)) {
    decision.reject_reason = "below_recover_threshold";
  } else if (decision.recovery_cost > RecoveryMaxCost(config)) {
    decision.reject_reason = "recovery_max_cost_reject";
  }
  return decision;
}

bool ValidScoreShape(const InactiveRecoverySolver::CandidateScore &score,
                     const std::vector<InactiveRecoverySolver::TrackInput> &tracks,
                     const std::vector<InactiveRecoverySolver::CandidateInput> &candidates) {
  return score.track_row >= 0 && score.candidate_col >= 0 &&
         score.track_row < static_cast<int>(tracks.size()) &&
         score.candidate_col < static_cast<int>(candidates.size());
}

}  // namespace

InactiveRecoverySolver::Result InactiveRecoverySolver::SelectBestSimilarity(
    const std::vector<TrackInput> &tracks,
    const std::vector<CandidateInput> &candidates,
    const std::vector<CandidateScore> &scores,
    const Config &config) {
  Result result;
  float best_similarity = -std::numeric_limits<float>::infinity();
  int best_decision_idx = -1;
  for (const auto &score : scores) {
    if (!ValidScoreShape(score, tracks, candidates)) {
      continue;
    }
    auto decision = BuildDecision(tracks[static_cast<std::size_t>(score.track_row)],
                                  candidates[static_cast<std::size_t>(score.candidate_col)],
                                  score,
                                  config);
    if (decision.accepted && decision.similarity > best_similarity) {
      best_similarity = decision.similarity;
      best_decision_idx = static_cast<int>(result.candidates.size());
    }
    result.candidates.push_back(std::move(decision));
  }
  if (best_decision_idx >= 0) {
    const auto &decision = result.candidates[static_cast<std::size_t>(best_decision_idx)];
    result.assignments.push_back(Assignment{decision.track_row,
                                            decision.candidate_col,
                                            decision.track_idx,
                                            decision.raw_track_id,
                                            decision.semantic_id,
                                            decision.similarity,
                                            decision.recovery_cost,
                                            0.0F});
  }
  return result;
}

InactiveRecoverySolver::Result InactiveRecoverySolver::SolveHungarian(
    const std::vector<TrackInput> &tracks,
    const std::vector<CandidateInput> &candidates,
    const std::vector<CandidateScore> &scores,
    const Config &config) {
  Result result;
  if (tracks.empty() || candidates.empty()) {
    return result;
  }

  std::vector<std::vector<float>> recovery_cost(tracks.size(), std::vector<float>(candidates.size(), kBigCost));
  for (const auto &score : scores) {
    if (!ValidScoreShape(score, tracks, candidates)) {
      continue;
    }
    auto decision = BuildDecision(tracks[static_cast<std::size_t>(score.track_row)],
                                  candidates[static_cast<std::size_t>(score.candidate_col)],
                                  score,
                                  config);
    if (decision.accepted) {
      recovery_cost[static_cast<std::size_t>(score.track_row)][static_cast<std::size_t>(score.candidate_col)] =
          decision.recovery_cost;
    }
    result.candidates.push_back(std::move(decision));
  }

  const std::vector<int> assignment = ActiveAssignmentSolver::HungarianAssignment(recovery_cost);
  for (std::size_t r = 0; r < tracks.size(); ++r) {
    if (r >= assignment.size()) {
      continue;
    }
    const int c = assignment[r];
    if (c < 0 || c >= static_cast<int>(candidates.size())) {
      continue;
    }
    const float cst = recovery_cost[r][static_cast<std::size_t>(c)];
    if (cst >= kBigCost * 0.5F) {
      continue;
    }
    const float margin = ActiveAssignmentSolver::AssignmentMargin(recovery_cost, r, c);
    result.assignments.push_back(Assignment{static_cast<int>(r),
                                            c,
                                            tracks[r].track_idx,
                                            tracks[r].raw_track_id,
                                            candidates[static_cast<std::size_t>(c)].semantic_id,
                                            1.0F - cst,
                                            cst,
                                            margin});
  }
  return result;
}

}  // namespace dog_patrol_perception_tracking
