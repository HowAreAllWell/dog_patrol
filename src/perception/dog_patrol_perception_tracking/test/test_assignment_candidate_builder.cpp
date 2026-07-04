#include <gtest/gtest.h>

#include <vector>

#include "active_assignment_solver.hpp"
#include "assignment_candidate_builder.hpp"
#include "inactive_recovery_solver.hpp"

namespace {

using vision_demo_host::ActiveAssignmentSolver;
using vision_demo_host::AssignmentCandidateBuilder;
using vision_demo_host::InactiveRecoverySolver;

AssignmentCandidateBuilder::ActiveTrackInput ActiveTrack(const int track_idx, const int raw_track_id) {
  AssignmentCandidateBuilder::ActiveTrackInput track;
  track.track_idx = track_idx;
  track.raw_track_id = raw_track_id;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

AssignmentCandidateBuilder::ActiveCandidateInput ActiveCandidate(const int semantic_id,
                                                                 const int missing_frames = 0) {
  AssignmentCandidateBuilder::ActiveCandidateInput candidate;
  candidate.semantic_id = semantic_id;
  candidate.missing_frames = missing_frames;
  return candidate;
}

AssignmentCandidateBuilder::CandidateScore CandidateScore(const int track_row,
                                                          const int candidate_col,
                                                          const float final_score,
                                                          const float app_cost,
                                                          const float geo_cost,
                                                          const float time_cost,
                                                          const bool passes_missing_identity_gate = true,
                                                          const bool passes_missing_appearance_gate = true) {
  AssignmentCandidateBuilder::CandidateScore score;
  score.track_row = track_row;
  score.candidate_col = candidate_col;
  score.app_cost = app_cost;
  score.geo_cost = geo_cost;
  score.time_cost = time_cost;
  score.final_score = final_score;
  score.passes_missing_identity_gate = passes_missing_identity_gate;
  score.passes_missing_appearance_gate = passes_missing_appearance_gate;
  return score;
}

InactiveRecoverySolver::CandidateDecision RecoveryDecision(const int track_idx,
                                                           const int raw_track_id,
                                                           const int semantic_id,
                                                           const float app_cost,
                                                           const float geo_cost,
                                                           const float similarity,
                                                           const bool accepted,
                                                           const char *reject_reason = "") {
  InactiveRecoverySolver::CandidateDecision decision;
  decision.track_idx = track_idx;
  decision.raw_track_id = raw_track_id;
  decision.semantic_id = semantic_id;
  decision.app_cost = app_cost;
  decision.geo_cost = geo_cost;
  decision.similarity = similarity;
  decision.accepted = accepted;
  decision.reject_reason = reject_reason;
  return decision;
}

}  // namespace

TEST(AssignmentCandidateBuilderTest, BuildsActiveCandidatesRowsAndSolverInputs) {
  const std::vector<AssignmentCandidateBuilder::ActiveTrackInput> tracks{ActiveTrack(4, 104)};
  const std::vector<AssignmentCandidateBuilder::ActiveCandidateInput> candidates{
      ActiveCandidate(9, 3),
      ActiveCandidate(10, 0),
  };
  const std::vector<AssignmentCandidateBuilder::CandidateScore> scores{
      CandidateScore(0, 0, 0.25F, 0.10F, 0.20F, 0.30F),
      CandidateScore(0, 1, 0.40F, 0.15F, 0.25F, 0.35F, false, true),
  };

  const auto result = AssignmentCandidateBuilder::BuildActiveAssignments(tracks, candidates, scores);

  ASSERT_EQ(result.solver_tracks.size(), 1U);
  EXPECT_EQ(result.solver_tracks[0].track_idx, 4);
  EXPECT_EQ(result.solver_tracks[0].raw_track_id, 104);
  ASSERT_EQ(result.solver_candidates.size(), 2U);
  EXPECT_EQ(result.solver_candidates[0].semantic_id, 9);
  EXPECT_EQ(result.solver_candidates[0].missing_frames, 3);
  ASSERT_EQ(result.cost_matrix.size(), 1U);
  ASSERT_EQ(result.cost_matrix[0].size(), 2U);
  EXPECT_FLOAT_EQ(result.cost_matrix[0][0], 0.25F);
  EXPECT_FLOAT_EQ(result.appearance_cost_matrix[0][0], 0.10F);
  EXPECT_GE(result.cost_matrix[0][1], ActiveAssignmentSolver::kBigCost * 0.5F);

  ASSERT_EQ(result.debug_rows.size(), 2U);
  EXPECT_EQ(result.debug_rows[0].stage, "assign_candidate");
  EXPECT_EQ(result.debug_rows[0].semantic_id, 9);
  EXPECT_EQ(result.debug_rows[0].reject_reason, "");
  EXPECT_EQ(result.debug_rows[1].semantic_id, 10);
  EXPECT_EQ(result.debug_rows[1].reject_reason, "missing_identity_gate_reject");
}

TEST(AssignmentCandidateBuilderTest, BuildsActiveMissingAppearanceRejectRows) {
  const std::vector<AssignmentCandidateBuilder::ActiveTrackInput> tracks{ActiveTrack(7, 107)};
  const std::vector<AssignmentCandidateBuilder::ActiveCandidateInput> candidates{ActiveCandidate(12)};
  const std::vector<AssignmentCandidateBuilder::CandidateScore> scores{
      CandidateScore(0, 0, 0.30F, 0.11F, 0.22F, 0.33F, true, false),
  };

  const auto result = AssignmentCandidateBuilder::BuildActiveAssignments(tracks, candidates, scores);

  ASSERT_EQ(result.debug_rows.size(), 1U);
  EXPECT_EQ(result.debug_rows[0].reject_reason, "missing_appearance_gate_reject");
  EXPECT_GE(result.cost_matrix[0][0], ActiveAssignmentSolver::kBigCost * 0.5F);
}

TEST(AssignmentCandidateBuilderTest, AppliesActiveSolverResultsAndPendingOverrideToRows) {
  std::vector<AssignmentCandidateBuilder::DebugRow> rows{
      AssignmentCandidateBuilder::MakeAssignCandidateRow(4, 104, 9, 0.10F, 0.20F, 0.30F, 0.25F, ""),
  };
  const std::vector<ActiveAssignmentSolver::Assignment> assignments{
      ActiveAssignmentSolver::Assignment{0, 0, 4, 9, 0.75F, 0.25F, 0.12F, true, "", true},
  };

  AssignmentCandidateBuilder::ApplyActiveSolverResults(assignments, true, &rows);

  ASSERT_EQ(rows.size(), 1U);
  EXPECT_TRUE(rows[0].selected);
  EXPECT_FLOAT_EQ(rows[0].margin, 0.12F);
  EXPECT_FALSE(rows[0].accepted);
  EXPECT_EQ(rows[0].reject_reason, "phase4_pairwise_assignment_pending");
}

TEST(AssignmentCandidateBuilderTest, BuildsInactiveRecoveryRowsAndAppliesAssignments) {
  const std::vector<InactiveRecoverySolver::CandidateDecision> decisions{
      RecoveryDecision(5, 205, 11, 0.10F, 0.22F, 0.90F, true),
      RecoveryDecision(5, 205, 12, 0.25F, 0.33F, 0.75F, false, "recovery_max_cost_reject"),
  };

  auto rows = AssignmentCandidateBuilder::BuildInactiveRecoveryRows(
      decisions);

  ASSERT_EQ(rows.size(), 2U);
  EXPECT_EQ(rows[0].stage, "inactive_recover_candidate");
  EXPECT_TRUE(rows[0].accepted);
  EXPECT_EQ(rows[1].reject_reason, "recovery_max_cost_reject");

  const std::vector<InactiveRecoverySolver::Assignment> assignments{
      InactiveRecoverySolver::Assignment{0, 0, 5, 205, 11, 0.90F, 0.10F, 0.18F},
  };
  AssignmentCandidateBuilder::ApplyInactiveRecoveryAssignments(assignments, &rows);

  EXPECT_TRUE(rows[0].selected);
  EXPECT_TRUE(rows[0].accepted);
  EXPECT_FLOAT_EQ(rows[0].margin, 0.18F);
  EXPECT_FALSE(rows[1].selected);
}
