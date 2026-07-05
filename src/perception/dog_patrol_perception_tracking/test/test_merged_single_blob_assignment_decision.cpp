#include <gtest/gtest.h>

#include <vector>

#include "merged_single_blob_assignment_decision.hpp"

namespace {

using vision_demo_host::InactiveRecoverySolver;
using vision_demo_host::MergedSingleBlobAssignmentDecision;

MergedSingleBlobAssignmentDecision::CandidateRow ActiveCandidate(
    const int semantic_id,
    const float app_cost,
    const float geo_cost,
    const float final_score,
    const int missing_frames = 0,
    const char *reject_reason = "") {
  MergedSingleBlobAssignmentDecision::CandidateRow row;
  row.track_idx = 3;
  row.raw_track_id = 103;
  row.semantic_id = semantic_id;
  row.missing_frames = missing_frames;
  row.app_cost = app_cost;
  row.geo_cost = geo_cost;
  row.time_cost = 0.01F;
  row.final_score = final_score;
  row.stage = "merged_candidate";
  row.reject_reason = reject_reason;
  return row;
}

MergedSingleBlobAssignmentDecision::CandidateRow InactiveRow(
    const int semantic_id,
    const float app_cost,
    const float geo_cost,
    const float similarity,
    const bool accepted = true) {
  MergedSingleBlobAssignmentDecision::CandidateRow row;
  row.track_idx = 3;
  row.raw_track_id = 103;
  row.semantic_id = semantic_id;
  row.app_cost = app_cost;
  row.geo_cost = geo_cost;
  row.final_score = similarity;
  row.stage = "inactive_recover_candidate";
  row.accepted = accepted;
  return row;
}

InactiveRecoverySolver::Assignment InactiveAssignment(const int semantic_id) {
  InactiveRecoverySolver::Assignment assignment;
  assignment.track_idx = 3;
  assignment.raw_track_id = 103;
  assignment.semantic_id = semantic_id;
  assignment.similarity = 0.91F;
  assignment.recovery_cost = 0.09F;
  assignment.margin = 0.20F;
  return assignment;
}

MergedSingleBlobAssignmentDecision::Input BaseInput(const int continuity_sid = 1) {
  MergedSingleBlobAssignmentDecision::Input input;
  input.track_idx = 3;
  input.raw_track_id = 103;
  input.continuity_semantic_id = continuity_sid;
  return input;
}

MergedSingleBlobAssignmentDecision::Config Config() {
  MergedSingleBlobAssignmentDecision::Config config;
  config.min_assignment_margin = 0.08F;
  config.max_missing_frames = 180;
  return config;
}

const MergedSingleBlobAssignmentDecision::CandidateRow *FindRow(
    const std::vector<MergedSingleBlobAssignmentDecision::CandidateRow> &rows,
    const int semantic_id,
    const char *stage) {
  for (const auto &row : rows) {
    if (row.semantic_id == semantic_id && row.stage == stage) {
      return &row;
    }
  }
  return nullptr;
}

}  // namespace

TEST(MergedSingleBlobAssignmentDecisionTest, KeepsContinuityWhenItIsOnlyAcceptedActiveCandidate) {
  auto input = BaseInput(1);
  input.active_candidates = {
      ActiveCandidate(1, 0.20F, 0.20F, 0.30F),
      ActiveCandidate(2, 0.10F, 0.20F, 0.25F, 0, "missing_identity_gate_reject"),
  };

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, 1);
  ASSERT_NE(FindRow(result.active_candidates, 1, "merged_candidate"), nullptr);
  EXPECT_TRUE(FindRow(result.active_candidates, 1, "merged_candidate")->selected);
  EXPECT_TRUE(FindRow(result.active_candidates, 1, "merged_candidate")->accepted);
}

TEST(MergedSingleBlobAssignmentDecisionTest, SelectsBestFinalCostWhenContinuityIsAbsent) {
  auto input = BaseInput(-1);
  input.active_candidates = {
      ActiveCandidate(1, 0.20F, 0.20F, 0.34F),
      ActiveCandidate(2, 0.30F, 0.10F, 0.21F),
  };

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, 2);
  EXPECT_TRUE(FindRow(result.active_candidates, 2, "merged_candidate")->selected);
}

TEST(MergedSingleBlobAssignmentDecisionTest, RetainsCloseContinuityCandidateByMargin) {
  auto input = BaseInput(1);
  input.active_candidates = {
      ActiveCandidate(1, 0.35F, 0.20F, 0.27F),
      ActiveCandidate(2, 0.30F, 0.20F, 0.22F),
  };

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, 1);
  EXPECT_TRUE(FindRow(result.active_candidates, 1, "merged_candidate")->selected);
}

TEST(MergedSingleBlobAssignmentDecisionTest, KeepsContinuityWhenHandoffIsEligibleForPhase4) {
  auto input = BaseInput(1);
  input.active_candidates = {
      ActiveCandidate(1, 0.20F, 0.20F, 0.30F),
      ActiveCandidate(2, 0.16F, 0.30F, 0.32F, 20),
  };

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, 1);
  EXPECT_TRUE(FindRow(result.active_candidates, 1, "merged_candidate")->selected);
}

TEST(MergedSingleBlobAssignmentDecisionTest, UsesInactiveRecoveryFallbackWhenActiveCandidatesFail) {
  auto input = BaseInput(-1);
  input.active_candidates = {
      ActiveCandidate(1, 0.20F, 0.20F, 0.30F, 0, "active_assign_max_cost_reject"),
  };
  input.inactive_recovery_rows = {InactiveRow(4, 0.09F, 0.20F, 0.91F)};
  input.inactive_recovery_assignments = {InactiveAssignment(4)};

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, 4);
  EXPECT_TRUE(result.used_inactive_recovery);
  EXPECT_TRUE(FindRow(result.inactive_recovery_rows, 4, "inactive_recover_candidate")->selected);
  EXPECT_TRUE(FindRow(result.inactive_recovery_rows, 4, "inactive_recover_candidate")->accepted);
}

TEST(MergedSingleBlobAssignmentDecisionTest, RequestsNewSemanticIdWhenNoActiveOrInactiveCandidateIsAccepted) {
  auto input = BaseInput(-1);
  input.active_candidates = {
      ActiveCandidate(1, 0.20F, 0.20F, 0.30F, 0, "missing_identity_gate_reject"),
  };
  input.inactive_recovery_rows = {InactiveRow(4, 0.40F, 0.20F, 0.60F, false)};

  const auto result = MergedSingleBlobAssignmentDecision::Decide(input, Config());

  EXPECT_EQ(result.semantic_id, -1);
  EXPECT_TRUE(result.needs_new_semantic_id);
  EXPECT_FALSE(FindRow(result.inactive_recovery_rows, 4, "inactive_recover_candidate")->selected);
}
