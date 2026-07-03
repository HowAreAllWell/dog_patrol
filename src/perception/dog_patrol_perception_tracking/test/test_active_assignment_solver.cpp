#include <gtest/gtest.h>

#include <vector>

#include "active_assignment_solver.hpp"

namespace {

using vision_demo_host::ActiveAssignmentSolver;

ActiveAssignmentSolver::TrackInput TrackInput(const int row) {
  ActiveAssignmentSolver::TrackInput track;
  track.track_idx = row;
  track.raw_track_id = 100 + row;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

ActiveAssignmentSolver::CandidateInput CandidateInput(const int semantic_id,
                                                      const int missing_frames = 0) {
  ActiveAssignmentSolver::CandidateInput candidate;
  candidate.semantic_id = semantic_id;
  candidate.missing_frames = missing_frames;
  return candidate;
}

}  // namespace

TEST(ActiveAssignmentSolverTest, SolvesHungarianAssignmentAndMargins) {
  const std::vector<ActiveAssignmentSolver::TrackInput> tracks{TrackInput(0), TrackInput(1)};
  const std::vector<ActiveAssignmentSolver::CandidateInput> candidates{CandidateInput(1), CandidateInput(2)};
  const std::vector<std::vector<float>> cost{{0.20F, 0.15F}, {0.25F, 0.80F}};
  const std::vector<std::vector<float>> app_cost{{0.20F, 0.15F}, {0.25F, 0.80F}};

  ActiveAssignmentSolver::Config config;
  config.active_assign_max_cost = 0.90F;
  config.min_assignment_margin = 0.01F;

  const auto result = ActiveAssignmentSolver::Solve(tracks, candidates, cost, app_cost, config);

  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.assignments[0].track_idx, 0);
  EXPECT_EQ(result.assignments[0].semantic_id, 2);
  EXPECT_FLOAT_EQ(result.assignments[0].cost, 0.15F);
  EXPECT_FLOAT_EQ(result.assignments[0].margin, 0.05F);
  EXPECT_TRUE(result.assignments[0].accepted);
  EXPECT_EQ(result.assignments[1].track_idx, 1);
  EXPECT_EQ(result.assignments[1].semantic_id, 1);
  EXPECT_FLOAT_EQ(result.assignments[1].cost, 0.25F);
  EXPECT_FLOAT_EQ(result.assignments[1].margin, 0.55F);
  EXPECT_TRUE(result.assignments[1].accepted);
}

TEST(ActiveAssignmentSolverTest, RejectsOverActiveMaxCost) {
  const std::vector<ActiveAssignmentSolver::TrackInput> tracks{TrackInput(3)};
  const std::vector<ActiveAssignmentSolver::CandidateInput> candidates{CandidateInput(4)};
  const std::vector<std::vector<float>> cost{{0.30F}};
  const std::vector<std::vector<float>> app_cost{{0.30F}};

  ActiveAssignmentSolver::Config config;
  config.active_assign_max_cost = 0.10F;
  config.min_assignment_margin = 0.0F;

  const auto result = ActiveAssignmentSolver::Solve(tracks, candidates, cost, app_cost, config);

  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_FALSE(result.assignments[0].accepted);
  EXPECT_EQ(result.assignments[0].reject_reason, "active_assign_max_cost_reject");
}

TEST(ActiveAssignmentSolverTest, RejectsInsufficientMarginUnlessRecentlyMissingCushionApplies) {
  const std::vector<ActiveAssignmentSolver::TrackInput> tracks{TrackInput(5), TrackInput(6)};
  const std::vector<ActiveAssignmentSolver::CandidateInput> candidates{
      CandidateInput(7, 0),
      CandidateInput(8, 4),
  };
  const std::vector<std::vector<float>> cost{{0.40F, 0.45F}, {0.445F, 0.40F}};
  const std::vector<std::vector<float>> app_cost{{0.40F, 0.45F}, {0.445F, 0.40F}};

  ActiveAssignmentSolver::Config config;
  config.max_missing_frames = 20;
  config.active_assign_max_cost = 0.38F;
  config.min_assignment_margin = 0.08F;

  const auto result = ActiveAssignmentSolver::Solve(tracks, candidates, cost, app_cost, config);

  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.assignments[0].semantic_id, 7);
  EXPECT_FALSE(result.assignments[0].accepted);
  EXPECT_EQ(result.assignments[0].reject_reason, "active_assign_max_cost_reject");
  EXPECT_EQ(result.assignments[1].semantic_id, 8);
  EXPECT_TRUE(result.assignments[1].accepted);
  EXPECT_EQ(result.assignments[1].reject_reason, "");
}

TEST(ActiveAssignmentSolverTest, PairwiseAppearanceOverrideSelectsAlternatePairing) {
  const std::vector<ActiveAssignmentSolver::TrackInput> tracks{TrackInput(9), TrackInput(10)};
  const std::vector<ActiveAssignmentSolver::CandidateInput> candidates{
      CandidateInput(1, 5),
      CandidateInput(2, 5),
  };
  const std::vector<std::vector<float>> cost{{0.20F, 0.23F}, {0.24F, 0.20F}};
  const std::vector<std::vector<float>> app_cost{{0.50F, 0.10F}, {0.10F, 0.50F}};

  ActiveAssignmentSolver::Config config;
  config.active_assign_max_cost = 0.55F;
  config.min_assignment_margin = 0.08F;

  const auto result = ActiveAssignmentSolver::Solve(tracks, candidates, cost, app_cost, config);

  ASSERT_EQ(result.pairwise_debug_rows.size(), 1U);
  EXPECT_TRUE(result.pairwise_debug_rows[0].appearance_override);
  EXPECT_FLOAT_EQ(result.pairwise_debug_rows[0].selected_final_cost, 0.40F);
  EXPECT_FLOAT_EQ(result.pairwise_debug_rows[0].alternate_final_cost, 0.47F);
  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.assignments[0].semantic_id, 2);
  EXPECT_EQ(result.assignments[1].semantic_id, 1);
  EXPECT_TRUE(result.assignments[0].pairwise_appearance_override);
  EXPECT_TRUE(result.assignments[1].pairwise_appearance_override);
  EXPECT_TRUE(result.assignments[0].accepted);
  EXPECT_TRUE(result.assignments[1].accepted);
}
