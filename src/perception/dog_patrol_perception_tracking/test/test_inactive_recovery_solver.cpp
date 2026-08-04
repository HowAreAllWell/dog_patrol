#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "inactive_recovery_solver.hpp"

namespace {

using dog_patrol_perception_tracking::InactiveRecoverySolver;

InactiveRecoverySolver::TrackInput TrackInput(const int row) {
  InactiveRecoverySolver::TrackInput track;
  track.track_idx = row;
  track.raw_track_id = 100 + row;
  return track;
}

InactiveRecoverySolver::CandidateInput CandidateInput(const int semantic_id) {
  InactiveRecoverySolver::CandidateInput candidate;
  candidate.semantic_id = semantic_id;
  return candidate;
}

InactiveRecoverySolver::CandidateScore Score(const int track_row,
                                             const int candidate_col,
                                             const float app_cost,
                                             const float threshold = 0.85F,
                                             const bool passes_gate = true) {
  InactiveRecoverySolver::CandidateScore score;
  score.track_row = track_row;
  score.candidate_col = candidate_col;
  score.app_cost = app_cost;
  score.geo_cost = 0.20F + 0.01F * static_cast<float>(candidate_col);
  score.similarity = 1.0F - app_cost;
  score.recover_threshold = threshold;
  score.passes_missing_identity_gate = passes_gate;
  return score;
}

}  // namespace

TEST(InactiveRecoverySolverTest, ClassifiesAcceptedAndRejectedRecoveryCandidates) {
  const std::vector<InactiveRecoverySolver::TrackInput> tracks{TrackInput(0)};
  const std::vector<InactiveRecoverySolver::CandidateInput> candidates{
      CandidateInput(1),
      CandidateInput(2),
      CandidateInput(3),
      CandidateInput(4),
  };
  const std::vector<InactiveRecoverySolver::CandidateScore> scores{
      Score(0, 0, 0.10F, 0.85F, true),
      Score(0, 1, 0.20F, 0.75F, true),
      Score(0, 2, 0.60F, 0.85F, true),
      Score(0, 3, 0.10F, 0.85F, false),
  };

  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.15F;

  const auto result = InactiveRecoverySolver::SelectBestSimilarity(tracks, candidates, scores, config);

  ASSERT_EQ(result.candidates.size(), 4U);
  EXPECT_TRUE(result.candidates[0].accepted);
  EXPECT_EQ(result.candidates[0].reject_reason, "");
  EXPECT_FALSE(result.candidates[1].accepted);
  EXPECT_EQ(result.candidates[1].reject_reason, "recovery_max_cost_reject");
  EXPECT_FALSE(result.candidates[2].accepted);
  EXPECT_EQ(result.candidates[2].reject_reason, "below_recover_threshold");
  EXPECT_FALSE(result.candidates[3].accepted);
  EXPECT_EQ(result.candidates[3].reject_reason, "missing_identity_gate_reject");
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments[0].semantic_id, 1);
  EXPECT_FLOAT_EQ(result.assignments[0].similarity, 0.90F);
}

TEST(InactiveRecoverySolverTest, RelaxedRecoverThresholdAllowsLowerSimilarity) {
  const std::vector<InactiveRecoverySolver::TrackInput> tracks{TrackInput(0)};
  const std::vector<InactiveRecoverySolver::CandidateInput> candidates{CandidateInput(9)};
  const std::vector<InactiveRecoverySolver::CandidateScore> scores{Score(0, 0, 0.24F, 0.75F, true)};

  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.30F;

  const auto result = InactiveRecoverySolver::SelectBestSimilarity(tracks, candidates, scores, config);

  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_TRUE(result.candidates[0].accepted);
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments[0].semantic_id, 9);
}

TEST(InactiveRecoverySolverTest, HungarianSelectionAssignsRecoverableInactiveIdentitiesWithMargins) {
  const std::vector<InactiveRecoverySolver::TrackInput> tracks{TrackInput(0), TrackInput(1)};
  const std::vector<InactiveRecoverySolver::CandidateInput> candidates{CandidateInput(1), CandidateInput(2)};
  const std::vector<InactiveRecoverySolver::CandidateScore> scores{
      Score(0, 0, 0.12F, 0.65F),
      Score(0, 1, 0.30F, 0.65F),
      Score(1, 0, 0.20F, 0.65F),
      Score(1, 1, 0.08F, 0.65F),
  };

  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.45F;

  const auto result = InactiveRecoverySolver::SolveHungarian(tracks, candidates, scores, config);

  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.assignments[0].track_idx, 0);
  EXPECT_EQ(result.assignments[0].semantic_id, 1);
  EXPECT_FLOAT_EQ(result.assignments[0].recovery_cost, 0.12F);
  EXPECT_FLOAT_EQ(result.assignments[0].margin, 0.18F);
  EXPECT_EQ(result.assignments[1].track_idx, 1);
  EXPECT_EQ(result.assignments[1].semantic_id, 2);
  EXPECT_FLOAT_EQ(result.assignments[1].recovery_cost, 0.08F);
  EXPECT_FLOAT_EQ(result.assignments[1].margin, 0.12F);
}

TEST(InactiveRecoverySolverTest, HungarianSelectionSkipsRejectedRecoveryCandidates) {
  const std::vector<InactiveRecoverySolver::TrackInput> tracks{TrackInput(0), TrackInput(1)};
  const std::vector<InactiveRecoverySolver::CandidateInput> candidates{CandidateInput(1), CandidateInput(2)};
  const std::vector<InactiveRecoverySolver::CandidateScore> scores{
      Score(0, 0, 0.10F),
      Score(0, 1, 0.05F, 0.85F, false),
      Score(1, 0, 0.09F),
      Score(1, 1, 0.20F, 0.75F),
  };

  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.45F;

  const auto result = InactiveRecoverySolver::SolveHungarian(tracks, candidates, scores, config);

  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.assignments[0].semantic_id, 1);
  EXPECT_EQ(result.assignments[1].semantic_id, 2);
  const auto rejected = std::find_if(result.candidates.begin(), result.candidates.end(), [](const auto &candidate) {
    return candidate.semantic_id == 2 && candidate.track_idx == 0;
  });
  ASSERT_NE(rejected, result.candidates.end());
  EXPECT_FALSE(rejected->accepted);
  EXPECT_EQ(rejected->reject_reason, "missing_identity_gate_reject");
}
