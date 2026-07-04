#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "assignment_application_plan.hpp"
#include "semantic_id_allocator.hpp"

namespace {

using vision_demo_host::AssignmentApplicationPlan;
using vision_demo_host::SemanticIdAllocator;

AssignmentApplicationPlan::TrackApplicationCandidate Candidate(const int track_idx,
                                                              const int raw_track_id,
                                                              const int semantic_id) {
  AssignmentApplicationPlan::TrackApplicationCandidate candidate;
  candidate.track_idx = track_idx;
  candidate.raw_track_id = raw_track_id;
  candidate.semantic_id = semantic_id;
  return candidate;
}

AssignmentApplicationPlan::AcceptedDebugRow AcceptedRow(const int frame_idx,
                                                       const int track_idx,
                                                       const int semantic_id,
                                                       const std::string &stage,
                                                       const float final_score,
                                                       const float margin) {
  AssignmentApplicationPlan::AcceptedDebugRow row;
  row.frame_idx = frame_idx;
  row.track_idx = track_idx;
  row.semantic_id = semantic_id;
  row.stage = stage;
  row.final_score = final_score;
  row.margin = margin;
  row.accepted = true;
  return row;
}

}  // namespace

TEST(AssignmentApplicationPlanTest, UsesOrdinaryAcceptedRowCostMarginAndBuildsRawMap) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(1, 101, 5),
      Candidate(3, 103, 7),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(9, 1, 5, "assign_candidate", 0.27F, 0.13F),
      AcceptedRow(9, 3, 7, "raw_continuity", 0.18F, 0.22F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 9, {5, 7}, &allocator);

  ASSERT_EQ(plan.applications.size(), 2U);
  EXPECT_EQ(plan.applications[0].track_idx, 1);
  EXPECT_EQ(plan.applications[0].raw_track_id, 101);
  EXPECT_EQ(plan.applications[0].semantic_id, 5);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_cost, 0.27F);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_margin, 0.13F);
  EXPECT_EQ(plan.applications[0].accepted_stage, "assign_candidate");
  EXPECT_TRUE(plan.applications[0].found_accepted_row);

  EXPECT_EQ(plan.applications[1].track_idx, 3);
  EXPECT_EQ(plan.applications[1].raw_track_id, 103);
  EXPECT_EQ(plan.applications[1].semantic_id, 7);
  EXPECT_FLOAT_EQ(plan.applications[1].assignment_cost, 0.18F);
  EXPECT_FLOAT_EQ(plan.applications[1].assignment_margin, 0.22F);

  const std::unordered_map<int, int> expected_raw_map{{101, 5}, {103, 7}};
  EXPECT_EQ(plan.next_raw_to_semantic, expected_raw_map);
}

TEST(AssignmentApplicationPlanTest, InvertsInactiveRecoverySimilarityIntoCost) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(0, 10, 4),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(12, 0, 4, "inactive_recover_candidate", 0.82F, 0.31F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 12, {4}, &allocator);

  ASSERT_EQ(plan.applications.size(), 1U);
  EXPECT_NEAR(plan.applications[0].assignment_cost, 0.18F, 1e-6F);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_margin, 0.31F);
  EXPECT_EQ(plan.applications[0].accepted_stage, "inactive_recover_candidate");
}

TEST(AssignmentApplicationPlanTest, NewSemanticRowsUseZeroCostAndOneMargin) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(2, 20, 6),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(13, 2, 6, "new_semantic", 0.44F, 0.02F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 13, {6}, &allocator);

  ASSERT_EQ(plan.applications.size(), 1U);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_cost, 0.0F);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_margin, 1.0F);
  EXPECT_EQ(plan.applications[0].accepted_stage, "new_semantic");
}

TEST(AssignmentApplicationPlanTest, Phase5NewSemanticRowsUseFinalScoreAndRowMargin) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(4, 40, 8),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(14, 4, 8, "phase5_new_semantic", 0.0F, 1.0F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 14, {8}, &allocator);

  ASSERT_EQ(plan.applications.size(), 1U);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_cost, 0.0F);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_margin, 1.0F);
  EXPECT_EQ(plan.applications[0].accepted_stage, "phase5_new_semantic");
}

TEST(AssignmentApplicationPlanTest, FallsBackToZeroCostOneMarginWhenNoAcceptedRowMatches) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(5, 50, 9),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(14, 5, 9, "assign_candidate", 0.7F, 0.1F),
      AcceptedRow(15, 5, 10, "assign_candidate", 0.8F, 0.2F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 15, {9}, &allocator);

  ASSERT_EQ(plan.applications.size(), 1U);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_cost, 0.0F);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_margin, 1.0F);
  EXPECT_EQ(plan.applications[0].accepted_stage, "");
  EXPECT_FALSE(plan.applications[0].found_accepted_row);
}

TEST(AssignmentApplicationPlanTest, DuplicateSemanticCollisionKeepsFirstOwnerAndAllocatesLaterOwner) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(1, 101, 5),
      Candidate(2, 102, 5),
      Candidate(3, 103, 5),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(16, 1, 5, "assign_candidate", 0.1F, 0.8F),
      AcceptedRow(16, 2, 5, "assign_candidate", 0.2F, 0.7F),
      AcceptedRow(16, 3, 5, "assign_candidate", 0.3F, 0.6F),
  };

  const auto plan = AssignmentApplicationPlan::Build(candidates, debug_rows, 16, {1, 2, 3, 5}, &allocator);

  ASSERT_EQ(plan.applications.size(), 3U);
  EXPECT_EQ(plan.applications[0].semantic_id, 5);
  EXPECT_FLOAT_EQ(plan.applications[0].assignment_cost, 0.1F);
  EXPECT_EQ(plan.applications[1].semantic_id, 4);
  EXPECT_FLOAT_EQ(plan.applications[1].assignment_cost, 0.0F);
  EXPECT_EQ(plan.applications[2].semantic_id, 6);
  EXPECT_FLOAT_EQ(plan.applications[2].assignment_cost, 0.0F);

  const std::unordered_map<int, int> expected_raw_map{{101, 5}, {102, 4}, {103, 6}};
  EXPECT_EQ(plan.next_raw_to_semantic, expected_raw_map);
}

TEST(AssignmentApplicationPlanTest, RawMapEntriesFollowRequestedTrackOrderWithoutChangingCollisionOwner) {
  SemanticIdAllocator allocator;
  const std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> candidates{
      Candidate(1, 101, 5),
      Candidate(2, 102, 5),
      Candidate(3, 103, 7),
  };
  const std::vector<AssignmentApplicationPlan::AcceptedDebugRow> debug_rows{
      AcceptedRow(17, 1, 5, "assign_candidate", 0.1F, 0.8F),
      AcceptedRow(17, 2, 5, "assign_candidate", 0.2F, 0.7F),
      AcceptedRow(17, 3, 7, "assign_candidate", 0.3F, 0.6F),
  };

  const auto plan =
      AssignmentApplicationPlan::Build(candidates, debug_rows, 17, {1, 2, 3, 5, 7}, &allocator, {3, 2, 1});

  ASSERT_EQ(plan.applications.size(), 3U);
  EXPECT_EQ(plan.applications[0].track_idx, 1);
  EXPECT_EQ(plan.applications[0].semantic_id, 5);
  EXPECT_EQ(plan.applications[1].track_idx, 2);
  EXPECT_EQ(plan.applications[1].semantic_id, 4);

  ASSERT_EQ(plan.next_raw_to_semantic_entries.size(), 3U);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[0].raw_track_id, 103);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[0].semantic_id, 7);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[1].raw_track_id, 102);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[1].semantic_id, 4);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[2].raw_track_id, 101);
  EXPECT_EQ(plan.next_raw_to_semantic_entries[2].semantic_id, 5);
}
