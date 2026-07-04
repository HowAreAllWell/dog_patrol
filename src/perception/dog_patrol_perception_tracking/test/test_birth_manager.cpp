#include <gtest/gtest.h>

#include <vector>

#include "birth_manager.hpp"

namespace {

using vision_demo_host::BirthManager;

BirthManager::Input BaseInput() {
  BirthManager::Input input;
  input.frame_index = 12;
  input.track_idx = 4;
  input.raw_track_id = 8;
  return input;
}

}  // namespace

TEST(BirthManagerTest, AmbiguousRecoveryPendingCreatesHiddenDebugRowWithoutAllocating) {
  BirthManager manager;
  auto input = BaseInput();
  input.hold_for_ambiguous_recovery = true;

  int allocation_calls = 0;
  const auto result = manager.Evaluate(input, [&]() {
    ++allocation_calls;
    return 21;
  });

  EXPECT_EQ(result.decision.action, vision_demo_host::BirthCandidateDecision::Action::kHideWithDebugRow);
  EXPECT_TRUE(result.has_debug_row);
  EXPECT_EQ(result.debug_row.stage, "birth_candidate");
  EXPECT_EQ(result.debug_row.reject_reason, "ambiguous_recovery_pending");
  EXPECT_EQ(result.debug_row.semantic_id, -1);
  EXPECT_FALSE(result.allocated_semantic_id);
  EXPECT_EQ(allocation_calls, 0);
}

TEST(BirthManagerTest, DuplicateSplitAndMorphologyHiddenRowsDoNotAllocateSemanticIds) {
  BirthManager manager;

  auto duplicate = BaseInput();
  duplicate.duplicate_split = true;

  int allocation_calls = 0;
  const auto duplicate_result = manager.Evaluate(duplicate, [&]() {
    ++allocation_calls;
    return 31;
  });
  EXPECT_TRUE(duplicate_result.has_debug_row);
  EXPECT_EQ(duplicate_result.debug_row.reject_reason, "duplicate_split_hidden");
  EXPECT_FALSE(duplicate_result.allocated_semantic_id);

  auto fragment = BaseInput();
  fragment.raw_track_id = 9;
  fragment.hide_reason = "wide_fragment_hidden";
  const auto fragment_result = manager.Evaluate(fragment, [&]() {
    ++allocation_calls;
    return 32;
  });
  EXPECT_TRUE(fragment_result.has_debug_row);
  EXPECT_EQ(fragment_result.debug_row.reject_reason, "wide_fragment_hidden");
  EXPECT_FALSE(fragment_result.allocated_semantic_id);
  EXPECT_EQ(allocation_calls, 0);
}

TEST(BirthManagerTest, LegacySmallPersonWaitsWithoutDebugRowThenPromotesAfterStableFrames) {
  BirthManager manager;

  auto input = BaseInput();
  input.small_person_requires_stability = true;

  int allocation_calls = 0;
  const auto first = manager.Evaluate(input, [&]() {
    ++allocation_calls;
    return 41;
  });
  EXPECT_EQ(first.decision.action,
            vision_demo_host::BirthCandidateDecision::Action::kLegacyPendingWithoutDebugRow);
  EXPECT_EQ(first.stable_observation_count, 1);
  EXPECT_FALSE(first.has_debug_row);
  EXPECT_FALSE(first.allocated_semantic_id);

  input.frame_index += 1;
  const auto second = manager.Evaluate(input, [&]() {
    ++allocation_calls;
    return 42;
  });
  EXPECT_EQ(second.decision.action,
            vision_demo_host::BirthCandidateDecision::Action::kAllocateNewSemantic);
  EXPECT_EQ(second.stable_observation_count, 2);
  EXPECT_TRUE(second.has_debug_row);
  EXPECT_TRUE(second.allocated_semantic_id);
  EXPECT_EQ(second.semantic_id, 42);
  EXPECT_EQ(second.debug_row.stage, "new_semantic");
  EXPECT_TRUE(second.debug_row.accepted);
  EXPECT_EQ(allocation_calls, 1);
}

TEST(BirthManagerTest, Phase5PendingCreatesDebugRowWithoutAllocating) {
  BirthManager manager;
  auto input = BaseInput();
  input.phase5_birth_manager_enabled = true;

  int allocation_calls = 0;
  const auto result = manager.Evaluate(input, [&]() {
    ++allocation_calls;
    return 51;
  });

  EXPECT_EQ(result.decision.action, vision_demo_host::BirthCandidateDecision::Action::kPhase5Pending);
  EXPECT_TRUE(result.has_debug_row);
  EXPECT_EQ(result.debug_row.stage, "phase5_birth_candidate");
  EXPECT_EQ(result.debug_row.reject_reason, "phase5_birth_manager_pending");
  EXPECT_FALSE(result.debug_row.accepted);
  EXPECT_FALSE(result.allocated_semantic_id);
  EXPECT_EQ(result.stable_observation_count, 0);
  EXPECT_EQ(allocation_calls, 0);
}

TEST(BirthManagerTest, Phase5AcceptedAllocationPromotesPendingRowAndAllocatesExactlyOnce) {
  BirthManager manager;
  BirthManager::DebugRow pending_row;
  pending_row.track_idx = 4;
  pending_row.raw_track_id = 8;
  pending_row.semantic_id = -1;
  pending_row.final_score = 0.0F;
  pending_row.selected = true;
  pending_row.stage = "phase5_birth_candidate";
  pending_row.margin = 1.0F;
  pending_row.accepted = false;
  pending_row.reject_reason = "phase5_birth_manager_pending";
  std::vector<BirthManager::DebugRow> rows{pending_row};

  int allocation_calls = 0;
  BirthManager::DebugRow accepted_row;
  int semantic_id = -1;
  const bool applied = manager.ApplyPhase5AcceptedAllocation(
      8, &rows,
      [&]() {
        ++allocation_calls;
        return 61;
      },
      &accepted_row, &semantic_id);

  ASSERT_TRUE(applied);
  EXPECT_EQ(allocation_calls, 1);
  EXPECT_EQ(semantic_id, 61);
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].semantic_id, 61);
  EXPECT_EQ(rows[0].stage, "phase5_new_semantic");
  EXPECT_TRUE(rows[0].selected);
  EXPECT_TRUE(rows[0].accepted);
  EXPECT_EQ(rows[0].reject_reason, "");
  EXPECT_FLOAT_EQ(rows[0].final_score, 0.0F);
  EXPECT_FLOAT_EQ(rows[0].margin, 1.0F);
  EXPECT_EQ(accepted_row.semantic_id, 61);
  EXPECT_EQ(accepted_row.stage, "phase5_new_semantic");
}

TEST(BirthManagerTest, HiddenAndPendingPathsNeverConsumeSemanticIds) {
  BirthManager manager;

  int allocation_calls = 0;
  auto hidden = BaseInput();
  hidden.hide_reason = "skinny_partial_hidden";
  manager.Evaluate(hidden, [&]() {
    ++allocation_calls;
    return 71;
  });

  auto pending = BaseInput();
  pending.raw_track_id = 9;
  pending.phase5_birth_manager_enabled = true;
  manager.Evaluate(pending, [&]() {
    ++allocation_calls;
    return 72;
  });

  auto legacy_pending = BaseInput();
  legacy_pending.raw_track_id = 10;
  legacy_pending.small_person_requires_stability = true;
  manager.Evaluate(legacy_pending, [&]() {
    ++allocation_calls;
    return 73;
  });

  EXPECT_EQ(allocation_calls, 0);
}
