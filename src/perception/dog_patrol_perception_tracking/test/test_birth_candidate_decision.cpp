#include <gtest/gtest.h>

#include "birth_candidate_decision.hpp"

namespace {

using dog_patrol_perception_tracking::BirthCandidateDecision;

BirthCandidateDecision::Input BaseInput() {
  BirthCandidateDecision::Input input;
  input.track_idx = 4;
  input.raw_track_id = 8;
  return input;
}

}  // namespace

TEST(BirthCandidateDecisionTest, HoldsAmbiguousRecoveryBeforeOtherBirthRules) {
  auto input = BaseInput();
  input.hold_for_ambiguous_recovery = true;
  input.duplicate_split = true;
  input.hide_reason = "skinny_partial_hidden";

  const auto decision = BirthCandidateDecision::Evaluate(input, BirthCandidateDecision::Config{});

  EXPECT_EQ(decision.action, BirthCandidateDecision::Action::kHideWithDebugRow);
  EXPECT_EQ(decision.track_idx, 4);
  EXPECT_EQ(decision.raw_track_id, 8);
  EXPECT_EQ(decision.semantic_id, -1);
  EXPECT_EQ(decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(decision.reject_reason, "ambiguous_recovery_pending");
  EXPECT_FALSE(decision.selected);
  EXPECT_FALSE(decision.accepted);
  EXPECT_TRUE(decision.clear_pending_candidate);
}

TEST(BirthCandidateDecisionTest, HidesDuplicateSplitBeforeShapeFragments) {
  auto input = BaseInput();
  input.duplicate_split = true;
  input.hide_reason = "wide_fragment_hidden";

  const auto decision = BirthCandidateDecision::Evaluate(input, BirthCandidateDecision::Config{});

  EXPECT_EQ(decision.action, BirthCandidateDecision::Action::kHideWithDebugRow);
  EXPECT_EQ(decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(decision.reject_reason, "duplicate_split_hidden");
  EXPECT_FALSE(decision.selected);
  EXPECT_FALSE(decision.accepted);
  EXPECT_TRUE(decision.clear_pending_candidate);
}

TEST(BirthCandidateDecisionTest, HidesSkinnyAndWideFragmentsWithoutSelecting) {
  auto skinny = BaseInput();
  skinny.hide_reason = "skinny_partial_hidden";
  const auto skinny_decision = BirthCandidateDecision::Evaluate(skinny, BirthCandidateDecision::Config{});
  EXPECT_EQ(skinny_decision.action, BirthCandidateDecision::Action::kHideWithDebugRow);
  EXPECT_EQ(skinny_decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(skinny_decision.reject_reason, "skinny_partial_hidden");
  EXPECT_FALSE(skinny_decision.selected);
  EXPECT_FALSE(skinny_decision.accepted);

  auto wide = BaseInput();
  wide.hide_reason = "wide_fragment_hidden";
  const auto wide_decision = BirthCandidateDecision::Evaluate(wide, BirthCandidateDecision::Config{});
  EXPECT_EQ(wide_decision.action, BirthCandidateDecision::Action::kHideWithDebugRow);
  EXPECT_EQ(wide_decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(wide_decision.reject_reason, "wide_fragment_hidden");
  EXPECT_FALSE(wide_decision.selected);
  EXPECT_FALSE(wide_decision.accepted);
}

TEST(BirthCandidateDecisionTest, LeavesAllocationPendingForManagerPath) {
  auto input = BaseInput();
  input.small_person_requires_stability = true;
  input.stable_observation_count = 2;

  const auto decision = BirthCandidateDecision::Evaluate(input, BirthCandidateDecision::Config{});

  EXPECT_EQ(decision.action, BirthCandidateDecision::Action::kPhase5Pending);
  EXPECT_EQ(decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(decision.reject_reason, "phase5_birth_manager_pending");
  EXPECT_TRUE(decision.selected);
  EXPECT_FALSE(decision.accepted);
  EXPECT_FLOAT_EQ(decision.final_score, 0.0F);
  EXPECT_FLOAT_EQ(decision.margin, 1.0F);
  EXPECT_FALSE(decision.clear_pending_candidate);
}

TEST(BirthCandidateDecisionTest, SmallPersonLeavesStabilityDecisionToPhase5Coordinator) {
  auto input = BaseInput();
  input.small_person_requires_stability = true;
  input.stable_observation_count = 1;

  const auto pending = BirthCandidateDecision::Evaluate(input, BirthCandidateDecision::Config{});

  EXPECT_EQ(pending.action, BirthCandidateDecision::Action::kPhase5Pending);
  EXPECT_EQ(pending.stage, "phase5_birth_candidate");
  EXPECT_EQ(pending.reject_reason, "phase5_birth_manager_pending");
  EXPECT_TRUE(pending.selected);
  EXPECT_FALSE(pending.accepted);
  EXPECT_FALSE(pending.clear_pending_candidate);

  input.stable_observation_count = 2;
  const auto promoted = BirthCandidateDecision::Evaluate(input, BirthCandidateDecision::Config{});

  EXPECT_EQ(promoted.action, BirthCandidateDecision::Action::kPhase5Pending);
  EXPECT_EQ(promoted.stage, "phase5_birth_candidate");
  EXPECT_EQ(promoted.reject_reason, "phase5_birth_manager_pending");
  EXPECT_TRUE(promoted.selected);
  EXPECT_FALSE(promoted.accepted);
  EXPECT_FLOAT_EQ(promoted.final_score, 0.0F);
  EXPECT_FLOAT_EQ(promoted.margin, 1.0F);
  EXPECT_FALSE(promoted.clear_pending_candidate);
}

TEST(BirthCandidateDecisionTest, KeepsFullNewSemanticAllocationPendingForBirthManager) {
  const auto decision = BirthCandidateDecision::Evaluate(BaseInput(), BirthCandidateDecision::Config{});

  EXPECT_EQ(decision.action, BirthCandidateDecision::Action::kPhase5Pending);
  EXPECT_EQ(decision.stage, "phase5_birth_candidate");
  EXPECT_EQ(decision.reject_reason, "phase5_birth_manager_pending");
  EXPECT_TRUE(decision.selected);
  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.clear_pending_candidate);
}
