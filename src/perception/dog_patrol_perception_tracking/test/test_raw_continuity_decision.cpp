#include <gtest/gtest.h>

#include "raw_continuity_decision.hpp"

namespace {

using dog_patrol_perception_tracking::RawContinuityDecision;

RawContinuityDecision::Input BaseInput() {
  RawContinuityDecision::Input input;
  input.track_idx = 3;
  input.raw_track_id = 7;
  input.semantic_id = 1;
  input.app_cost = 0.10F;
  input.geo_cost = 0.20F;
  input.time_cost = 0.30F;
  input.final_cost = 0.40F;
  return input;
}

}  // namespace

TEST(RawContinuityDecisionTest, AcceptsSaneRawContinuityAndReportsDebugPayload) {
  RawContinuityDecision::Config config;
  config.raw_continuity_max_cost = 0.55F;

  const auto decision = RawContinuityDecision::Evaluate(BaseInput(), config);

  EXPECT_EQ(decision.track_idx, 3);
  EXPECT_EQ(decision.raw_track_id, 7);
  EXPECT_EQ(decision.semantic_id, 1);
  EXPECT_FLOAT_EQ(decision.app_cost, 0.10F);
  EXPECT_FLOAT_EQ(decision.geo_cost, 0.20F);
  EXPECT_FLOAT_EQ(decision.time_cost, 0.30F);
  EXPECT_FLOAT_EQ(decision.final_score, 0.40F);
  EXPECT_FLOAT_EQ(decision.margin, 0.60F);
  EXPECT_TRUE(decision.selected);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reject_reason, "");
  EXPECT_TRUE(decision.continuity_used);
}

TEST(RawContinuityDecisionTest, RejectsWhenIdentityIsMissingWithoutAcceptingContinuity) {
  auto input = BaseInput();
  input.identity_found = false;
  input.final_cost = 1.0F;

  const auto decision = RawContinuityDecision::Evaluate(input, RawContinuityDecision::Config{});

  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.selected);
  EXPECT_EQ(decision.reject_reason, "identity_not_found");
  EXPECT_FLOAT_EQ(decision.margin, 0.0F);
}

TEST(RawContinuityDecisionTest, RejectsMissingIdentityGateBeforeCostThreshold) {
  auto input = BaseInput();
  input.passes_missing_identity_gate = false;
  input.final_cost = 0.90F;

  RawContinuityDecision::Config config;
  config.raw_continuity_max_cost = 0.10F;
  const auto decision = RawContinuityDecision::Evaluate(input, config);

  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.selected);
  EXPECT_EQ(decision.reject_reason, "missing_identity_gate_reject");
  EXPECT_FLOAT_EQ(decision.margin, 0.10F);
}

TEST(RawContinuityDecisionTest, RejectsOverRawContinuityMaxCostBeforeWeakAssociation) {
  auto input = BaseInput();
  input.final_cost = 0.70F;
  input.weak_mot_association = true;

  RawContinuityDecision::Config config;
  config.raw_continuity_max_cost = 0.55F;
  const auto decision = RawContinuityDecision::Evaluate(input, config);

  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.selected);
  EXPECT_EQ(decision.reject_reason, "raw_continuity_max_cost_reject");
  EXPECT_FLOAT_EQ(decision.margin, 0.30F);
}

TEST(RawContinuityDecisionTest, RejectsWeakMotAssociationAfterCostAndGatePass) {
  auto input = BaseInput();
  input.weak_mot_association = true;

  RawContinuityDecision::Config config;
  config.raw_continuity_max_cost = 0.55F;
  const auto decision = RawContinuityDecision::Evaluate(input, config);

  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.selected);
  EXPECT_EQ(decision.reject_reason, "weak_mot_association");
  EXPECT_FLOAT_EQ(decision.margin, 0.60F);
}
