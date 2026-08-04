#include <gtest/gtest.h>

#include "dog_patrol_perception_tracking/modules/feature_update_policy.hpp"

namespace dog_patrol_perception_tracking {
namespace {

TEST(FeatureUpdatePolicyTest, AllowsReliableAcceptedUpdatesForNewIdentity) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.reliable_observation = true;
  input.has_existing_feature_bank = false;
  input.stable_update_frames = 0;
  input.stable_frames_before_feature_update = 3;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_TRUE(decision.feature_update_allowed);
  EXPECT_TRUE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "allowed_update");
  EXPECT_EQ(decision.geometry_update_reason, "allowed_update");
}

TEST(FeatureUpdatePolicyTest, ReportsRejectedAssignmentsWithoutUpdates) {
  FeatureUpdatePolicy::Input input;
  input.accepted = false;
  input.reliable_observation = true;
  input.force_geometry_update = true;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_FALSE(decision.feature_update_allowed);
  EXPECT_FALSE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "update_blocked_by_rejected_assignment");
  EXPECT_EQ(decision.geometry_update_reason, "update_blocked_by_rejected_assignment");
}

TEST(FeatureUpdatePolicyTest, PreservesOverlapReasonPrecedenceOverGlobalFreeze) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.global_freeze = true;
  input.overlap_freeze = true;
  input.reliable_observation = false;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_FALSE(decision.feature_update_allowed);
  EXPECT_FALSE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "overlapping_track_freeze");
  EXPECT_EQ(decision.geometry_update_reason, "overlapping_track_freeze");
}

TEST(FeatureUpdatePolicyTest, ReportsGlobalFreezeWhenNoOverlapFreeze) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.global_freeze = true;
  input.overlap_freeze = false;
  input.reliable_observation = false;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_FALSE(decision.feature_update_allowed);
  EXPECT_FALSE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "global_merge_split_freeze");
  EXPECT_EQ(decision.geometry_update_reason, "global_merge_split_freeze");
}

TEST(FeatureUpdatePolicyTest, BlocksUnreliableLowQualityObservations) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.reliable_observation = false;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_FALSE(decision.feature_update_allowed);
  EXPECT_FALSE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "unreliable_low_quality_observation");
  EXPECT_EQ(decision.geometry_update_reason, "unreliable_low_quality_observation");
}

TEST(FeatureUpdatePolicyTest, AllowsGeometryWhileFeatureWaitsForStableFrames) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.reliable_observation = true;
  input.has_existing_feature_bank = true;
  input.stable_update_frames = 1;
  input.stable_frames_before_feature_update = 3;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_TRUE(decision.feature_update_allowed);
  EXPECT_TRUE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "insufficient_stable_frames");
  EXPECT_EQ(decision.geometry_update_reason, "allowed_update");
}

TEST(FeatureUpdatePolicyTest, ForceGeometryStillHonorsFeatureFreeze) {
  FeatureUpdatePolicy::Input input;
  input.accepted = true;
  input.overlap_freeze = true;
  input.reliable_observation = false;
  input.force_geometry_update = true;

  const auto decision = FeatureUpdatePolicy::Decide(input);

  EXPECT_FALSE(decision.feature_update_allowed);
  EXPECT_TRUE(decision.geometry_update_allowed);
  EXPECT_EQ(decision.feature_update_reason, "overlapping_track_freeze");
  EXPECT_EQ(decision.geometry_update_reason, "allowed_update");
}

}  // namespace
}  // namespace dog_patrol_perception_tracking
