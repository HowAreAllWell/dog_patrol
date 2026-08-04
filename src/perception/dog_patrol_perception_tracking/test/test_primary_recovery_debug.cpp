#include <gtest/gtest.h>

#include <string>

#include "dog_patrol_perception_tracking/modules/primary_recovery_debug.hpp"

namespace {

dog_patrol_perception_tracking::PrimaryTargetResult PendingPrimary(const int semantic_id = 7) {
  dog_patrol_perception_tracking::PrimaryTargetResult primary;
  primary.state = dog_patrol_perception_tracking::PrimaryState::kPendingRecovery;
  primary.primary_target_id = semantic_id;
  primary.raw_track_id = -1;
  primary.missing_frames = 2;
  return primary;
}

dog_patrol_perception_tracking::IdentityObservation Identity(const int semantic_id,
                                               const dog_patrol_perception_tracking::IdentityState state,
                                               const int raw_id = -1) {
  dog_patrol_perception_tracking::IdentityObservation identity;
  identity.semantic_id = semantic_id;
  identity.state = state;
  identity.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  identity.visible = raw_id > 0;
  if (raw_id > 0) {
    identity.supporting_raw_track_id = raw_id;
  }
  return identity;
}

}  // namespace

TEST(PrimaryRecoveryDebugTest, MapsSanityRejectReasonsToCompactTokens) {
  const auto primary = PendingPrimary();
  const dog_patrol_perception_tracking::IdentityManagerResult identities;

  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_center_jump"),
            "center_jump");
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_low_score_update"),
            "low_score");
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_assoc_gate_failed"),
            "assoc_gate");
}

TEST(PrimaryRecoveryDebugTest, MapsIdentityLifecycleToCompactTokens) {
  auto primary = PendingPrimary(7);
  dog_patrol_perception_tracking::IdentityManagerResult merged_result;
  merged_result.identities.push_back(Identity(7, dog_patrol_perception_tracking::IdentityState::kMerged, 42));
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(
                primary, merged_result, "pending_recovery_from_identity_state", ""),
            "merged");

  dog_patrol_perception_tracking::IdentityManagerResult split_result;
  split_result.identities.push_back(Identity(7, dog_patrol_perception_tracking::IdentityState::kSplitRecovery, 43));
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(
                primary, split_result, "pending_recovery_from_identity_state", ""),
            "split_recovery");
}

TEST(PrimaryRecoveryDebugTest, FallsBackToPendingOnlyForPendingRecovery) {
  const dog_patrol_perception_tracking::IdentityManagerResult identities;
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(PendingPrimary(), identities,
                                                        "pending_recovery_unknown", ""),
            "pending");

  dog_patrol_perception_tracking::PrimaryTargetResult locked;
  locked.state = dog_patrol_perception_tracking::PrimaryState::kLocked;
  locked.primary_target_id = 7;
  EXPECT_TRUE(dog_patrol_perception_tracking::PrimaryRecoveryReasonToken(locked, identities, "locked_visible_primary_identity", "")
                  .empty());
}

TEST(PrimaryRecoveryDebugTest, BuildsCompactOverlayLineWithReasonAndFreezeMarker) {
  auto primary = PendingPrimary(7);
  dog_patrol_perception_tracking::IdentityManagerResult identities;
  identities.identities.push_back(Identity(7, dog_patrol_perception_tracking::IdentityState::kMerged, 42));
  identities.feature_update_frozen = true;

  const std::string line = dog_patrol_perception_tracking::BuildPrimaryOverlayLine(
      primary, identities, "pending_recovery_from_identity_state", "");

  EXPECT_EQ(line, "PENDING_RECOVERY id=7 raw=42 reason=merged freeze");
}

TEST(PrimaryRecoveryDebugTest, TrackLabelPointStaysTopAnchoredWhenBoxHeightChanges) {
  const cv::Size frame_size(1280, 720);
  const cv::Rect2f short_box(120.0F, 80.0F, 64.0F, 90.0F);
  const cv::Rect2f tall_box(120.0F, 80.0F, 64.0F, 320.0F);

  const cv::Point short_label = dog_patrol_perception_tracking::CompactOverlayTrackLabelPoint(frame_size, short_box);
  const cv::Point tall_label = dog_patrol_perception_tracking::CompactOverlayTrackLabelPoint(frame_size, tall_box);

  EXPECT_EQ(short_label.y, tall_label.y);
  EXPECT_EQ(short_label.y, 96);
}

TEST(PrimaryRecoveryDebugTest, TrackLabelPointTopBoundaryClampIsVisibleAndDeterministic) {
  const cv::Size frame_size(1280, 720);
  const cv::Rect2f top_clipped_box(120.0F, -40.0F, 64.0F, 180.0F);
  const cv::Rect2f top_clipped_tall_box(120.0F, -40.0F, 64.0F, 360.0F);

  const cv::Point label = dog_patrol_perception_tracking::CompactOverlayTrackLabelPoint(frame_size, top_clipped_box);
  const cv::Point tall_label =
      dog_patrol_perception_tracking::CompactOverlayTrackLabelPoint(frame_size, top_clipped_tall_box);

  EXPECT_EQ(label.y, 14);
  EXPECT_EQ(tall_label.y, label.y);
}
