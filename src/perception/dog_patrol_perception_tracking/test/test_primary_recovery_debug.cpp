#include <gtest/gtest.h>

#include <string>

#include "vision_demo_host/modules/primary_recovery_debug.hpp"

namespace {

vision_demo_host::PrimaryTargetResult PendingPrimary(const int semantic_id = 7) {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kPendingRecovery;
  primary.primary_target_id = semantic_id;
  primary.raw_track_id = -1;
  primary.missing_frames = 2;
  return primary;
}

vision_demo_host::IdentityObservation Identity(const int semantic_id,
                                               const vision_demo_host::IdentityState state,
                                               const int raw_id = -1) {
  vision_demo_host::IdentityObservation identity;
  identity.semantic_id = semantic_id;
  identity.state = state;
  identity.class_id = vision_demo_host::ClassId::kPerson;
  identity.visible = raw_id > 0;
  if (raw_id > 0) {
    identity.supporting_raw_track_id = raw_id;
  }
  return identity;
}

}  // namespace

TEST(PrimaryRecoveryDebugTest, MapsSanityRejectReasonsToCompactTokens) {
  const auto primary = PendingPrimary();
  const vision_demo_host::IdentityManagerResult identities;

  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_center_jump"),
            "center_jump");
  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_low_score_update"),
            "low_score");
  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(
                primary, identities, "pending_recovery_visible_primary_sanity_rejected",
                "visible_primary_assoc_gate_failed"),
            "assoc_gate");
}

TEST(PrimaryRecoveryDebugTest, MapsIdentityLifecycleToCompactTokens) {
  auto primary = PendingPrimary(7);
  vision_demo_host::IdentityManagerResult merged_result;
  merged_result.identities.push_back(Identity(7, vision_demo_host::IdentityState::kMerged, 42));
  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(
                primary, merged_result, "pending_recovery_from_identity_state", ""),
            "merged");

  vision_demo_host::IdentityManagerResult split_result;
  split_result.identities.push_back(Identity(7, vision_demo_host::IdentityState::kSplitRecovery, 43));
  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(
                primary, split_result, "pending_recovery_from_identity_state", ""),
            "split_recovery");
}

TEST(PrimaryRecoveryDebugTest, FallsBackToPendingOnlyForPendingRecovery) {
  const vision_demo_host::IdentityManagerResult identities;
  EXPECT_EQ(vision_demo_host::PrimaryRecoveryReasonToken(PendingPrimary(), identities,
                                                        "pending_recovery_unknown", ""),
            "pending");

  vision_demo_host::PrimaryTargetResult locked;
  locked.state = vision_demo_host::PrimaryState::kLocked;
  locked.primary_target_id = 7;
  EXPECT_TRUE(vision_demo_host::PrimaryRecoveryReasonToken(locked, identities, "locked_visible_primary_identity", "")
                  .empty());
}

TEST(PrimaryRecoveryDebugTest, BuildsCompactOverlayLineWithReasonAndFreezeMarker) {
  auto primary = PendingPrimary(7);
  vision_demo_host::IdentityManagerResult identities;
  identities.identities.push_back(Identity(7, vision_demo_host::IdentityState::kMerged, 42));
  identities.feature_update_frozen = true;

  const std::string line = vision_demo_host::BuildPrimaryOverlayLine(
      primary, identities, "pending_recovery_from_identity_state", "");

  EXPECT_EQ(line, "PENDING_RECOVERY id=7 raw=42 reason=merged freeze");
}

TEST(PrimaryRecoveryDebugTest, TrackLabelPointStaysTopAnchoredWhenBoxHeightChanges) {
  const cv::Size frame_size(1280, 720);
  const cv::Rect2f short_box(120.0F, 80.0F, 64.0F, 90.0F);
  const cv::Rect2f tall_box(120.0F, 80.0F, 64.0F, 320.0F);

  const cv::Point short_label = vision_demo_host::CompactOverlayTrackLabelPoint(frame_size, short_box);
  const cv::Point tall_label = vision_demo_host::CompactOverlayTrackLabelPoint(frame_size, tall_box);

  EXPECT_EQ(short_label.y, tall_label.y);
  EXPECT_EQ(short_label.y, 96);
}

TEST(PrimaryRecoveryDebugTest, TrackLabelPointTopBoundaryClampIsVisibleAndDeterministic) {
  const cv::Size frame_size(1280, 720);
  const cv::Rect2f top_clipped_box(120.0F, -40.0F, 64.0F, 180.0F);
  const cv::Rect2f top_clipped_tall_box(120.0F, -40.0F, 64.0F, 360.0F);

  const cv::Point label = vision_demo_host::CompactOverlayTrackLabelPoint(frame_size, top_clipped_box);
  const cv::Point tall_label =
      vision_demo_host::CompactOverlayTrackLabelPoint(frame_size, top_clipped_tall_box);

  EXPECT_EQ(label.y, 14);
  EXPECT_EQ(tall_label.y, label.y);
}
