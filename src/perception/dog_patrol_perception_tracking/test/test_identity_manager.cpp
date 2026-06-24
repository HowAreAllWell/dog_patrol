#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "vision_demo_host/modules/identity_manager.hpp"

namespace {

vision_demo_host::Track MakePersonTrack(const int raw_id, const cv::Rect2f &bbox,
                                        const std::vector<float> &feature = {1.0F, 0.0F, 0.0F}) {
  vision_demo_host::Track track;
  track.id = raw_id;
  track.class_id = vision_demo_host::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = bbox;
  track.is_confirmed = true;
  track.appearance_feature = feature;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

vision_demo_host::PrimaryTargetResult IdlePrimary() {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kIdle;
  return primary;
}

const vision_demo_host::IdentityObservation *FindIdentity(
    const std::vector<vision_demo_host::IdentityObservation> &identities, const int semantic_id) {
  const auto it = std::find_if(identities.begin(), identities.end(), [&](const auto &identity) {
    return identity.semantic_id == semantic_id;
  });
  return it == identities.end() ? nullptr : &(*it);
}

}  // namespace

TEST(IdentityManagerTest, ProducesVisibleIdentityObservations) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;

  vision_demo_host::IdentityManager manager(cfg);

  const std::vector<vision_demo_host::Track> tracks{
      MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50)),
      MakePersonTrack(20, cv::Rect2f(200, 0, 50, 50), {0.0F, 1.0F, 0.0F}),
  };
  const auto primary = IdlePrimary();

  const auto result = manager.Update(vision_demo_host::TrackletObservationsFromTracks(tracks), primary);

  ASSERT_EQ(result.identities.size(), tracks.size());
  EXPECT_EQ(result.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(result.SemanticIdForRawTrack(20), 2);

  const auto *identity1 = FindIdentity(result.identities, 1);
  ASSERT_NE(identity1, nullptr);
  ASSERT_TRUE(identity1->supporting_raw_track_id.has_value());
  EXPECT_EQ(*identity1->supporting_raw_track_id, 10);
  ASSERT_TRUE(identity1->supporting_tracklet.has_value());
  EXPECT_EQ(identity1->supporting_tracklet->raw_track_id, 10);
  EXPECT_TRUE(identity1->visible);
  EXPECT_EQ(identity1->missing_frames, 0);
  EXPECT_EQ(identity1->state, vision_demo_host::IdentityState::kActive);

  const auto *identity2 = FindIdentity(result.identities, 2);
  ASSERT_NE(identity2, nullptr);
  ASSERT_TRUE(identity2->supporting_raw_track_id.has_value());
  EXPECT_EQ(*identity2->supporting_raw_track_id, 20);
  EXPECT_TRUE(identity2->visible);
  EXPECT_EQ(identity2->state, vision_demo_host::IdentityState::kActive);
}

TEST(IdentityManagerTest, CarriesAssignmentEvidenceAndPrimaryFlag) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.raw_continuity_max_cost = 0.10F;
  cfg.active_assign_max_cost = 0.90F;
  vision_demo_host::IdentityManager manager(cfg);

  auto first = MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50));
  vision_demo_host::PrimaryTargetResult primary = IdlePrimary();
  primary.primary_target_id = 1;

  auto first_result = manager.Update(vision_demo_host::TrackletObservationsFromTracks({first}), primary);
  ASSERT_EQ(first_result.identities.size(), 1U);
  EXPECT_EQ(first_result.primary_semantic_id, 1);
  EXPECT_TRUE(first_result.identities.front().primary);

  auto jumped = MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50));
  const auto second_result = manager.Update(vision_demo_host::TrackletObservationsFromTracks({jumped}), primary);

  ASSERT_EQ(second_result.identities.size(), 1U);
  const auto &identity = second_result.identities.front();
  EXPECT_EQ(identity.semantic_id, 1);
  EXPECT_EQ(identity.assignment.stage, "assign_candidate");
  EXPECT_TRUE(identity.assignment.accepted);

  const auto &debug_rows = manager.LastScoreDebugRows();
  const auto raw_reject = std::find_if(debug_rows.begin(), debug_rows.end(), [](const auto &row) {
    return row.stage == "raw_continuity" && row.reject_reason == "raw_continuity_max_cost_reject";
  });
  ASSERT_NE(raw_reject, debug_rows.end());
  EXPECT_TRUE(raw_reject->continuity_used);
  EXPECT_FALSE(raw_reject->accepted);
}

TEST(IdentityManagerTest, DefaultMissingWindowKeepsIdentityOccludedAcrossFourSecondAbsence) {
  vision_demo_host::IdentityManager manager;

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}),
      IdlePrimary());
  ASSERT_EQ(first.identities.size(), 1U);
  EXPECT_EQ(first.identities.front().state, vision_demo_host::IdentityState::kActive);

  vision_demo_host::IdentityManagerResult missing_result;
  for (int i = 0; i < 120; ++i) {
    missing_result = manager.Update({}, IdlePrimary());
  }

  ASSERT_EQ(missing_result.identities.size(), 1U);
  EXPECT_EQ(missing_result.identities.front().semantic_id, 1);
  EXPECT_EQ(missing_result.identities.front().state, vision_demo_host::IdentityState::kOccluded);
  EXPECT_EQ(missing_result.identities.front().missing_frames, 120);
}

TEST(IdentityManagerTest, EmitsOccludedAndLostIdentityLifecycleWithoutVisibleTracklet) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 1;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityManager manager(cfg);

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}),
      IdlePrimary());
  ASSERT_EQ(first.identities.size(), 1U);
  EXPECT_EQ(first.identities.front().state, vision_demo_host::IdentityState::kActive);

  const auto second = manager.Update({}, IdlePrimary());
  ASSERT_EQ(second.identities.size(), 1U);
  EXPECT_EQ(second.identities.front().semantic_id, 1);
  EXPECT_EQ(second.identities.front().state, vision_demo_host::IdentityState::kOccluded);
  EXPECT_FALSE(second.identities.front().visible);
  EXPECT_FALSE(second.identities.front().supporting_raw_track_id.has_value());
  EXPECT_EQ(second.identities.front().missing_frames, 1);

  const auto third = manager.Update({}, IdlePrimary());
  ASSERT_EQ(third.identities.size(), 1U);
  EXPECT_EQ(third.identities.front().semantic_id, 1);
  EXPECT_EQ(third.identities.front().state, vision_demo_host::IdentityState::kLost);
  EXPECT_EQ(third.identities.front().missing_frames, 2);
}
