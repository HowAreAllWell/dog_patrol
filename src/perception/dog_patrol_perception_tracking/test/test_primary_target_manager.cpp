#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"

namespace {

dog_patrol_perception_tracking::IdentityObservation MakeIdentity(const int sid, const int raw_id,
                                                   const dog_patrol_perception_tracking::ClassId class_id,
                                                   const cv::Rect2f &bbox, const float conf = 0.9F) {
  dog_patrol_perception_tracking::IdentityObservation identity;
  identity.semantic_id = sid;
  identity.state = dog_patrol_perception_tracking::IdentityState::kActive;
  identity.supporting_raw_track_id = raw_id;
  dog_patrol_perception_tracking::Track track;
  track.id = raw_id;
  track.class_id = class_id;
  track.confidence = conf;
  track.bbox = bbox;
  track.is_confirmed = true;
  identity.supporting_tracklet = dog_patrol_perception_tracking::TrackletObservation::FromTrack(track);
  identity.class_id = class_id;
  identity.confidence = conf;
  identity.bbox = bbox;
  identity.visible = true;
  identity.association.stage = "stage1_confirmed_high";
  identity.association.passed_final_cost_gate = true;
  return identity;
}

dog_patrol_perception_tracking::IdentityObservation MakeLifecycleIdentity(const int sid,
                                                            const dog_patrol_perception_tracking::IdentityState state,
                                                            const int missing_frames,
                                                            const cv::Rect2f &bbox) {
  dog_patrol_perception_tracking::IdentityObservation identity;
  identity.semantic_id = sid;
  identity.state = state;
  identity.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  identity.bbox = bbox;
  identity.missing_frames = missing_frames;
  identity.visible = false;
  return identity;
}

}  // namespace

TEST(PrimaryTargetManagerTest, FirstLockChoosesLargestPersonIdentityOnly) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  const auto state = mgr.Update({
      MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kCar, cv::Rect2f(0, 0, 120, 120)),
      MakeIdentity(21, 201, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 30, 30)),
      MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50)),
  });
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(state.primary_target_id, 22);
  EXPECT_EQ(state.primary_track->id, 202);
  EXPECT_EQ(state.primary_track->class_id, dog_patrol_perception_tracking::ClassId::kPerson);
}

TEST(PrimaryTargetManagerTest, PatrolCycleDoesNotCreateLocalExclusions) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint now{};

  const auto handled = MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto eligible = MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));

  const auto first = mgr.UpdateForPatrol({handled}, now);
  ASSERT_TRUE(first.primary_track.has_value());
  EXPECT_EQ(first.primary_target_id, 11);

  mgr.ResetForPatrolCycle();
  const auto state = mgr.UpdateForPatrol({handled, eligible}, now);

  EXPECT_TRUE(mgr.IsMissionEligible(handled));
  EXPECT_EQ(state.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(state.primary_target_id, 11);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_track->id, 101);
  EXPECT_EQ(handled.semantic_id, 11);
  EXPECT_EQ(handled.supporting_raw_track_id, 101);
}

TEST(PrimaryTargetManagerTest,
     MissionReturnToPatrolDoesNotExcludeWithoutReidDecision) {
  using namespace std::chrono_literals;

  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint start{};
  const auto handled =
      MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto next_eligible =
      MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));

  const dog_patrol_perception_tracking::MissionSnapshot first_patrol{
      10U, dog_patrol_perception_tracking::MissionPhase::kPatrol, 0};
  const auto first = mgr.UpdateForMission(
      {handled, next_eligible}, first_patrol, std::nullopt, start);
  ASSERT_TRUE(first.primary_track.has_value());
  ASSERT_EQ(first.primary_target_id, 11);

  const dog_patrol_perception_tracking::MissionSnapshot verification{
      11U, dog_patrol_perception_tracking::MissionPhase::kVerifyIdentity, 11};
  const auto verified = mgr.UpdateForMission(
      {handled, next_eligible}, verification, first_patrol, start + 1ms);
  ASSERT_EQ(verified.primary_target_id, 11);

  const dog_patrol_perception_tracking::MissionSnapshot next_patrol{
      12U, dog_patrol_perception_tracking::MissionPhase::kPatrol, 0, 11};
  const auto selected = mgr.UpdateForMission(
      {handled, next_eligible}, next_patrol, verification, start + 2ms);

  EXPECT_TRUE(mgr.IsMissionEligible(handled));
  ASSERT_TRUE(selected.primary_track.has_value());
  EXPECT_EQ(selected.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(selected.primary_target_id, 11);
  EXPECT_EQ(selected.primary_track->id, 101);
  EXPECT_EQ(handled.semantic_id, 11);
  EXPECT_EQ(handled.supporting_raw_track_id, 101);
}

TEST(PrimaryTargetManagerTest, ActiveMissionBindsOnlyTheAuthoritativeTargetId) {
  using namespace dog_patrol_perception_tracking;
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const auto authoritative =
      MakeIdentity(11, 101, ClassId::kPerson, cv::Rect2f(0, 0, 40, 40));
  const auto larger_other =
      MakeIdentity(22, 202, ClassId::kPerson, cv::Rect2f(100, 0, 100, 100));
  const MissionSnapshot approach{11U, MissionPhase::kApproachTarget, 11};

  const auto selected = mgr.UpdateForMission(
      {authoritative, larger_other}, approach, std::nullopt,
      PrimaryTargetManager::TimePoint{});

  ASSERT_EQ(selected.state, PrimaryState::kLocked);
  ASSERT_TRUE(selected.primary_track.has_value());
  EXPECT_EQ(selected.primary_target_id, 11);
  EXPECT_EQ(selected.primary_track->id, 101);
}

TEST(PrimaryTargetManagerTest, ActiveMissionNeverReplacesMissingTargetWithAnotherPerson) {
  using namespace dog_patrol_perception_tracking;
  PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 1;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const MissionSnapshot verify{12U, MissionPhase::kVerifyIdentity, 11};
  const auto other =
      MakeIdentity(22, 202, ClassId::kPerson, cv::Rect2f(100, 0, 100, 100));

  const auto first = mgr.UpdateForMission(
      {other}, verify, std::nullopt, PrimaryTargetManager::TimePoint{});
  EXPECT_NE(first.primary_target_id, 22);
  const auto second = mgr.UpdateForMission(
      {other}, verify, verify, PrimaryTargetManager::TimePoint{});
  EXPECT_NE(second.primary_target_id, 22);
  EXPECT_FALSE(second.primary_track.has_value());
}

TEST(PrimaryTargetManagerTest, PreviousTaskPhaseAloneDoesNotExcludeUnfinishedTarget) {
  using namespace dog_patrol_perception_tracking;
  const auto person = MakeIdentity(11, 101, ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  for (const auto phase : {MissionPhase::kConfirmTarget, MissionPhase::kApproachTarget,
                           MissionPhase::kVerifyIdentity, MissionPhase::kTrackIntruder,
                           MissionPhase::kRecoverPatrol}) {
    PrimaryTargetManager mgr(PrimaryTargetManager::Config{});
    const MissionSnapshot previous{10U, phase, 11};
    const MissionSnapshot patrol{11U, MissionPhase::kPatrol, 0, 0};
    mgr.UpdateForMission({person}, previous, std::nullopt, PrimaryTargetManager::TimePoint{});
    const auto selected = mgr.UpdateForMission(
        {person}, patrol, previous, PrimaryTargetManager::TimePoint{});
    EXPECT_TRUE(mgr.IsMissionEligible(person));
    EXPECT_EQ(selected.state, PrimaryState::kLocked);
    EXPECT_EQ(selected.primary_target_id, 11);
  }
}

TEST(PrimaryTargetManagerTest, PatrolSelectionKeepsMultipleReidIdentitiesExcluded) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint now{};
  const auto first_handled =
      MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 100, 100));
  const auto second_handled =
      MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto eligible = MakeIdentity(33, 303, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));

  mgr.ExcludeForReidDedup(11);
  mgr.UpdateForPatrol({first_handled}, now);
  mgr.ExcludeForReidDedup(22);
  const auto state = mgr.UpdateForPatrol({first_handled, second_handled, eligible}, now);

  EXPECT_FALSE(mgr.IsMissionEligible(first_handled));
  EXPECT_FALSE(mgr.IsMissionEligible(second_handled));
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 33);
  EXPECT_EQ(state.primary_track->id, 303);
}

TEST(PrimaryTargetManagerTest, ReadyClearsPrimaryWithoutSelectingOrDeletingReidHistory) {
  using namespace dog_patrol_perception_tracking;
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const auto person = MakeIdentity(11, 101, ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto eligible = MakeIdentity(22, 202, ClassId::kPerson, cv::Rect2f(100, 0, 60, 60));
  const MissionSnapshot patrol{10U, MissionPhase::kPatrol, 0, 11};
  const MissionSnapshot ready{11U, MissionPhase::kReady, 0, 11};

  mgr.ExcludeForReidDedup(11);
  const auto selected = mgr.UpdateForMission({person, eligible}, patrol, std::nullopt,
                                              PrimaryTargetManager::TimePoint{});
  ASSERT_EQ(selected.primary_target_id, 22);
  const auto standby = mgr.UpdateForMission({person, eligible}, ready, patrol,
                                             PrimaryTargetManager::TimePoint{} + std::chrono::seconds{1});
  EXPECT_EQ(standby.state, PrimaryState::kIdle);
  EXPECT_EQ(standby.primary_target_id, -1);
  EXPECT_FALSE(mgr.IsMissionEligible(person));
}

TEST(PrimaryTargetManagerTest, StartupResetClearsPrimaryAndReidHistory) {
  using namespace dog_patrol_perception_tracking;
  PrimaryTargetManager mgr(PrimaryTargetManager::Config{});
  const auto person = MakeIdentity(11, 101, ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const MissionSnapshot patrol{10U, MissionPhase::kPatrol, 0, 11};
  const MissionSnapshot startup{11U, MissionPhase::kStartup, 0, 0};

  mgr.UpdateForMission({person}, patrol, std::nullopt, PrimaryTargetManager::TimePoint{});
  mgr.ExcludeForReidDedup(11);
  EXPECT_FALSE(mgr.IsMissionEligible(person));
  const auto reset = mgr.UpdateForMission({person}, startup, patrol,
                                          PrimaryTargetManager::TimePoint{});
  EXPECT_TRUE(mgr.IsMissionEligible(person));
  EXPECT_EQ(reset.state, PrimaryState::kIdle);
  EXPECT_EQ(reset.primary_target_id, -1);
}

TEST(PrimaryTargetManagerTest, ActivePatrolPrimaryDoesNotSwitchWhenAnotherEligiblePersonBecomesLarger) {
  using namespace std::chrono_literals;

  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint start{};
  const auto primary = MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));
  const auto candidate = MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50));

  const auto first = mgr.UpdateForPatrol({primary, candidate}, start);
  ASSERT_TRUE(first.primary_track.has_value());
  EXPECT_EQ(first.primary_target_id, 11);

  const auto larger_candidate = MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 100, 100));
  const auto second = mgr.UpdateForPatrol({primary, larger_candidate}, start + 1ms);
  ASSERT_TRUE(second.primary_track.has_value());
  EXPECT_EQ(second.primary_target_id, 11);
  EXPECT_EQ(second.primary_track->id, 101);
}

TEST(PrimaryTargetManagerTest, LockedContinuityPreferredBeforeReplacement) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 40, 40))});
  ASSERT_TRUE(s1.primary_track.has_value());
  const int business_id = s1.primary_target_id;
  EXPECT_EQ(s1.primary_track->id, 7);

  // Raw tracker id changes, but same person should be rebound under same business primary_target_id.
  auto s2 = mgr.Update({MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60))});
  ASSERT_TRUE(s2.primary_track.has_value());
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s2.primary_track->id, 8);
  EXPECT_EQ(s2.primary_target_id, business_id);

  // Short temporary miss should not immediately replace primary target.
  auto s3 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s3.primary_target_id, business_id);

  // After long miss, a new business primary_target_id can be assigned.
  auto s4 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s4.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  auto s5 = mgr.Update({MakeIdentity(2, 9, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(200, 0, 60, 60))});
  EXPECT_EQ(s5.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_FALSE(s5.primary_track.has_value());

  auto s6 = mgr.Update({MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60))});
  ASSERT_TRUE(s6.primary_track.has_value());
  EXPECT_EQ(s6.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s6.primary_track->id, 8);
  EXPECT_EQ(s6.primary_target_id, business_id);
}

TEST(PrimaryTargetManagerTest, DefaultMissingWindowKeepsPrimaryAcrossFourSecondAbsence) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());
  EXPECT_EQ(s1.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s1.primary_target_id, 1);

  dog_patrol_perception_tracking::PrimaryTargetResult missing_state;
  // The production input is 10 Hz and the internal default is 100 frames;
  // keep this scenario within the four-second occlusion window.
  for (int i = 0; i < 40; ++i) {
    missing_state = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  }
  EXPECT_EQ(missing_state.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(missing_state.primary_target_id, 1);

  auto distractor = MakeIdentity(2, 20, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(100, 0, 80, 80));
  auto s2 = mgr.Update({distractor});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_FALSE(s2.primary_track.has_value());

  auto s3 = mgr.Update({MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s3.primary_track.has_value());
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s3.primary_target_id, 1);
  EXPECT_EQ(s3.primary_track->id, 8);
}

TEST(PrimaryTargetManagerTest, NonPersonCannotBecomePrimaryTarget) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s = mgr.Update({MakeIdentity(101, 101, dog_patrol_perception_tracking::ClassId::kCar, cv::Rect2f(0, 0, 200, 200))});
  EXPECT_EQ(s.state, dog_patrol_perception_tracking::PrimaryState::kIdle);
  EXPECT_FALSE(s.primary_track.has_value());
  EXPECT_EQ(s.primary_target_id, -1);
}

TEST(PrimaryTargetManagerTest, PendingRecoveryStateSerializesPublicString) {
  EXPECT_EQ(dog_patrol_perception_tracking::PrimaryStateToString(dog_patrol_perception_tracking::PrimaryState::kPendingRecovery),
            "PENDING_RECOVERY");
}

TEST(PrimaryTargetManagerTest, ReacquireCanSwitchPrimarySemanticIdAfterLongLoss) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 2;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 10, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80))});
  ASSERT_TRUE(s1.primary_track.has_value());
  EXPECT_EQ(s1.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s1.primary_target_id, 1);

  auto s2 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  auto s3 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  auto s4 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s4.state, dog_patrol_perception_tracking::PrimaryState::kLost);
  EXPECT_EQ(s4.primary_target_id, -1);

  auto s5 = mgr.Update({MakeIdentity(2, 20, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(20, 0, 90, 90))});
  ASSERT_TRUE(s5.primary_track.has_value());
  EXPECT_EQ(s5.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s5.primary_target_id, 2);
  EXPECT_EQ(s5.primary_track->id, 20);
}

TEST(PrimaryTargetManagerTest, SuspiciousVisiblePrimaryKeepsSemanticIdPendingRecovery) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  cfg.max_center_jump_norm = 1.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());
  EXPECT_EQ(s1.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s1.primary_target_id, 1);

  auto suspicious = MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(500, 0, 50, 50));
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_FALSE(s2.primary_track.has_value());
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_EQ(mgr.LastRejectReason(), "visible_primary_center_jump");
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_visible_primary_sanity_rejected");
}

TEST(PrimaryTargetManagerTest, PendingRecoveryHoldsForConfiguredFramesThenFallsBackToOccluded) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 5;
  cfg.pending_recovery_frames = 2;
  cfg.min_person_area_px = 100.0F;
  cfg.max_center_jump_norm = 1.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto suspicious = MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(500, 0, 50, 50));
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_visible_primary_sanity_rejected");

  auto s3 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(s3.primary_target_id, 1);
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_hold_missing_identity_evidence");

  auto s4 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s4.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s4.primary_target_id, 1);
  EXPECT_EQ(mgr.LastDecisionReason(), "occluded_within_lost_threshold");
}

TEST(PrimaryTargetManagerTest, PendingRecoveryReturnsToLockedWithStableEvidence) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 5;
  cfg.pending_recovery_frames = 3;
  cfg.min_person_area_px = 100.0F;
  cfg.max_center_jump_norm = 1.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto suspicious = MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(500, 0, 50, 50));
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);

  auto recovered = MakeIdentity(1, 9, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(2, 0, 50, 50));
  auto s3 = mgr.Update({recovered});
  ASSERT_TRUE(s3.primary_track.has_value());
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s3.primary_target_id, 1);
  EXPECT_EQ(s3.primary_track->id, 9);
  EXPECT_EQ(mgr.LastDecisionReason(), "locked_visible_primary_identity");
  EXPECT_TRUE(mgr.LastRejectReason().empty());
}

TEST(PrimaryTargetManagerTest, StableNewScaleRecoversSameSemanticTarget) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 10;
  cfg.pending_recovery_frames = 3;
  cfg.min_person_area_px = 100.0F;
  cfg.max_area_ratio = 2.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  ASSERT_EQ(mgr.Update({MakeIdentity(
                1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
                cv::Rect2f(0, 0, 50, 50))})
                .state,
            dog_patrol_perception_tracking::PrimaryState::kLocked);

  const auto first = mgr.Update({MakeIdentity(
      1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
      cv::Rect2f(0, 0, 90, 90))});
  EXPECT_EQ(first.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(mgr.LastRejectReason(), "visible_primary_area_ratio");

  const auto second = mgr.Update({MakeIdentity(
      1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
      cv::Rect2f(0, 0, 92, 92))});
  EXPECT_EQ(second.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);

  const auto recovered = mgr.Update({MakeIdentity(
      1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
      cv::Rect2f(0, 0, 94, 94))});
  ASSERT_EQ(recovered.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  ASSERT_TRUE(recovered.primary_track.has_value());
  EXPECT_EQ(recovered.primary_target_id, 1);
  EXPECT_EQ(recovered.primary_track->id, 7);
  EXPECT_EQ(mgr.LastDecisionReason(), "locked_stable_geometry_recovery");
  EXPECT_TRUE(mgr.LastRejectReason().empty());
}

TEST(PrimaryTargetManagerTest, GeometryRecoveryCannotBridgeToDifferentRawTrack) {
  using namespace dog_patrol_perception_tracking;
  PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 10;
  cfg.pending_recovery_frames = 3;
  cfg.min_person_area_px = 100.0F;
  cfg.max_area_ratio = 2.0F;
  PrimaryTargetManager mgr(cfg);

  ASSERT_EQ(mgr.Update({MakeIdentity(1, 7, ClassId::kPerson,
                                     cv::Rect2f(0, 0, 50, 50))}).state,
            PrimaryState::kLocked);
  for (int frame = 0; frame < 4; ++frame) {
    const auto state = mgr.Update({MakeIdentity(
        1, 8, ClassId::kPerson,
        cv::Rect2f(0, 0, 90 + frame, 90 + frame))});
    EXPECT_EQ(state.state, PrimaryState::kPendingRecovery);
    EXPECT_EQ(state.primary_target_id, 1);
    EXPECT_FALSE(state.primary_track.has_value());
  }
}

TEST(PrimaryTargetManagerTest, DisabledPendingRecoveryDoesNotAcceptGeometryOutlier) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 10;
  cfg.pending_recovery_frames = 0;
  cfg.min_person_area_px = 100.0F;
  cfg.max_area_ratio = 2.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  ASSERT_EQ(mgr.Update({MakeIdentity(
                1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
                cv::Rect2f(0, 0, 50, 50))})
                .state,
            dog_patrol_perception_tracking::PrimaryState::kLocked);

  for (int frame = 0; frame < 3; ++frame) {
    const auto state = mgr.Update({MakeIdentity(
        1, 7, dog_patrol_perception_tracking::ClassId::kPerson,
        cv::Rect2f(0, 0, 90 + frame, 90 + frame))});
    EXPECT_EQ(state.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
    EXPECT_FALSE(state.primary_track.has_value());
  }
}

TEST(PrimaryTargetManagerTest, PendingRecoveryLossThresholdBecomesLost) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 2;
  cfg.pending_recovery_frames = 5;
  cfg.min_person_area_px = 100.0F;
  cfg.max_center_jump_norm = 1.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto suspicious = MakeIdentity(1, 8, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(500, 0, 50, 50));
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);

  auto s3 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);

  auto s4 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s4.state, dog_patrol_perception_tracking::PrimaryState::kLost);
  EXPECT_EQ(s4.primary_target_id, -1);
  EXPECT_EQ(mgr.LastDecisionReason(), "lost_after_threshold");
}

TEST(PrimaryTargetManagerTest, AssignmentGateFailureEntersPendingRecoveryWithRejectReason) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto rejected = MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50));
  rejected.association.stage = "inactive_recover_candidate";
  rejected.association.passed_final_cost_gate = false;
  auto s2 = mgr.Update({rejected});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_EQ(mgr.LastRejectReason(), "visible_primary_assoc_gate_failed");
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_visible_primary_sanity_rejected");
}

TEST(PrimaryTargetManagerTest, OcclusionSuspectVisiblePrimaryRejected) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto suspicious = MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50));
  suspicious.occlusion_suspect = true;
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_FALSE(s2.primary_track.has_value());
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_EQ(mgr.LastRejectReason(), "visible_primary_occlusion_suspect");
}

TEST(PrimaryTargetManagerTest, LowScoreVisiblePrimaryRejected) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto suspicious = MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50));
  suspicious.low_score_update = true;
  suspicious.association.low_score_detection = true;
  auto s2 = mgr.Update({suspicious});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_FALSE(s2.primary_track.has_value());
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_EQ(mgr.LastRejectReason(), "visible_primary_low_score_update");
}

TEST(PrimaryTargetManagerTest, OccludedIdentityLifecyclePreventsImmediateReplacement) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());
  EXPECT_EQ(s1.state, dog_patrol_perception_tracking::PrimaryState::kLocked);

  auto occluded_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kOccluded, 1,
                                                cv::Rect2f(0, 0, 50, 50));
  auto neighbor = MakeIdentity(2, 20, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(100, 0, 80, 80));
  auto s2 = mgr.Update({occluded_primary, neighbor});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_FALSE(s2.primary_track.has_value());
  EXPECT_EQ(mgr.LastDecisionReason(), "occluded_from_identity_state");
}

TEST(PrimaryTargetManagerTest, MergedAndSplitRecoveryIdentityLifecycleArePendingRecovery) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());
  EXPECT_EQ(s1.state, dog_patrol_perception_tracking::PrimaryState::kLocked);

  auto merged_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kMerged, 1,
                                              cv::Rect2f(0, 0, 50, 50));
  auto merged_state = mgr.Update({merged_primary});
  EXPECT_EQ(merged_state.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(merged_state.primary_target_id, 1);
  EXPECT_FALSE(merged_state.primary_track.has_value());
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_from_identity_state");

  auto split_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kSplitRecovery, 2,
                                             cv::Rect2f(0, 0, 50, 50));
  auto split_state = mgr.Update({split_primary});
  EXPECT_EQ(split_state.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(split_state.primary_target_id, 1);
  EXPECT_FALSE(split_state.primary_track.has_value());
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_from_identity_state");
}

TEST(PrimaryTargetManagerTest, MergedPendingRecoveryFallsBackToOccludedWhenLifecycleBecomesOrdinaryOcclusion) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 5;
  cfg.pending_recovery_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto merged_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kMerged, 1,
                                              cv::Rect2f(0, 0, 50, 50));
  auto s2 = mgr.Update({merged_primary});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_from_identity_state");

  auto occluded_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kOccluded, 2,
                                                cv::Rect2f(0, 0, 50, 50));
  auto s3 = mgr.Update({occluded_primary});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_hold_missing_identity_evidence");

  auto s4 = mgr.Update({occluded_primary});
  EXPECT_EQ(s4.state, dog_patrol_perception_tracking::PrimaryState::kPendingRecovery);
  EXPECT_EQ(mgr.LastDecisionReason(), "pending_recovery_hold_missing_identity_evidence");

  auto s5 = mgr.Update({occluded_primary});
  EXPECT_EQ(s5.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s5.primary_target_id, 1);
  EXPECT_EQ(mgr.LastDecisionReason(), "occluded_from_identity_state");
}

TEST(PrimaryTargetManagerTest, OrdinaryOcclusionDoesNotEnterPendingRecovery) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.pending_recovery_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto s2 = mgr.Update(std::vector<dog_patrol_perception_tracking::IdentityObservation>{});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s2.primary_target_id, 1);
  EXPECT_EQ(mgr.LastDecisionReason(), "occluded_within_lost_threshold");

  auto occluded_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kOccluded, 2,
                                                cv::Rect2f(0, 0, 50, 50));
  auto s3 = mgr.Update({occluded_primary});
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kOccluded);
  EXPECT_EQ(s3.primary_target_id, 1);
  EXPECT_EQ(mgr.LastDecisionReason(), "occluded_from_identity_state");
}

TEST(PrimaryTargetManagerTest, LostIdentityLifecycleReleasesPrimaryForNewIdentity) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.lost_threshold_frames = 3;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);

  auto s1 = mgr.Update({MakeIdentity(1, 7, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 50, 50))});
  ASSERT_TRUE(s1.primary_track.has_value());

  auto lost_primary = MakeLifecycleIdentity(1, dog_patrol_perception_tracking::IdentityState::kLost, 4,
                                            cv::Rect2f(0, 0, 50, 50));
  auto s2 = mgr.Update({lost_primary});
  EXPECT_EQ(s2.state, dog_patrol_perception_tracking::PrimaryState::kLost);
  EXPECT_EQ(s2.primary_target_id, -1);
  EXPECT_EQ(mgr.LastDecisionReason(), "lost_from_identity_state");

  auto s3 = mgr.Update({MakeIdentity(2, 20, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(10, 0, 80, 80))});
  ASSERT_TRUE(s3.primary_track.has_value());
  EXPECT_EQ(s3.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(s3.primary_target_id, 2);
}
