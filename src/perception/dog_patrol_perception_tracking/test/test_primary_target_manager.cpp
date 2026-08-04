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

TEST(PrimaryTargetManagerTest, PatrolCycleExcludesHandledIdentityAndChoosesNextLargestEligiblePerson) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint now{};

  const auto handled = MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto eligible = MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));

  const auto first = mgr.UpdateForPatrol({handled}, now);
  ASSERT_TRUE(first.primary_track.has_value());
  EXPECT_EQ(first.primary_target_id, 11);

  mgr.ResetForPatrolCycle(first.primary_target_id);
  const auto state = mgr.UpdateForPatrol({handled, eligible}, now);

  EXPECT_FALSE(mgr.IsMissionEligible(handled));
  EXPECT_EQ(state.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(state.primary_target_id, 22);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_track->id, 202);
  EXPECT_EQ(handled.semantic_id, 11);
  EXPECT_EQ(handled.supporting_raw_track_id, 101);
}

TEST(PrimaryTargetManagerTest,
     MissionReturnToPatrolMarksHandledTargetAndSelectsNextEligibleOnFirstFrame) {
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
      10U, dog_patrol_perception_tracking::MissionPhase::kPatrol, 0, false,
      dog_patrol_perception_tracking::MissionBlockCause::kNone};
  const auto first = mgr.UpdateForMission(
      {handled, next_eligible}, first_patrol, std::nullopt, start);
  ASSERT_TRUE(first.primary_track.has_value());
  ASSERT_EQ(first.primary_target_id, 11);

  const dog_patrol_perception_tracking::MissionSnapshot verification{
      11U, dog_patrol_perception_tracking::MissionPhase::kVerifyIdentity, 11, false,
      dog_patrol_perception_tracking::MissionBlockCause::kNone};
  const auto verified = mgr.UpdateForMission(
      {handled, next_eligible}, verification, first_patrol, start + 1ms);
  ASSERT_EQ(verified.primary_target_id, 11);

  const dog_patrol_perception_tracking::MissionSnapshot next_patrol{
      12U, dog_patrol_perception_tracking::MissionPhase::kPatrol, 0, false,
      dog_patrol_perception_tracking::MissionBlockCause::kNone};
  const auto selected = mgr.UpdateForMission(
      {handled, next_eligible}, next_patrol, verification, start + 2ms);

  EXPECT_FALSE(mgr.IsMissionEligible(handled));
  ASSERT_TRUE(selected.primary_track.has_value());
  EXPECT_EQ(selected.state, dog_patrol_perception_tracking::PrimaryState::kLocked);
  EXPECT_EQ(selected.primary_target_id, 22);
  EXPECT_EQ(selected.primary_track->id, 202);
  EXPECT_EQ(handled.semantic_id, 11);
  EXPECT_EQ(handled.supporting_raw_track_id, 101);
}

TEST(PrimaryTargetManagerTest, HandledIdentityUsesDefaultThirtySecondContinuousAbsenceBeforePatrolEligibility) {
  using namespace std::chrono_literals;

  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint start{};
  const auto handled = MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));

  mgr.ResetForPatrolCycle(11);
  mgr.UpdateForPatrol({handled}, start);
  mgr.UpdateForPatrol({}, start);
  mgr.UpdateForPatrol({}, start + 29s);
  EXPECT_FALSE(mgr.IsMissionEligible(handled));

  mgr.UpdateForPatrol({}, start + 30s);
  EXPECT_TRUE(mgr.IsMissionEligible(handled));

  const auto state = mgr.UpdateForPatrol({handled}, start + 30s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

TEST(PrimaryTargetManagerTest, ReappearingHandledIdentityRestartsConfiguredAbsenceTimer) {
  using namespace std::chrono_literals;

  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  cfg.handled_ignore_absence = 2s;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint start{};
  const auto handled = MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));

  mgr.ResetForPatrolCycle(11);
  mgr.UpdateForPatrol({}, start);
  mgr.UpdateForPatrol({handled}, start + 1500ms);
  mgr.UpdateForPatrol({}, start + 1600ms);
  mgr.UpdateForPatrol({}, start + 3500ms);
  EXPECT_FALSE(mgr.IsMissionEligible(handled));

  mgr.UpdateForPatrol({}, start + 3600ms);
  EXPECT_TRUE(mgr.IsMissionEligible(handled));
}

TEST(PrimaryTargetManagerTest, PatrolSelectionKeepsMultipleHandledIdentitiesExcluded) {
  dog_patrol_perception_tracking::PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  dog_patrol_perception_tracking::PrimaryTargetManager mgr(cfg);
  const dog_patrol_perception_tracking::PrimaryTargetManager::TimePoint now{};
  const auto first_handled =
      MakeIdentity(11, 101, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 100, 100));
  const auto second_handled =
      MakeIdentity(22, 202, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 80, 80));
  const auto eligible = MakeIdentity(33, 303, dog_patrol_perception_tracking::ClassId::kPerson, cv::Rect2f(0, 0, 60, 60));

  mgr.ResetForPatrolCycle(11);
  mgr.UpdateForPatrol({first_handled}, now);
  mgr.ResetForPatrolCycle(22);
  const auto state = mgr.UpdateForPatrol({first_handled, second_handled, eligible}, now);

  EXPECT_FALSE(mgr.IsMissionEligible(first_handled));
  EXPECT_FALSE(mgr.IsMissionEligible(second_handled));
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 33);
  EXPECT_EQ(state.primary_track->id, 303);
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
  for (int i = 0; i < 120; ++i) {
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
