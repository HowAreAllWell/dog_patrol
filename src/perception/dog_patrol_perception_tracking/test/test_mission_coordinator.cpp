#include <chrono>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "vision_demo_host/modules/mission_coordinator.hpp"

namespace {

using vision_demo_host::ClassId;
using vision_demo_host::IdentityObservation;
using vision_demo_host::IdentityState;
using vision_demo_host::MissionCoordinator;
using vision_demo_host::MissionBlockCause;
using vision_demo_host::MissionEventAction;
using vision_demo_host::PerceptionMissionEvent;
using vision_demo_host::MissionPhase;
using vision_demo_host::MissionSnapshot;
using vision_demo_host::PrimaryState;
using vision_demo_host::PrimaryTargetResult;
using vision_demo_host::Track;

IdentityObservation TrustedPerson(const int semantic_id, const int raw_track_id,
                                  const cv::Rect2f bbox = cv::Rect2f{10.0F, 20.0F, 40.0F, 80.0F}) {
  IdentityObservation identity;
  identity.semantic_id = semantic_id;
  identity.state = IdentityState::kActive;
  identity.supporting_raw_track_id = raw_track_id;
  identity.class_id = ClassId::kPerson;
  identity.confidence = 0.92F;
  identity.bbox = bbox;
  identity.visible = true;
  return identity;
}

PrimaryTargetResult LockedPrimary(const int semantic_id, const int raw_track_id,
                                  const cv::Rect2f bbox = cv::Rect2f{10.0F, 20.0F, 40.0F, 80.0F}) {
  Track track;
  track.id = raw_track_id;
  track.class_id = ClassId::kPerson;
  track.confidence = 0.92F;
  track.bbox = bbox;
  track.is_confirmed = true;
  track.authoritative = true;

  PrimaryTargetResult primary;
  primary.state = PrimaryState::kLocked;
  primary.primary_target_id = semantic_id;
  primary.raw_track_id = raw_track_id;
  primary.primary_track = track;
  return primary;
}

MissionSnapshot ActiveMission(const MissionPhase phase = MissionPhase::kConfirmTarget,
                              const std::uint32_t state_seq = 17U, const int target_id = 42) {
  MissionSnapshot mission;
  mission.phase = phase;
  mission.state_seq = state_seq;
  mission.target_id = target_id;
  return mission;
}

MissionSnapshot TargetLostBlock(const std::uint32_t state_seq, const int target_id = 42) {
  MissionSnapshot mission = ActiveMission(MissionPhase::kConfirmTarget, state_seq, target_id);
  mission.blocked = true;
  mission.block_cause = MissionBlockCause::kTargetLost;
  return mission;
}

MissionSnapshot ExecutionErrorBlock(const std::uint32_t state_seq, const int target_id = 42) {
  MissionSnapshot mission = ActiveMission(MissionPhase::kConfirmTarget, state_seq, target_id);
  mission.blocked = true;
  mission.block_cause = MissionBlockCause::kExecutionError;
  return mission;
}

void ExpectSingleEvent(const std::vector<MissionEventAction> &events, const PerceptionMissionEvent event,
                       const int target_id, const std::uint32_t state_seq) {
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().event, event);
  EXPECT_EQ(events.front().target_id, target_id);
  EXPECT_EQ(events.front().observed_state_seq, state_seq);
}

TEST(MissionCoordinatorTest, PublishesOnlyCurrentTrustedTargetBoxInAllowedMissionState) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint now{};

  const auto output = coordinator.Update({ActiveMission(), {TrustedPerson(42, 7), TrustedPerson(99, 8)},
                                          LockedPrimary(42, 7), now});

  ASSERT_TRUE(output.target_box.has_value());
  EXPECT_EQ(output.target_box->target_id, 42);
  EXPECT_EQ(output.target_box->observed_state_seq, 17U);
  EXPECT_EQ(output.target_box->source_time, now);
  EXPECT_EQ(output.target_box->bbox, cv::Rect2f(10.0F, 20.0F, 40.0F, 80.0F));
  EXPECT_TRUE(output.events.empty());
}

TEST(MissionCoordinatorTest, GatesFreshBoxesByMissionPhaseAndCurrentPrimaryEvidence) {
  const MissionCoordinator::TimePoint now{};
  const auto identities = std::vector<IdentityObservation>{TrustedPerson(42, 7)};
  const auto primary = LockedPrimary(42, 7);

  for (const MissionPhase phase : {MissionPhase::kStartup, MissionPhase::kPatrol}) {
    MissionCoordinator coordinator;
    auto mission = ActiveMission(phase);
    mission.target_id = phase == MissionPhase::kPatrol ? 0 : 42;
    const auto output = coordinator.Update({mission, identities, primary, now});
    EXPECT_FALSE(output.target_box.has_value());
  }

  for (const MissionPhase phase : {MissionPhase::kConfirmTarget, MissionPhase::kApproachTarget,
                                   MissionPhase::kVerifyIdentity, MissionPhase::kTrackIntruder}) {
    MissionCoordinator coordinator;
    const auto output = coordinator.Update({ActiveMission(phase), identities, primary, now});
    ASSERT_TRUE(output.target_box.has_value());
    EXPECT_EQ(output.target_box->target_id, 42);
  }

  MissionCoordinator coordinator;
  const auto no_current_identity = coordinator.Update({ActiveMission(), {}, primary, now});
  EXPECT_FALSE(no_current_identity.target_box.has_value());
  EXPECT_TRUE(no_current_identity.events.empty());

  const auto wrong_primary = coordinator.Update(
      {ActiveMission(), identities, LockedPrimary(99, 8), now + std::chrono::milliseconds{1}});
  EXPECT_FALSE(wrong_primary.target_box.has_value());
  EXPECT_TRUE(wrong_primary.events.empty());
}

TEST(MissionCoordinatorTest, EmitsOneLossAtDefaultTimeoutWithoutPublishingReplacementBox) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto mission = ActiveMission();

  ASSERT_TRUE(coordinator.Update({mission, {TrustedPerson(42, 7)}, LockedPrimary(42, 7), start})
                  .target_box.has_value());

  const auto before_timeout = coordinator.Update(
      {mission, {TrustedPerson(99, 8)}, LockedPrimary(99, 8), start + std::chrono::milliseconds{499}});
  EXPECT_FALSE(before_timeout.target_box.has_value());
  EXPECT_TRUE(before_timeout.events.empty());

  const auto at_timeout = coordinator.Update(
      {mission, {TrustedPerson(99, 8)}, LockedPrimary(99, 8), start + std::chrono::milliseconds{500}});
  EXPECT_FALSE(at_timeout.target_box.has_value());
  ExpectSingleEvent(at_timeout.events, PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto after_timeout = coordinator.Update(
      {mission, {TrustedPerson(99, 8)}, LockedPrimary(99, 8), start + std::chrono::milliseconds{501}});
  EXPECT_FALSE(after_timeout.target_box.has_value());
  EXPECT_TRUE(after_timeout.events.empty());
}

TEST(MissionCoordinatorTest, ReacquiresOnlySameTargetAgainstNewCompatibleLostBlockAndResumesAfterUnblock) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto mission = ActiveMission();
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({mission, {trusted}, primary, start}).target_box.has_value());
  ExpectSingleEvent(
      coordinator.Update({mission, {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{500}}).events,
      PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto before_block = coordinator.Update(
      {mission, {trusted}, primary, start + std::chrono::milliseconds{600}});
  EXPECT_TRUE(before_block.events.empty());
  EXPECT_FALSE(before_block.target_box.has_value());

  const auto reacquired = coordinator.Update(
      {TargetLostBlock(18U), {trusted}, primary, start + std::chrono::milliseconds{601}});
  ExpectSingleEvent(reacquired.events, PerceptionMissionEvent::kTargetReacquired, 42, 18U);
  EXPECT_FALSE(reacquired.target_box.has_value());

  const auto duplicate_block = coordinator.Update(
      {TargetLostBlock(18U), {trusted}, primary, start + std::chrono::milliseconds{602}});
  EXPECT_TRUE(duplicate_block.events.empty());
  EXPECT_FALSE(duplicate_block.target_box.has_value());

  const auto resumed = coordinator.Update(
      {ActiveMission(MissionPhase::kConfirmTarget, 19U), {trusted}, primary, start + std::chrono::milliseconds{603}});
  EXPECT_TRUE(resumed.events.empty());
  ASSERT_TRUE(resumed.target_box.has_value());
  EXPECT_EQ(resumed.target_box->target_id, 42);
  EXPECT_EQ(resumed.target_box->observed_state_seq, 19U);
}

TEST(MissionCoordinatorTest, PreventsReacquisitionForExpiredStaleOrExecutionErrorState) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto mission = ActiveMission();
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({mission, {trusted}, primary, start}).target_box.has_value());
  ExpectSingleEvent(
      coordinator.Update({mission, {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{500}}).events,
      PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto execution_error = coordinator.Update(
      {ExecutionErrorBlock(18U), {trusted}, primary, start + std::chrono::milliseconds{600}});
  EXPECT_TRUE(execution_error.events.empty());
  EXPECT_FALSE(execution_error.target_box.has_value());

  const auto stale = coordinator.Update(
      {TargetLostBlock(17U), {trusted}, primary, start + std::chrono::milliseconds{601}});
  EXPECT_TRUE(stale.events.empty());
  EXPECT_FALSE(stale.target_box.has_value());

  const auto expired = coordinator.Update(
      {TargetLostBlock(19U), {trusted}, primary, start + std::chrono::seconds{6}});
  EXPECT_TRUE(expired.events.empty());
  EXPECT_FALSE(expired.target_box.has_value());
}

TEST(MissionCoordinatorTest, SupportsRepeatedCyclesAndNonDefaultTimeConfiguration) {
  MissionCoordinator::Config config;
  config.lost_event_timeout = std::chrono::milliseconds{100};
  config.reacquire_retention = std::chrono::seconds{2};
  MissionCoordinator coordinator(config);
  const MissionCoordinator::TimePoint start{};
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({ActiveMission(), {trusted}, primary, start}).target_box.has_value());
  EXPECT_TRUE(coordinator.Update({ActiveMission(), {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{99}})
                  .events.empty());
  ExpectSingleEvent(
      coordinator.Update({ActiveMission(), {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{100}}).events,
      PerceptionMissionEvent::kTargetLost, 42, 17U);
  ExpectSingleEvent(
      coordinator.Update({TargetLostBlock(18U), {trusted}, primary, start + std::chrono::milliseconds{101}}).events,
      PerceptionMissionEvent::kTargetReacquired, 42, 18U);
  ASSERT_TRUE(coordinator.Update(
                  {ActiveMission(MissionPhase::kConfirmTarget, 19U), {trusted}, primary,
                   start + std::chrono::milliseconds{102}})
                  .target_box.has_value());

  ExpectSingleEvent(
      coordinator.Update({ActiveMission(MissionPhase::kConfirmTarget, 19U), {}, PrimaryTargetResult{},
                          start + std::chrono::milliseconds{202}})
          .events,
      PerceptionMissionEvent::kTargetLost, 42, 19U);
  ExpectSingleEvent(
      coordinator.Update({TargetLostBlock(20U), {trusted}, primary, start + std::chrono::milliseconds{203}}).events,
      PerceptionMissionEvent::kTargetReacquired, 42, 20U);
}

TEST(MissionCoordinatorTest, RejectsInvalidTimeoutConfiguration) {
  MissionCoordinator::Config config;
  config.lost_event_timeout = MissionCoordinator::Duration::zero();
  EXPECT_THROW(MissionCoordinator invalid(config), std::invalid_argument);

  config.lost_event_timeout = std::chrono::seconds{6};
  config.reacquire_retention = std::chrono::seconds{6};
  EXPECT_THROW(MissionCoordinator invalid(config), std::invalid_argument);
}

TEST(MissionCoordinatorTest, DoesNotReuseSourceFrameOrPublishAZeroAreaBoxAsFreshObservation) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({ActiveMission(), {trusted}, primary, start}).target_box.has_value());

  const auto repeated_source_frame = coordinator.Update(
      {ActiveMission(MissionPhase::kApproachTarget, 18U), {trusted}, primary, start});
  EXPECT_FALSE(repeated_source_frame.target_box.has_value());

  const auto zero_box = TrustedPerson(42, 7, cv::Rect2f{10.0F, 20.0F, 0.0F, 80.0F});
  const auto zero_primary = LockedPrimary(42, 7, zero_box.bbox);
  const auto fabricated = coordinator.Update(
      {ActiveMission(MissionPhase::kConfirmTarget, 19U), {zero_box}, zero_primary,
       start + std::chrono::milliseconds{1}});
  EXPECT_FALSE(fabricated.target_box.has_value());
}

TEST(MissionCoordinatorTest, DoesNotReacquireAfterAnIncompatibleBlockInTheSameLossCycle) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({ActiveMission(), {trusted}, primary, start}).target_box.has_value());
  ExpectSingleEvent(
      coordinator.Update({ActiveMission(), {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{500}}).events,
      PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto error = coordinator.Update(
      {ExecutionErrorBlock(18U), {trusted}, primary, start + std::chrono::milliseconds{600}});
  EXPECT_TRUE(error.events.empty());

  const auto incompatible_recovery = coordinator.Update(
      {TargetLostBlock(19U), {trusted}, primary, start + std::chrono::milliseconds{601}});
  EXPECT_TRUE(incompatible_recovery.events.empty());
  EXPECT_FALSE(incompatible_recovery.target_box.has_value());
}

TEST(MissionCoordinatorTest, RetainsOnlyTheLostSemanticTargetAcrossRawTrackReplacement) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint start{};
  const auto original = TrustedPerson(42, 7);
  const auto original_primary = LockedPrimary(42, 7);
  const auto replacement = TrustedPerson(42, 70, cv::Rect2f{12.0F, 22.0F, 41.0F, 81.0F});
  const auto replacement_primary = LockedPrimary(42, 70, replacement.bbox);

  ASSERT_TRUE(coordinator.Update({ActiveMission(), {original}, original_primary, start}).target_box.has_value());
  ExpectSingleEvent(
      coordinator.Update({ActiveMission(), {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{500}}).events,
      PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto different_target = coordinator.Update(
      {ActiveMission(MissionPhase::kConfirmTarget, 18U, 99), {TrustedPerson(99, 8)}, LockedPrimary(99, 8),
       start + std::chrono::milliseconds{600}});
  EXPECT_TRUE(different_target.events.empty());
  EXPECT_FALSE(different_target.target_box.has_value());

  const auto reacquired = coordinator.Update(
      {TargetLostBlock(19U), {replacement}, replacement_primary, start + std::chrono::milliseconds{601}});
  ExpectSingleEvent(reacquired.events, PerceptionMissionEvent::kTargetReacquired, 42, 19U);
}

}  // namespace
