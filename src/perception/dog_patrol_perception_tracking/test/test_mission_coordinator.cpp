#include <chrono>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityObservation;
using dog_patrol_perception_tracking::IdentityState;
using dog_patrol_perception_tracking::MissionCoordinator;
using dog_patrol_perception_tracking::MissionEventAction;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::PerceptionMissionEvent;
using dog_patrol_perception_tracking::PrimaryState;
using dog_patrol_perception_tracking::PrimaryTargetResult;
using dog_patrol_perception_tracking::Track;

IdentityObservation TrustedPerson(
    const int semantic_id, const int raw_track_id,
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
                              const std::uint32_t state_seq = 17U,
                              const int target_id = 42) {
  MissionSnapshot mission;
  mission.phase = phase;
  mission.state_seq = state_seq;
  mission.target_id = target_id;
  return mission;
}

void ExpectSingleEvent(const std::vector<MissionEventAction> &events,
                       const PerceptionMissionEvent event, const int target_id,
                       const std::uint32_t state_seq) {
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().event, event);
  EXPECT_EQ(events.front().target_id, target_id);
  EXPECT_EQ(events.front().observed_state_seq, state_seq);
}

TEST(MissionCoordinatorTest, PublishesOnlyCurrentTrustedTargetBoxInAllowedMissionState) {
  MissionCoordinator coordinator;
  const MissionCoordinator::TimePoint now{};
  const auto output = coordinator.Update(
      {ActiveMission(), {TrustedPerson(42, 7), TrustedPerson(99, 8)},
       LockedPrimary(42, 7), now});

  ASSERT_TRUE(output.target_box.has_value());
  EXPECT_EQ(output.target_box->target_id, 42);
  EXPECT_EQ(output.target_box->observed_state_seq, 17U);
  EXPECT_EQ(output.target_box->source_time, now);
  EXPECT_TRUE(output.events.empty());
}

TEST(MissionCoordinatorTest, DoesNotPublishBoxesOutsideTargetLifecycleStates) {
  const MissionCoordinator::TimePoint now{};
  const auto identities = std::vector<IdentityObservation>{TrustedPerson(42, 7)};
  const auto primary = LockedPrimary(42, 7);
  for (const MissionPhase phase : {MissionPhase::kStartup, MissionPhase::kPatrol,
                                   MissionPhase::kRecoverPatrol}) {
    MissionCoordinator coordinator;
    auto mission = ActiveMission(phase);
    mission.target_id = phase == MissionPhase::kPatrol || phase == MissionPhase::kStartup ? 0 : 42;
    EXPECT_FALSE(coordinator.Update({mission, identities, primary, now}).target_box.has_value());
  }
}

TEST(MissionCoordinatorTest, EmitsOneTargetLostAndNeverReacquiresImplicitly) {
  MissionCoordinator::Config config;
  config.lost_event_timeout = std::chrono::milliseconds{500};
  MissionCoordinator coordinator(config);
  const MissionCoordinator::TimePoint start{};
  const auto mission = ActiveMission();
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);

  ASSERT_TRUE(coordinator.Update({mission, {trusted}, primary, start}).target_box.has_value());
  EXPECT_TRUE(coordinator.Update(
      {mission, {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{499}}).events.empty());
  ExpectSingleEvent(coordinator.Update(
      {mission, {}, PrimaryTargetResult{}, start + std::chrono::milliseconds{500}}).events,
                    PerceptionMissionEvent::kTargetLost, 42, 17U);

  const auto same_state_reappearance = coordinator.Update(
      {mission, {trusted}, primary, start + std::chrono::milliseconds{600}});
  EXPECT_TRUE(same_state_reappearance.events.empty());
  EXPECT_FALSE(same_state_reappearance.target_box.has_value());

  const auto recovery = ActiveMission(MissionPhase::kRecoverPatrol, 18U, 42);
  EXPECT_TRUE(coordinator.Update({recovery, {}, PrimaryTargetResult{},
                                  start + std::chrono::milliseconds{700}})
                  .events.empty());
  const auto next_mission = ActiveMission(MissionPhase::kConfirmTarget, 19U, 42);
  const auto resumed = coordinator.Update({next_mission, {trusted}, primary,
                                            start + std::chrono::milliseconds{800}});
  ASSERT_TRUE(resumed.target_box.has_value());
  EXPECT_EQ(resumed.target_box->observed_state_seq, 19U);
}

TEST(MissionCoordinatorTest, RejectsInvalidTimeoutAndStaleSourceConfiguration) {
  MissionCoordinator::Config config;
  config.lost_event_timeout = MissionCoordinator::Duration::zero();
  EXPECT_THROW(MissionCoordinator invalid(config), std::invalid_argument);

  MissionCoordinator coordinator;
  const auto start = MissionCoordinator::TimePoint{};
  const auto mission = ActiveMission();
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);
  ASSERT_TRUE(coordinator.Update({mission, {trusted}, primary, start}).target_box.has_value());
  EXPECT_FALSE(coordinator.Update({mission, {trusted}, primary, start}).target_box.has_value());
}

}  // namespace
