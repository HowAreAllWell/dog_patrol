#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "dog_patrol_perception_tracking/modules/mission_frame_transaction.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityObservation;
using dog_patrol_perception_tracking::IdentityState;
using dog_patrol_perception_tracking::MissionCoordinator;
using dog_patrol_perception_tracking::MissionFrameTransaction;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::PerceptionMissionEvent;
using dog_patrol_perception_tracking::PrimaryState;
using dog_patrol_perception_tracking::SourceFrameMetadata;

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

MissionSnapshot Mission(const MissionPhase phase, const std::uint32_t state_seq,
                        const int target_id = 0, const int handled_target_id = 0) {
  MissionSnapshot mission;
  mission.phase = phase;
  mission.state_seq = state_seq;
  mission.target_id = target_id;
  mission.handled_target_id = handled_target_id;
  return mission;
}

SourceFrameMetadata Metadata() {
  SourceFrameMetadata metadata;
  metadata.source_timestamp_ns = 1710000000000000000ULL;
  metadata.image_width = 640;
  metadata.image_height = 480;
  metadata.optical_frame_id = "hik_camera_optical_frame";
  return metadata;
}

MissionFrameTransaction ConfiguredTransaction() {
  MissionFrameTransaction::Config config;
  config.primary.min_person_area_px = 100.0F;
  return MissionFrameTransaction(config);
}

TEST(MissionFrameTransactionTest, ThrottlesDuplicatePatrolConfirmation) {
  auto transaction = ConfiguredTransaction();
  const auto patrol = Mission(MissionPhase::kPatrol, 101U);
  const auto source_time = MissionCoordinator::TimePoint{};
  const std::vector<IdentityObservation> identities{
      TrustedPerson(42, 7, cv::Rect2f{10.0F, 10.0F, 20.0F, 20.0F}),
      TrustedPerson(99, 8, cv::Rect2f{50.0F, 10.0F, 40.0F, 40.0F})};

  const auto first = transaction.Update({patrol, std::nullopt, identities, source_time, Metadata()});

  EXPECT_EQ(first.primary.state, PrimaryState::kLocked);
  EXPECT_EQ(first.primary.primary_target_id, 99);
  ASSERT_EQ(first.events.size(), 1U);
  EXPECT_EQ(first.events.front().event, PerceptionMissionEvent::kTargetConfirmed);
  EXPECT_EQ(first.events.front().target_id, 99);
  EXPECT_EQ(first.events.front().observed_state_seq, 101U);
  EXPECT_FALSE(first.target_box.has_value());

  const auto duplicate = transaction.Update(
      {patrol, std::nullopt, identities, source_time + std::chrono::milliseconds{1}, Metadata()});

  EXPECT_EQ(duplicate.primary.primary_target_id, 99);
  EXPECT_TRUE(duplicate.events.empty());
  EXPECT_FALSE(duplicate.target_box.has_value());
}

TEST(MissionFrameTransactionTest, RetriesPatrolConfirmationWhenStateHasNotAdvanced) {
  auto transaction = ConfiguredTransaction();
  const auto startup = Mission(MissionPhase::kStartup, 100U);
  const auto patrol = Mission(MissionPhase::kPatrol, 101U);
  const std::vector<IdentityObservation> identities{TrustedPerson(42, 7)};
  const auto start = MissionCoordinator::TimePoint{};

  transaction.Update({startup, std::nullopt, identities, start, Metadata()});
  const auto first = transaction.Update({patrol, startup, identities, start + std::chrono::milliseconds{1}, Metadata()});
  ASSERT_EQ(first.events.size(), 1U);
  EXPECT_EQ(first.events.front().event, PerceptionMissionEvent::kTargetConfirmed);

  const auto before_retry = transaction.Update(
      {patrol, startup, identities, start + std::chrono::milliseconds{50}, Metadata()});
  EXPECT_TRUE(before_retry.events.empty());

  const auto retry = transaction.Update(
      {patrol, startup, identities, start + std::chrono::milliseconds{101}, Metadata()});
  ASSERT_EQ(retry.events.size(), 1U);
  EXPECT_EQ(retry.events.front().event, PerceptionMissionEvent::kTargetConfirmed);
  EXPECT_EQ(retry.events.front().observed_state_seq, 101U);
}

TEST(MissionFrameTransactionTest, TreatsUnrepresentableTargetBoxAsMissing) {
  auto transaction = ConfiguredTransaction();
  const auto confirm = Mission(MissionPhase::kConfirmTarget, 17U, 42);
  const auto source_time = MissionCoordinator::TimePoint{};
  const std::vector<IdentityObservation> trusted{TrustedPerson(42, 7)};

  const auto fresh = transaction.Update({confirm, std::nullopt, trusted, source_time, Metadata()});

  ASSERT_TRUE(fresh.target_box.has_value());
  EXPECT_EQ(fresh.target_box->target_id, 42);
  EXPECT_TRUE(fresh.events.empty());

  const std::vector<IdentityObservation> off_image{
      TrustedPerson(42, 7, cv::Rect2f{700.0F, 2.0F, 4.0F, 4.0F})};
  const auto lost = transaction.Update(
      {confirm, std::nullopt, off_image, source_time + std::chrono::seconds{10}, Metadata()});

  EXPECT_FALSE(lost.target_box.has_value());
  ASSERT_EQ(lost.events.size(), 1U);
  EXPECT_EQ(lost.events.front().event, PerceptionMissionEvent::kTargetLost);
  EXPECT_EQ(lost.events.front().target_id, 42);
  EXPECT_EQ(lost.events.front().observed_state_seq, 17U);
}

TEST(MissionFrameTransactionTest, SkipsHandledTargetWhenMissionReturnsToPatrol) {
  auto transaction = ConfiguredTransaction();
  const auto verify = Mission(MissionPhase::kVerifyIdentity, 200U, 42);
  const auto patrol = Mission(MissionPhase::kPatrol, 201U, 0, 42);
  const auto source_time = MissionCoordinator::TimePoint{};
  const std::vector<IdentityObservation> current_target{
      TrustedPerson(42, 7, cv::Rect2f{10.0F, 10.0F, 80.0F, 80.0F})};

  const auto verification = transaction.Update(
      {verify, std::nullopt, current_target, source_time, Metadata()});
  ASSERT_TRUE(verification.target_box.has_value());
  EXPECT_EQ(verification.primary.primary_target_id, 42);

  const std::vector<IdentityObservation> patrol_candidates{
      TrustedPerson(42, 7, cv::Rect2f{10.0F, 10.0F, 80.0F, 80.0F}),
      TrustedPerson(99, 8, cv::Rect2f{120.0F, 10.0F, 30.0F, 30.0F})};
  const auto selection = transaction.Update(
      {patrol, verify, patrol_candidates, source_time + std::chrono::milliseconds{100}, Metadata()});

  EXPECT_EQ(selection.primary.state, PrimaryState::kLocked);
  EXPECT_EQ(selection.primary.primary_target_id, 99);
  ASSERT_EQ(selection.events.size(), 1U);
  EXPECT_EQ(selection.events.front().event, PerceptionMissionEvent::kTargetConfirmed);
  EXPECT_EQ(selection.events.front().target_id, 99);
  EXPECT_FALSE(selection.target_box.has_value());
}

TEST(MissionFrameTransactionTest, FailedPeopleRemainSelectableAfterAnotherPersonWasHandled) {
  using namespace std::chrono_literals;
  MissionFrameTransaction::Config config;
  config.primary.handled_ignore_absence = 60s;
  MissionFrameTransaction transaction(config);
  const auto start = MissionCoordinator::TimePoint{};
  const auto a = TrustedPerson(11, 101, cv::Rect2f{10, 10, 80, 80});
  const auto b = TrustedPerson(22, 202, cv::Rect2f{120, 10, 70, 70});
  const auto c = TrustedPerson(33, 303, cv::Rect2f{230, 10, 60, 60});
  const std::vector<IdentityObservation> people{a, b, c};
  const auto verify = Mission(MissionPhase::kVerifyIdentity, 100U, 11);
  const auto authorized_recovery = Mission(MissionPhase::kRecoverPatrol, 101U, 11, 11);
  const auto patrol_after_a = Mission(MissionPhase::kPatrol, 102U, 0, 11);
  transaction.Update({verify, std::nullopt, people, start, Metadata()});
  transaction.Update({authorized_recovery, verify, people, start + 1s, Metadata()});
  auto output = transaction.Update(
      {patrol_after_a, authorized_recovery, people, start + 2s, Metadata()});
  ASSERT_EQ(output.primary.primary_target_id, 22);
  ASSERT_EQ(output.events.size(), 1U);
  EXPECT_EQ(output.events.front().target_id, 22);

  const auto confirm_b = Mission(MissionPhase::kConfirmTarget, 103U, 22);
  const auto failed_b = Mission(MissionPhase::kRecoverPatrol, 104U, 22);
  const auto patrol_after_b = Mission(MissionPhase::kPatrol, 105U);
  transaction.Update({confirm_b, patrol_after_a, people, start + 3s, Metadata()});
  transaction.Update({failed_b, confirm_b, people, start + 8s, Metadata()});
  output = transaction.Update({patrol_after_b, failed_b, people, start + 9s, Metadata()});
  // A remains excluded; failed B is retried instead of becoming handled.
  EXPECT_EQ(output.primary.primary_target_id, 22);
  ASSERT_EQ(output.events.size(), 1U);
  EXPECT_EQ(output.events.front().event, PerceptionMissionEvent::kTargetConfirmed);
  EXPECT_EQ(output.events.front().target_id, 22);

  // B leaves on the next failed attempt; C is eligible as well.
  const auto confirm_b_again = Mission(MissionPhase::kConfirmTarget, 106U, 22);
  const auto failed_b_again = Mission(MissionPhase::kRecoverPatrol, 107U, 22);
  const auto patrol_for_c = Mission(MissionPhase::kPatrol, 108U);
  const std::vector<IdentityObservation> without_b{a, c};
  transaction.Update({confirm_b_again, patrol_after_b, people, start + 10s, Metadata()});
  transaction.Update({failed_b_again, confirm_b_again, without_b, start + 15s, Metadata()});
  output = transaction.Update({patrol_for_c, failed_b_again, without_b, start + 16s, Metadata()});
  ASSERT_EQ(output.primary.primary_target_id, 33);
  const auto confirm_c = Mission(MissionPhase::kConfirmTarget, 109U, 33);
  const auto failed_c = Mission(MissionPhase::kRecoverPatrol, 110U, 33);
  const auto final_patrol = Mission(MissionPhase::kPatrol, 111U);
  transaction.Update({confirm_c, patrol_for_c, without_b, start + 17s, Metadata()});
  transaction.Update({failed_c, confirm_c, without_b, start + 22s, Metadata()});
  output = transaction.Update({final_patrol, failed_c, people, start + 23s, Metadata()});
  EXPECT_EQ(output.primary.state, PrimaryState::kLocked);
  EXPECT_EQ(output.primary.primary_target_id, 22);
  ASSERT_EQ(output.events.size(), 1U);
  EXPECT_EQ(output.events.front().target_id, 22);
  // Without B, the same fresh patrol outcome still permits C.
  const auto retry_b = Mission(MissionPhase::kConfirmTarget, 112U, 22);
  const auto recovery_b = Mission(MissionPhase::kRecoverPatrol, 113U, 22);
  transaction.Update({retry_b, final_patrol, people, start + 24s, Metadata()});
  transaction.Update({recovery_b, retry_b, without_b, start + 29s, Metadata()});
  output = transaction.Update({Mission(MissionPhase::kPatrol, 114U), recovery_b,
                               without_b, start + 30s, Metadata()});
  EXPECT_EQ(output.primary.primary_target_id, 33);
}

}  // namespace
