#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "vision_demo_host/modules/mission_frame_transaction.hpp"

namespace {

using vision_demo_host::ClassId;
using vision_demo_host::IdentityObservation;
using vision_demo_host::IdentityState;
using vision_demo_host::MissionBlockCause;
using vision_demo_host::MissionCoordinator;
using vision_demo_host::MissionFrameTransaction;
using vision_demo_host::MissionPhase;
using vision_demo_host::MissionSnapshot;
using vision_demo_host::PerceptionMissionEvent;
using vision_demo_host::PrimaryState;
using vision_demo_host::SourceFrameMetadata;

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
                        const int target_id = 0) {
  MissionSnapshot mission;
  mission.phase = phase;
  mission.state_seq = state_seq;
  mission.target_id = target_id;
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

TEST(MissionFrameTransactionTest, ConfirmsLargestPatrolTargetOnce) {
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
      {confirm, std::nullopt, off_image, source_time + std::chrono::milliseconds{500}, Metadata()});

  EXPECT_FALSE(lost.target_box.has_value());
  ASSERT_EQ(lost.events.size(), 1U);
  EXPECT_EQ(lost.events.front().event, PerceptionMissionEvent::kTargetLost);
  EXPECT_EQ(lost.events.front().target_id, 42);
  EXPECT_EQ(lost.events.front().observed_state_seq, 17U);
}

TEST(MissionFrameTransactionTest, SkipsHandledTargetWhenMissionReturnsToPatrol) {
  auto transaction = ConfiguredTransaction();
  const auto verify = Mission(MissionPhase::kVerifyIdentity, 200U, 42);
  const auto patrol = Mission(MissionPhase::kPatrol, 201U);
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

}  // namespace
