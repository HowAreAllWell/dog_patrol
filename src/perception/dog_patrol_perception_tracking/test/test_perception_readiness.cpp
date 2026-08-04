#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"

namespace {

using dog_patrol_perception_tracking::DetectionTrackingReadinessContributor;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::MutableReadinessContributor;
using dog_patrol_perception_tracking::PerceptionReadiness;
using dog_patrol_perception_tracking::PerceptionReadinessAggregator;
using dog_patrol_perception_tracking::PlaceholderReadinessContributor;

MissionSnapshot StartupMission(const std::uint32_t state_seq) {
  MissionSnapshot mission;
  mission.phase = MissionPhase::kStartup;
  mission.state_seq = state_seq;
  return mission;
}

TEST(PerceptionReadinessAggregatorTest, EmitsOnceWhenRealAndExplicitPlaceholderContributorsAreReady) {
  PerceptionReadinessAggregator aggregator;
  auto detection_tracking = std::make_unique<DetectionTrackingReadinessContributor>();
  DetectionTrackingReadinessContributor *const runtime = detection_tracking.get();
  aggregator.AddRequiredContributor(std::move(detection_tracking));
  aggregator.AddRequiredContributor(std::make_unique<PlaceholderReadinessContributor>(
      "authorization", "dog_patrol authorization module", "ReplaceRequiredContributor", PerceptionReadiness::kReady));
  aggregator.AddRequiredContributor(std::make_unique<PlaceholderReadinessContributor>(
      "liveness", "future liveness module", "ReplaceRequiredContributor", PerceptionReadiness::kReady));

  const auto startup = StartupMission(31U);
  EXPECT_FALSE(aggregator.Update(startup).ready.has_value());

  runtime->ReportRuntimeStatus({true, false, {}});
  EXPECT_FALSE(aggregator.Update(startup).ready.has_value());

  runtime->ReportRuntimeStatus({true, true, {}});
  const auto ready = aggregator.Update(startup);
  ASSERT_TRUE(ready.ready.has_value());
  EXPECT_EQ(ready.ready->observed_state_seq, 31U);

  EXPECT_FALSE(aggregator.Update(startup).ready.has_value());
}

TEST(PerceptionReadinessAggregatorTest, FailureOrNotReadyBlocksAStartupSequenceUntilRuntimeRecovers) {
  PerceptionReadinessAggregator aggregator;
  auto detection_tracking = std::make_unique<DetectionTrackingReadinessContributor>();
  DetectionTrackingReadinessContributor *const runtime = detection_tracking.get();
  runtime->ReportRuntimeStatus({true, true, {}});
  aggregator.AddRequiredContributor(std::move(detection_tracking));
  aggregator.AddRequiredContributor(std::make_unique<PlaceholderReadinessContributor>(
      "authorization", "dog_patrol authorization module", "ReplaceRequiredContributor", PerceptionReadiness::kReady));

  ASSERT_TRUE(aggregator.Update(StartupMission(31U)).ready.has_value());

  runtime->ReportRuntimeStatus({true, true, "tracker no longer accepts frames"});
  EXPECT_EQ(runtime->Contribution().readiness, PerceptionReadiness::kFailure);
  EXPECT_EQ(runtime->Contribution().detail, "tracker no longer accepts frames");
  EXPECT_FALSE(aggregator.Update(StartupMission(32U)).ready.has_value());

  runtime->ReportRuntimeStatus({true, false, {}});
  EXPECT_EQ(runtime->Contribution().readiness, PerceptionReadiness::kNotReady);
  EXPECT_FALSE(aggregator.Update(StartupMission(32U)).ready.has_value());

  runtime->ReportRuntimeStatus({true, true, {}});
  const auto recovered = aggregator.Update(StartupMission(32U));
  ASSERT_TRUE(recovered.ready.has_value());
  EXPECT_EQ(recovered.ready->observed_state_seq, 32U);
}

TEST(PerceptionReadinessAggregatorTest, EmitsOnlyForCurrentStartupMissionSequence) {
  PerceptionReadinessAggregator aggregator;
  auto detection_tracking = std::make_unique<DetectionTrackingReadinessContributor>();
  detection_tracking->ReportRuntimeStatus({true, true, {}});
  aggregator.AddRequiredContributor(std::move(detection_tracking));
  aggregator.AddRequiredContributor(std::make_unique<PlaceholderReadinessContributor>(
      "authorization", "dog_patrol authorization module", "ReplaceRequiredContributor", PerceptionReadiness::kReady));

  MissionSnapshot patrol = StartupMission(40U);
  patrol.phase = MissionPhase::kPatrol;
  EXPECT_FALSE(aggregator.Update(patrol).ready.has_value());

  EXPECT_FALSE(aggregator.Update(StartupMission(39U)).ready.has_value());

  const auto startup = aggregator.Update(StartupMission(41U));
  ASSERT_TRUE(startup.ready.has_value());
  EXPECT_EQ(startup.ready->observed_state_seq, 41U);
}

TEST(PerceptionReadinessAggregatorTest, ReplacesAPlaceholderWithARealContributorWithoutPolicyChanges) {
  PerceptionReadinessAggregator aggregator;
  auto detection_tracking = std::make_unique<DetectionTrackingReadinessContributor>();
  detection_tracking->ReportRuntimeStatus({true, true, {}});
  aggregator.AddRequiredContributor(std::move(detection_tracking));
  aggregator.AddRequiredContributor(std::make_unique<PlaceholderReadinessContributor>(
      "authorization", "dog_patrol authorization module", "ReplaceRequiredContributor", PerceptionReadiness::kNotReady));

  const auto startup = StartupMission(51U);
  EXPECT_FALSE(aggregator.Update(startup).ready.has_value());

  EXPECT_TRUE(aggregator.ReplaceRequiredContributor(
      "authorization", std::make_unique<MutableReadinessContributor>(
                           "authorization", PerceptionReadiness::kReady, "authorization runtime ready")));
  const auto ready = aggregator.Update(startup);
  ASSERT_TRUE(ready.ready.has_value());
  EXPECT_EQ(ready.ready->observed_state_seq, 51U);
}

}  // namespace
