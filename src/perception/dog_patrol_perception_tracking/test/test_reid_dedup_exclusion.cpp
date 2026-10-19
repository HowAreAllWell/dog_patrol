#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"

// ReID 去重排除与选人边界的行为回归（评审 N01/N03/N04）。
// 契约（N04，与 gate/provider 同口径）：supervisor 处置过的人当天免重复
// 处置——ReID 排除项当日有效，不随持续不可见释放；跨天（ClearReidDedup
// Exclusions）与 STARTUP/reset（ResetMissionHistory）才清除；业务 handled
// 表的 60 秒不可见释放规则不受影响。
namespace {

using namespace dog_patrol_perception_tracking;
using namespace std::chrono_literals;

dog_patrol_perception_tracking::IdentityObservation MakePerson(const int sid, const int raw_id,
                                                   const cv::Rect2f &bbox) {
  dog_patrol_perception_tracking::IdentityObservation identity;
  identity.semantic_id = sid;
  identity.state = dog_patrol_perception_tracking::IdentityState::kActive;
  identity.supporting_raw_track_id = raw_id;
  dog_patrol_perception_tracking::Track track;
  track.id = raw_id;
  track.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = bbox;
  track.is_confirmed = true;
  identity.supporting_tracklet =
      dog_patrol_perception_tracking::TrackletObservation::FromTrack(track);
  identity.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  identity.confidence = 0.9F;
  identity.bbox = bbox;
  identity.visible = true;
  identity.association.stage = "stage1_confirmed_high";
  identity.association.passed_final_cost_gate = true;
  return identity;
}

// N01：排除结论异步注入后，在位巡逻 primary 必须让位，同屏新人可被选择。
TEST(ReidDedupExclusionTest, ExclusionReleasesCurrentPatrolPrimaryForNextCandidate) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  const auto first = mgr.UpdateForPatrol({a, b}, now);
  ASSERT_EQ(first.primary_target_id, 11);

  mgr.ExcludeForReidDedup(11);
  const auto second = mgr.UpdateForPatrol({a, b}, now + 1s);
  ASSERT_TRUE(second.primary_track.has_value());
  EXPECT_EQ(second.primary_target_id, 22);
  EXPECT_EQ(second.primary_track->id, 202);
}

// N01：同屏全员被排除时进入无有效 primary 的等待状态。
TEST(ReidDedupExclusionTest, ExclusionOfEveryVisibleCandidateLeavesPatrolIdle) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  ASSERT_EQ(mgr.UpdateForPatrol({a, b}, now).primary_target_id, 11);

  mgr.ExcludeForReidDedup(11);
  mgr.ExcludeForReidDedup(22);
  const auto state = mgr.UpdateForPatrol({a, b}, now + 1s);
  EXPECT_EQ(state.state, PrimaryState::kIdle);
  EXPECT_EQ(state.primary_target_id, -1);
  EXPECT_FALSE(state.primary_track.has_value());
}

// N01：排除未持有 primary 的人不扰动当前巡逻目标。
TEST(ReidDedupExclusionTest, ExclusionOfNonPrimaryDoesNotDisturbCurrentPrimary) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  ASSERT_EQ(mgr.UpdateForPatrol({a, b}, now).primary_target_id, 11);

  mgr.ExcludeForReidDedup(22);
  const auto state = mgr.UpdateForPatrol({a, b}, now + 1s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

// N01 对照：活动目标阶段只绑定 supervisor.target_id，排除不得换人。
TEST(ReidDedupExclusionTest, ExclusionDoesNotReplaceActiveMissionTarget) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 40, 40));
  const auto larger_other = MakePerson(22, 202, cv::Rect2f(100, 0, 100, 100));
  const MissionSnapshot patrol{10U, MissionPhase::kPatrol, 0, 0};
  const MissionSnapshot approach{11U, MissionPhase::kApproachTarget, 11};

  ASSERT_EQ(mgr.UpdateForMission({a}, patrol, std::nullopt, now).primary_target_id, 11);

  mgr.ExcludeForReidDedup(11);
  const auto state =
      mgr.UpdateForMission({a, larger_other}, approach, patrol, now + 1s);
  ASSERT_EQ(state.state, PrimaryState::kLocked);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

// ReID 排除不受目标是否持续可见、离场时长或新巡逻周期影响。
TEST(ReidDedupExclusionTest, ExclusionSurvivesAbsenceAndNewPatrolCycle) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint start{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  ASSERT_EQ(mgr.UpdateForPatrol({a, b}, start).primary_target_id, 11);

  mgr.ExcludeForReidDedup(11);
  mgr.UpdateForPatrol({}, start + 1s);
  mgr.UpdateForPatrol({}, start + 61s);
  EXPECT_FALSE(mgr.IsMissionEligible(a));

  mgr.ResetForPatrolCycle();
  const auto state = mgr.UpdateForPatrol({a, b}, start + 62s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 22);
}

// 跨天清理让所有被 ReID 排除的身份重新具备巡逻选择资格。
TEST(ReidDedupExclusionTest, ClearReidDedupExclusionsRestoresEligibility) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  mgr.ExcludeForReidDedup(22);
  mgr.ExcludeForReidDedup(11);
  mgr.ClearReidDedupExclusions();

  EXPECT_TRUE(mgr.IsMissionEligible(a));
  EXPECT_TRUE(mgr.IsMissionEligible(b));
  const auto state = mgr.UpdateForPatrol({a, b}, now);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

// reset（重进 STARTUP）清除 ReID 排除项，被排除者恢复可选。
TEST(ReidDedupExclusionTest, StartupResetClearsReidDedupExclusions) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint now{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const MissionSnapshot patrol{10U, MissionPhase::kPatrol, 0, 0};
  const MissionSnapshot startup{20U, MissionPhase::kStartup, 0, 0};

  ASSERT_EQ(mgr.UpdateForMission({a}, patrol, std::nullopt, now).primary_target_id, 11);
  mgr.ExcludeForReidDedup(11);
  EXPECT_FALSE(mgr.IsMissionEligible(a));

  mgr.UpdateForMission({a}, startup, patrol, now + 1s);
  const MissionSnapshot patrol_again{30U, MissionPhase::kPatrol, 0, 0};
  const auto state = mgr.UpdateForMission({a}, patrol_again, startup, now + 2s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

}  // namespace
