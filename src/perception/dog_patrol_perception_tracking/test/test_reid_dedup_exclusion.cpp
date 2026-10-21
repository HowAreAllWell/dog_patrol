#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"

// ReID 去重排除与选人边界的行为回归（评审 N01/N03/N04 + 重复核验忽略时长）。
// 契约（与 gate/provider 同口径）：核验通过的人在"重复核验忽略时长"内免
// 重复核验——ReID 排除项携带有效期截止，到期在巡逻选人边界自动释放（同人
// 重新可选、重新核验）；不随持续不可见释放，跨天持续（资格由截止时刻
// 治理，无按日清理）；STARTUP/reset（ResetMissionHistory）提前全清。
namespace {

using namespace dog_patrol_perception_tracking;
using namespace std::chrono_literals;

// 测试辅助：相对本文件假时间轴（epoch 附近）的排除截止时刻；默认给足
// 余量等效"长忽略时长"，到期释放行为单独用例覆盖
PrimaryTargetManager::TimePoint ExcludedUntil() {
  return PrimaryTargetManager::TimePoint{} + 24h;
}

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

  mgr.ExcludeForReidDedup(11, ExcludedUntil());
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

  mgr.ExcludeForReidDedup(11, ExcludedUntil());
  mgr.ExcludeForReidDedup(22, ExcludedUntil());
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

  mgr.ExcludeForReidDedup(22, ExcludedUntil());
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

  mgr.ExcludeForReidDedup(11, ExcludedUntil());
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

  mgr.ExcludeForReidDedup(11, ExcludedUntil());
  mgr.UpdateForPatrol({}, start + 1s);
  mgr.UpdateForPatrol({}, start + 61s);
  EXPECT_FALSE(mgr.IsMissionEligible(a));

  mgr.ResetForPatrolCycle();
  const auto state = mgr.UpdateForPatrol({a, b}, start + 62s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 22);
}

// "重复核验忽略时长"到期：排除在巡逻选人边界自动释放，A 恢复选人资格；
// 在位的可见 primary（B）不因此被抢占（保持 merge 的选人连续性），B 离开
// 后 A 重新当选。
TEST(ReidDedupExclusionTest, ExclusionExpiresAtDeadlineAndPersonIsReselectable) {
  PrimaryTargetManager::Config cfg;
  cfg.min_person_area_px = 100.0F;
  PrimaryTargetManager mgr(cfg);
  const PrimaryTargetManager::TimePoint start{};

  const auto a = MakePerson(11, 101, cv::Rect2f(0, 0, 80, 80));
  const auto b = MakePerson(22, 202, cv::Rect2f(100, 0, 60, 60));

  ASSERT_EQ(mgr.UpdateForPatrol({a, b}, start).primary_target_id, 11);

  const auto ignore_until = start + 60s;  // 忽略时长 60 秒
  mgr.ExcludeForReidDedup(11, ignore_until);
  EXPECT_EQ(mgr.UpdateForPatrol({a, b}, start + 1s).primary_target_id, 22);
  EXPECT_EQ(mgr.UpdateForPatrol({a, b}, start + 59s).primary_target_id, 22);
  EXPECT_FALSE(mgr.IsMissionEligible(a));

  // 到期释放：A 资格恢复；B 仍可见且在位，保持为 primary（不按面积抢占；
  // B 单帧消失只进入遮挡等待，不触发换选——与选人保持逻辑一致）
  const auto state = mgr.UpdateForPatrol({a, b}, start + 60s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 22);
  EXPECT_TRUE(mgr.IsMissionEligible(a));

  // 新巡逻周期重选：资格恢复的 A 按面积重新当选
  mgr.ResetForPatrolCycle();
  const auto reselected = mgr.UpdateForPatrol({a, b}, start + 61s);
  ASSERT_TRUE(reselected.primary_track.has_value());
  EXPECT_EQ(reselected.primary_target_id, 11);
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
  mgr.ExcludeForReidDedup(11, ExcludedUntil());
  EXPECT_FALSE(mgr.IsMissionEligible(a));

  mgr.UpdateForMission({a}, startup, patrol, now + 1s);
  const MissionSnapshot patrol_again{30U, MissionPhase::kPatrol, 0, 0};
  const auto state = mgr.UpdateForMission({a}, patrol_again, startup, now + 2s);
  ASSERT_TRUE(state.primary_track.has_value());
  EXPECT_EQ(state.primary_target_id, 11);
}

}  // namespace
