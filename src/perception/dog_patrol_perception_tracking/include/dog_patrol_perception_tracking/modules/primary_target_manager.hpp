#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class PrimaryTargetManager {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Config {
    int lost_threshold_frames{100};
    float min_person_area_px{1000.0F};
    float max_center_jump_norm{2.0F};
    float min_area_ratio{0.25F};
    float max_area_ratio{4.0F};
    int pending_recovery_frames{3};
  };

  explicit PrimaryTargetManager(Config config);

  PrimaryTargetResult Update(const std::vector<IdentityObservation> &identities);

  // Starts a new mission patrol cycle without changing ReID eligibility.
  void ResetForPatrolCycle();

  // ReID 去重排除（seen+handled 的外观结论，评审 R04/N04）：仅影响巡逻选人
  // 资格；有效期截止时刻由 provider"重复核验忽略时长"的剩余量决定（随
  // gate 排除结论注入），到期在巡逻选人边界自动释放（同人重新可选、重新
  // 核验）；跨天（N03）与 STARTUP/reset 提前全清。不随持续不可见释放。
  // 由帧处理路径经 MissionFrameTransaction 注入。
  void ExcludeForReidDedup(int semantic_id, TimePoint valid_until);

  // Clears the current primary and ReID exclusion history. This is used
  // only for STARTUP/reset or a full process session boundary.
  void ResetMissionHistory();

  // Evaluates ReID dedup eligibility only. Person validity,
  // confidence, and tracking continuity remain selection-layer concerns.
  bool IsMissionEligible(const IdentityObservation &identity) const;

  // Patrol eligibility is controlled by ReID, without a local absence timer.
  PrimaryTargetResult UpdateForPatrol(const std::vector<IdentityObservation> &identities, TimePoint now);

  // Applies the authoritative mission transition and primary selection as one
  // public runtime operation. A new patrol cycle releases the previous primary;
  // successful-verification results are forwarded to ReID by the runtime.
  PrimaryTargetResult UpdateForMission(
      const std::vector<IdentityObservation> &identities,
      const std::optional<MissionSnapshot> &mission,
      const std::optional<MissionSnapshot> &previous_mission,
      TimePoint now);

  PrimaryTargetResult GetState() const;
  const std::string &LastDecisionReason() const { return last_decision_reason_; }
  const std::string &LastRejectReason() const { return last_reject_reason_; }

 private:
  std::optional<IdentityObservation> FindVisibleIdentityBySemanticId(
      const std::vector<IdentityObservation> &identities, int semantic_id) const;
  std::optional<IdentityObservation> FindIdentityBySemanticId(
      const std::vector<IdentityObservation> &identities, int semantic_id) const;
  std::optional<IdentityObservation> SelectLargestValidPersonIdentity(
      const std::vector<IdentityObservation> &identities, bool require_mission_eligibility) const;
  PrimaryTargetResult UpdateInternal(const std::vector<IdentityObservation> &identities,
                                     bool require_mission_eligibility);
  PrimaryTargetResult UpdateForActiveMissionTarget(
      const std::vector<IdentityObservation> &identities, int target_id);
  bool IsVisibleIdentity(const IdentityObservation &identity) const;
  bool IsVisiblePrimarySane(const Track &track, std::string *reject_reason) const;
  bool IsGeometryConsistent(const Track &reference, const Track &candidate) const;
  bool ObserveStableGeometryRecovery(const Track &track);
  PrimaryTargetResult LockPrimary(const Track &track, int semantic_id,
                                  const std::string &decision_reason);
  Track TrackFromIdentityObservation(const IdentityObservation &identity) const;
  PrimaryTargetResult EnterPendingRecovery(const std::string &decision_reason);
  PrimaryTargetResult EnterOccluded(const std::string &decision_reason);
  PrimaryTargetResult EnterLost(const std::string &decision_reason, int missing_frames);
  void ResetCurrentPrimary();

  Config config_;
  PrimaryTargetResult state_{};
  int primary_target_id_{-1};
  int bound_raw_track_id_{-1};
  std::optional<Track> last_primary_track_{std::nullopt};
  int pending_recovery_frames_{0};
  std::optional<Track> pending_geometry_candidate_{std::nullopt};
  int pending_geometry_stable_frames_{0};
  std::string last_decision_reason_;
  std::string last_reject_reason_;
  // ReID 去重排除：值 = 排除有效期截止（provider"重复核验忽略时长"随 gate
  // 结论注入；UpdateForPatrol 每帧先释放到期项，同人重新可选、重新核验）
  std::unordered_map<int, TimePoint> reid_dedup_excluded_until_;
  std::optional<MissionSnapshot> last_mission_for_primary_;
};

}  // namespace dog_patrol_perception_tracking
