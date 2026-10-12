#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class PrimaryTargetManager {
 public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  struct Config {
    int lost_threshold_frames{100};
    float min_person_area_px{1000.0F};
    float max_center_jump_norm{2.0F};
    float min_area_ratio{0.25F};
    float max_area_ratio{4.0F};
    int pending_recovery_frames{3};
    Duration handled_ignore_absence{std::chrono::seconds{60}};
  };

  explicit PrimaryTargetManager(Config config);

  PrimaryTargetResult Update(const std::vector<IdentityObservation> &identities);

  // Starts a new mission patrol cycle. A positive semantic ID is temporarily
  // excluded from patrol-primary selection; perception ownership stays with
  // the detector, tracker, and identity manager.
  void ResetForPatrolCycle(int handled_semantic_id = -1);

  // ReID 去重排除（seen+handled 的外观结论，评审 R04/N04）：仅影响巡逻选人
  // 资格；当日有效（契约：处置过的人当天免重复处置），不随持续不可见释放，
  // 仅跨天或 STARTUP/reset 清除；业务 handled 表的 60 秒释放规则不受影响。
  // 由帧处理路径经 MissionFrameTransaction 注入。
  void ExcludeForReidDedup(int semantic_id);

  // 跨天失效选人层的 ReID 排除项（评审 N03）：与 gate 锁存/provider 当日库
  // 的日期清零同步。业务 handled 表按原有规则维护，不在此清理。
  void ClearReidDedupExclusions();

  // Clears all mission-owned primary and handled-target history. This is used
  // only for STARTUP/reset or a full process session boundary.
  void ResetMissionHistory();

  // Evaluates the mission-specific handled-ID policy only. Person validity,
  // confidence, and tracking continuity remain selection-layer concerns.
  bool IsMissionEligible(const IdentityObservation &identity) const;

  // Uses injected monotonic time so handled-ID absence expiry is deterministic
  // for both runtime callers and tests.
  PrimaryTargetResult UpdateForPatrol(const std::vector<IdentityObservation> &identities, TimePoint now);

  // Applies the authoritative mission transition and primary selection as one
  // public runtime operation. A new patrol cycle applies the current mission's
  // explicit handled_target_id before the first frame is selected. Previous
  // phases alone do not establish a business result.
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
  void UpdateHandledIdentityAbsence(const std::vector<IdentityObservation> &identities, TimePoint now);
  // 业务 handled 表的"持续不可见释放"扫描（ReID 排除项当日有效，不参与）
  void UpdateAbsenceTable(std::unordered_map<int, std::optional<TimePoint>> &table,
                          const std::vector<IdentityObservation> &identities, TimePoint now);
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
  std::unordered_map<int, std::optional<TimePoint>> handled_identity_absence_started_at_;
  // ReID 去重排除（当日有效）：无需不可见计时，值即"今日已免处置"事实
  std::unordered_set<int> reid_dedup_excluded_ids_;
  std::optional<MissionSnapshot> last_mission_for_primary_;
};

}  // namespace dog_patrol_perception_tracking
