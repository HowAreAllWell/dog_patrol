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
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  struct Config {
    int lost_threshold_frames{180};
    float min_person_area_px{1000.0F};
    float max_center_jump_norm{2.0F};
    float min_area_ratio{0.25F};
    float max_area_ratio{4.0F};
    int pending_recovery_frames{3};
    Duration handled_ignore_absence{std::chrono::seconds{30}};
  };

  explicit PrimaryTargetManager(Config config);

  PrimaryTargetResult Update(const std::vector<IdentityObservation> &identities);

  // Starts a new mission patrol cycle. A positive semantic ID is temporarily
  // excluded from patrol-primary selection; perception ownership stays with
  // the detector, tracker, and identity manager.
  void ResetForPatrolCycle(int handled_semantic_id = -1);

  // Evaluates the mission-specific handled-ID policy only. Person validity,
  // confidence, and tracking continuity remain selection-layer concerns.
  bool IsMissionEligible(const IdentityObservation &identity) const;

  // Uses injected monotonic time so handled-ID absence expiry is deterministic
  // for both runtime callers and tests.
  PrimaryTargetResult UpdateForPatrol(const std::vector<IdentityObservation> &identities, TimePoint now);

  // Applies the authoritative mission transition and primary selection as one
  // public runtime operation. Returning from verification or intruder tracking
  // to a new patrol cycle marks the prior semantic target handled before the
  // first patrol frame is selected.
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
  PrimaryTargetResult UpdateInternal(const std::vector<IdentityObservation> &identities,
                                     bool require_mission_eligibility);
  bool IsVisibleIdentity(const IdentityObservation &identity) const;
  bool IsVisiblePrimarySane(const Track &track, std::string *reject_reason) const;
  Track TrackFromIdentityObservation(const IdentityObservation &identity) const;
  PrimaryTargetResult EnterPendingRecovery(const std::string &decision_reason);
  PrimaryTargetResult EnterOccluded(const std::string &decision_reason);
  PrimaryTargetResult EnterLost(const std::string &decision_reason, int missing_frames);

  Config config_;
  PrimaryTargetResult state_{};
  int primary_target_id_{-1};
  int bound_raw_track_id_{-1};
  std::optional<Track> last_primary_track_{std::nullopt};
  int pending_recovery_frames_{0};
  std::string last_decision_reason_;
  std::string last_reject_reason_;
  std::unordered_map<int, std::optional<TimePoint>> handled_identity_absence_started_at_;
  std::optional<MissionSnapshot> last_mission_for_primary_;
};

}  // namespace dog_patrol_perception_tracking
