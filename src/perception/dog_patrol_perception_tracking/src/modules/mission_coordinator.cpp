#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"

#include <cmath>
#include <stdexcept>

namespace dog_patrol_perception_tracking {
MissionCoordinator::MissionCoordinator() : MissionCoordinator(Config{}) {}

MissionCoordinator::MissionCoordinator(Config config) : config_(config) {
  if (config_.lost_event_timeout <= Duration::zero()) {
    throw std::invalid_argument("lost_event_timeout must be positive");
  }
}

bool MissionCoordinator::AcceptsFreshTargetBox(const MissionPhase phase) {
  switch (phase) {
    case MissionPhase::kConfirmTarget:
    case MissionPhase::kApproachTarget:
    case MissionPhase::kVerifyIdentity:
    case MissionPhase::kTrackIntruder:
      return true;
    case MissionPhase::kStartup:
    case MissionPhase::kPatrol:
    case MissionPhase::kRecoverPatrol:
      return false;
  }
  return false;
}

bool MissionCoordinator::IsCurrentMissionState(const MissionSnapshot &mission) {
  return state_sequence_.AcceptsCurrentOrNewer(mission.state_seq);
}

bool MissionCoordinator::IsCurrentSourceTime(const TimePoint source_time) const {
  return !latest_source_time_.has_value() || source_time >= latest_source_time_.value();
}

bool MissionCoordinator::IsTrustedCurrentObservation(const FrameInput &input,
                                                      const IdentityObservation **observation) const {
  if (!IsTargetLifecycleActive(input.mission) || input.primary.state != PrimaryState::kLocked ||
      input.primary.primary_target_id != input.mission.target_id ||
      !input.primary.primary_track.has_value()) {
    return false;
  }

  const Track &primary_track = input.primary.primary_track.value();
  if (!primary_track.authoritative || primary_track.class_id != ClassId::kPerson ||
      !primary_track.is_confirmed || input.primary.raw_track_id != primary_track.id) {
    return false;
  }

  for (const auto &identity : input.identities) {
    if (identity.semantic_id != input.mission.target_id || identity.state != IdentityState::kActive ||
        !identity.visible || identity.class_id != ClassId::kPerson ||
        !identity.supporting_raw_track_id.has_value() ||
        identity.supporting_raw_track_id.value() != primary_track.id || identity.bbox.width <= 0.0F ||
        identity.bbox.height <= 0.0F || !std::isfinite(identity.bbox.x) || !std::isfinite(identity.bbox.y) ||
        !std::isfinite(identity.bbox.width) || !std::isfinite(identity.bbox.height)) {
      continue;
    }
    if (observation != nullptr) {
      *observation = &identity;
    }
    return true;
  }
  return false;
}

bool MissionCoordinator::IsTargetLifecycleActive(const MissionSnapshot &mission) const {
  return mission.target_id > 0 && AcceptsFreshTargetBox(mission.phase);
}

bool MissionCoordinator::CanPublishForSourceTime(const TimePoint source_time) const {
  return !last_published_source_time_.has_value() || source_time > last_published_source_time_.value();
}

void MissionCoordinator::ResetMissionTarget() {
  tracked_target_id_.reset();
  last_fresh_observation_at_.reset();
  loss_event_state_seq_.reset();
}

MissionCoordinator::Output MissionCoordinator::Update(const FrameInput &input) {
  Output output;
  if (!IsCurrentMissionState(input.mission) || !IsCurrentSourceTime(input.source_time)) {
    return output;
  }
  latest_source_time_ = input.source_time;

  if (input.mission.phase == MissionPhase::kPatrol || input.mission.phase == MissionPhase::kStartup ||
      input.mission.phase == MissionPhase::kRecoverPatrol ||
      input.mission.target_id <= 0) {
    ResetMissionTarget();
    return output;
  }

  // A final loss closes this target lifecycle. Do not let a late reappearance
  // publish a new bbox before the supervisor has advanced the mission state.
  if (loss_event_state_seq_.has_value() &&
      loss_event_state_seq_.value() != input.mission.state_seq) {
    ResetMissionTarget();
  }

  if (tracked_target_id_.has_value() && tracked_target_id_.value() != input.mission.target_id) {
    ResetMissionTarget();
  }

  const IdentityObservation *trusted_observation = nullptr;
  const bool has_trusted_observation = IsTrustedCurrentObservation(input, &trusted_observation);

  if (has_trusted_observation && loss_event_state_seq_.has_value() &&
      loss_event_state_seq_.value() == input.mission.state_seq) {
    return output;
  }

  if (!has_trusted_observation) {
    if (!has_trusted_observation && tracked_target_id_.has_value() &&
        tracked_target_id_.value() == input.mission.target_id && last_fresh_observation_at_.has_value() &&
        input.source_time >= last_fresh_observation_at_.value() &&
        input.source_time - last_fresh_observation_at_.value() >= config_.lost_event_timeout &&
        loss_event_state_seq_ != input.mission.state_seq) {
      loss_event_state_seq_ = input.mission.state_seq;
      output.events.push_back(
          {PerceptionMissionEvent::kTargetLost, input.mission.target_id, input.mission.state_seq});
    }
    return output;
  }

  tracked_target_id_ = input.mission.target_id;
  last_fresh_observation_at_ = input.source_time;
  if (CanPublishForSourceTime(input.source_time)) {
    output.target_box = FreshTargetBoxAction{input.mission.target_id, input.mission.state_seq, input.source_time,
                                              trusted_observation->bbox, trusted_observation->confidence};
    last_published_source_time_ = input.source_time;
  }
  return output;
}

}  // namespace dog_patrol_perception_tracking
