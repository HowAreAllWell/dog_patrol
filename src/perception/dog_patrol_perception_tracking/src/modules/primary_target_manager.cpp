#include "vision_demo_host/modules/primary_target_manager.hpp"

#include <algorithm>
#include <utility>

#include "vision_demo_host/modules/association_utils.hpp"

namespace vision_demo_host {

PrimaryTargetManager::PrimaryTargetManager(Config config) : config_(std::move(config)) {
  if (config_.lost_threshold_frames < 1) {
    config_.lost_threshold_frames = 1;
  }
  config_.max_center_jump_norm = std::max(0.0F, config_.max_center_jump_norm);
  config_.min_area_ratio = std::max(0.0F, config_.min_area_ratio);
  config_.max_area_ratio = std::max(config_.min_area_ratio, config_.max_area_ratio);
  config_.pending_recovery_frames = std::max(0, config_.pending_recovery_frames);
  if (config_.handled_ignore_absence <= Duration::zero()) {
    config_.handled_ignore_absence = std::chrono::milliseconds{1};
  }
}

std::optional<IdentityObservation> PrimaryTargetManager::FindVisibleIdentityBySemanticId(
    const std::vector<IdentityObservation> &identities, const int semantic_id) const {
  const auto identity = FindIdentityBySemanticId(identities, semantic_id);
  if (identity.has_value() && IsVisibleIdentity(identity.value())) {
    return identity;
  }
  return std::nullopt;
}

std::optional<IdentityObservation> PrimaryTargetManager::FindIdentityBySemanticId(
    const std::vector<IdentityObservation> &identities, const int semantic_id) const {
  if (semantic_id <= 0) {
    return std::nullopt;
  }
  for (const auto &identity : identities) {
    if (identity.semantic_id == semantic_id) {
      return identity;
    }
  }
  return std::nullopt;
}

std::optional<IdentityObservation> PrimaryTargetManager::SelectLargestValidPersonIdentity(
    const std::vector<IdentityObservation> &identities, const bool require_mission_eligibility) const {
  std::optional<IdentityObservation> best = std::nullopt;
  float best_area = 0.0F;

  for (const auto &identity : identities) {
    if (!IsVisibleIdentity(identity)) {
      continue;
    }
    if (identity.class_id != ClassId::kPerson) {
      continue;
    }
    if (require_mission_eligibility && !IsMissionEligible(identity)) {
      continue;
    }

    const float area = identity.bbox.area();
    if (area < config_.min_person_area_px) {
      continue;
    }

    if (!best.has_value() || area > best_area) {
      best = identity;
      best_area = area;
    }
  }

  return best;
}

void PrimaryTargetManager::ResetForPatrolCycle(const int handled_semantic_id) {
  if (handled_semantic_id > 0) {
    handled_identity_absence_started_at_.insert_or_assign(handled_semantic_id, std::nullopt);
  }

  state_ = PrimaryTargetResult{};
  primary_target_id_ = -1;
  bound_raw_track_id_ = -1;
  last_primary_track_ = std::nullopt;
  pending_recovery_frames_ = 0;
  last_decision_reason_.clear();
  last_reject_reason_.clear();
}

bool PrimaryTargetManager::IsMissionEligible(const IdentityObservation &identity) const {
  return identity.semantic_id > 0 &&
         handled_identity_absence_started_at_.find(identity.semantic_id) == handled_identity_absence_started_at_.end();
}

void PrimaryTargetManager::UpdateHandledIdentityAbsence(const std::vector<IdentityObservation> &identities,
                                                        const TimePoint now) {
  for (auto it = handled_identity_absence_started_at_.begin(); it != handled_identity_absence_started_at_.end();) {
    const auto visible_identity = FindVisibleIdentityBySemanticId(identities, it->first);
    if (visible_identity.has_value()) {
      it->second.reset();
      ++it;
      continue;
    }

    if (!it->second.has_value()) {
      it->second = now;
      ++it;
      continue;
    }

    if (now >= it->second.value() && now - it->second.value() >= config_.handled_ignore_absence) {
      it = handled_identity_absence_started_at_.erase(it);
      continue;
    }
    ++it;
  }
}

bool PrimaryTargetManager::IsVisibleIdentity(const IdentityObservation &identity) const {
  return identity.state == IdentityState::kActive && identity.visible && identity.supporting_raw_track_id.has_value();
}

Track PrimaryTargetManager::TrackFromIdentityObservation(const IdentityObservation &identity) const {
  Track track;
  track.id = identity.supporting_raw_track_id.value_or(-1);
  track.class_id = identity.class_id;
  track.confidence = identity.confidence;
  track.bbox = identity.bbox;
  track.is_confirmed = identity.visible;
  track.occlusion_suspect = identity.occlusion_suspect;
  track.association = identity.association;
  track.just_recovered = identity.just_recovered;
  track.low_score_update = identity.low_score_update;
  return track;
}

PrimaryTargetResult PrimaryTargetManager::EnterPendingRecovery(const std::string &decision_reason) {
  pending_recovery_frames_ = std::min(pending_recovery_frames_ + 1, config_.pending_recovery_frames);
  state_.state = PrimaryState::kPendingRecovery;
  last_decision_reason_ = decision_reason;
  return state_;
}

PrimaryTargetResult PrimaryTargetManager::EnterOccluded(const std::string &decision_reason) {
  pending_recovery_frames_ = 0;
  state_.state = PrimaryState::kOccluded;
  last_decision_reason_ = decision_reason;
  return state_;
}

PrimaryTargetResult PrimaryTargetManager::EnterLost(const std::string &decision_reason, const int missing_frames) {
  bound_raw_track_id_ = -1;
  primary_target_id_ = -1;
  last_primary_track_ = std::nullopt;
  pending_recovery_frames_ = 0;
  state_.primary_track = std::nullopt;
  state_.primary_target_id = -1;
  state_.raw_track_id = -1;
  state_.missing_frames = missing_frames;
  state_.state = PrimaryState::kLost;
  last_decision_reason_ = decision_reason;
  return state_;
}

bool PrimaryTargetManager::IsVisiblePrimarySane(const Track &track, std::string *reject_reason) const {
  auto reject = [&](const std::string &reason) {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    return false;
  };

  if (track.class_id != ClassId::kPerson) {
    return reject("visible_primary_not_person");
  }
  if (track.bbox.area() < config_.min_person_area_px) {
    return reject("visible_primary_area_too_small");
  }
  if (!track.is_confirmed) {
    return reject("visible_primary_unconfirmed");
  }
  if (track.occlusion_suspect) {
    return reject("visible_primary_occlusion_suspect");
  }
  if (!track.association.stage.empty() && !track.association.passed_final_cost_gate) {
    return reject("visible_primary_assoc_gate_failed");
  }
  if (track.low_score_update || track.association.low_score_detection) {
    return reject("visible_primary_low_score_update");
  }
  if (track.just_recovered || track.association.recovered_from_lost) {
    return reject("visible_primary_just_recovered");
  }

  if (last_primary_track_.has_value()) {
    const float jump = association::CenterDistanceNormByDiag(last_primary_track_->bbox, track.bbox);
    if (jump > config_.max_center_jump_norm) {
      return reject("visible_primary_center_jump");
    }

    const float area_ratio = association::AreaRatio(track.bbox, last_primary_track_->bbox);
    if (area_ratio < config_.min_area_ratio || area_ratio > config_.max_area_ratio) {
      return reject("visible_primary_area_ratio");
    }
  }

  if (reject_reason != nullptr) {
    reject_reason->clear();
  }
  return true;
}

PrimaryTargetResult PrimaryTargetManager::Update(const std::vector<IdentityObservation> &identities) {
  return UpdateInternal(identities, false);
}

PrimaryTargetResult PrimaryTargetManager::UpdateForPatrol(const std::vector<IdentityObservation> &identities,
                                                          const TimePoint now) {
  UpdateHandledIdentityAbsence(identities, now);
  return UpdateInternal(identities, true);
}

PrimaryTargetResult PrimaryTargetManager::UpdateForMission(
    const std::vector<IdentityObservation> &identities,
    const std::optional<MissionSnapshot> &mission,
    const std::optional<MissionSnapshot> &previous_mission,
    const TimePoint now) {
  if (!mission.has_value()) {
    return Update(identities);
  }

  if (mission->phase == MissionPhase::kPatrol &&
      (!last_mission_for_primary_.has_value() ||
       last_mission_for_primary_->state_seq != mission->state_seq ||
       last_mission_for_primary_->phase != MissionPhase::kPatrol)) {
    int handled_semantic_id = -1;
    const auto preceding_mission =
        previous_mission.has_value() && previous_mission->state_seq != mission->state_seq
            ? previous_mission
            : last_mission_for_primary_;
    if (preceding_mission.has_value() && preceding_mission->target_id > 0 &&
        (preceding_mission->phase == MissionPhase::kVerifyIdentity ||
         preceding_mission->phase == MissionPhase::kTrackIntruder)) {
      handled_semantic_id = preceding_mission->target_id;
    }
    ResetForPatrolCycle(handled_semantic_id);
  }

  last_mission_for_primary_ = mission;
  return mission->phase == MissionPhase::kPatrol ? UpdateForPatrol(identities, now)
                                                 : Update(identities);
}

PrimaryTargetResult PrimaryTargetManager::UpdateInternal(const std::vector<IdentityObservation> &identities,
                                                         const bool require_mission_eligibility) {
  last_decision_reason_.clear();
  last_reject_reason_.clear();

  const int current_primary_id =
      primary_target_id_ > 0 ? primary_target_id_ : (state_.primary_target_id > 0 ? state_.primary_target_id : -1);
  const auto current_primary_identity = FindIdentityBySemanticId(identities, current_primary_id);
  const auto visible_primary_identity = FindVisibleIdentityBySemanticId(identities, current_primary_id);
  if (visible_primary_identity.has_value()) {
    auto primary_track = TrackFromIdentityObservation(visible_primary_identity.value());
    std::string reject_reason;
    if (!IsVisiblePrimarySane(primary_track, &reject_reason)) {
      last_reject_reason_ = reject_reason;
      if (current_primary_id > 0) {
        primary_target_id_ = current_primary_id;
        state_.missing_frames = std::max(state_.missing_frames + 1, visible_primary_identity->missing_frames);
        state_.primary_track = std::nullopt;
        state_.raw_track_id = -1;
        state_.primary_target_id = primary_target_id_;
        if (state_.missing_frames > config_.lost_threshold_frames) {
          return EnterLost("lost_after_pending_recovery_threshold", state_.missing_frames);
        }
        return EnterPendingRecovery("pending_recovery_visible_primary_sanity_rejected");
      }
      last_decision_reason_ = "visible_primary_sanity_rejected_no_primary";
    } else {
      primary_target_id_ = visible_primary_identity->semantic_id;
      bound_raw_track_id_ = primary_track.id;
      primary_track.authoritative = true;
      state_.state = PrimaryState::kLocked;
      state_.primary_track = primary_track;
      state_.primary_target_id = primary_target_id_;
      state_.raw_track_id = primary_track.id;
      state_.missing_frames = 0;
      pending_recovery_frames_ = 0;
      last_primary_track_ = primary_track;
      last_decision_reason_ = "locked_visible_primary_identity";
      return state_;
    }
  }

  if (current_primary_identity.has_value() && current_primary_id > 0) {
    primary_target_id_ = current_primary_id;
    bound_raw_track_id_ = -1;
    state_.primary_track = std::nullopt;
    state_.raw_track_id = -1;
    state_.primary_target_id = primary_target_id_;
    state_.missing_frames = std::max(state_.missing_frames + 1, current_primary_identity->missing_frames);

    if (current_primary_identity->state == IdentityState::kLost ||
        current_primary_identity->state == IdentityState::kInactive) {
      return EnterLost("lost_from_identity_state", std::max(state_.missing_frames, current_primary_identity->missing_frames));
    }

    if (state_.missing_frames > config_.lost_threshold_frames) {
      return EnterLost("lost_after_threshold", state_.missing_frames);
    }

    if (current_primary_identity->state == IdentityState::kMerged ||
        current_primary_identity->state == IdentityState::kSplitRecovery) {
      return EnterPendingRecovery("pending_recovery_from_identity_state");
    }

    if (state_.state == PrimaryState::kPendingRecovery && pending_recovery_frames_ > 0 &&
        pending_recovery_frames_ < config_.pending_recovery_frames) {
      return EnterPendingRecovery("pending_recovery_hold_missing_identity_evidence");
    }

    return EnterOccluded("occluded_from_identity_state");
  }

  if (primary_target_id_ > 0 || bound_raw_track_id_ > 0 || state_.state == PrimaryState::kLocked ||
      state_.state == PrimaryState::kOccluded || state_.state == PrimaryState::kPendingRecovery) {
    state_.missing_frames += 1;
    state_.primary_track = std::nullopt;
    state_.raw_track_id = -1;
    state_.primary_target_id = (primary_target_id_ > 0) ? primary_target_id_ : -1;

    if (state_.missing_frames <= config_.lost_threshold_frames) {
      if (state_.state == PrimaryState::kPendingRecovery && pending_recovery_frames_ > 0 &&
          pending_recovery_frames_ < config_.pending_recovery_frames) {
        return EnterPendingRecovery("pending_recovery_hold_missing_identity_evidence");
      }
      return EnterOccluded("occluded_within_lost_threshold");
    }

    return EnterLost("lost_after_threshold", state_.missing_frames);
  }

  auto new_identity = SelectLargestValidPersonIdentity(identities, require_mission_eligibility);
  if (new_identity.has_value()) {
    auto new_target = TrackFromIdentityObservation(new_identity.value());
    primary_target_id_ = new_identity->semantic_id > 0 ? new_identity->semantic_id : 1;
    bound_raw_track_id_ = new_target.id;
    new_target.authoritative = true;
    state_.state = PrimaryState::kLocked;
    state_.primary_track = new_target;
    state_.primary_target_id = primary_target_id_;
    state_.raw_track_id = new_target.id;
    state_.missing_frames = 0;
    pending_recovery_frames_ = 0;
    last_primary_track_ = new_target;
    last_decision_reason_ = "locked_largest_valid_identity";
    return state_;
  }

  state_.state = PrimaryState::kIdle;
  state_.primary_target_id = -1;
  state_.raw_track_id = -1;
  state_.primary_track = std::nullopt;
  pending_recovery_frames_ = 0;
  last_decision_reason_ = "idle_no_valid_identity";
  return state_;
}

PrimaryTargetResult PrimaryTargetManager::GetState() const { return state_; }

}  // namespace vision_demo_host
