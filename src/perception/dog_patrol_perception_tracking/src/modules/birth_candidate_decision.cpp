#include "birth_candidate_decision.hpp"

#include <algorithm>

namespace vision_demo_host {

BirthCandidateDecision::Decision BirthCandidateDecision::Evaluate(const Input &input,
                                                                  const Config &config) {
  Decision decision;
  decision.track_idx = input.track_idx;
  decision.raw_track_id = input.raw_track_id;

  const auto birth_candidate_stage = [&]() {
    return input.phase5_birth_manager_enabled ? "phase5_birth_candidate" : "birth_candidate";
  };

  if (input.hold_for_ambiguous_recovery) {
    decision.action = Action::kHideWithDebugRow;
    decision.stage = birth_candidate_stage();
    decision.reject_reason = "ambiguous_recovery_pending";
    decision.clear_pending_candidate = true;
    return decision;
  }

  if (input.duplicate_split || !input.hide_reason.empty()) {
    decision.action = Action::kHideWithDebugRow;
    decision.stage = birth_candidate_stage();
    decision.reject_reason = input.duplicate_split ? "duplicate_split_hidden" : input.hide_reason;
    decision.clear_pending_candidate = true;
    return decision;
  }

  if (input.phase5_birth_manager_enabled) {
    decision.action = Action::kPhase5Pending;
    decision.stage = "phase5_birth_candidate";
    decision.reject_reason = "phase5_birth_manager_pending";
    decision.final_score = 0.0F;
    decision.margin = 1.0F;
    decision.selected = true;
    decision.accepted = false;
    return decision;
  }

  if (input.small_person_requires_stability &&
      input.stable_observation_count < std::max(1, config.small_person_stable_frames_required)) {
    decision.action = Action::kLegacyPendingWithoutDebugRow;
    decision.reject_reason = "small_new_person_pending";
    return decision;
  }

  decision.action = Action::kAllocateNewSemantic;
  decision.stage = "new_semantic";
  decision.final_score = 0.0F;
  decision.margin = 1.0F;
  decision.selected = true;
  decision.accepted = true;
  decision.clear_pending_candidate = input.small_person_requires_stability;
  return decision;
}

}  // namespace vision_demo_host
