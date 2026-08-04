#include "birth_candidate_decision.hpp"

#include <algorithm>

namespace dog_patrol_perception_tracking {

BirthCandidateDecision::Decision BirthCandidateDecision::Evaluate(const Input &input,
                                                                  const Config &config) {
  (void)config;
  Decision decision;
  decision.track_idx = input.track_idx;
  decision.raw_track_id = input.raw_track_id;

  if (input.hold_for_ambiguous_recovery) {
    decision.action = Action::kHideWithDebugRow;
    decision.stage = "phase5_birth_candidate";
    decision.reject_reason = "ambiguous_recovery_pending";
    decision.clear_pending_candidate = true;
    return decision;
  }

  if (input.duplicate_split || !input.hide_reason.empty()) {
    decision.action = Action::kHideWithDebugRow;
    decision.stage = "phase5_birth_candidate";
    decision.reject_reason = input.duplicate_split ? "duplicate_split_hidden" : input.hide_reason;
    decision.clear_pending_candidate = true;
    return decision;
  }

  decision.action = Action::kPhase5Pending;
  decision.stage = "phase5_birth_candidate";
  decision.reject_reason = "phase5_birth_manager_pending";
  decision.final_score = 0.0F;
  decision.margin = 1.0F;
  decision.selected = true;
  decision.accepted = false;
  return decision;
}

}  // namespace dog_patrol_perception_tracking
