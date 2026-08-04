#include "raw_continuity_decision.hpp"

#include <algorithm>

namespace dog_patrol_perception_tracking {

RawContinuityDecision::Decision RawContinuityDecision::Evaluate(const Input &input,
                                                                const Config &config) {
  Decision decision;
  decision.track_idx = input.track_idx;
  decision.raw_track_id = input.raw_track_id;
  decision.semantic_id = input.semantic_id;
  decision.app_cost = input.app_cost;
  decision.geo_cost = input.geo_cost;
  decision.time_cost = input.time_cost;
  decision.final_score = input.final_cost;
  decision.margin = std::max(0.0F, 1.0F - std::clamp(input.final_cost, 0.0F, 1.0F));

  if (!input.identity_found) {
    decision.reject_reason = "identity_not_found";
    return decision;
  }
  if (!input.passes_missing_identity_gate) {
    decision.reject_reason = "missing_identity_gate_reject";
    return decision;
  }
  if (input.final_cost > std::clamp(config.raw_continuity_max_cost, 0.0F, 1.0F)) {
    decision.reject_reason = "raw_continuity_max_cost_reject";
    return decision;
  }
  if (input.weak_mot_association) {
    decision.reject_reason = "weak_mot_association";
    return decision;
  }

  decision.selected = true;
  decision.accepted = true;
  return decision;
}

}  // namespace dog_patrol_perception_tracking
