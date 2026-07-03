#include "vision_demo_host/modules/feature_update_policy.hpp"

#include <algorithm>

namespace vision_demo_host {

FeatureUpdatePolicy::Decision FeatureUpdatePolicy::Decide(const Input &input) {
  Decision decision;
  if (!input.accepted) {
    decision.feature_update_reason = "update_blocked_by_rejected_assignment";
    decision.geometry_update_reason = "update_blocked_by_rejected_assignment";
    return decision;
  }

  std::string feature_gate_reason;
  const bool allow_feature_gate = !input.global_freeze && !input.overlap_freeze;
  if (input.overlap_freeze) {
    feature_gate_reason = "overlapping_track_freeze";
  } else if (input.global_freeze) {
    feature_gate_reason = "global_merge_split_freeze";
  }

  const bool reliable_geometry = input.force_geometry_update || input.reliable_observation;
  decision.geometry_update_allowed = reliable_geometry;
  if (reliable_geometry) {
    decision.geometry_update_reason = "allowed_update";
  } else if (!feature_gate_reason.empty()) {
    decision.geometry_update_reason = feature_gate_reason;
  } else {
    decision.geometry_update_reason = "unreliable_low_quality_observation";
  }

  decision.feature_update_allowed = allow_feature_gate && reliable_geometry;
  if (!allow_feature_gate) {
    decision.feature_update_reason = feature_gate_reason;
  } else if (!reliable_geometry) {
    decision.feature_update_reason = "unreliable_low_quality_observation";
  } else {
    const int stable_required = std::max(1, input.stable_frames_before_feature_update);
    const bool stable_enough =
        !input.has_existing_feature_bank || input.stable_update_frames + 1 >= stable_required;
    decision.feature_update_reason = stable_enough ? "allowed_update" : "insufficient_stable_frames";
  }

  return decision;
}

}  // namespace vision_demo_host
