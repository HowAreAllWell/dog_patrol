#include "merged_single_blob_assignment_decision.hpp"

#include <algorithm>

namespace vision_demo_host {

MergedSingleBlobAssignmentDecision::Result MergedSingleBlobAssignmentDecision::Decide(
    const Input &input,
    const Config &config) {
  Result result;
  result.active_candidates = input.active_candidates;
  result.inactive_recovery_rows = input.inactive_recovery_rows;

  int best_sid = -1;
  float best_cost = kBigCost;
  float continuity_cost = kBigCost;
  int best_app_sid = -1;
  int best_app_missing_frames = 0;
  float best_app_cost = kBigCost;
  float best_app_final = kBigCost;
  float best_app_geo = kBigCost;
  float continuity_app_cost = kBigCost;

  for (const auto &row : result.active_candidates) {
    if (row.stage != "merged_candidate" || !row.reject_reason.empty()) {
      continue;
    }
    if (row.semantic_id == input.continuity_semantic_id) {
      continuity_cost = row.final_score;
      continuity_app_cost = row.app_cost;
    }
    if (row.app_cost < best_app_cost) {
      best_app_cost = row.app_cost;
      best_app_final = row.final_score;
      best_app_geo = row.geo_cost;
      best_app_sid = row.semantic_id;
      best_app_missing_frames = row.missing_frames;
    }
    if (row.final_score < best_cost) {
      best_cost = row.final_score;
      best_sid = row.semantic_id;
    }
  }

  if (input.continuity_semantic_id > 0 && input.continuity_semantic_id != best_sid &&
      continuity_cost < kBigCost * 0.5F && best_cost < kBigCost * 0.5F) {
    const float margin = continuity_cost - best_cost;
    if (margin <= std::max(0.0F, config.min_assignment_margin)) {
      best_sid = input.continuity_semantic_id;
      best_cost = continuity_cost;
    }
  }

  bool legacy_handoff_available = false;
  if (best_app_sid > 0 && best_app_sid != input.continuity_semantic_id && best_app_missing_frames >= 18 &&
      best_app_final < kBigCost * 0.5F && best_cost < kBigCost * 0.5F && best_app_geo <= 0.75F &&
      best_app_final <= best_cost + 0.04F) {
    const bool best_app_was_missing = best_app_missing_frames > 0 &&
                                      best_app_missing_frames <= config.max_missing_frames;
    if (best_app_was_missing && best_app_cost + 0.025F <= continuity_app_cost) {
      legacy_handoff_available = true;
    }
  }

  if (legacy_handoff_available && input.continuity_semantic_id > 0) {
    best_sid = input.continuity_semantic_id;
    if (continuity_cost < kBigCost * 0.5F) {
      best_cost = continuity_cost;
    }
  }
  if (input.continuity_semantic_id > 0 && continuity_cost < kBigCost * 0.5F &&
      best_sid != input.continuity_semantic_id) {
    best_sid = input.continuity_semantic_id;
    best_cost = continuity_cost;
  }

  if (best_sid < 0 && !input.inactive_recovery_assignments.empty()) {
    best_sid = input.inactive_recovery_assignments.front().semantic_id;
    result.used_inactive_recovery = true;
  }

  if (best_sid < 0) {
    result.needs_new_semantic_id = true;
  }

  result.semantic_id = best_sid;
  for (auto &row : result.active_candidates) {
    if (row.track_idx == input.track_idx && row.raw_track_id == input.raw_track_id &&
        row.semantic_id == best_sid && row.stage == "merged_candidate") {
      row.selected = true;
      row.accepted = true;
    }
  }
  for (auto &row : result.inactive_recovery_rows) {
    if (row.track_idx == input.track_idx && row.raw_track_id == input.raw_track_id &&
        row.semantic_id == best_sid && row.stage == "inactive_recover_candidate") {
      row.selected = true;
      row.accepted = true;
    }
  }

  return result;
}

}  // namespace vision_demo_host
