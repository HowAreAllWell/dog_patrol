#include "unresolved_track_final_resolution_coordinator.hpp"

#include <algorithm>

#include "vision_demo_host/modules/association_utils.hpp"

namespace vision_demo_host {
namespace {

std::vector<float> FeatureForTrack(const std::vector<int> &person_track_indices,
                                   const std::vector<std::vector<float>> &person_features,
                                   const int track_idx) {
  const auto rel_it = std::find(person_track_indices.begin(), person_track_indices.end(), track_idx);
  if (rel_it == person_track_indices.end()) {
    return {};
  }
  const auto rel = static_cast<std::size_t>(std::distance(person_track_indices.begin(), rel_it));
  return rel < person_features.size() ? person_features[rel] : std::vector<float>{};
}

float IntersectionArea(const cv::Rect2f &a, const cv::Rect2f &b) {
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  return std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
}

bool LooksLikeFullBodyEdgePerson(const Track &track) {
  const float height = std::max(1.0F, track.bbox.height);
  const float area = std::max(0.0F, track.bbox.area());
  return height >= 1000.0F && area >= 200000.0F;
}

std::string NewSemanticBirthHideReason(const Track &track) {
  const float height = std::max(1.0F, track.bbox.height);
  const float aspect = track.bbox.width / height;
  if (aspect < 0.25F && !LooksLikeFullBodyEdgePerson(track)) {
    return "skinny_partial_hidden";
  }
  if (aspect > 1.45F && height < 300.0F) {
    return "wide_fragment_hidden";
  }
  return {};
}

bool ShouldQuickConfirmSmallNewPerson(const Track &track) {
  const float height = std::max(1.0F, track.bbox.height);
  const float aspect = track.bbox.width / height;
  return track.bbox.area() < 20000.0F && height < 300.0F && aspect >= 0.25F && aspect <= 0.60F;
}

bool LooksLikeDuplicateSplitOfAssignedIdentity(const Track &candidate,
                                               const std::vector<Track> &tracks,
                                               const std::unordered_map<int, int> &track_idx_to_sid,
                                               const std::unordered_map<int, int> &prev_raw_to_semantic) {
  for (const auto &[track_idx, semantic_id] : track_idx_to_sid) {
    (void)semantic_id;
    if (track_idx < 0 || track_idx >= static_cast<int>(tracks.size())) {
      continue;
    }
    const Track &assigned = tracks[static_cast<std::size_t>(track_idx)];
    if (prev_raw_to_semantic.find(assigned.id) == prev_raw_to_semantic.end()) {
      continue;
    }
    if (assigned.class_id != candidate.class_id) {
      continue;
    }
    const float candidate_area = std::max(1.0F, candidate.bbox.area());
    const float containment = IntersectionArea(candidate.bbox, assigned.bbox) / candidate_area;
    const float iou = association::BBoxIoU(candidate.bbox, assigned.bbox);
    if (containment >= 0.75F && iou >= 0.25F) {
      return true;
    }
  }
  return false;
}

}  // namespace

UnresolvedTrackFinalResolutionCoordinator::Result
UnresolvedTrackFinalResolutionCoordinator::Resolve(const Input &input) {
  Result result;
  if (input.tracks == nullptr || input.person_track_indices == nullptr || input.person_features == nullptr ||
      input.assigned_track_to_sid == nullptr || input.sid_used == nullptr || input.active_semantic_ids == nullptr ||
      input.prev_raw_to_semantic == nullptr || input.score_debug_rows == nullptr) {
    return result;
  }

  result.assigned_track_to_sid = *input.assigned_track_to_sid;
  result.sid_used = *input.sid_used;

  for (const int track_idx : *input.person_track_indices) {
    if (result.assigned_track_to_sid.find(track_idx) != result.assigned_track_to_sid.end()) {
      continue;
    }
    if (track_idx < 0 || track_idx >= static_cast<int>(input.tracks->size())) {
      continue;
    }
    const Track &track = (*input.tracks)[static_cast<std::size_t>(track_idx)];
    if (track.occlusion_suspect) {
      continue;
    }

    bool hold_for_ambiguous_recovery = false;
    if (input.find_identity && input.active_assignment_max_cost) {
      for (const auto &row : *input.score_debug_rows) {
        if (row.track_idx != track_idx || row.stage != "assign_candidate" || !row.selected || row.accepted ||
            row.reject_reason != "assignment_margin_reject") {
          continue;
        }
        const auto *identity = input.find_identity(row.semantic_id);
        if (identity == nullptr) {
          continue;
        }
        if (identity->missing_frames > 0 &&
            identity->missing_frames <= std::max(1, input.config.max_missing_frames) &&
            row.final_score <= std::min(1.0F, input.active_assignment_max_cost(*identity, track.association) + 0.10F)) {
          hold_for_ambiguous_recovery = true;
          break;
        }
      }
    }

    int side_recovery_sid = -1;
    ScoreEvidence side_recovery_evidence;
    if (input.find_identity && input.score_evidence && input.looks_like_merged_side_reappearance) {
      const auto feature = FeatureForTrack(*input.person_track_indices, *input.person_features, track_idx);
      for (const int sid : *input.active_semantic_ids) {
        const auto used_it = result.sid_used.find(sid);
        if (used_it != result.sid_used.end() && used_it->second) {
          continue;
        }
        const auto *identity = input.find_identity(sid);
        if (identity == nullptr || identity->missing_frames <= 0) {
          continue;
        }
        const auto evidence = input.score_evidence(track, *identity, feature);
        if (input.looks_like_merged_side_reappearance(track, *identity, *input.tracks, track_idx,
                                                      result.assigned_track_to_sid, evidence.app_cost) &&
            (side_recovery_sid < 0 || evidence.app_cost < side_recovery_evidence.app_cost)) {
          side_recovery_sid = sid;
          side_recovery_evidence = evidence;
        }
      }
    }

    if (side_recovery_sid > 0) {
      result.erase_pending_raw_track_ids.push_back(track.id);
      continue;
    }

    BirthManager::Input birth_input;
    birth_input.frame_index = input.frame_index;
    birth_input.track_idx = track_idx;
    birth_input.raw_track_id = track.id;
    birth_input.hold_for_ambiguous_recovery = hold_for_ambiguous_recovery;
    birth_input.duplicate_split =
        LooksLikeDuplicateSplitOfAssignedIdentity(track, *input.tracks, result.assigned_track_to_sid,
                                                  *input.prev_raw_to_semantic);
    birth_input.hide_reason = birth_input.duplicate_split ? "" : NewSemanticBirthHideReason(track);
    birth_input.phase5_birth_manager_enabled = input.config.disable_legacy_birth_allocation;
    birth_input.small_person_requires_stability = ShouldQuickConfirmSmallNewPerson(track);
    const auto birth_result = input.evaluate_birth ? input.evaluate_birth(birth_input) : BirthManager::Result{};

    if (birth_result.allocated_semantic_id) {
      result.assigned_track_to_sid[track_idx] = birth_result.semantic_id;
    }
    if (birth_result.has_debug_row) {
      DebugRow row;
      row.track_idx = birth_result.debug_row.track_idx;
      row.raw_track_id = birth_result.debug_row.raw_track_id;
      row.semantic_id = birth_result.debug_row.semantic_id;
      row.final_score = birth_result.debug_row.final_score;
      row.selected = birth_result.debug_row.selected;
      row.stage = birth_result.debug_row.stage;
      row.margin = birth_result.debug_row.margin;
      row.accepted = birth_result.debug_row.accepted;
      row.reject_reason = birth_result.debug_row.reject_reason;
      result.debug_rows.push_back(std::move(row));
    }
  }

  return result;
}

}  // namespace vision_demo_host
