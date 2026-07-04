#include "identity_observation_projection.hpp"

#include <algorithm>

namespace vision_demo_host {
namespace {

const IdentityManager::ScoreDebugRow *FindBestDebugRow(
    const std::vector<IdentityManager::ScoreDebugRow> &rows, const int raw_track_id, const int semantic_id) {
  const IdentityManager::ScoreDebugRow *fallback = nullptr;
  for (const auto &row : rows) {
    if (row.raw_track_id != raw_track_id || row.semantic_id != semantic_id) {
      continue;
    }
    if (row.accepted) {
      return &row;
    }
    if (fallback == nullptr || row.selected) {
      fallback = &row;
    }
  }
  return fallback;
}

IdentityState StateFromSnapshot(const LegacyIdentitySnapshot &snapshot, const IdentityManager::Mode mode,
                                const int max_missing_frames) {
  if (snapshot.seen_this_frame) {
    return IdentityState::kActive;
  }
  if (mode == IdentityManager::Mode::kMerged) {
    return IdentityState::kMerged;
  }
  if (mode == IdentityManager::Mode::kSplitRecovery) {
    return IdentityState::kSplitRecovery;
  }
  if (snapshot.missing_frames <= std::max(0, max_missing_frames)) {
    return IdentityState::kOccluded;
  }
  return IdentityState::kLost;
}

IdentityAssignmentEvidence AssignmentEvidenceFromDebug(const IdentityManager::ScoreDebugRow &row) {
  IdentityAssignmentEvidence out;
  out.app_cost = row.app_cost;
  out.geo_cost = row.geo_cost;
  out.time_cost = row.time_cost;
  out.final_score = row.final_score;
  out.margin = row.margin;
  out.selected = row.selected;
  out.accepted = row.accepted;
  out.continuity_used = row.continuity_used;
  out.feature_update_allowed = row.feature_update_allowed;
  out.geometry_update_allowed = row.geometry_update_allowed;
  out.feature_update_reason = row.feature_update_reason;
  out.geometry_update_reason = row.geometry_update_reason;
  out.stage = row.stage;
  out.reject_reason = row.reject_reason;
  return out;
}

}  // namespace

IdentityManagerResult IdentityObservationProjection::Build(const Input &input) {
  IdentityManagerResult built;
  built.primary_semantic_id = input.primary_semantic_id;
  built.feature_update_frozen = input.feature_update_frozen;
  built.identities.reserve(input.snapshots.size());

  for (const auto &snapshot : input.snapshots) {
    if (snapshot.semantic_id <= 0) {
      continue;
    }

    IdentityObservation identity;
    identity.semantic_id = snapshot.semantic_id;
    identity.state = StateFromSnapshot(snapshot, input.mode, input.max_missing_frames);
    identity.class_id = snapshot.class_id;
    identity.confidence = snapshot.confidence;
    identity.bbox = snapshot.has_reliable_geometry ? snapshot.reliable_bbox : snapshot.bbox;
    identity.missing_frames = snapshot.missing_frames;
    identity.visible = snapshot.seen_this_frame;
    identity.primary = (built.primary_semantic_id > 0 && identity.semantic_id == built.primary_semantic_id);

    if (snapshot.supporting_raw_track_id > 0) {
      identity.supporting_raw_track_id = snapshot.supporting_raw_track_id;
      const auto obs_it = input.observations_by_raw_track_id.find(snapshot.supporting_raw_track_id);
      if (obs_it != input.observations_by_raw_track_id.end()) {
        const auto &obs = obs_it->second;
        identity.supporting_tracklet = obs;
        identity.class_id = obs.class_id;
        identity.confidence = obs.confidence;
        identity.bbox = obs.bbox;
        identity.occlusion_suspect = obs.occlusion_suspect;
        identity.low_score_update = obs.low_score_update;
        identity.just_recovered = obs.just_recovered;
        identity.association = obs.association;

        if (const auto *row = FindBestDebugRow(input.debug_rows, obs.raw_track_id, identity.semantic_id);
            row != nullptr) {
          identity.assignment = AssignmentEvidenceFromDebug(*row);
        }
      }
    }

    built.identities.push_back(std::move(identity));
  }

  return built;
}

}  // namespace vision_demo_host
