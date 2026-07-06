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

bool StartsWith(const std::string &value, const std::string &prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool ContainsSemanticIdToken(const std::string &semantic_ids, const int semantic_id) {
  const std::string token = std::to_string(semantic_id);
  std::size_t begin = 0;
  while (begin <= semantic_ids.size()) {
    const std::size_t end = semantic_ids.find('|', begin);
    const std::string part = semantic_ids.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    if (part == token) {
      return true;
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return false;
}

bool HasMergedEvidenceForIdentity(const IdentityObservation &identity,
                                  const std::vector<IdentityManager::Phase3ShadowDebugRow> &rows) {
  for (const auto &row : rows) {
    if (row.event_type == "merged_group_enter" || row.event_type == "merged_group_update" ||
        row.event_type == "merged_group_end") {
      if (row.carrier_semantic_id == identity.semantic_id ||
          ContainsSemanticIdToken(row.semantic_ids, identity.semantic_id)) {
        return true;
      }
    }
    if (StartsWith(row.event_type, "phase4_merged") && row.candidate_semantic_id == identity.semantic_id) {
      return true;
    }
  }
  return false;
}

bool HasSplitEvidenceForIdentity(const IdentityObservation &identity,
                                 const std::vector<IdentityManager::Phase3ShadowDebugRow> &rows) {
  for (const auto &row : rows) {
    if ((row.event_type == "split_candidate_enter" || row.event_type == "split_candidate_update" ||
         row.event_type == "split_candidate_end") &&
        (row.candidate_semantic_id == identity.semantic_id ||
         row.carrier_semantic_id == identity.semantic_id ||
         row.related_raw_track_id == identity.supporting_raw_track_id.value_or(-1))) {
      return true;
    }
  }
  return false;
}

bool HasBirthEvidenceForIdentity(const IdentityObservation &identity,
                                 const std::vector<IdentityManager::Phase3ShadowDebugRow> &rows) {
  for (const auto &row : rows) {
    if (!StartsWith(row.event_type, "new_birth_candidate_")) {
      continue;
    }
    if (row.candidate_semantic_id == identity.semantic_id ||
        row.candidate_raw_track_id == identity.supporting_raw_track_id.value_or(-1)) {
      return true;
    }
  }
  return false;
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

IdentityObservationProjection::TargetLifecycle IdentityObservationProjection::ProjectTargetLifecycle(
    const IdentityObservation &identity, const IdentityManager::Mode mode,
    const std::vector<IdentityManager::Phase3ShadowDebugRow> &phase3_rows) {
  if (HasBirthEvidenceForIdentity(identity, phase3_rows)) {
    return TargetLifecycle::kNewBirthCandidate;
  }
  if (identity.state == IdentityState::kMerged || mode == IdentityManager::Mode::kMerged ||
      HasMergedEvidenceForIdentity(identity, phase3_rows)) {
    return TargetLifecycle::kMergedGroup;
  }
  if (identity.state == IdentityState::kSplitRecovery || mode == IdentityManager::Mode::kSplitRecovery ||
      HasSplitEvidenceForIdentity(identity, phase3_rows)) {
    return TargetLifecycle::kSplitCandidate;
  }
  if (identity.state == IdentityState::kLost || identity.state == IdentityState::kInactive) {
    return TargetLifecycle::kLostIdentity;
  }
  if (identity.visible || identity.state == IdentityState::kActive) {
    return TargetLifecycle::kVisibleIdentity;
  }
  if (identity.state == IdentityState::kOccluded) {
    return TargetLifecycle::kOccludedIdentity;
  }
  return TargetLifecycle::kLostIdentity;
}

std::string TargetLifecycleToString(const IdentityObservationProjection::TargetLifecycle lifecycle) {
  using TargetLifecycle = IdentityObservationProjection::TargetLifecycle;
  switch (lifecycle) {
    case TargetLifecycle::kVisibleIdentity:
      return "VisibleIdentity";
    case TargetLifecycle::kOccludedIdentity:
      return "OccludedIdentity";
    case TargetLifecycle::kMergedGroup:
      return "MergedGroup";
    case TargetLifecycle::kSplitCandidate:
      return "SplitCandidate";
    case TargetLifecycle::kNewBirthCandidate:
      return "NewBirthCandidate";
    case TargetLifecycle::kLostIdentity:
      return "LostIdentity";
  }
  return "LostIdentity";
}

}  // namespace vision_demo_host
