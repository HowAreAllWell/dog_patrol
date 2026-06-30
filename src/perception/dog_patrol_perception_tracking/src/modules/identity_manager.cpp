#include "vision_demo_host/modules/identity_manager.hpp"

#include <algorithm>
#include <optional>
#include <unordered_map>
#include <utility>

#include "legacy_identity_matcher.hpp"

namespace vision_demo_host {
namespace {

LegacyIdentityMatcher::Config ToLegacyConfig(const IdentityManager::Config &config) {
  LegacyIdentityMatcher::Config out;
  out.max_missing_frames = config.max_missing_frames;
  out.feat_bank_size = config.feat_bank_size;
  out.recover_sim_thresh_strict = config.recover_sim_thresh_strict;
  out.recover_sim_thresh_relaxed = config.recover_sim_thresh_relaxed;
  out.recover_relaxed_max_missing_frames = config.recover_relaxed_max_missing_frames;
  out.occlusion_protect_frames = config.occlusion_protect_frames;
  out.missing_assign_min_area_ratio = config.missing_assign_min_area_ratio;
  out.missing_assign_max_area_ratio = config.missing_assign_max_area_ratio;
  out.missing_assign_max_center_dist_norm = config.missing_assign_max_center_dist_norm;
  out.missing_assign_max_app_cost = config.missing_assign_max_app_cost;
  out.overlap_iou_freeze = config.overlap_iou_freeze;
  out.split_stable_frames = config.split_stable_frames;
  out.merge_hold_frames = config.merge_hold_frames;
  out.app_w = config.app_w;
  out.geo_w = config.geo_w;
  out.time_w = config.time_w;
  out.active_assign_max_cost = config.active_assign_max_cost;
  out.recovery_max_cost = config.recovery_max_cost;
  out.raw_continuity_max_cost = config.raw_continuity_max_cost;
  out.min_assignment_margin = config.min_assignment_margin;
  out.stable_frames_before_feature_update = config.stable_frames_before_feature_update;
  out.merged_requires_overlap = config.merged_requires_overlap;
  out.reid_enable = config.reid_enable;
  out.reid_backend = config.reid_backend;
  out.reid_model_path = config.reid_model_path;
  out.reid_input_width = config.reid_input_width;
  out.reid_input_height = config.reid_input_height;
  return out;
}

IdentityManager::Mode FromLegacyMode(const LegacyIdentityMode mode) {
  switch (mode) {
    case LegacyIdentityMode::kMerged:
      return IdentityManager::Mode::kMerged;
    case LegacyIdentityMode::kSplitRecovery:
      return IdentityManager::Mode::kSplitRecovery;
    case LegacyIdentityMode::kNormalResumed:
      return IdentityManager::Mode::kNormalResumed;
    case LegacyIdentityMode::kNormal:
    default:
      return IdentityManager::Mode::kNormal;
  }
}

IdentityManager::ScoreDebugRow FromLegacyDebugRow(const LegacyIdentityMatcher::ScoreDebugRow &row) {
  IdentityManager::ScoreDebugRow out;
  out.frame_idx = row.frame_idx;
  out.mode = FromLegacyMode(row.mode);
  out.track_idx = row.track_idx;
  out.raw_track_id = row.raw_track_id;
  out.semantic_id = row.semantic_id;
  out.app_cost = row.app_cost;
  out.geo_cost = row.geo_cost;
  out.time_cost = row.time_cost;
  out.final_score = row.final_score;
  out.selected = row.selected;
  out.stage = row.stage;
  out.margin = row.margin;
  out.accepted = row.accepted;
  out.reject_reason = row.reject_reason;
  out.continuity_used = row.continuity_used;
  out.feature_update_allowed = row.feature_update_allowed;
  out.geometry_update_allowed = row.geometry_update_allowed;
  return out;
}

std::string TrackletHypothesisStatusToDebugString(const TrackletHypothesisStatus status) {
  switch (status) {
    case TrackletHypothesisStatus::kTracked:
      return "tracked";
    case TrackletHypothesisStatus::kTentative:
      return "tentative";
    case TrackletHypothesisStatus::kLostPrediction:
      return "lost_prediction";
    case TrackletHypothesisStatus::kSuppressedDuplicateCandidate:
      return "suppressed_duplicate_candidate";
    case TrackletHypothesisStatus::kSplitCandidate:
      return "split_candidate";
    case TrackletHypothesisStatus::kLowQualityCandidate:
      return "low_quality_candidate";
    default:
      return "unknown";
  }
}

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

IdentityState StateFromSnapshot(const LegacyIdentityMatcher::IdentitySnapshot &snapshot,
                                const IdentityManager::Mode mode,
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

}  // namespace

class IdentityManager::Impl {
 public:
  Impl() = default;
  explicit Impl(Config config) : config(std::move(config)), legacy_identity_matcher(ToLegacyConfig(this->config)) {}

  Config config;
  LegacyIdentityMatcher legacy_identity_matcher;
  std::vector<ScoreDebugRow> last_score_debug_rows;
  std::vector<Phase3ShadowDebugRow> last_phase3_shadow_debug_rows;
};

IdentityManager::IdentityManager() : impl_(std::make_shared<Impl>()) {}

IdentityManager::IdentityManager(Config config) : impl_(std::make_shared<Impl>(config)) {}

bool IdentityManager::Initialize(std::string *error) { return impl_->legacy_identity_matcher.Initialize(error); }

void IdentityManager::Reset() {
  impl_->legacy_identity_matcher.Reset();
  impl_->last_score_debug_rows.clear();
  impl_->last_phase3_shadow_debug_rows.clear();
  raw_to_semantic_id_.clear();
}

IdentityManagerResult IdentityManager::Update(const std::vector<TrackletObservation> &observations,
                                              const PrimaryTargetResult &primary, const cv::Mat *frame) {
  return Update(observations, {}, primary, frame);
}

IdentityManagerResult IdentityManager::Update(const std::vector<TrackletObservation> &observations,
                                              const std::vector<TrackletHypothesis> &shadow_hypotheses,
                                              const PrimaryTargetResult &primary, const cv::Mat *frame) {
  const std::vector<Track> tracks = TracksFromObservations(observations);
  raw_to_semantic_id_ = impl_->legacy_identity_matcher.Update(tracks, primary, frame);
  impl_->last_score_debug_rows.clear();
  const auto &legacy_debug_rows = impl_->legacy_identity_matcher.LastScoreDebugRows();
  impl_->last_score_debug_rows.reserve(legacy_debug_rows.size());
  for (const auto &row : legacy_debug_rows) {
    impl_->last_score_debug_rows.push_back(FromLegacyDebugRow(row));
  }

  impl_->last_phase3_shadow_debug_rows.clear();
  impl_->last_phase3_shadow_debug_rows.reserve(shadow_hypotheses.size());
  const int current_frame_idx = impl_->last_score_debug_rows.empty() ? -1 : impl_->last_score_debug_rows.front().frame_idx;
  int event_idx = 0;
  for (const auto &hypothesis : shadow_hypotheses) {
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = "hypothesis_input";
    row.candidate_raw_track_id = hypothesis.raw_track_id;
    row.reason = hypothesis.candidate_reason;
    row.related_raw_track_id = hypothesis.related_raw_track_id.value_or(-1);
    row.hypothesis_status = TrackletHypothesisStatusToDebugString(hypothesis.status);
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  }

  IdentityManagerResult result;
  result.primary_semantic_id = impl_->legacy_identity_matcher.CurrentPrimarySemanticId();
  result.feature_update_frozen = impl_->legacy_identity_matcher.IsFeatureUpdateFrozen();
  const auto mode = CurrentMode();
  const auto snapshots = impl_->legacy_identity_matcher.IdentitySnapshots();
  result.identities.reserve(snapshots.size());

  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id;
  observations_by_raw_track_id.reserve(observations.size());
  for (const auto &obs : observations) {
    observations_by_raw_track_id[obs.raw_track_id] = obs;
  }

  const auto &debug_rows = impl_->last_score_debug_rows;
  for (const auto &snapshot : snapshots) {
    if (snapshot.semantic_id <= 0) {
      continue;
    }

    IdentityObservation identity;
    identity.semantic_id = snapshot.semantic_id;
    identity.state = StateFromSnapshot(snapshot, mode, impl_->config.max_missing_frames);
    identity.class_id = snapshot.class_id;
    identity.confidence = snapshot.confidence;
    identity.bbox = snapshot.has_reliable_geometry ? snapshot.reliable_bbox : snapshot.bbox;
    identity.missing_frames = snapshot.missing_frames;
    identity.visible = snapshot.seen_this_frame;
    identity.primary = (result.primary_semantic_id > 0 && identity.semantic_id == result.primary_semantic_id);
    if (snapshot.supporting_raw_track_id > 0) {
      identity.supporting_raw_track_id = snapshot.supporting_raw_track_id;
      auto obs_it = observations_by_raw_track_id.find(snapshot.supporting_raw_track_id);
      if (obs_it != observations_by_raw_track_id.end()) {
        const auto &obs = obs_it->second;
        identity.supporting_tracklet = obs;
        identity.class_id = obs.class_id;
        identity.confidence = obs.confidence;
        identity.bbox = obs.bbox;
        identity.occlusion_suspect = obs.occlusion_suspect;
        identity.low_score_update = obs.low_score_update;
        identity.just_recovered = obs.just_recovered;
        identity.association = obs.association;

        if (const auto *row = FindBestDebugRow(debug_rows, obs.raw_track_id, identity.semantic_id); row != nullptr) {
          identity.assignment = AssignmentEvidenceFromDebug(*row);
        }
      }
    }

    result.identities.push_back(std::move(identity));
  }

  return result;
}

IdentityManager::Mode IdentityManager::CurrentMode() const {
  return FromLegacyMode(impl_->legacy_identity_matcher.CurrentMode());
}

bool IdentityManager::IsFeatureUpdateFrozen() const { return impl_->legacy_identity_matcher.IsFeatureUpdateFrozen(); }

const std::vector<IdentityManager::ScoreDebugRow> &IdentityManager::LastScoreDebugRows() const {
  return impl_->last_score_debug_rows;
}

const std::vector<IdentityManager::Phase3ShadowDebugRow> &IdentityManager::LastPhase3ShadowDebugRows() const {
  return impl_->last_phase3_shadow_debug_rows;
}

std::vector<Track> IdentityManager::TracksFromObservations(const std::vector<TrackletObservation> &observations) {
  std::vector<Track> tracks;
  tracks.reserve(observations.size());
  for (const auto &obs : observations) {
    tracks.push_back(obs.ToTrack());
  }
  return tracks;
}

IdentityAssignmentEvidence IdentityManager::AssignmentEvidenceFromDebug(const ScoreDebugRow &row) {
  IdentityAssignmentEvidence evidence;
  evidence.app_cost = row.app_cost;
  evidence.geo_cost = row.geo_cost;
  evidence.time_cost = row.time_cost;
  evidence.final_score = row.final_score;
  evidence.margin = row.margin;
  evidence.selected = row.selected;
  evidence.accepted = row.accepted;
  evidence.continuity_used = row.continuity_used;
  evidence.feature_update_allowed = row.feature_update_allowed;
  evidence.geometry_update_allowed = row.geometry_update_allowed;
  evidence.stage = row.stage;
  evidence.reject_reason = row.reject_reason;
  return evidence;
}

std::vector<TrackletObservation> TrackletObservationsFromTracks(const std::vector<Track> &tracks) {
  std::vector<TrackletObservation> observations;
  observations.reserve(tracks.size());
  for (const auto &track : tracks) {
    observations.push_back(TrackletObservation::FromTrack(track));
  }
  return observations;
}

std::string IdentityModeToString(const IdentityManager::Mode mode) {
  switch (mode) {
    case IdentityManager::Mode::kMerged:
      return "MERGED";
    case IdentityManager::Mode::kSplitRecovery:
      return "SPLIT_RECOVERY";
    case IdentityManager::Mode::kNormalResumed:
      return "NORMAL_RESUMED";
    case IdentityManager::Mode::kNormal:
    default:
      return "NORMAL";
  }
}

}  // namespace vision_demo_host
