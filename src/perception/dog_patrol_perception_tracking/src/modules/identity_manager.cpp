#include "vision_demo_host/modules/identity_manager.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <sstream>
#include <set>
#include <unordered_map>
#include <utility>

#include "identity_observation_projection.hpp"
#include "legacy_identity_matcher.hpp"
#include "occlusion_group_shadow_lifecycle.hpp"
#include "phase4_handoff_coordinator.hpp"
#include "phase5_birth_coordinator.hpp"

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
  out.auto_apply_phase5_birth_allocations = false;
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
  out.feature_update_reason = row.feature_update_reason;
  out.geometry_update_reason = row.geometry_update_reason;
  return out;
}

IdentityManager::Phase3ShadowDebugRow FromLegacyPairwiseDebugRow(
    const LegacyIdentityMatcher::PairwiseAssignmentDebugRow &row) {
  IdentityManager::Phase3ShadowDebugRow out;
  out.frame_idx = row.frame_idx - 1;
  out.event_type = "pairwise_assignment_matrix";
  out.reason = row.appearance_override ? "pairwise_appearance_override" : "pairwise_assignment_matrix";
  out.pairwise_selected_pairs = row.selected_pairs;
  out.pairwise_alternate_pairs = row.alternate_pairs;
  out.pairwise_selected_final_cost = row.selected_final_cost;
  out.pairwise_alternate_final_cost = row.alternate_final_cost;
  out.pairwise_selected_app_cost = row.selected_app_cost;
  out.pairwise_alternate_app_cost = row.alternate_app_cost;
  out.pairwise_margin = row.margin;
  out.pairwise_appearance_override = row.appearance_override;
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

std::string JoinSemanticIds(const std::set<int> &semantic_ids) {
  std::ostringstream oss;
  bool first = true;
  for (const int semantic_id : semantic_ids) {
    if (!first) {
      oss << "|";
    }
    first = false;
    oss << semantic_id;
  }
  return oss.str();
}

bool IsSplitCandidateHypothesis(const TrackletHypothesis &hypothesis) {
  if (hypothesis.class_id != ClassId::kPerson || hypothesis.raw_track_id <= 0) {
    return false;
  }
  return hypothesis.status == TrackletHypothesisStatus::kTracked ||
         hypothesis.status == TrackletHypothesisStatus::kSuppressedDuplicateCandidate ||
         hypothesis.status == TrackletHypothesisStatus::kSplitCandidate;
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
  OcclusionGroupShadowLifecycle::State occlusion_group_shadow_state;
  std::map<int, Phase5BirthCoordinator::ShadowState> new_birth_candidate_shadow_by_raw_id;
  int phase3_frame_index{0};
};

IdentityManager::IdentityManager() : impl_(std::make_shared<Impl>()) {}

IdentityManager::IdentityManager(Config config) : impl_(std::make_shared<Impl>(config)) {}

bool IdentityManager::Initialize(std::string *error) { return impl_->legacy_identity_matcher.Initialize(error); }

void IdentityManager::Reset() {
  impl_->legacy_identity_matcher.Reset();
  impl_->last_score_debug_rows.clear();
  impl_->last_phase3_shadow_debug_rows.clear();
  OcclusionGroupShadowLifecycle::Reset(&impl_->occlusion_group_shadow_state);
  impl_->new_birth_candidate_shadow_by_raw_id.clear();
  impl_->phase3_frame_index = 0;
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
  const auto prev_raw_to_semantic = raw_to_semantic_id_;
  raw_to_semantic_id_ = impl_->legacy_identity_matcher.Update(tracks, primary, frame);
  impl_->last_score_debug_rows.clear();
  const auto &legacy_debug_rows = impl_->legacy_identity_matcher.LastScoreDebugRows();
  impl_->last_score_debug_rows.reserve(legacy_debug_rows.size());
  for (const auto &row : legacy_debug_rows) {
    impl_->last_score_debug_rows.push_back(FromLegacyDebugRow(row));
  }

  impl_->last_phase3_shadow_debug_rows.clear();
  impl_->last_phase3_shadow_debug_rows.reserve(shadow_hypotheses.size());
  const int current_frame_idx = impl_->phase3_frame_index;
  int event_idx = 0;
  for (const auto &hypothesis : shadow_hypotheses) {
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = "hypothesis_input";
    row.candidate_raw_track_id = hypothesis.raw_track_id;
    row.candidate_bbox = hypothesis.bbox;
    row.candidate_confidence = hypothesis.confidence;
    row.reason = hypothesis.candidate_reason;
    row.related_raw_track_id = hypothesis.related_raw_track_id.value_or(-1);
    row.hypothesis_status = TrackletHypothesisStatusToDebugString(hypothesis.status);
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  }
  for (const auto &legacy_pairwise_row : impl_->legacy_identity_matcher.LastPairwiseAssignmentDebugRows()) {
    Phase3ShadowDebugRow row = FromLegacyPairwiseDebugRow(legacy_pairwise_row);
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  }

  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id;
  observations_by_raw_track_id.reserve(observations.size());
  for (const auto &obs : observations) {
    observations_by_raw_track_id[obs.raw_track_id] = obs;
  }

  const auto refresh_legacy_debug_rows = [&]() {
    impl_->last_score_debug_rows.clear();
    const auto &rows = impl_->legacy_identity_matcher.LastScoreDebugRows();
    impl_->last_score_debug_rows.reserve(rows.size());
    for (const auto &row : rows) {
      impl_->last_score_debug_rows.push_back(FromLegacyDebugRow(row));
    }
  };

  const auto apply_phase5_birth_manager = [&]() {
    Phase5BirthCoordinator::ApplyInput input;
    input.enabled = true;
    input.current_frame_idx = current_frame_idx;
    input.observations_by_raw_track_id = &observations_by_raw_track_id;
    input.score_rows = &impl_->last_score_debug_rows;
    input.shadow_by_raw_track_id = &impl_->new_birth_candidate_shadow_by_raw_id;
    input.raw_to_semantic_id = &raw_to_semantic_id_;
    Phase5BirthCoordinator::ApplyAcceptedBirths(
        input,
        [&](const int raw_track_id) {
          impl_->legacy_identity_matcher.ApplyPhase5BirthAllocation(tracks, raw_track_id, frame);
        },
        [&]() { refresh_legacy_debug_rows(); });
  };

  const auto build_result_from_legacy = [&]() {
    IdentityObservationProjection::Input input;
    input.snapshots = impl_->legacy_identity_matcher.IdentitySnapshots();
    input.observations_by_raw_track_id = observations_by_raw_track_id;
    input.debug_rows = impl_->last_score_debug_rows;
    input.mode = CurrentMode();
    input.primary_semantic_id = impl_->legacy_identity_matcher.CurrentPrimarySemanticId();
    input.feature_update_frozen = impl_->legacy_identity_matcher.IsFeatureUpdateFrozen();
    input.max_missing_frames = impl_->config.max_missing_frames;
    return IdentityObservationProjection::Build(input);
  };

  apply_phase5_birth_manager();
  IdentityManagerResult result = build_result_from_legacy();
  const auto refresh_outputs = [&]() {
    refresh_legacy_debug_rows();
    result = build_result_from_legacy();
  };

  Phase4HandoffCoordinator::PairwiseInput pairwise_input;
  pairwise_input.enabled = true;
  pairwise_input.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
  pairwise_input.raw_to_semantic_id = &raw_to_semantic_id_;
  pairwise_input.next_event_idx = &event_idx;
  Phase4HandoffCoordinator::ApplyPairwiseAssignment(
      pairwise_input,
      [&](const int first_raw_track_id, const int first_semantic_id,
          const int second_raw_track_id, const int second_semantic_id) {
        return impl_->legacy_identity_matcher.ApplyPhase4PairwiseAssignment(
            tracks, first_raw_track_id, first_semantic_id, second_raw_track_id, second_semantic_id, frame);
      },
      refresh_outputs);

  const auto mode = CurrentMode();
  int phase4_continuity_raw = -1;
  int phase4_continuity_sid = -1;
  if (impl_->occlusion_group_shadow_state.merged_group.active &&
      observations_by_raw_track_id.size() >= 2) {
    for (const auto &[raw_id, semantic_id] : prev_raw_to_semantic) {
      if (impl_->occlusion_group_shadow_state.merged_group.semantic_ids.count(semantic_id) == 0) {
        continue;
      }
      if (observations_by_raw_track_id.find(raw_id) == observations_by_raw_track_id.end()) {
        continue;
      }
      phase4_continuity_raw = raw_id;
      phase4_continuity_sid = semantic_id;
      break;
    }
  }

  std::set<int> person_semantic_ids;
  int carrier_semantic_id = -1;
  int carrier_raw_track_id = -1;
  for (const auto &identity : result.identities) {
    if (identity.class_id != ClassId::kPerson || identity.semantic_id <= 0) {
      continue;
    }
    person_semantic_ids.insert(identity.semantic_id);
    if (carrier_semantic_id < 0 && identity.visible && identity.supporting_raw_track_id.has_value()) {
      carrier_semantic_id = identity.semantic_id;
      carrier_raw_track_id = *identity.supporting_raw_track_id;
    }
  }

  const auto append_single_blob_decision_rows = [&]() {
    if (!impl_->occlusion_group_shadow_state.merged_group.active || observations_by_raw_track_id.size() != 1) {
      return;
    }

    const auto &carrier_entry = *observations_by_raw_track_id.begin();
    const int carrier_raw_id = carrier_entry.first;
    const auto continuity_it = prev_raw_to_semantic.find(carrier_raw_id);
    const int continuity_semantic_id = continuity_it == prev_raw_to_semantic.end() ? -1 : continuity_it->second;
    if (continuity_semantic_id <= 0 ||
        impl_->occlusion_group_shadow_state.merged_group.semantic_ids.count(continuity_semantic_id) == 0) {
      return;
    }

    std::unordered_map<int, int> missing_frames_by_semantic_id;
    for (const auto &snapshot : impl_->legacy_identity_matcher.IdentitySnapshots()) {
      missing_frames_by_semantic_id[snapshot.semantic_id] = snapshot.missing_frames;
    }

    for (const auto &score : impl_->last_score_debug_rows) {
      if (score.stage != "merged_candidate" || score.raw_track_id != carrier_raw_id ||
          impl_->occlusion_group_shadow_state.merged_group.semantic_ids.count(score.semantic_id) == 0) {
        continue;
      }

      std::string reason;
      if (score.selected && score.accepted && score.semantic_id == continuity_semantic_id) {
        reason = "single_blob_continuity_kept";
      } else if (score.selected && score.accepted) {
        reason = "single_blob_handoff_accepted";
      } else if (!score.reject_reason.empty()) {
        reason = "single_blob_rejected_by_appearance_or_geometry_margin";
      } else if (score.semantic_id != continuity_semantic_id &&
                 missing_frames_by_semantic_id[score.semantic_id] < 18) {
        reason = "single_blob_rejected_by_missing_age";
      } else if (score.semantic_id != continuity_semantic_id) {
        reason = "single_blob_handoff_eligible";
      } else {
        reason = "single_blob_continuity_candidate";
      }

      Phase3ShadowDebugRow row;
      row.frame_idx = current_frame_idx;
      row.event_idx = event_idx++;
      row.event_type = "single_blob_handoff_decision";
      row.group_id = impl_->occlusion_group_shadow_state.merged_group.group_id;
      row.semantic_ids = JoinSemanticIds(impl_->occlusion_group_shadow_state.merged_group.semantic_ids);
      row.carrier_semantic_id = continuity_semantic_id;
      row.carrier_raw_track_id = carrier_raw_id;
      row.candidate_raw_track_id = carrier_raw_id;
      row.candidate_semantic_id = score.semantic_id;
      row.candidate_bbox = carrier_entry.second.bbox;
      row.candidate_confidence = carrier_entry.second.confidence;
      row.reason = reason;
      row.related_raw_track_id = carrier_raw_id;
      row.hypothesis_status = "single_blob_visible";
      row.group_age_frames = impl_->occlusion_group_shadow_state.merged_group.age_frames;
      row.group_last_update_frame = impl_->occlusion_group_shadow_state.merged_group.last_update_frame;
      row.decision_app_cost = score.app_cost;
      row.decision_geo_cost = score.geo_cost;
      row.decision_time_cost = score.time_cost;
      row.decision_final_score = score.final_score;
      row.decision_margin = score.margin;
      row.decision_selected = score.selected;
      row.decision_accepted = score.accepted;
      impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
    }
  };

  const auto shadow_rows_context = [&]() {
    OcclusionGroupShadowLifecycle::ShadowRowsContext context;
    context.current_frame_idx = current_frame_idx;
    context.next_event_idx = &event_idx;
    context.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
    return context;
  };

  const auto recovery_group = [&]() {
    return OcclusionGroupShadowLifecycle::RecoveryGroup(impl_->occlusion_group_shadow_state, current_frame_idx);
  };

  const auto append_side_reappearance_rows = [&]() {
    const OcclusionGroupShadowLifecycle::MergedGroupShadowState *group = recovery_group();
    if (group == nullptr || group->group_id <= 0) {
      return;
    }

    for (const auto &hypothesis : shadow_hypotheses) {
      if (hypothesis.class_id != ClassId::kPerson || hypothesis.raw_track_id <= 0) {
        continue;
      }

      const ScoreDebugRow *side_recovery_row = nullptr;
      for (const auto &row : impl_->last_score_debug_rows) {
        if (row.raw_track_id == hypothesis.raw_track_id && row.stage == "merged_side_recovery" && row.accepted) {
          side_recovery_row = &row;
          break;
        }
      }
      if (side_recovery_row == nullptr) {
        continue;
      }
      Phase3ShadowDebugRow row;
      row.frame_idx = current_frame_idx;
      row.event_idx = event_idx++;
      row.event_type = "side_reappearance_candidate";
      row.group_id = group->group_id;
      row.semantic_ids = JoinSemanticIds(group->semantic_ids);
      row.carrier_semantic_id = group->carrier_semantic_id;
      row.carrier_raw_track_id = group->carrier_raw_track_id;
      row.candidate_raw_track_id = hypothesis.raw_track_id;
      row.candidate_semantic_id = side_recovery_row->semantic_id;
      row.candidate_bbox = hypothesis.bbox;
      row.candidate_confidence = hypothesis.confidence;
      row.reason = "side_reappearance_candidate";
      row.related_raw_track_id = group->carrier_raw_track_id;
      row.hypothesis_status = TrackletHypothesisStatusToDebugString(hypothesis.status);
      row.candidate_stable_frames = 1;
      row.group_age_frames = group->age_frames;
      row.group_last_update_frame = group->last_update_frame;
      impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
    }
  };

  OcclusionGroupShadowLifecycle::SyncGroupModeInput sync_input;
  sync_input.mode = mode;
  sync_input.person_semantic_ids = person_semantic_ids;
  sync_input.carrier_semantic_id = carrier_semantic_id;
  sync_input.carrier_raw_track_id = carrier_raw_track_id;
  sync_input.phase4_continuity_raw = phase4_continuity_raw;
  OcclusionGroupShadowLifecycle::SyncGroupMode(
      &impl_->occlusion_group_shadow_state, sync_input, shadow_rows_context());

  append_single_blob_decision_rows();
  Phase4HandoffCoordinator::SingleBlobInput single_blob_input;
  single_blob_input.enabled = true;
  single_blob_input.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
  single_blob_input.raw_to_semantic_id = &raw_to_semantic_id_;
  single_blob_input.next_event_idx = &event_idx;
  Phase4HandoffCoordinator::ApplyMergedSingleBlobHandoff(
      single_blob_input,
      [&](const int carrier_raw_track_id, const int carrier_semantic_id, const int candidate_semantic_id) {
        return impl_->legacy_identity_matcher.ApplyPhase4MergedSingleBlobHandoff(
            tracks, carrier_raw_track_id, carrier_semantic_id, candidate_semantic_id, frame);
      },
      refresh_outputs);

  Phase4HandoffCoordinator::SideRecoveryInput side_recovery_input;
  side_recovery_input.enabled = true;
  side_recovery_input.current_frame_idx = current_frame_idx;
  side_recovery_input.shadow_hypotheses = &shadow_hypotheses;
  side_recovery_input.observations_by_raw_track_id = &observations_by_raw_track_id;
  side_recovery_input.score_rows = &impl_->last_score_debug_rows;
  side_recovery_input.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
  side_recovery_input.raw_to_semantic_id = &raw_to_semantic_id_;
  side_recovery_input.occlusion_state = &impl_->occlusion_group_shadow_state;
  side_recovery_input.next_event_idx = &event_idx;
  Phase4HandoffCoordinator::ApplyMergedSideRecovery(
      side_recovery_input,
      [&](const int carrier_raw_track_id, const int carrier_semantic_id,
          const int candidate_raw_track_id, const int candidate_semantic_id) {
        return impl_->legacy_identity_matcher.ApplyPhase4MergedSideRecovery(
            tracks, carrier_raw_track_id, carrier_semantic_id, candidate_raw_track_id, candidate_semantic_id, frame);
      },
      refresh_outputs);
  append_side_reappearance_rows();

  if (impl_->occlusion_group_shadow_state.merged_group.active) {
    OcclusionGroupShadowLifecycle::MarkSplitCandidatesUnseen(&impl_->occlusion_group_shadow_state);

    for (const auto &hypothesis : shadow_hypotheses) {
      if (!IsSplitCandidateHypothesis(hypothesis)) {
        continue;
      }

      const int related_raw_track_id = hypothesis.related_raw_track_id.value_or(-1);
      const bool linked_to_carrier =
          related_raw_track_id == impl_->occlusion_group_shadow_state.merged_group.carrier_raw_track_id;
      const bool is_carrier_raw =
          hypothesis.raw_track_id == impl_->occlusion_group_shadow_state.merged_group.carrier_raw_track_id;
      if (is_carrier_raw || hypothesis.raw_track_id == phase4_continuity_raw) {
        continue;
      }
      const bool known_group_raw =
          raw_to_semantic_id_.find(hypothesis.raw_track_id) != raw_to_semantic_id_.end() &&
          impl_->occlusion_group_shadow_state.merged_group.semantic_ids.count(raw_to_semantic_id_[hypothesis.raw_track_id]) > 0;
      const bool tracked_split_candidate =
          hypothesis.status == TrackletHypothesisStatus::kTracked && observations_by_raw_track_id.size() >= 2;
      if (!linked_to_carrier && !known_group_raw && !tracked_split_candidate) {
        continue;
      }

      int candidate_semantic_id = -1;
      const auto semantic_it = raw_to_semantic_id_.find(hypothesis.raw_track_id);
      if (semantic_it != raw_to_semantic_id_.end() &&
          impl_->occlusion_group_shadow_state.merged_group.semantic_ids.count(semantic_it->second) > 0) {
        candidate_semantic_id = semantic_it->second;
      } else {
        for (const int semantic_id : impl_->occlusion_group_shadow_state.merged_group.semantic_ids) {
          if (semantic_id != impl_->occlusion_group_shadow_state.merged_group.carrier_semantic_id) {
            candidate_semantic_id = semantic_id;
            break;
          }
        }
      }
      OcclusionGroupShadowLifecycle::ObserveSplitCandidateInput candidate_input;
      candidate_input.hypothesis = hypothesis;
      candidate_input.related_raw_track_id = related_raw_track_id;
      candidate_input.candidate_semantic_id = candidate_semantic_id;
      OcclusionGroupShadowLifecycle::ObserveSplitCandidate(
          &impl_->occlusion_group_shadow_state, candidate_input, shadow_rows_context());
    }

    Phase4HandoffCoordinator::MergedSplitInput merged_split_input;
    merged_split_input.enabled = true;
    merged_split_input.current_frame_idx = current_frame_idx;
    merged_split_input.phase4_continuity_raw = phase4_continuity_raw;
    merged_split_input.phase4_continuity_sid = phase4_continuity_sid;
    merged_split_input.prev_raw_to_semantic = &prev_raw_to_semantic;
    merged_split_input.observations_by_raw_track_id = &observations_by_raw_track_id;
    merged_split_input.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
    merged_split_input.raw_to_semantic_id = &raw_to_semantic_id_;
    merged_split_input.occlusion_state = &impl_->occlusion_group_shadow_state;
    merged_split_input.next_event_idx = &event_idx;
    Phase4HandoffCoordinator::ApplyMergedSplitHandoff(
        merged_split_input,
        [&](const int continuity_raw_track_id, const int exposed_partial_sid,
            const int candidate_raw_track_id, const int continuity_sid) {
          return impl_->legacy_identity_matcher.ApplyPhase4MergedSplitHandoff(
              tracks, continuity_raw_track_id, exposed_partial_sid, candidate_raw_track_id, continuity_sid, frame);
        },
        refresh_outputs);

    OcclusionGroupShadowLifecycle::EndMissingSplitCandidates(&impl_->occlusion_group_shadow_state,
                                                             shadow_rows_context());
  }

  Phase5BirthCoordinator::ShadowRowsInput shadow_rows_input;
  shadow_rows_input.current_frame_idx = current_frame_idx;
  shadow_rows_input.observations = &observations;
  shadow_rows_input.observations_by_raw_track_id = &observations_by_raw_track_id;
  shadow_rows_input.raw_to_semantic_id = &raw_to_semantic_id_;
  shadow_rows_input.score_rows = &impl_->last_score_debug_rows;
  shadow_rows_input.shadow_by_raw_track_id = &impl_->new_birth_candidate_shadow_by_raw_id;
  shadow_rows_input.phase3_rows = &impl_->last_phase3_shadow_debug_rows;
  shadow_rows_input.next_event_idx = &event_idx;
  Phase5BirthCoordinator::AppendShadowLifecycleRows(shadow_rows_input);

  impl_->phase3_frame_index += 1;
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
  evidence.feature_update_reason = row.feature_update_reason;
  evidence.geometry_update_reason = row.geometry_update_reason;
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
