#include "vision_demo_host/modules/identity_manager.hpp"

#include <algorithm>
#include <map>
#include <sstream>
#include <optional>
#include <set>
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
  out.disable_legacy_merged_split_handoff = config.enable_phase4_merged_split_handoff;
  out.disable_legacy_merged_side_recovery = config.enable_phase4_merged_side_recovery;
  out.disable_legacy_merged_single_blob_handoff = config.enable_phase4_merged_single_blob_handoff;
  out.disable_legacy_pairwise_assignment = config.enable_phase4_pairwise_assignment;
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

std::vector<std::pair<int, int>> ParsePairwisePairs(const std::string &pairs) {
  std::vector<std::pair<int, int>> out;
  std::stringstream ss(pairs);
  std::string token;
  while (std::getline(ss, token, '|')) {
    const auto sep = token.find("->");
    if (sep == std::string::npos) {
      return {};
    }
    try {
      const int raw_id = std::stoi(token.substr(0, sep));
      const int semantic_id = std::stoi(token.substr(sep + 2));
      out.emplace_back(raw_id, semantic_id);
    } catch (...) {
      return {};
    }
  }
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

bool LooksLikeMergedSplitHandoff(const cv::Rect2f &carrier_bbox, const cv::Rect2f &candidate_bbox) {
  const float carrier_area = std::max(1.0F, carrier_bbox.area());
  const float candidate_area = std::max(1.0F, candidate_bbox.area());
  const float carrier_aspect = carrier_bbox.width / std::max(1.0F, carrier_bbox.height);
  const bool candidate_is_taller_and_higher =
      candidate_bbox.height >= carrier_bbox.height * 1.08F &&
      candidate_bbox.y + carrier_bbox.height * 0.05F < carrier_bbox.y;
  return carrier_aspect <= 0.40F &&
         (carrier_area <= 0.85F * candidate_area || candidate_is_taller_and_higher);
}

bool LooksLikeMergedSideRecovery(const cv::Rect2f &carrier_bbox, const cv::Rect2f &candidate_bbox,
                                 const float candidate_confidence) {
  const float candidate_height = std::max(1.0F, candidate_bbox.height);
  const float candidate_aspect = candidate_bbox.width / candidate_height;
  if (candidate_confidence < 0.45F || candidate_height < 300.0F || candidate_aspect < 0.25F ||
      candidate_aspect > 0.70F) {
    return false;
  }

  const float vertical_overlap =
      std::max(0.0F, std::min(candidate_bbox.y + candidate_bbox.height, carrier_bbox.y + carrier_bbox.height) -
                         std::max(candidate_bbox.y, carrier_bbox.y));
  if (vertical_overlap / candidate_height < 0.45F) {
    return false;
  }

  const float carrier_left = carrier_bbox.x;
  const float carrier_right = carrier_bbox.x + carrier_bbox.width;
  const float candidate_left = candidate_bbox.x;
  const float candidate_right = candidate_bbox.x + candidate_bbox.width;
  const bool touches_carrier_side =
      (candidate_left < carrier_left && candidate_right > carrier_left) ||
      (candidate_left < carrier_right && candidate_right > carrier_right);
  const float side_gap = std::min(std::abs(candidate_right - carrier_left),
                                  std::abs(candidate_left - carrier_right));
  const bool close_to_carrier_side = side_gap <= std::max(25.0F, carrier_bbox.width * 0.20F);
  return touches_carrier_side || close_to_carrier_side;
}

}  // namespace

class IdentityManager::Impl {
 public:
  Impl() = default;
  explicit Impl(Config config) : config(std::move(config)), legacy_identity_matcher(ToLegacyConfig(this->config)) {}

  struct MergedGroupShadowState {
    int group_id{-1};
    std::set<int> semantic_ids;
    int carrier_semantic_id{-1};
    int carrier_raw_track_id{-1};
    int age_frames{0};
    int last_update_frame{-1};
    bool active{false};
  };

  struct SplitCandidateShadowState {
    int group_id{-1};
    int candidate_raw_track_id{-1};
    int candidate_semantic_id{-1};
    cv::Rect2f bbox;
    float confidence{0.0F};
    std::string reason;
    int related_raw_track_id{-1};
    std::string hypothesis_status;
    int stable_frames{0};
    int last_update_frame{-1};
    bool seen_this_frame{false};
  };

  struct NewBirthCandidateShadowState {
    cv::Rect2f bbox;
    float confidence{0.0F};
    int stable_frames{0};
    int last_update_frame{-1};
  };

  Config config;
  LegacyIdentityMatcher legacy_identity_matcher;
  std::vector<ScoreDebugRow> last_score_debug_rows;
  std::vector<Phase3ShadowDebugRow> last_phase3_shadow_debug_rows;
  MergedGroupShadowState merged_group_shadow;
  MergedGroupShadowState recent_merged_group_shadow;
  bool has_recent_merged_group_shadow{false};
  std::map<int, SplitCandidateShadowState> split_candidate_shadow_by_raw_id;
  std::map<int, NewBirthCandidateShadowState> new_birth_candidate_shadow_by_raw_id;
  int next_merged_group_id{1};
  int phase3_frame_index{0};
};

IdentityManager::IdentityManager() : impl_(std::make_shared<Impl>()) {}

IdentityManager::IdentityManager(Config config) : impl_(std::make_shared<Impl>(config)) {}

bool IdentityManager::Initialize(std::string *error) { return impl_->legacy_identity_matcher.Initialize(error); }

void IdentityManager::Reset() {
  impl_->legacy_identity_matcher.Reset();
  impl_->last_score_debug_rows.clear();
  impl_->last_phase3_shadow_debug_rows.clear();
  impl_->merged_group_shadow = Impl::MergedGroupShadowState{};
  impl_->recent_merged_group_shadow = Impl::MergedGroupShadowState{};
  impl_->has_recent_merged_group_shadow = false;
  impl_->split_candidate_shadow_by_raw_id.clear();
  impl_->new_birth_candidate_shadow_by_raw_id.clear();
  impl_->next_merged_group_id = 1;
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

  const auto build_result_from_legacy = [&]() {
    IdentityManagerResult built;
    built.primary_semantic_id = impl_->legacy_identity_matcher.CurrentPrimarySemanticId();
    built.feature_update_frozen = impl_->legacy_identity_matcher.IsFeatureUpdateFrozen();
    const auto mode = CurrentMode();
    const auto snapshots = impl_->legacy_identity_matcher.IdentitySnapshots();
    built.identities.reserve(snapshots.size());
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
      identity.primary = (built.primary_semantic_id > 0 && identity.semantic_id == built.primary_semantic_id);
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

      built.identities.push_back(std::move(identity));
    }
    return built;
  };

  IdentityManagerResult result = build_result_from_legacy();

  const auto append_phase4_pairwise_rows = [&](const Phase3ShadowDebugRow &matrix_row,
                                               const std::vector<std::pair<int, int>> &pairs) {
    for (std::size_t i = 0; i < pairs.size(); ++i) {
      Phase3ShadowDebugRow row = matrix_row;
      row.event_idx = event_idx++;
      row.event_type = "phase4_pairwise_assignment";
      row.reason = "pairwise_appearance_override";
      row.candidate_raw_track_id = pairs[i].first;
      row.candidate_semantic_id = pairs[i].second;
      row.related_raw_track_id = pairs[pairs.size() - 1 - i].first;
      impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
    }
  };

  const auto apply_phase4_pairwise_assignment = [&]() {
    if (!impl_->config.enable_phase4_pairwise_assignment) {
      return;
    }
    for (const auto &row : impl_->last_phase3_shadow_debug_rows) {
      if (row.event_type != "pairwise_assignment_matrix" || !row.pairwise_appearance_override) {
        continue;
      }
      const auto alternate_pairs = ParsePairwisePairs(row.pairwise_alternate_pairs);
      if (alternate_pairs.size() != 2U || alternate_pairs[0].first <= 0 || alternate_pairs[0].second <= 0 ||
          alternate_pairs[1].first <= 0 || alternate_pairs[1].second <= 0 ||
          alternate_pairs[0].first == alternate_pairs[1].first ||
          alternate_pairs[0].second == alternate_pairs[1].second) {
        continue;
      }
      const bool applied = impl_->legacy_identity_matcher.ApplyPhase4PairwiseAssignment(
          tracks, alternate_pairs[0].first, alternate_pairs[0].second,
          alternate_pairs[1].first, alternate_pairs[1].second, frame);
      if (!applied) {
        continue;
      }
      raw_to_semantic_id_[alternate_pairs[0].first] = alternate_pairs[0].second;
      raw_to_semantic_id_[alternate_pairs[1].first] = alternate_pairs[1].second;
      const Phase3ShadowDebugRow matrix_row = row;
      refresh_legacy_debug_rows();
      result = build_result_from_legacy();
      append_phase4_pairwise_rows(matrix_row, alternate_pairs);
      break;
    }
  };

  apply_phase4_pairwise_assignment();

  const auto mode = CurrentMode();
  int phase4_continuity_raw = -1;
  int phase4_continuity_sid = -1;
  if (impl_->config.enable_phase4_merged_split_handoff && impl_->merged_group_shadow.active &&
      observations_by_raw_track_id.size() >= 2) {
    for (const auto &[raw_id, semantic_id] : prev_raw_to_semantic) {
      if (impl_->merged_group_shadow.semantic_ids.count(semantic_id) == 0) {
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

  const auto append_group_row = [&](const std::string &event_type, const std::string &reason) {
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = event_type;
    row.group_id = impl_->merged_group_shadow.group_id;
    row.semantic_ids = JoinSemanticIds(impl_->merged_group_shadow.semantic_ids);
    row.carrier_semantic_id = impl_->merged_group_shadow.carrier_semantic_id;
    row.carrier_raw_track_id = impl_->merged_group_shadow.carrier_raw_track_id;
    row.reason = reason;
    row.group_age_frames = impl_->merged_group_shadow.age_frames;
    row.group_last_update_frame = impl_->merged_group_shadow.last_update_frame;
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  };

  const auto new_birth_hidden_status = [](const std::string &reason) {
    if (reason == "ambiguous_recovery_pending") {
      return "pending_recovery";
    }
    if (reason == "duplicate_split_hidden") {
      return "hidden_duplicate_split";
    }
    if (reason == "skinny_partial_hidden" || reason == "wide_fragment_hidden") {
      return "hidden_partial_fragment";
    }
    return "hidden";
  };

  const auto append_new_birth_candidate_row = [&](const ScoreDebugRow &score,
                                                  const TrackletObservation *observation,
                                                  const std::string &event_type,
                                                  const std::string &reason,
                                                  const std::string &status,
                                                  const int stable_frames) {
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = event_type;
    row.candidate_raw_track_id = score.raw_track_id;
    row.candidate_semantic_id = score.semantic_id;
    if (observation != nullptr) {
      row.candidate_bbox = observation->bbox;
      row.candidate_confidence = observation->confidence;
    }
    row.reason = reason;
    row.hypothesis_status = status;
    row.candidate_stable_frames = stable_frames;
    row.decision_app_cost = score.app_cost;
    row.decision_geo_cost = score.geo_cost;
    row.decision_time_cost = score.time_cost;
    row.decision_final_score = score.final_score;
    row.decision_margin = score.margin;
    row.decision_selected = score.selected;
    row.decision_accepted = score.accepted;
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  };

  const auto append_phase5_new_birth_candidate_rows = [&]() {
    std::set<int> scored_new_birth_raw_ids;
    for (const auto &score : impl_->last_score_debug_rows) {
      if (score.stage != "birth_candidate" && score.stage != "new_semantic") {
        continue;
      }
      if (score.raw_track_id <= 0) {
        continue;
      }
      scored_new_birth_raw_ids.insert(score.raw_track_id);
      const auto obs_it = observations_by_raw_track_id.find(score.raw_track_id);
      const TrackletObservation *observation =
          obs_it == observations_by_raw_track_id.end() ? nullptr : &obs_it->second;
      const auto pending_it = impl_->new_birth_candidate_shadow_by_raw_id.find(score.raw_track_id);
      const int stable_frames = pending_it == impl_->new_birth_candidate_shadow_by_raw_id.end()
                                    ? 0
                                    : pending_it->second.stable_frames + 1;
      if (score.stage == "birth_candidate") {
        append_new_birth_candidate_row(score, observation, "new_birth_candidate_hidden",
                                       score.reject_reason.empty() ? "birth_candidate_hidden"
                                                                   : score.reject_reason,
                                       new_birth_hidden_status(score.reject_reason), 0);
        impl_->new_birth_candidate_shadow_by_raw_id.erase(score.raw_track_id);
        continue;
      }
      if (score.stage == "new_semantic" && score.accepted) {
        append_new_birth_candidate_row(score, observation, "new_birth_candidate_allocated",
                                       pending_it == impl_->new_birth_candidate_shadow_by_raw_id.end()
                                           ? "new_semantic_allocated"
                                           : "small_stable_new_person_promoted",
                                       "allocated", stable_frames);
        impl_->new_birth_candidate_shadow_by_raw_id.erase(score.raw_track_id);
      }
    }

    for (const auto &obs : observations) {
      if (obs.class_id != ClassId::kPerson || obs.raw_track_id <= 0 ||
          raw_to_semantic_id_.find(obs.raw_track_id) != raw_to_semantic_id_.end() ||
          scored_new_birth_raw_ids.count(obs.raw_track_id) > 0) {
        continue;
      }

      auto &candidate = impl_->new_birth_candidate_shadow_by_raw_id[obs.raw_track_id];
      if (candidate.last_update_frame != current_frame_idx - 1) {
        candidate.stable_frames = 0;
      }
      candidate.stable_frames += 1;
      candidate.bbox = obs.bbox;
      candidate.confidence = obs.confidence;
      candidate.last_update_frame = current_frame_idx;

      ScoreDebugRow pending_score;
      pending_score.raw_track_id = obs.raw_track_id;
      pending_score.semantic_id = -1;
      append_new_birth_candidate_row(pending_score, &obs, "new_birth_candidate_pending",
                                     "small_new_person_pending", "pending_stability",
                                     candidate.stable_frames);
    }

    for (auto it = impl_->new_birth_candidate_shadow_by_raw_id.begin();
         it != impl_->new_birth_candidate_shadow_by_raw_id.end();) {
      if (it->second.last_update_frame == current_frame_idx) {
        ++it;
        continue;
      }
      it = impl_->new_birth_candidate_shadow_by_raw_id.erase(it);
    }
  };

  const auto append_split_candidate_row = [&](const std::string &event_type,
                                              const Impl::SplitCandidateShadowState &candidate,
                                              const std::string &reason_override = {}) {
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = event_type;
    row.group_id = candidate.group_id;
    row.semantic_ids = JoinSemanticIds(impl_->merged_group_shadow.semantic_ids);
    row.carrier_semantic_id = impl_->merged_group_shadow.carrier_semantic_id;
    row.carrier_raw_track_id = impl_->merged_group_shadow.carrier_raw_track_id;
    row.candidate_raw_track_id = candidate.candidate_raw_track_id;
    row.candidate_semantic_id = candidate.candidate_semantic_id;
    row.candidate_bbox = candidate.bbox;
    row.candidate_confidence = candidate.confidence;
    row.reason = reason_override.empty() ? candidate.reason : reason_override;
    row.related_raw_track_id = candidate.related_raw_track_id;
    row.hypothesis_status = candidate.hypothesis_status;
    row.candidate_stable_frames = candidate.stable_frames;
    row.group_age_frames = impl_->merged_group_shadow.age_frames;
    row.group_last_update_frame = impl_->merged_group_shadow.last_update_frame;
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  };

  const auto append_single_blob_decision_rows = [&]() {
    if (!impl_->merged_group_shadow.active || observations_by_raw_track_id.size() != 1) {
      return;
    }

    const auto &carrier_entry = *observations_by_raw_track_id.begin();
    const int carrier_raw_id = carrier_entry.first;
    const auto continuity_it = prev_raw_to_semantic.find(carrier_raw_id);
    const int continuity_semantic_id = continuity_it == prev_raw_to_semantic.end() ? -1 : continuity_it->second;
    if (continuity_semantic_id <= 0 ||
        impl_->merged_group_shadow.semantic_ids.count(continuity_semantic_id) == 0) {
      return;
    }

    std::unordered_map<int, int> missing_frames_by_semantic_id;
    for (const auto &snapshot : impl_->legacy_identity_matcher.IdentitySnapshots()) {
      missing_frames_by_semantic_id[snapshot.semantic_id] = snapshot.missing_frames;
    }

    for (const auto &score : impl_->last_score_debug_rows) {
      if (score.stage != "merged_candidate" || score.raw_track_id != carrier_raw_id ||
          impl_->merged_group_shadow.semantic_ids.count(score.semantic_id) == 0) {
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
      row.group_id = impl_->merged_group_shadow.group_id;
      row.semantic_ids = JoinSemanticIds(impl_->merged_group_shadow.semantic_ids);
      row.carrier_semantic_id = continuity_semantic_id;
      row.carrier_raw_track_id = carrier_raw_id;
      row.candidate_raw_track_id = carrier_raw_id;
      row.candidate_semantic_id = score.semantic_id;
      row.candidate_bbox = carrier_entry.second.bbox;
      row.candidate_confidence = carrier_entry.second.confidence;
      row.reason = reason;
      row.related_raw_track_id = carrier_raw_id;
      row.hypothesis_status = "single_blob_visible";
      row.group_age_frames = impl_->merged_group_shadow.age_frames;
      row.group_last_update_frame = impl_->merged_group_shadow.last_update_frame;
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

  const auto append_phase4_single_blob_handoff_row = [&](const Phase3ShadowDebugRow &decision_row) {
    Phase3ShadowDebugRow row = decision_row;
    row.event_idx = event_idx++;
    row.event_type = "phase4_merged_single_blob_handoff";
    row.reason = "merged_single_blob_handoff";
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  };

  const auto apply_phase4_single_blob_handoff = [&]() {
    if (!impl_->config.enable_phase4_merged_single_blob_handoff) {
      return;
    }

    const Phase3ShadowDebugRow *continuity_decision = nullptr;
    for (int i = 0; i < static_cast<int>(impl_->last_phase3_shadow_debug_rows.size()); ++i) {
      const auto &row = impl_->last_phase3_shadow_debug_rows[static_cast<std::size_t>(i)];
      if (row.event_type == "single_blob_handoff_decision" &&
          row.reason == "single_blob_continuity_kept" &&
          row.carrier_raw_track_id > 0 &&
          row.carrier_semantic_id > 0 &&
          row.candidate_semantic_id == row.carrier_semantic_id) {
        continuity_decision = &row;
      }
    }

    int accepted_decision_index = -1;
    for (int i = 0; i < static_cast<int>(impl_->last_phase3_shadow_debug_rows.size()); ++i) {
      const auto &row = impl_->last_phase3_shadow_debug_rows[static_cast<std::size_t>(i)];
      if (row.event_type != "single_blob_handoff_decision" ||
          (row.reason != "single_blob_handoff_accepted" &&
           row.reason != "single_blob_handoff_eligible") ||
          row.carrier_raw_track_id <= 0 || row.carrier_semantic_id <= 0 ||
          row.candidate_semantic_id <= 0 ||
          row.carrier_semantic_id == row.candidate_semantic_id) {
        continue;
      }
      if (row.reason == "single_blob_handoff_eligible" && continuity_decision != nullptr) {
        const bool handoff_margin_ok =
            row.decision_geo_cost <= 0.75F &&
            row.decision_final_score <= continuity_decision->decision_final_score + 0.04F &&
            row.decision_app_cost + 0.025F <= continuity_decision->decision_app_cost;
        if (!handoff_margin_ok) {
          continue;
        }
      }
      accepted_decision_index = i;
      break;
    }
    if (accepted_decision_index < 0) {
      return;
    }
    auto &accepted_decision =
        impl_->last_phase3_shadow_debug_rows[static_cast<std::size_t>(accepted_decision_index)];

    const bool applied = impl_->legacy_identity_matcher.ApplyPhase4MergedSingleBlobHandoff(
        tracks, accepted_decision.carrier_raw_track_id, accepted_decision.carrier_semantic_id,
        accepted_decision.candidate_semantic_id, frame);
    if (!applied) {
      return;
    }

    accepted_decision.reason = "single_blob_handoff_accepted";
    accepted_decision.decision_selected = true;
    accepted_decision.decision_accepted = true;
    raw_to_semantic_id_[accepted_decision.carrier_raw_track_id] = accepted_decision.candidate_semantic_id;
    refresh_legacy_debug_rows();
    result = build_result_from_legacy();
    append_phase4_single_blob_handoff_row(accepted_decision);
  };

  const auto recovery_group = [&]() -> const Impl::MergedGroupShadowState * {
    if (impl_->merged_group_shadow.active) {
      return &impl_->merged_group_shadow;
    }
    if (impl_->has_recent_merged_group_shadow &&
        current_frame_idx - impl_->recent_merged_group_shadow.last_update_frame <= 2) {
      return &impl_->recent_merged_group_shadow;
    }
    return nullptr;
  };

  const auto side_recovery_carrier = [&](const Impl::MergedGroupShadowState &group,
                                         const int candidate_raw_track_id) {
    int related_raw_track_id = group.carrier_raw_track_id;
    int related_semantic_id = group.carrier_semantic_id;
    for (const auto &row : impl_->last_score_debug_rows) {
      if (row.raw_track_id == candidate_raw_track_id || !row.accepted ||
          group.semantic_ids.count(row.semantic_id) == 0) {
        continue;
      }
      if (row.stage == "raw_continuity" || row.stage == "assign_candidate" ||
          row.stage == "phase4_merged_split_handoff") {
        related_raw_track_id = row.raw_track_id;
        related_semantic_id = row.semantic_id;
        break;
      }
    }
    return std::pair<int, int>{related_raw_track_id, related_semantic_id};
  };

  const auto append_side_recovery_row = [&](const std::string &event_type,
                                            const Impl::MergedGroupShadowState &group,
                                            const TrackletHypothesis &hypothesis,
                                            const int candidate_semantic_id,
                                            const int related_raw_track_id,
                                            const int related_semantic_id,
                                            const std::string &reason) {
    const auto candidate_it = impl_->split_candidate_shadow_by_raw_id.find(hypothesis.raw_track_id);
    Phase3ShadowDebugRow row;
    row.frame_idx = current_frame_idx;
    row.event_idx = event_idx++;
    row.event_type = event_type;
    row.group_id = group.group_id;
    row.semantic_ids = JoinSemanticIds(group.semantic_ids);
    row.carrier_semantic_id = related_semantic_id;
    row.carrier_raw_track_id = related_raw_track_id;
    row.candidate_raw_track_id = hypothesis.raw_track_id;
    row.candidate_semantic_id = candidate_semantic_id;
    row.candidate_bbox = hypothesis.bbox;
    row.candidate_confidence = hypothesis.confidence;
    row.reason = reason;
    row.related_raw_track_id = related_raw_track_id;
    row.hypothesis_status = TrackletHypothesisStatusToDebugString(hypothesis.status);
    row.candidate_stable_frames =
        candidate_it == impl_->split_candidate_shadow_by_raw_id.end() ? 1 : candidate_it->second.stable_frames;
    row.group_age_frames = group.age_frames;
    row.group_last_update_frame = group.last_update_frame;
    impl_->last_phase3_shadow_debug_rows.push_back(std::move(row));
  };

  const auto append_side_reappearance_rows = [&]() {
    const Impl::MergedGroupShadowState *group = recovery_group();
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

      const auto [related_raw_track_id, related_semantic_id] =
          side_recovery_carrier(*group, hypothesis.raw_track_id);
      append_side_recovery_row("side_reappearance_candidate", *group, hypothesis, side_recovery_row->semantic_id,
                               related_raw_track_id, related_semantic_id, "side_reappearance_candidate");
    }
  };

  const auto apply_phase4_side_recovery = [&]() {
    if (!impl_->config.enable_phase4_merged_side_recovery) {
      return;
    }
    const Impl::MergedGroupShadowState *group = recovery_group();
    if (group == nullptr || group->group_id <= 0 || group->semantic_ids.size() < 2) {
      return;
    }

    for (const auto &hypothesis : shadow_hypotheses) {
      if (hypothesis.class_id != ClassId::kPerson || hypothesis.raw_track_id <= 0 ||
          hypothesis.status != TrackletHypothesisStatus::kTracked ||
          hypothesis.raw_track_id == group->carrier_raw_track_id ||
          raw_to_semantic_id_.find(hypothesis.raw_track_id) != raw_to_semantic_id_.end()) {
        continue;
      }

      const auto [related_raw_track_id, related_semantic_id] =
          side_recovery_carrier(*group, hypothesis.raw_track_id);
      if (related_raw_track_id <= 0 || related_semantic_id <= 0 ||
          group->semantic_ids.count(related_semantic_id) == 0) {
        continue;
      }
      const auto carrier_obs_it = observations_by_raw_track_id.find(related_raw_track_id);
      if (carrier_obs_it == observations_by_raw_track_id.end() ||
          !LooksLikeMergedSideRecovery(carrier_obs_it->second.bbox, hypothesis.bbox, hypothesis.confidence)) {
        continue;
      }

      int candidate_semantic_id = -1;
      for (const int semantic_id : group->semantic_ids) {
        if (semantic_id != related_semantic_id) {
          candidate_semantic_id = semantic_id;
          break;
        }
      }
      if (candidate_semantic_id <= 0) {
        continue;
      }

      const bool applied = impl_->legacy_identity_matcher.ApplyPhase4MergedSideRecovery(
          tracks, related_raw_track_id, related_semantic_id, hypothesis.raw_track_id,
          candidate_semantic_id, frame);
      if (!applied) {
        continue;
      }
      raw_to_semantic_id_[hypothesis.raw_track_id] = candidate_semantic_id;
      refresh_legacy_debug_rows();
      result = build_result_from_legacy();
      append_side_recovery_row("phase4_merged_side_recovery", *group, hypothesis,
                               candidate_semantic_id, related_raw_track_id,
                               related_semantic_id, "merged_side_recovery");
      break;
    }
  };

  const bool legacy_group_mode = mode == Mode::kMerged || mode == Mode::kSplitRecovery;
  if (legacy_group_mode) {
    if (!impl_->merged_group_shadow.active) {
      impl_->merged_group_shadow = Impl::MergedGroupShadowState{};
      impl_->merged_group_shadow.group_id = impl_->next_merged_group_id++;
      impl_->merged_group_shadow.active = true;
      impl_->merged_group_shadow.age_frames = 1;
      impl_->merged_group_shadow.semantic_ids = std::move(person_semantic_ids);
      impl_->merged_group_shadow.carrier_semantic_id = carrier_semantic_id;
      impl_->merged_group_shadow.carrier_raw_track_id = carrier_raw_track_id;
      impl_->merged_group_shadow.last_update_frame = current_frame_idx;
      append_group_row("merged_group_enter", mode == Mode::kMerged ? "legacy_mode_merged_enter"
                                                                   : "legacy_mode_split_recovery_enter");
    } else {
      impl_->merged_group_shadow.age_frames += 1;
      impl_->merged_group_shadow.semantic_ids.insert(person_semantic_ids.begin(), person_semantic_ids.end());
      if (carrier_semantic_id > 0 && phase4_continuity_raw <= 0) {
        impl_->merged_group_shadow.carrier_semantic_id = carrier_semantic_id;
        impl_->merged_group_shadow.carrier_raw_track_id = carrier_raw_track_id;
      }
      impl_->merged_group_shadow.last_update_frame = current_frame_idx;
      append_group_row("merged_group_update", mode == Mode::kMerged ? "legacy_mode_merged_hold"
                                                                    : "legacy_mode_split_recovery_hold");
    }
  } else if (impl_->merged_group_shadow.active) {
    impl_->merged_group_shadow.last_update_frame = current_frame_idx;
    for (auto &entry : impl_->split_candidate_shadow_by_raw_id) {
      append_split_candidate_row("split_candidate_end", entry.second, "group_end");
    }
    impl_->split_candidate_shadow_by_raw_id.clear();
    append_group_row("merged_group_end", mode == Mode::kNormalResumed ? "legacy_mode_normal_resumed"
                                                                      : "legacy_mode_normal");
    impl_->recent_merged_group_shadow = impl_->merged_group_shadow;
    impl_->has_recent_merged_group_shadow = true;
    impl_->merged_group_shadow = Impl::MergedGroupShadowState{};
  }

  append_single_blob_decision_rows();
  apply_phase4_single_blob_handoff();
  apply_phase4_side_recovery();
  append_side_reappearance_rows();

  if (impl_->merged_group_shadow.active) {
    for (auto &entry : impl_->split_candidate_shadow_by_raw_id) {
      entry.second.seen_this_frame = false;
    }

    for (const auto &hypothesis : shadow_hypotheses) {
      if (!IsSplitCandidateHypothesis(hypothesis)) {
        continue;
      }

      const int related_raw_track_id = hypothesis.related_raw_track_id.value_or(-1);
      const bool linked_to_carrier = related_raw_track_id == impl_->merged_group_shadow.carrier_raw_track_id;
      const bool is_carrier_raw = hypothesis.raw_track_id == impl_->merged_group_shadow.carrier_raw_track_id;
      if (is_carrier_raw || hypothesis.raw_track_id == phase4_continuity_raw) {
        continue;
      }
      const bool known_group_raw =
          raw_to_semantic_id_.find(hypothesis.raw_track_id) != raw_to_semantic_id_.end() &&
          impl_->merged_group_shadow.semantic_ids.count(raw_to_semantic_id_[hypothesis.raw_track_id]) > 0;
      const bool tracked_split_candidate =
          hypothesis.status == TrackletHypothesisStatus::kTracked && observations_by_raw_track_id.size() >= 2;
      if (!linked_to_carrier && !known_group_raw && !tracked_split_candidate) {
        continue;
      }

      auto &candidate = impl_->split_candidate_shadow_by_raw_id[hypothesis.raw_track_id];
      const bool first_seen = candidate.stable_frames == 0 || candidate.group_id != impl_->merged_group_shadow.group_id;
      candidate.group_id = impl_->merged_group_shadow.group_id;
      candidate.candidate_raw_track_id = hypothesis.raw_track_id;
      candidate.candidate_semantic_id = -1;
      const auto semantic_it = raw_to_semantic_id_.find(hypothesis.raw_track_id);
      if (semantic_it != raw_to_semantic_id_.end() &&
          impl_->merged_group_shadow.semantic_ids.count(semantic_it->second) > 0) {
        candidate.candidate_semantic_id = semantic_it->second;
      } else {
        for (const int semantic_id : impl_->merged_group_shadow.semantic_ids) {
          if (semantic_id != impl_->merged_group_shadow.carrier_semantic_id) {
            candidate.candidate_semantic_id = semantic_id;
            break;
          }
        }
      }
      candidate.bbox = hypothesis.bbox;
      candidate.confidence = hypothesis.confidence;
      candidate.reason = hypothesis.candidate_reason;
      candidate.related_raw_track_id = related_raw_track_id;
      candidate.hypothesis_status = TrackletHypothesisStatusToDebugString(hypothesis.status);
      candidate.stable_frames = first_seen ? 1 : candidate.stable_frames + 1;
      candidate.last_update_frame = current_frame_idx;
      candidate.seen_this_frame = true;
      append_split_candidate_row(first_seen ? "split_candidate_enter" : "split_candidate_update", candidate);
    }

    if (impl_->config.enable_phase4_merged_split_handoff) {
      for (auto &entry : impl_->split_candidate_shadow_by_raw_id) {
        auto &candidate = entry.second;
        if (!candidate.seen_this_frame || phase4_continuity_sid <= 0) {
          continue;
        }
        if (prev_raw_to_semantic.find(candidate.candidate_raw_track_id) != prev_raw_to_semantic.end()) {
          continue;
        }
        const int continuity_raw = phase4_continuity_raw;
        const int continuity_sid = phase4_continuity_sid;
        int exposed_partial_sid = -1;
        for (const int semantic_id : impl_->merged_group_shadow.semantic_ids) {
          if (semantic_id != continuity_sid) {
            exposed_partial_sid = semantic_id;
            break;
          }
        }
        if (continuity_raw <= 0 || continuity_sid <= 0 || exposed_partial_sid <= 0 ||
            continuity_raw == candidate.candidate_raw_track_id || exposed_partial_sid == continuity_sid) {
          continue;
        }
        const auto carrier_obs_it = observations_by_raw_track_id.find(continuity_raw);
        if (carrier_obs_it == observations_by_raw_track_id.end() ||
            !LooksLikeMergedSplitHandoff(carrier_obs_it->second.bbox, candidate.bbox)) {
          continue;
        }
        const bool applied = impl_->legacy_identity_matcher.ApplyPhase4MergedSplitHandoff(
            tracks, continuity_raw, exposed_partial_sid, candidate.candidate_raw_track_id, continuity_sid,
            frame);
        if (!applied) {
          continue;
        }
        raw_to_semantic_id_[continuity_raw] = exposed_partial_sid;
        raw_to_semantic_id_[candidate.candidate_raw_track_id] = continuity_sid;
        candidate.candidate_semantic_id = continuity_sid;
        impl_->merged_group_shadow.carrier_raw_track_id = continuity_raw;
        impl_->merged_group_shadow.carrier_semantic_id = exposed_partial_sid;
        refresh_legacy_debug_rows();
        result = build_result_from_legacy();
        append_split_candidate_row("phase4_merged_split_handoff", candidate, "merged_split_handoff");
        break;
      }
    }

    for (auto it = impl_->split_candidate_shadow_by_raw_id.begin();
         it != impl_->split_candidate_shadow_by_raw_id.end();) {
      if (it->second.seen_this_frame) {
        ++it;
        continue;
      }
      append_split_candidate_row("split_candidate_end", it->second, "candidate_missing");
      it = impl_->split_candidate_shadow_by_raw_id.erase(it);
    }
  }

  append_phase5_new_birth_candidate_rows();

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
