#include "identity_assignment_engine_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "vision_demo_host/modules/association_utils.hpp"
#include "active_assignment_solver.hpp"
#include "assignment_cost.hpp"
#include "assignment_candidate_builder.hpp"
#include "identity_assignment_frame_transaction.hpp"
#include "inactive_recovery_solver.hpp"
#include "identity_runtime_record_lifecycle.hpp"
#include "merged_single_blob_assignment_decision.hpp"

namespace vision_demo_host {
namespace {

constexpr float kBigCost = ActiveAssignmentSolver::kBigCost;

IdentityAssignmentEngineAdapter::ScoreDebugRow MakeInactiveRecoverDebugRow(
    const int frame_index,
    const IdentityLifecycleMode mode,
    const InactiveRecoverySolver::CandidateDecision &candidate) {
  IdentityAssignmentEngineAdapter::ScoreDebugRow row;
  row.frame_idx = frame_index;
  row.mode = mode;
  row.track_idx = candidate.track_idx;
  row.raw_track_id = candidate.raw_track_id;
  row.semantic_id = candidate.semantic_id;
  row.app_cost = candidate.app_cost;
  row.geo_cost = candidate.geo_cost;
  row.final_score = candidate.similarity;
  row.stage = "inactive_recover_candidate";
  row.accepted = candidate.accepted;
  row.reject_reason = candidate.reject_reason;
  return row;
}

IdentityAssignmentEngineAdapter::ScoreDebugRow MakeScoreDebugRow(
    const int frame_index,
    const IdentityLifecycleMode mode,
    const MergedSingleBlobAssignmentDecision::CandidateRow &debug_row) {
  IdentityAssignmentEngineAdapter::ScoreDebugRow row;
  row.frame_idx = frame_index;
  row.mode = mode;
  row.track_idx = debug_row.track_idx;
  row.raw_track_id = debug_row.raw_track_id;
  row.semantic_id = debug_row.semantic_id;
  row.app_cost = debug_row.app_cost;
  row.geo_cost = debug_row.geo_cost;
  row.time_cost = debug_row.time_cost;
  row.final_score = debug_row.final_score;
  row.selected = debug_row.selected;
  row.stage = debug_row.stage;
  row.margin = debug_row.margin;
  row.accepted = debug_row.accepted;
  row.reject_reason = debug_row.reject_reason;
  return row;
}

std::string FormatPairwisePairs(const std::vector<int> &track_indices,
                                const std::vector<int> &candidate_semantic_ids,
                                const std::vector<Track> &tracks,
                                const int first_col,
                                const int second_col) {
  if (track_indices.size() < 2 || candidate_semantic_ids.size() < 2 ||
      first_col < 0 || second_col < 0 ||
      first_col >= static_cast<int>(candidate_semantic_ids.size()) ||
      second_col >= static_cast<int>(candidate_semantic_ids.size())) {
    return {};
  }
  std::ostringstream oss;
  oss << tracks[static_cast<std::size_t>(track_indices[0])].id << "->"
      << candidate_semantic_ids[static_cast<std::size_t>(first_col)] << "|"
      << tracks[static_cast<std::size_t>(track_indices[1])].id << "->"
      << candidate_semantic_ids[static_cast<std::size_t>(second_col)];
  return oss.str();
}

bool AssociationEvidenceWeakForIdentity(const AssociationEvidence &association) {
  if (!association.stage.empty() && !association.passed_final_cost_gate) {
    return true;
  }
  return association.low_score_detection || association.recovered_from_lost;
}

}  // namespace

std::string IdentityLifecycleModeToString(const IdentityLifecycleMode mode) {
  switch (mode) {
    case IdentityLifecycleMode::kNormal:
      return "NORMAL";
    case IdentityLifecycleMode::kMerged:
      return "MERGED";
    case IdentityLifecycleMode::kSplitRecovery:
      return "SPLIT_RECOVERY";
    case IdentityLifecycleMode::kNormalResumed:
      return "NORMAL_RESUMED";
    default:
      return "UNKNOWN";
  }
}

IdentityAssignmentEngineAdapter::IdentityAssignmentEngineAdapter(Config config, RuntimeState *runtime_state)
    : config_(std::move(config)), runtime_state_(runtime_state), appearance_features_() {}

bool IdentityAssignmentEngineAdapter::Initialize(std::string *error) {
  appearance_features_ = AppearanceFeatureService(
      AppearanceFeatureService::Config{config_.reid_backend, config_.reid_model_path, config_.reid_input_width,
                                       config_.reid_input_height, 16, 8},
      AppearanceFeatureService::Profile::kIdentity);
  initialized_ = appearance_features_.Initialize(error);
  return initialized_;
}

void IdentityAssignmentEngineAdapter::ResetRuntimeState(RuntimeState *runtime_state) {
  if (runtime_state == nullptr) {
    return;
  }
  runtime_state->semantic_id_allocator.Reset();
  runtime_state->primary_initialized = false;
  runtime_state->current_primary_semantic_id = -1;
  runtime_state->frame_index = 0;
  runtime_state->occlusion_mode.prev_visible_person_count = 0;
  runtime_state->occlusion_mode.prev_had_overlap = false;
  runtime_state->occlusion_mode.merged_frames = 0;
  runtime_state->occlusion_mode.split_stable_count = 0;
  runtime_state->occlusion_mode.mode = IdentityLifecycleMode::kNormal;
  runtime_state->occlusion_mode.feature_update_frozen = false;
  runtime_state->last_score_debug_rows.clear();
  runtime_state->last_pairwise_assignment_debug_rows.clear();
  runtime_state->identity_store.Reset();
  runtime_state->raw_semantic_bindings.Reset();
  runtime_state->birth_manager.Reset();
}

void IdentityAssignmentEngineAdapter::ResetAdapter() {
  initialized_ = false;
  appearance_features_ = AppearanceFeatureService();
}

void IdentityAssignmentEngineAdapter::Reset() {
  ResetRuntimeState(runtime_state_);
  ResetAdapter();
}

std::vector<float> IdentityAssignmentEngineAdapter::ExtractFeature(const cv::Mat &frame, const Track &track) const {
  if (!track.appearance_feature.empty()) {
    return track.appearance_feature;
  }
  return appearance_features_.ExtractIdentityFeature(frame, track.bbox);
}

ReliableGeometryCost::State IdentityAssignmentEngineAdapter::ReliableGeometryState(const IdentityRuntimeRecord &identity) const {
  ReliableGeometryCost::State state;
  state.latest_bbox = identity.last_bbox;
  state.latest_center = identity.last_center;
  state.reliable_bbox = identity.feature_geometry.reliable_bbox;
  state.reliable_center = identity.feature_geometry.reliable_center;
  state.reliable_velocity = identity.feature_geometry.reliable_velocity;
  state.missing_frames = identity.missing_frames;
  state.has_reliable_geometry = identity.feature_geometry.has_reliable_geometry;
  return state;
}

bool IdentityAssignmentEngineAdapter::PassesMissingIdentityGate(const Track &track, const IdentityRuntimeRecord &identity,
                                                       const float app_cost, const float geo_cost) const {
  ReliableGeometryCost::MissingGateConfig gate_config;
  gate_config.min_area_ratio = config_.missing_assign_min_area_ratio;
  gate_config.max_area_ratio = config_.missing_assign_max_area_ratio;
  gate_config.max_center_dist_norm = config_.missing_assign_max_center_dist_norm;
  gate_config.max_app_cost = config_.missing_assign_max_app_cost;
  gate_config.active_max_cost = config_.active_assign_max_cost;
  return ReliableGeometryCost::PassesMissingIdentityGate(track.bbox, ReliableGeometryState(identity), app_cost,
                                                        geo_cost, gate_config);
}

bool IdentityAssignmentEngineAdapter::PassesMissingAppearanceGate(const IdentityRuntimeRecord &identity, const float app_cost,
                                                         const float geo_cost) const {
  return ReliableGeometryCost::PassesShortMissingAppearanceGate(
      ReliableGeometryState(identity), app_cost, geo_cost, config_.missing_assign_max_app_cost);
}

void IdentityAssignmentEngineAdapter::ComputeCosts(const Track &track, const IdentityRuntimeRecord &identity, const std::vector<float> &feature,
                                      float *app, float *geo, float *tim, float *final) const {
  AssignmentCost::Config cost_config;
  cost_config.max_missing_frames = config_.max_missing_frames;
  cost_config.app_weight = config_.app_w;
  cost_config.geometry_weight = config_.geo_w;
  cost_config.time_weight = config_.time_w;
  const auto cost = AssignmentCost::Compute(track, identity, feature, cost_config);
  if (app != nullptr) {
    *app = cost.appearance;
  }
  if (geo != nullptr) {
    *geo = cost.geometry;
  }
  if (tim != nullptr) {
    *tim = cost.time;
  }
  if (final != nullptr) {
    *final = cost.final;
  }
}

float IdentityAssignmentEngineAdapter::AssignmentScore(const Track &track, const IdentityRuntimeRecord &identity,
                                          const std::vector<float> &feature) const {
  float final = 0.0F;
  ComputeCosts(track, identity, feature, nullptr, nullptr, nullptr, &final);
  return final;
}

float IdentityAssignmentEngineAdapter::ActiveAssignmentMaxCost(const IdentityRuntimeRecord &identity,
                                                     const AssociationEvidence &association) const {
  float max_cost = std::clamp(config_.active_assign_max_cost, 0.0F, 1.0F);
  if (identity.missing_frames > 0 && identity.missing_frames <= std::max(1, config_.max_missing_frames) &&
      association.passed_final_cost_gate && !association.stage.empty()) {
    max_cost = std::min(1.0F, max_cost + std::max(0.0F, config_.min_assignment_margin));
  }
  return max_cost;
}

bool IdentityAssignmentEngineAdapter::IsReliableObservation(const Track &track, const bool allow_feat_update,
                                                const float assignment_cost,
                                                const float assignment_margin) const {
  if (!allow_feat_update || track.occlusion_suspect) {
    return false;
  }
  if (AssociationEvidenceWeakForIdentity(track.association)) {
    return false;
  }
  if (assignment_cost > std::clamp(config_.active_assign_max_cost, 0.0F, 1.0F)) {
    return false;
  }
  if (assignment_margin < std::max(0.0F, config_.min_assignment_margin)) {
    return false;
  }
  return true;
}

FeatureUpdatePolicy::Decision IdentityAssignmentEngineAdapter::EvaluateUpdatePolicy(
    const Track &track, const std::vector<Track> &tracks, const int self_idx, const IdentityRuntimeRecord *identity,
    const bool accepted, const float assignment_cost, const float assignment_margin,
    const bool force_geometry_update) const {
  const bool overlapping = TrackOverlapsAny(track, tracks, self_idx);
  const bool globally_frozen = runtime_state_->occlusion_mode.feature_update_frozen;
  const bool allow_feature_gate = !globally_frozen && !overlapping;
  FeatureUpdatePolicy::Input input;
  input.accepted = accepted;
  input.global_freeze = globally_frozen;
  input.overlap_freeze = overlapping;
  input.reliable_observation = IsReliableObservation(track, allow_feature_gate, assignment_cost, assignment_margin);
  input.force_geometry_update = force_geometry_update;
  input.has_existing_feature_bank = identity != nullptr && !identity->feature_geometry.feature_bank.empty();
  input.stable_update_frames = identity == nullptr ? 0 : identity->feature_geometry.stable_update_frames;
  input.stable_frames_before_feature_update = config_.stable_frames_before_feature_update;
  return FeatureUpdatePolicy::Decide(input);
}

void IdentityAssignmentEngineAdapter::UpdateIdentityObservation(IdentityRuntimeRecord *identity, const Track &track,
                                                   const float assignment_cost,
                                                   const float assignment_margin) const {
  IdentityRuntimeRecordLifecycle::ApplyObservation(track, runtime_state_->frame_index, assignment_cost, assignment_margin, identity);
}

void IdentityAssignmentEngineAdapter::UpsertIdentity(const Track &track, const int semantic_id, const std::vector<float> &feature,
                                        const FeatureUpdatePolicy::Decision &update_policy,
                                        const float assignment_cost, const float assignment_margin) {
  IdentityRuntimeRecord &id = runtime_state_->identity_store.Upsert(semantic_id);
  id.semantic_id = semantic_id;
  UpdateIdentityObservation(&id, track, assignment_cost, assignment_margin);

  FeatureGeometryUpdateState::Observation observation;
  observation.bbox = track.bbox;
  observation.frame_index = runtime_state_->frame_index;
  observation.feature = feature;

  FeatureGeometryUpdateState::Config update_config;
  update_config.feature_bank_max_size = config_.feat_bank_size;
  update_config.stable_frames_before_feature_update = config_.stable_frames_before_feature_update;
  FeatureGeometryUpdateState::Apply(update_policy, observation, update_config, &id.feature_geometry);
}

std::vector<IdentityAssignmentEngineAdapter::Assignment> IdentityAssignmentEngineAdapter::SolveAssignments(
    const AssignmentCandidateBuilder::ActiveBuildResult &build_result, const std::vector<Track> &tracks) {
  if (build_result.solver_tracks.empty() || build_result.solver_candidates.empty()) {
    return {};
  }
  ActiveAssignmentSolver::Config solver_config;
  solver_config.max_missing_frames = config_.max_missing_frames;
  solver_config.active_assign_max_cost = config_.active_assign_max_cost;
  solver_config.min_assignment_margin = config_.min_assignment_margin;
  const auto solved = ActiveAssignmentSolver::Solve(build_result.solver_tracks, build_result.solver_candidates,
                                                    build_result.cost_matrix, build_result.appearance_cost_matrix,
                                                    solver_config);
  std::vector<int> track_indices;
  track_indices.reserve(build_result.solver_tracks.size());
  for (const auto &track : build_result.solver_tracks) {
    track_indices.push_back(track.track_idx);
  }
  std::vector<int> candidate_semantic_ids;
  candidate_semantic_ids.reserve(build_result.solver_candidates.size());
  for (const auto &candidate : build_result.solver_candidates) {
    candidate_semantic_ids.push_back(candidate.semantic_id);
  }
  for (const auto &debug : solved.pairwise_debug_rows) {
    PairwiseAssignmentDebugRow row;
    row.frame_idx = runtime_state_->frame_index;
    row.mode = runtime_state_->occlusion_mode.mode;
    row.selected_pairs = FormatPairwisePairs(track_indices, candidate_semantic_ids, tracks,
                                             debug.selected_first_col, debug.selected_second_col);
    row.alternate_pairs = FormatPairwisePairs(track_indices, candidate_semantic_ids, tracks,
                                              debug.alternate_first_col, debug.alternate_second_col);
    row.selected_final_cost = debug.selected_final_cost;
    row.alternate_final_cost = debug.alternate_final_cost;
    row.selected_app_cost = debug.selected_app_cost;
    row.alternate_app_cost = debug.alternate_app_cost;
    row.margin = debug.margin;
    row.appearance_override = debug.appearance_override;
    runtime_state_->last_pairwise_assignment_debug_rows.push_back(std::move(row));
  }
  std::vector<Assignment> out;
  out.reserve(solved.assignments.size());
  for (const auto &assignment : solved.assignments) {
    out.push_back(Assignment{assignment.track_idx,
                             assignment.semantic_id,
                             assignment.score,
                             assignment.cost,
                             assignment.margin,
                             assignment.accepted,
                             assignment.reject_reason,
                             assignment.pairwise_appearance_override});
  }
  return out;
}

bool IdentityAssignmentEngineAdapter::TrackOverlapsAny(const Track &track, const std::vector<Track> &tracks, const int self_idx) const {
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (static_cast<int>(i) == self_idx) {
      continue;
    }
    if (tracks[i].class_id != ClassId::kPerson) {
      continue;
    }
    if (association::BBoxIoU(track.bbox, tracks[i].bbox) >= config_.overlap_iou_freeze) {
      return true;
    }
  }
  return false;
}

bool IdentityAssignmentEngineAdapter::LooksLikeMergedSideReappearance(
    const Track &candidate, const IdentityRuntimeRecord &identity, const std::vector<Track> &tracks, const int candidate_idx,
    const std::unordered_map<int, int> &track_idx_to_sid, const float app_cost) const {
  if (identity.missing_frames < std::max(1, config_.merge_hold_frames) ||
      identity.missing_frames > std::max(1, config_.max_missing_frames) ||
      app_cost > std::min(0.80F, std::clamp(config_.missing_assign_max_app_cost, 0.0F, 1.0F) + 0.30F)) {
    return false;
  }

  const float candidate_height = std::max(1.0F, candidate.bbox.height);
  const float candidate_aspect = candidate.bbox.width / candidate_height;
  if (candidate.confidence < 0.45F || candidate_height < 300.0F || candidate_aspect < 0.25F ||
      candidate_aspect > 0.70F) {
    return false;
  }

  const cv::Point2f hidden_center = identity.feature_geometry.has_reliable_geometry
                                        ? identity.feature_geometry.reliable_center
                                        : identity.last_center;
  const cv::Point2f candidate_center = association::BBoxCenter(candidate.bbox);
  for (const auto &[track_idx, sid] : track_idx_to_sid) {
    (void)sid;
    if (track_idx == candidate_idx || track_idx < 0 || track_idx >= static_cast<int>(tracks.size())) {
      continue;
    }
    const Track &carrier = tracks[static_cast<std::size_t>(track_idx)];
    if (carrier.class_id != candidate.class_id) {
      continue;
    }

    const float vertical_overlap =
        std::max(0.0F, std::min(candidate.bbox.y + candidate.bbox.height, carrier.bbox.y + carrier.bbox.height) -
                           std::max(candidate.bbox.y, carrier.bbox.y));
    if (vertical_overlap / candidate_height < 0.45F) {
      continue;
    }

    const float carrier_left = carrier.bbox.x;
    const float carrier_right = carrier.bbox.x + carrier.bbox.width;
    const float candidate_left = candidate.bbox.x;
    const float candidate_right = candidate.bbox.x + candidate.bbox.width;
    const float hidden_dx = hidden_center.x - association::BBoxCenter(carrier.bbox).x;
    const float candidate_dx = candidate_center.x - association::BBoxCenter(carrier.bbox).x;
    const bool crossed_carrier_side = hidden_dx * candidate_dx < 0.0F;
    const bool touches_carrier_side =
        (candidate_left < carrier_left && candidate_right > carrier_left) ||
        (candidate_left < carrier_right && candidate_right > carrier_right);
    const float side_gap =
        std::min(std::abs(candidate_right - carrier_left), std::abs(candidate_left - carrier_right));
    const bool close_to_carrier_side = side_gap <= std::max(25.0F, carrier.bbox.width * 0.20F);
    if (crossed_carrier_side && (touches_carrier_side || close_to_carrier_side)) {
      return true;
    }
  }
  return false;
}

int IdentityAssignmentEngineAdapter::AllocateNewSemanticId() {
  std::unordered_set<int> occupied_semantic_ids;
  occupied_semantic_ids = runtime_state_->identity_store.OccupiedSemanticIds();
  return runtime_state_->semantic_id_allocator.Allocate(occupied_semantic_ids);
}

int IdentityAssignmentEngineAdapter::SelectBestSemanticForMerged(const std::vector<int> &candidate_semantic_ids,
                                                     const Track &track,
                                                     const std::vector<float> &feature) const {
  int best_id = -1;
  float best_cost = kBigCost;
  for (const int semantic_id : candidate_semantic_ids) {
    const auto *identity = runtime_state_->identity_store.Find(semantic_id);
    if (identity == nullptr) {
      continue;
    }
    const float cost = AssignmentScore(track, *identity, feature);
    if (cost < best_cost) {
      best_cost = cost;
      best_id = semantic_id;
    }
  }
  return best_id;
}

float IdentityAssignmentEngineAdapter::RecoverThresholdForSemantic(const IdentityRuntimeRecord &identity) const {
  float threshold = std::clamp(config_.recover_sim_thresh_strict, 0.0F, 1.0F);
  if (identity.missing_frames > std::max(1, config_.max_missing_frames) &&
      identity.missing_frames <= std::max(1, config_.recover_relaxed_max_missing_frames)) {
    threshold = std::min(threshold, std::clamp(config_.recover_sim_thresh_relaxed, 0.0F, 1.0F));
  }
  return threshold;
}

bool IdentityAssignmentEngineAdapter::CanRecoverInactiveIdentity(const IdentityRuntimeRecord &identity) const {
  return identity.occlusion_protect_remaining <= 0;
}

const std::unordered_map<int, int> &IdentityAssignmentEngineAdapter::Update(const std::vector<Track> &tracks,
                                                                const PrimaryTargetResult &primary,
                                                                const cv::Mat *frame) {
  if (!initialized_) {
    std::string error;
    if (!Initialize(&error)) {
      runtime_state_->last_score_debug_rows.clear();
      return runtime_state_->raw_semantic_bindings.Current();
    }
  }
  ++runtime_state_->frame_index;
  runtime_state_->last_score_debug_rows.clear();
  runtime_state_->last_pairwise_assignment_debug_rows.clear();
  if (primary.primary_target_id > 0) {
    runtime_state_->current_primary_semantic_id = primary.primary_target_id;
  } else if (primary.state == PrimaryState::kLost || primary.state == PrimaryState::kIdle) {
    runtime_state_->current_primary_semantic_id = -1;
  }
  const auto prev_raw_to_semantic = runtime_state_->raw_semantic_bindings.PreviousSnapshot();
  runtime_state_->identity_store.BeginFrame();
  runtime_state_->raw_semantic_bindings.Clear();

  std::vector<int> person_track_indices;
  person_track_indices.reserve(tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i].class_id == ClassId::kPerson) {
      person_track_indices.push_back(static_cast<int>(i));
    }
  }

  const int visible_person_count = static_cast<int>(person_track_indices.size());
  bool has_overlap = false;
  for (std::size_t a = 0; a < person_track_indices.size() && !has_overlap; ++a) {
    for (std::size_t b = a + 1; b < person_track_indices.size(); ++b) {
      const auto &ta = tracks[static_cast<std::size_t>(person_track_indices[a])];
      const auto &tb = tracks[static_cast<std::size_t>(person_track_indices[b])];
      if (association::BBoxIoU(ta.bbox, tb.bbox) >= config_.overlap_iou_freeze) {
        has_overlap = true;
        break;
      }
    }
  }

  OcclusionModeState::Config mode_config;
  mode_config.split_stable_frames = config_.split_stable_frames;
  mode_config.merge_hold_frames = config_.merge_hold_frames;
  mode_config.merged_requires_overlap = config_.merged_requires_overlap;
  OcclusionModeState::Input mode_input;
  mode_input.visible_person_count = visible_person_count;
  mode_input.has_overlap = has_overlap;
  runtime_state_->occlusion_mode = OcclusionModeState::Advance(mode_config, runtime_state_->occlusion_mode, mode_input);

  int bootstrap_track_idx = -1;
  if (!runtime_state_->primary_initialized && !person_track_indices.empty()) {
    float best_area = -1.0F;
    for (const int track_idx : person_track_indices) {
      const float area = tracks[static_cast<std::size_t>(track_idx)].bbox.area();
      if (area > best_area) {
        best_area = area;
        bootstrap_track_idx = track_idx;
      }
    }
    runtime_state_->primary_initialized = true;
    runtime_state_->current_primary_semantic_id = 1;
  }

  std::vector<int> active_semantic_ids;
  std::vector<int> inactive_semantic_ids;
  active_semantic_ids = runtime_state_->identity_store.PersonSemanticIds(true, config_.max_missing_frames);
  inactive_semantic_ids = runtime_state_->identity_store.PersonSemanticIds(false, config_.max_missing_frames);

  std::unordered_map<int, int> track_idx_to_sid;
  std::unordered_map<int, bool> sid_used;

  std::vector<std::vector<float>> person_features(person_track_indices.size());
  const cv::Mat empty_frame;
  const cv::Mat *feature_frame = (frame != nullptr && !frame->empty()) ? frame : &empty_frame;
  for (std::size_t i = 0; i < person_track_indices.size(); ++i) {
    const Track &track = tracks[static_cast<std::size_t>(person_track_indices[i])];
    if (!track.appearance_feature.empty() || !feature_frame->empty()) {
      person_features[i] = ExtractFeature(*feature_frame, track);
    }
  }
  if (runtime_state_->occlusion_mode.mode == IdentityLifecycleMode::kMerged && person_track_indices.size() == 1) {
    const int track_idx = person_track_indices[0];
    const Track &track = tracks[static_cast<std::size_t>(track_idx)];
    const auto continuity_it = prev_raw_to_semantic.find(track.id);
    const int continuity_sid = continuity_it == prev_raw_to_semantic.end() ? -1 : continuity_it->second;
    MergedSingleBlobAssignmentDecision::Input single_blob_input;
    single_blob_input.track_idx = track_idx;
    single_blob_input.raw_track_id = track.id;
    single_blob_input.continuity_semantic_id = continuity_sid;
    for (const int semantic_id : active_semantic_ids) {
      const auto *identity = runtime_state_->identity_store.Find(semantic_id);
      if (identity == nullptr) {
        continue;
      }
      float app = 0.0F;
      float geo = 0.0F;
      float tim = 0.0F;
      float final = 0.0F;
      ComputeCosts(track, *identity, person_features[0], &app, &geo, &tim, &final);
      ScoreDebugRow row;
      row.frame_idx = runtime_state_->frame_index;
      row.mode = runtime_state_->occlusion_mode.mode;
      row.track_idx = track_idx;
      row.raw_track_id = track.id;
      row.semantic_id = semantic_id;
      row.app_cost = app;
      row.geo_cost = geo;
      row.time_cost = tim;
      row.final_score = final;
      row.stage = "merged_candidate";
      MergedSingleBlobAssignmentDecision::CandidateRow helper_row;
      helper_row.track_idx = row.track_idx;
      helper_row.raw_track_id = row.raw_track_id;
      helper_row.semantic_id = row.semantic_id;
      helper_row.missing_frames = identity->missing_frames;
      helper_row.app_cost = row.app_cost;
      helper_row.geo_cost = row.geo_cost;
      helper_row.time_cost = row.time_cost;
      helper_row.final_score = row.final_score;
      helper_row.stage = row.stage;
      if (!PassesMissingIdentityGate(track, *identity, app, geo)) {
        row.reject_reason = "missing_identity_gate_reject";
      } else if (!PassesMissingAppearanceGate(*identity, app, geo)) {
        row.reject_reason = "missing_appearance_gate_reject";
      } else if (final > ActiveAssignmentMaxCost(*identity, track.association)) {
        row.reject_reason = "active_assign_max_cost_reject";
      }
      helper_row.reject_reason = row.reject_reason;
      single_blob_input.active_candidates.push_back(std::move(helper_row));
    }
    MergedSingleBlobAssignmentDecision::Config single_blob_config;
    single_blob_config.min_assignment_margin = config_.min_assignment_margin;
    single_blob_config.max_missing_frames = config_.max_missing_frames;
    auto single_blob_decision =
        MergedSingleBlobAssignmentDecision::Decide(single_blob_input, single_blob_config);
    if (single_blob_decision.needs_new_semantic_id) {
      std::vector<InactiveRecoverySolver::TrackInput> recovery_tracks;
      InactiveRecoverySolver::TrackInput recovery_track;
      recovery_track.track_idx = track_idx;
      recovery_track.raw_track_id = track.id;
      recovery_tracks.push_back(recovery_track);
      std::vector<InactiveRecoverySolver::CandidateInput> recovery_candidates;
      std::vector<InactiveRecoverySolver::CandidateScore> recovery_scores;
      for (const int semantic_id : inactive_semantic_ids) {
        const auto *identity = runtime_state_->identity_store.Find(semantic_id);
        if (identity == nullptr) {
          continue;
        }
        if (!CanRecoverInactiveIdentity(*identity)) {
          continue;
        }
        float app_cost = 0.0F;
        float geo_cost = 0.0F;
        ComputeCosts(track, *identity, person_features[0], &app_cost, &geo_cost, nullptr, nullptr);
        const float sim = 1.0F - app_cost;
        InactiveRecoverySolver::CandidateInput candidate;
        candidate.semantic_id = semantic_id;
        recovery_candidates.push_back(candidate);
        InactiveRecoverySolver::CandidateScore score;
        score.track_row = 0;
        score.candidate_col = static_cast<int>(recovery_candidates.size() - 1);
        score.app_cost = app_cost;
        score.geo_cost = geo_cost;
        score.similarity = sim;
        score.recover_threshold = RecoverThresholdForSemantic(*identity);
        score.passes_missing_identity_gate = PassesMissingIdentityGate(track, *identity, app_cost, geo_cost);
        recovery_scores.push_back(score);
      }
      InactiveRecoverySolver::Config recovery_config;
      recovery_config.recovery_max_cost = config_.recovery_max_cost;
      const auto recovery_result = InactiveRecoverySolver::SelectBestSimilarity(
          recovery_tracks, recovery_candidates, recovery_scores, recovery_config);
      for (const auto &candidate : recovery_result.candidates) {
        const auto row = MakeInactiveRecoverDebugRow(runtime_state_->frame_index, runtime_state_->occlusion_mode.mode, candidate);
        MergedSingleBlobAssignmentDecision::CandidateRow helper_row;
        helper_row.track_idx = row.track_idx;
        helper_row.raw_track_id = row.raw_track_id;
        helper_row.semantic_id = row.semantic_id;
        helper_row.app_cost = row.app_cost;
        helper_row.geo_cost = row.geo_cost;
        helper_row.time_cost = row.time_cost;
        helper_row.final_score = row.final_score;
        helper_row.selected = row.selected;
        helper_row.stage = row.stage;
        helper_row.margin = row.margin;
        helper_row.accepted = row.accepted;
        helper_row.reject_reason = row.reject_reason;
        single_blob_input.inactive_recovery_rows.push_back(std::move(helper_row));
      }
      single_blob_input.inactive_recovery_assignments = recovery_result.assignments;
      single_blob_decision = MergedSingleBlobAssignmentDecision::Decide(single_blob_input, single_blob_config);
    }
    int best_sid = single_blob_decision.semantic_id;
    if (best_sid < 0) {
      best_sid = AllocateNewSemanticId();
    }
    track_idx_to_sid[track_idx] = best_sid;
    sid_used[best_sid] = true;
    for (const auto &row : single_blob_decision.active_candidates) {
      runtime_state_->last_score_debug_rows.push_back(MakeScoreDebugRow(runtime_state_->frame_index, runtime_state_->occlusion_mode.mode, row));
    }
    for (const auto &row : single_blob_decision.inactive_recovery_rows) {
      runtime_state_->last_score_debug_rows.push_back(MakeScoreDebugRow(runtime_state_->frame_index, runtime_state_->occlusion_mode.mode, row));
    }
  }

  IdentityAssignmentFrameTransaction::Input transaction_input;
  transaction_input.tracks = &tracks;
  transaction_input.person_track_indices = &person_track_indices;
  transaction_input.person_features = &person_features;
  transaction_input.active_semantic_ids = &active_semantic_ids;
  transaction_input.inactive_semantic_ids = &inactive_semantic_ids;
  transaction_input.prev_raw_to_semantic = &prev_raw_to_semantic;
  transaction_input.initial_track_idx_to_sid = std::move(track_idx_to_sid);
  transaction_input.initial_sid_used = std::move(sid_used);
  transaction_input.bootstrap_track_idx = bootstrap_track_idx;
  transaction_input.resolve_assignments =
      runtime_state_->occlusion_mode.mode != IdentityLifecycleMode::kMerged ||
      person_track_indices.size() != 1;
  transaction_input.has_overlap = has_overlap;
  transaction_input.frame = frame;
  transaction_input.config = config_;
  transaction_input.runtime_state = runtime_state_;
  transaction_input.adapter = this;
  transaction_input.appearance_features = &appearance_features_;

  const auto frame_result = IdentityAssignmentFrameTransaction::Execute(transaction_input);
  runtime_state_->last_score_debug_rows = frame_result.score_debug_rows;
  runtime_state_->last_pairwise_assignment_debug_rows = frame_result.pairwise_assignment_debug_rows;
  return runtime_state_->raw_semantic_bindings.Current();
}

std::vector<IdentityAssignmentEngineAdapter::IdentitySnapshot> IdentityAssignmentEngineAdapter::IdentitySnapshots() const {
  return runtime_state_->identity_store.Snapshots();
}

int IdentityAssignmentEngineAdapter::SemanticIdForRawTrack(const int raw_track_id) const {
  return runtime_state_->raw_semantic_bindings.SemanticIdForRawTrack(raw_track_id);
}

}  // namespace vision_demo_host
