#include "identity_runtime_mutation_applier.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "vision_demo_host/modules/association_utils.hpp"
#include "vision_demo_host/modules/feature_geometry_update_state.hpp"
#include "assignment_cost.hpp"
#include "legacy_identity_record_lifecycle.hpp"

namespace vision_demo_host {
namespace {

bool AssociationEvidenceWeakForIdentity(const AssociationEvidence &association) {
  if (!association.stage.empty() && !association.passed_final_cost_gate) {
    return true;
  }
  return association.low_score_detection || association.recovered_from_lost;
}

}  // namespace

IdentityRuntimeMutationApplier::IdentityRuntimeMutationApplier(
    IdentityAssignmentEngineAdapter::Config config,
    RuntimeState *runtime_state,
    AppearanceFeatureService *appearance_features)
    : config_(std::move(config)),
      runtime_state_(runtime_state),
      appearance_features_(appearance_features) {}

std::vector<float> IdentityRuntimeMutationApplier::ExtractFeature(
    const cv::Mat &frame, const Track &track) const {
  if (!track.appearance_feature.empty()) {
    return track.appearance_feature;
  }
  if (appearance_features_ == nullptr) {
    return {};
  }
  return appearance_features_->ExtractIdentityFeature(frame, track.bbox);
}

ReliableGeometryCost::State IdentityRuntimeMutationApplier::ReliableGeometryState(
    const LegacyIdentityRecord &identity) const {
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

bool IdentityRuntimeMutationApplier::TrackOverlapsAny(const Track &track,
                                                      const std::vector<Track> &tracks,
                                                      const int self_idx) const {
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (static_cast<int>(i) == self_idx || tracks[i].class_id != ClassId::kPerson) {
      continue;
    }
    if (association::BBoxIoU(track.bbox, tracks[i].bbox) >= config_.overlap_iou_freeze) {
      return true;
    }
  }
  return false;
}

bool IdentityRuntimeMutationApplier::IsReliableObservation(
    const Track &track, const bool allow_feat_update, const float assignment_cost,
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

FeatureUpdatePolicy::Decision IdentityRuntimeMutationApplier::EvaluateUpdatePolicy(
    const Track &track, const std::vector<Track> &tracks, const int self_idx,
    const LegacyIdentityRecord *identity, const bool accepted,
    const float assignment_cost, const float assignment_margin,
    const bool force_geometry_update) const {
  const bool overlapping = TrackOverlapsAny(track, tracks, self_idx);
  const bool globally_frozen = runtime_state_->occlusion_mode.feature_update_frozen;
  const bool allow_feature_gate = !globally_frozen && !overlapping;
  FeatureUpdatePolicy::Input input;
  input.accepted = accepted;
  input.global_freeze = globally_frozen;
  input.overlap_freeze = overlapping;
  input.reliable_observation = IsReliableObservation(track, allow_feature_gate, assignment_cost,
                                                     assignment_margin);
  input.force_geometry_update = force_geometry_update;
  input.has_existing_feature_bank = identity != nullptr && !identity->feature_geometry.feature_bank.empty();
  input.stable_update_frames = identity == nullptr ? 0 : identity->feature_geometry.stable_update_frames;
  input.stable_frames_before_feature_update = config_.stable_frames_before_feature_update;
  return FeatureUpdatePolicy::Decide(input);
}

void IdentityRuntimeMutationApplier::UpdateIdentityObservation(
    LegacyIdentityRecord *identity, const Track &track, const float assignment_cost,
    const float assignment_margin) const {
  LegacyIdentityRecordLifecycle::ApplyObservation(track, runtime_state_->frame_index,
                                                  assignment_cost, assignment_margin, identity);
}

void IdentityRuntimeMutationApplier::UpsertIdentity(
    const Track &track, const int semantic_id, const std::vector<float> &feature,
    const FeatureUpdatePolicy::Decision &update_policy, const float assignment_cost,
    const float assignment_margin) {
  LegacyIdentityRecord &id = runtime_state_->identity_store.Upsert(semantic_id);
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

void IdentityRuntimeMutationApplier::ComputeCosts(
    const Track &track, const LegacyIdentityRecord &identity,
    const std::vector<float> &feature, float *app, float *geo, float *tim,
    float *final) const {
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

int IdentityRuntimeMutationApplier::AllocateNewSemanticId() {
  std::unordered_set<int> occupied_semantic_ids;
  occupied_semantic_ids = runtime_state_->identity_store.OccupiedSemanticIds();
  return runtime_state_->semantic_id_allocator.Allocate(occupied_semantic_ids);
}

bool IdentityRuntimeMutationApplier::ApplyPhase4DirectActions(
    const std::vector<Track> &tracks,
    const std::vector<Phase4DirectApplyHelper::Action> &actions,
    const cv::Mat *frame) {
  Phase4DirectApplyHelper::Input<ScoreDebugRow> input;
  input.tracks = &tracks;
  input.frame = frame;
  input.frame_index = runtime_state_->frame_index;
  input.mode = runtime_state_->occlusion_mode.mode;
  input.actions = actions;
  input.debug_rows = &runtime_state_->last_score_debug_rows;
  input.contains_semantic_id = [&](const int semantic_id) {
    return runtime_state_->identity_store.Contains(semantic_id);
  };
  input.find_identity = [&](const int semantic_id) {
    return runtime_state_->identity_store.Find(semantic_id);
  };
  input.extract_feature = [&](const cv::Mat &feature_frame, const Track &track) {
    return ExtractFeature(feature_frame, track);
  };
  input.compute_costs = [&](const Track &track, const LegacyIdentityRecord &identity,
                            const std::vector<float> &feature, float *app, float *geo,
                            float *tim, float *final) {
    ComputeCosts(track, identity, feature, app, geo, tim, final);
  };
  input.evaluate_update_policy =
      [&](const Track &track, const int track_idx, const LegacyIdentityRecord *identity,
          const float assignment_cost, const float assignment_margin,
          const bool force_geometry_update) {
        return EvaluateUpdatePolicy(track, tracks, track_idx, identity, true,
                                    assignment_cost, assignment_margin,
                                    force_geometry_update);
      };
  input.upsert_identity = [&](const Track &track, const int semantic_id,
                              const std::vector<float> &feature,
                              const FeatureUpdatePolicy::Decision &update_policy,
                              const float assignment_cost, const float assignment_margin) {
    UpsertIdentity(track, semantic_id, feature, update_policy, assignment_cost,
                   assignment_margin);
  };
  input.bind_raw_semantic = [&](const int raw_track_id, const int semantic_id) {
    runtime_state_->raw_semantic_bindings.Bind(raw_track_id, semantic_id);
  };
  input.erase_birth_candidate = [&](const int raw_track_id) {
    runtime_state_->birth_manager.Erase(raw_track_id);
  };
  return Phase4DirectApplyHelper::Apply(input);
}

bool IdentityRuntimeMutationApplier::ApplyPhase4MergedSplitHandoff(
    const std::vector<Track> &tracks, const int continuity_raw_track_id,
    const int continuity_semantic_id, const int candidate_raw_track_id,
    const int candidate_semantic_id, const cv::Mat *frame) {
  if (continuity_raw_track_id <= 0 || continuity_semantic_id <= 0 ||
      candidate_raw_track_id <= 0 || candidate_semantic_id <= 0 ||
      continuity_raw_track_id == candidate_raw_track_id ||
      continuity_semantic_id == candidate_semantic_id) {
    return false;
  }

  return ApplyPhase4DirectActions(
      tracks,
      {{continuity_raw_track_id, continuity_semantic_id, "phase4_merged_split_handoff", false},
       {candidate_raw_track_id, candidate_semantic_id, "phase4_merged_split_handoff", false}},
      frame);
}

bool IdentityRuntimeMutationApplier::ApplyPhase4MergedSideRecovery(
    const std::vector<Track> &tracks, const int carrier_raw_track_id,
    const int carrier_semantic_id, const int candidate_raw_track_id,
    const int candidate_semantic_id, const cv::Mat *frame) {
  if (carrier_raw_track_id <= 0 || carrier_semantic_id <= 0 ||
      candidate_raw_track_id <= 0 || candidate_semantic_id <= 0 ||
      carrier_raw_track_id == candidate_raw_track_id ||
      carrier_semantic_id == candidate_semantic_id) {
    return false;
  }

  if (!runtime_state_->identity_store.Contains(carrier_semantic_id)) {
    return false;
  }

  return ApplyPhase4DirectActions(
      tracks,
      {{candidate_raw_track_id, candidate_semantic_id, "phase4_merged_side_recovery", true}},
      frame);
}

bool IdentityRuntimeMutationApplier::ApplyPhase4MergedSingleBlobHandoff(
    const std::vector<Track> &tracks, const int carrier_raw_track_id,
    const int carrier_semantic_id, const int candidate_semantic_id,
    const cv::Mat *frame) {
  if (carrier_raw_track_id <= 0 || carrier_semantic_id <= 0 ||
      candidate_semantic_id <= 0 || carrier_semantic_id == candidate_semantic_id) {
    return false;
  }

  if (!runtime_state_->identity_store.Contains(carrier_semantic_id)) {
    return false;
  }

  const bool applied = ApplyPhase4DirectActions(
      tracks,
      {{carrier_raw_track_id, candidate_semantic_id, "phase4_merged_single_blob_handoff", false}},
      frame);
  if (applied) {
    runtime_state_->identity_store.MarkCarrierMissingForHandoff(carrier_semantic_id);
  }
  return applied;
}

bool IdentityRuntimeMutationApplier::ApplyPhase4PairwiseAssignment(
    const std::vector<Track> &tracks, const int first_raw_track_id,
    const int first_semantic_id, const int second_raw_track_id,
    const int second_semantic_id, const cv::Mat *frame) {
  if (first_raw_track_id <= 0 || first_semantic_id <= 0 ||
      second_raw_track_id <= 0 || second_semantic_id <= 0 ||
      first_raw_track_id == second_raw_track_id ||
      first_semantic_id == second_semantic_id) {
    return false;
  }

  return ApplyPhase4DirectActions(
      tracks,
      {{first_raw_track_id, first_semantic_id, "phase4_pairwise_assignment", false},
       {second_raw_track_id, second_semantic_id, "phase4_pairwise_assignment", false}},
      frame);
}

bool IdentityRuntimeMutationApplier::ApplyPhase5BirthAllocation(
    const std::vector<Track> &tracks, const int raw_track_id, const cv::Mat *frame) {
  if (raw_track_id <= 0 || runtime_state_->raw_semantic_bindings.HasBinding(raw_track_id)) {
    return false;
  }

  const auto track_it = std::find_if(tracks.begin(), tracks.end(), [&](const auto &track) {
    return track.id == raw_track_id && track.class_id == ClassId::kPerson && !track.occlusion_suspect;
  });
  if (track_it == tracks.end()) {
    return false;
  }

  int proposal_track_idx = -1;
  for (const auto &row : runtime_state_->last_score_debug_rows) {
    if (row.frame_idx == runtime_state_->frame_index && row.raw_track_id == raw_track_id &&
        row.stage == "phase5_birth_candidate" && row.selected && !row.accepted &&
        row.reject_reason == "phase5_birth_manager_pending") {
      proposal_track_idx = row.track_idx;
      break;
    }
  }
  if (proposal_track_idx < 0) {
    return false;
  }

  BirthManager::DebugRow accepted_birth_row;
  int new_sid = -1;
  if (!runtime_state_->birth_manager.ApplyPhase5AcceptedAllocation(
          raw_track_id, &runtime_state_->last_score_debug_rows,
          [&]() { return AllocateNewSemanticId(); },
          &accepted_birth_row, &new_sid)) {
    return false;
  }

  const Track &track = *track_it;
  const std::vector<float> feature = ExtractFeature(frame == nullptr ? cv::Mat{} : *frame, track);
  constexpr float kAssignmentCost = 0.0F;
  constexpr float kAssignmentMargin = 1.0F;
  const auto update_policy =
      EvaluateUpdatePolicy(track, tracks, proposal_track_idx, nullptr, true,
                           kAssignmentCost, kAssignmentMargin);

  for (auto &row : runtime_state_->last_score_debug_rows) {
    if (row.frame_idx == runtime_state_->frame_index &&
        row.raw_track_id == raw_track_id &&
        row.track_idx == accepted_birth_row.track_idx &&
        row.semantic_id == accepted_birth_row.semantic_id &&
        row.stage == accepted_birth_row.stage &&
        row.accepted) {
      row.feature_update_allowed = update_policy.feature_update_allowed;
      row.geometry_update_allowed = update_policy.geometry_update_allowed;
      row.feature_update_reason = update_policy.feature_update_reason;
      row.geometry_update_reason = update_policy.geometry_update_reason;
      break;
    }
  }

  UpsertIdentity(track, new_sid, feature, update_policy, kAssignmentCost, kAssignmentMargin);
  runtime_state_->raw_semantic_bindings.Bind(track.id, new_sid);
  return true;
}

}  // namespace vision_demo_host
