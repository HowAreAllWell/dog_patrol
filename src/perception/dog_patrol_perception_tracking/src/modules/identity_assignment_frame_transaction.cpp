#include "identity_assignment_frame_transaction.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "active_assignment_input_collector.hpp"
#include "active_assignment_solver.hpp"
#include "assignment_application_executor.hpp"
#include "assignment_application_plan.hpp"
#include "assignment_candidate_builder.hpp"
#include "birth_manager.hpp"
#include "identity_runtime_lifecycle_coordinator.hpp"
#include "identity_runtime_mutation_applier.hpp"
#include "inactive_recovery_input_collector.hpp"
#include "inactive_recovery_solver.hpp"
#include "raw_continuity_decision.hpp"
#include "unresolved_track_final_resolution_coordinator.hpp"

namespace vision_demo_host {
namespace {

bool AssociationEvidenceWeakForIdentity(const AssociationEvidence &association) {
  if (!association.stage.empty() && !association.passed_final_cost_gate) {
    return true;
  }
  return association.low_score_detection || association.recovered_from_lost;
}

IdentityAssignmentFrameTransaction::ScoreDebugRow MakeScoreDebugRow(
    const int frame_index,
    const IdentityLifecycleMode mode,
    const AssignmentCandidateBuilder::DebugRow &debug_row) {
  IdentityAssignmentFrameTransaction::ScoreDebugRow row;
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

IdentityAssignmentFrameTransaction::ScoreDebugRow MakeRawContinuityDebugRow(
    const int frame_index,
    const IdentityLifecycleMode mode,
    const RawContinuityDecision::Decision &decision) {
  IdentityAssignmentFrameTransaction::ScoreDebugRow row;
  row.frame_idx = frame_index;
  row.mode = mode;
  row.track_idx = decision.track_idx;
  row.raw_track_id = decision.raw_track_id;
  row.semantic_id = decision.semantic_id;
  row.app_cost = decision.app_cost;
  row.geo_cost = decision.geo_cost;
  row.time_cost = decision.time_cost;
  row.final_score = decision.final_score;
  row.margin = decision.margin;
  row.selected = decision.selected;
  row.stage = "raw_continuity";
  row.accepted = decision.accepted;
  row.reject_reason = decision.reject_reason;
  row.continuity_used = decision.continuity_used;
  return row;
}

}  // namespace

IdentityAssignmentFrameTransaction::Result IdentityAssignmentFrameTransaction::Execute(const Input &input) {
  Result result;
  result.track_idx_to_sid = input.initial_track_idx_to_sid;
  result.sid_used = input.initial_sid_used;

  if (input.tracks == nullptr || input.person_track_indices == nullptr || input.person_features == nullptr ||
      input.active_semantic_ids == nullptr || input.inactive_semantic_ids == nullptr ||
      input.prev_raw_to_semantic == nullptr || input.runtime_state == nullptr || input.adapter == nullptr) {
    return result;
  }

  const std::vector<Track> &tracks = *input.tracks;
  const std::vector<int> &person_track_indices = *input.person_track_indices;
  const std::vector<std::vector<float>> &person_features = *input.person_features;
  const std::vector<int> &active_semantic_ids = *input.active_semantic_ids;
  const std::vector<int> &inactive_semantic_ids = *input.inactive_semantic_ids;
  const std::unordered_map<int, int> &prev_raw_to_semantic = *input.prev_raw_to_semantic;
  auto &runtime_state = *input.runtime_state;
  auto &adapter = *input.adapter;
  auto &track_idx_to_sid = result.track_idx_to_sid;
  auto &sid_used = result.sid_used;

  if (input.resolve_assignments) {
    if (input.bootstrap_track_idx >= 0) {
      track_idx_to_sid[input.bootstrap_track_idx] = 1;
      sid_used[1] = true;
    }

    // Continuity keeps an existing raw->semantic binding only when the current
    // observation still passes the same quality and cost gates as before.
    for (std::size_t i = 0; i < person_track_indices.size(); ++i) {
      const int track_idx = person_track_indices[i];
      const Track &track = tracks[static_cast<std::size_t>(track_idx)];
      if (track.occlusion_suspect) {
        continue;
      }
      const int raw_id = track.id;
      const auto continuity_it = prev_raw_to_semantic.find(raw_id);
      if (continuity_it != prev_raw_to_semantic.end() && !sid_used[continuity_it->second]) {
        const int semantic_id = continuity_it->second;
        RawContinuityDecision::Input continuity_input;
        continuity_input.track_idx = track_idx;
        continuity_input.raw_track_id = raw_id;
        continuity_input.semantic_id = semantic_id;
        const auto *identity = runtime_state.identity_store.Find(semantic_id);
        if (identity == nullptr) {
          continuity_input.identity_found = false;
        } else {
          adapter.ComputeCosts(track,
                               *identity,
                               person_features[i],
                               &continuity_input.app_cost,
                               &continuity_input.geo_cost,
                               &continuity_input.time_cost,
                               &continuity_input.final_cost);
          continuity_input.passes_missing_identity_gate =
              adapter.PassesMissingIdentityGate(track, *identity, continuity_input.app_cost,
                                                continuity_input.geo_cost);
        }
        continuity_input.weak_mot_association =
            AssociationEvidenceWeakForIdentity(track.association);
        RawContinuityDecision::Config continuity_config;
        continuity_config.raw_continuity_max_cost = input.config.raw_continuity_max_cost;
        const auto decision = RawContinuityDecision::Evaluate(continuity_input, continuity_config);
        runtime_state.last_score_debug_rows.push_back(
            MakeRawContinuityDebugRow(runtime_state.frame_index, runtime_state.occlusion_mode.mode, decision));
        if (decision.accepted) {
          track_idx_to_sid[track_idx] = semantic_id;
          sid_used[semantic_id] = true;
        }
      }
    }

    ActiveAssignmentInputCollector::Input active_input;
    active_input.tracks = &tracks;
    active_input.person_track_indices = &person_track_indices;
    active_input.person_features = &person_features;
    active_input.assigned_track_to_sid = &track_idx_to_sid;
    active_input.active_semantic_ids = &active_semantic_ids;
    active_input.semantic_id_used = [&](const int semantic_id) {
      const auto it = sid_used.find(semantic_id);
      return it != sid_used.end() && it->second;
    };
    active_input.find_identity = [&](const int semantic_id) -> const IdentityRuntimeRecord * {
      return runtime_state.identity_store.Find(semantic_id);
    };
    active_input.score_evidence = [&](const Track &track,
                                      const IdentityRuntimeRecord &identity,
                                      const std::vector<float> &feature) {
      ActiveAssignmentInputCollector::ScoreEvidence evidence;
      adapter.ComputeCosts(track, identity, feature, &evidence.app_cost, &evidence.geo_cost,
                           &evidence.time_cost, &evidence.final_score);
      evidence.passes_missing_identity_gate =
          adapter.PassesMissingIdentityGate(track, identity, evidence.app_cost, evidence.geo_cost);
      evidence.passes_missing_appearance_gate =
          adapter.PassesMissingAppearanceGate(identity, evidence.app_cost, evidence.geo_cost);
      return evidence;
    };
    const auto active_input_result = ActiveAssignmentInputCollector::Collect(active_input);

    if (!active_input_result.builder_tracks.empty() && !active_input_result.builder_candidates.empty()) {
      const auto active_build =
          AssignmentCandidateBuilder::BuildActiveAssignments(active_input_result.builder_tracks,
                                                             active_input_result.builder_candidates,
                                                             active_input_result.builder_scores);
      for (const auto &row : active_build.debug_rows) {
        runtime_state.last_score_debug_rows.push_back(
            MakeScoreDebugRow(runtime_state.frame_index, runtime_state.occlusion_mode.mode, row));
      }
      const auto assigns = adapter.SolveAssignments(active_build, tracks);
      const bool phase4_pairwise_override_pending =
          std::any_of(runtime_state.last_pairwise_assignment_debug_rows.begin(),
                      runtime_state.last_pairwise_assignment_debug_rows.end(),
                      [&](const auto &row) {
                        return row.frame_idx == runtime_state.frame_index && row.appearance_override;
                      });
      std::vector<AssignmentCandidateBuilder::DebugRow> active_rows = active_build.debug_rows;
      std::vector<ActiveAssignmentSolver::Assignment> solver_assignments;
      solver_assignments.reserve(assigns.size());
      for (const auto &assignment : assigns) {
        ActiveAssignmentSolver::Assignment solver_assignment;
        solver_assignment.track_idx = assignment.track_idx;
        solver_assignment.semantic_id = assignment.semantic_id;
        solver_assignment.score = assignment.score;
        solver_assignment.cost = assignment.cost;
        solver_assignment.margin = assignment.margin;
        solver_assignment.accepted = assignment.accepted;
        solver_assignment.reject_reason = assignment.reject_reason;
        solver_assignment.pairwise_appearance_override = assignment.pairwise_appearance_override;
        solver_assignments.push_back(std::move(solver_assignment));
      }
      AssignmentCandidateBuilder::ApplyActiveSolverResults(solver_assignments, phase4_pairwise_override_pending,
                                                           &active_rows);
      for (const auto &row : active_rows) {
        for (auto &matcher_row : runtime_state.last_score_debug_rows) {
          if (matcher_row.frame_idx == runtime_state.frame_index &&
              matcher_row.mode == runtime_state.occlusion_mode.mode &&
              matcher_row.track_idx == row.track_idx && matcher_row.semantic_id == row.semantic_id &&
              matcher_row.stage == row.stage) {
            matcher_row.selected = row.selected;
            matcher_row.margin = row.margin;
            matcher_row.accepted = row.accepted;
            matcher_row.reject_reason = row.reject_reason;
          }
        }
      }
      for (const auto &assignment : assigns) {
        if (track_idx_to_sid.find(assignment.track_idx) != track_idx_to_sid.end()) {
          continue;
        }
        if (sid_used[assignment.semantic_id]) {
          continue;
        }
        if (!assignment.accepted || phase4_pairwise_override_pending) {
          continue;
        }
        track_idx_to_sid[assignment.track_idx] = assignment.semantic_id;
        sid_used[assignment.semantic_id] = true;
      }
    }

    InactiveRecoveryInputCollector::Input recovery_input;
    recovery_input.tracks = &tracks;
    recovery_input.person_track_indices = &person_track_indices;
    recovery_input.person_features = &person_features;
    recovery_input.assigned_track_to_sid = &track_idx_to_sid;
    recovery_input.inactive_semantic_ids = &inactive_semantic_ids;
    recovery_input.semantic_id_used = [&](const int semantic_id) {
      const auto it = sid_used.find(semantic_id);
      return it != sid_used.end() && it->second;
    };
    recovery_input.find_identity = [&](const int semantic_id) -> const IdentityRuntimeRecord * {
      return runtime_state.identity_store.Find(semantic_id);
    };
    recovery_input.can_recover_identity = [&](const IdentityRuntimeRecord &identity) {
      return adapter.CanRecoverInactiveIdentity(identity);
    };
    recovery_input.score_evidence =
        [&](const Track &track, const IdentityRuntimeRecord &identity, const std::vector<float> &feature) {
          InactiveRecoveryInputCollector::ScoreEvidence evidence;
          adapter.ComputeCosts(track, identity, feature, &evidence.app_cost, &evidence.geo_cost, nullptr, nullptr);
          evidence.similarity = 1.0F - evidence.app_cost;
          evidence.recover_threshold = adapter.RecoverThresholdForSemantic(identity);
          evidence.passes_missing_identity_gate =
              adapter.PassesMissingIdentityGate(track, identity, evidence.app_cost, evidence.geo_cost);
          return evidence;
        };
    const auto recovery_input_result = InactiveRecoveryInputCollector::Collect(recovery_input);
    if (!recovery_input_result.solver_tracks.empty() && !recovery_input_result.solver_candidates.empty()) {
      InactiveRecoverySolver::Config recovery_config;
      recovery_config.recovery_max_cost = input.config.recovery_max_cost;
      const auto recovery_result = InactiveRecoverySolver::SolveHungarian(
          recovery_input_result.solver_tracks, recovery_input_result.solver_candidates,
          recovery_input_result.solver_scores, recovery_config);
      const auto recovery_rows = AssignmentCandidateBuilder::BuildInactiveRecoveryRows(recovery_result.candidates);
      for (const auto &row : recovery_rows) {
        runtime_state.last_score_debug_rows.push_back(
            MakeScoreDebugRow(runtime_state.frame_index, runtime_state.occlusion_mode.mode, row));
      }
      auto selected_recovery_rows = recovery_rows;
      AssignmentCandidateBuilder::ApplyInactiveRecoveryAssignments(recovery_result.assignments,
                                                                    &selected_recovery_rows);
      for (const auto &row : selected_recovery_rows) {
        for (auto &matcher_row : runtime_state.last_score_debug_rows) {
          if (matcher_row.frame_idx == runtime_state.frame_index &&
              matcher_row.mode == runtime_state.occlusion_mode.mode &&
              matcher_row.track_idx == row.track_idx && matcher_row.semantic_id == row.semantic_id &&
              matcher_row.stage == row.stage) {
            matcher_row.selected = row.selected;
            matcher_row.margin = row.margin;
            matcher_row.accepted = row.accepted;
          }
        }
      }
      for (const auto &assignment : recovery_result.assignments) {
        const int track_idx = assignment.track_idx;
        const int sid = assignment.semantic_id;
        if (track_idx_to_sid.find(track_idx) != track_idx_to_sid.end() || sid_used[sid]) {
          continue;
        }
        track_idx_to_sid[track_idx] = sid;
        sid_used[sid] = true;
      }
    }

    std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> final_resolution_debug_rows;
    final_resolution_debug_rows.reserve(runtime_state.last_score_debug_rows.size());
    for (const auto &row : runtime_state.last_score_debug_rows) {
      UnresolvedTrackFinalResolutionCoordinator::DebugRow final_row;
      final_row.track_idx = row.track_idx;
      final_row.raw_track_id = row.raw_track_id;
      final_row.semantic_id = row.semantic_id;
      final_row.app_cost = row.app_cost;
      final_row.geo_cost = row.geo_cost;
      final_row.time_cost = row.time_cost;
      final_row.final_score = row.final_score;
      final_row.selected = row.selected;
      final_row.stage = row.stage;
      final_row.margin = row.margin;
      final_row.accepted = row.accepted;
      final_row.reject_reason = row.reject_reason;
      final_resolution_debug_rows.push_back(std::move(final_row));
    }
    UnresolvedTrackFinalResolutionCoordinator::Input final_resolution_input;
    final_resolution_input.frame_index = runtime_state.frame_index;
    final_resolution_input.tracks = &tracks;
    final_resolution_input.person_track_indices = &person_track_indices;
    final_resolution_input.person_features = &person_features;
    final_resolution_input.assigned_track_to_sid = &track_idx_to_sid;
    final_resolution_input.sid_used = &sid_used;
    final_resolution_input.active_semantic_ids = &active_semantic_ids;
    final_resolution_input.prev_raw_to_semantic = &prev_raw_to_semantic;
    final_resolution_input.score_debug_rows = &final_resolution_debug_rows;
    final_resolution_input.config.max_missing_frames = input.config.max_missing_frames;
    final_resolution_input.config.defer_small_phase5_birth_allocation =
        input.config.auto_apply_phase5_birth_allocations;
    final_resolution_input.find_identity = [&](const int semantic_id) -> const IdentityRuntimeRecord * {
      return runtime_state.identity_store.Find(semantic_id);
    };
    final_resolution_input.active_assignment_max_cost =
        [&](const IdentityRuntimeRecord &identity, const AssociationEvidence &association) {
          return adapter.ActiveAssignmentMaxCost(identity, association);
        };
    final_resolution_input.score_evidence =
        [&](const Track &track, const IdentityRuntimeRecord &identity, const std::vector<float> &feature) {
          UnresolvedTrackFinalResolutionCoordinator::ScoreEvidence evidence;
          adapter.ComputeCosts(track, identity, feature, &evidence.app_cost, &evidence.geo_cost,
                               &evidence.time_cost, &evidence.final_score);
          return evidence;
        };
    final_resolution_input.looks_like_merged_side_reappearance =
        [&](const Track &candidate, const IdentityRuntimeRecord &identity,
            const std::vector<Track> &all_tracks, const int candidate_idx,
            const std::unordered_map<int, int> &assigned_track_to_sid, const float app_cost) {
          return adapter.LooksLikeMergedSideReappearance(candidate, identity, all_tracks, candidate_idx,
                                                         assigned_track_to_sid, app_cost);
        };
    final_resolution_input.evaluate_birth = [&](const BirthManager::Input &birth_input) {
      return runtime_state.birth_manager.Evaluate(birth_input, [&]() {
        return adapter.AllocateNewSemanticId();
      });
    };
    const auto final_resolution = UnresolvedTrackFinalResolutionCoordinator::Resolve(final_resolution_input);
    track_idx_to_sid = final_resolution.assigned_track_to_sid;
    sid_used = final_resolution.sid_used;
    for (const int raw_track_id : final_resolution.erase_pending_raw_track_ids) {
      runtime_state.birth_manager.Erase(raw_track_id);
    }
    for (const auto &debug_row : final_resolution.debug_rows) {
      ScoreDebugRow row;
      row.frame_idx = runtime_state.frame_index;
      row.mode = runtime_state.occlusion_mode.mode;
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
      runtime_state.last_score_debug_rows.push_back(std::move(row));
    }

    if (input.config.auto_apply_phase5_birth_allocations) {
      IdentityRuntimeMutationApplier mutation_applier(input.config, &runtime_state, input.appearance_features);
      std::vector<int> phase5_birth_allocation_raw_ids;
      for (const auto &row : runtime_state.last_score_debug_rows) {
        if (row.frame_idx == runtime_state.frame_index && row.stage == "phase5_birth_candidate" &&
            row.selected && !row.accepted && row.reject_reason == "phase5_birth_manager_pending") {
          phase5_birth_allocation_raw_ids.push_back(row.raw_track_id);
        }
      }
      for (const int raw_track_id : phase5_birth_allocation_raw_ids) {
        if (!mutation_applier.ApplyPhase5BirthAllocation(tracks, raw_track_id, input.frame)) {
          continue;
        }
        const int semantic_id = runtime_state.raw_semantic_bindings.SemanticIdForRawTrack(raw_track_id);
        if (semantic_id <= 0) {
          continue;
        }
        const auto track_it = std::find_if(tracks.begin(), tracks.end(), [&](const auto &track) {
          return track.id == raw_track_id;
        });
        if (track_it == tracks.end()) {
          continue;
        }
        const int track_idx = static_cast<int>(std::distance(tracks.begin(), track_it));
        track_idx_to_sid[track_idx] = semantic_id;
        sid_used[semantic_id] = true;
      }
    }
  }

  // Preserve semantic continuity from the prepared assignment map. Primary raw
  // tracks do not override this map in NORMAL mode.
  std::vector<AssignmentApplicationPlan::TrackApplicationCandidate> application_candidates;
  application_candidates.reserve(track_idx_to_sid.size());
  for (const int track_idx : person_track_indices) {
    const auto sid_it = track_idx_to_sid.find(track_idx);
    if (sid_it == track_idx_to_sid.end()) {
      continue;
    }
    AssignmentApplicationPlan::TrackApplicationCandidate candidate;
    candidate.track_idx = track_idx;
    candidate.raw_track_id = tracks[static_cast<std::size_t>(track_idx)].id;
    candidate.semantic_id = sid_it->second;
    application_candidates.push_back(candidate);
  }

  std::vector<AssignmentApplicationPlan::AcceptedDebugRow> accepted_debug_rows;
  accepted_debug_rows.reserve(runtime_state.last_score_debug_rows.size());
  for (const auto &row : runtime_state.last_score_debug_rows) {
    AssignmentApplicationPlan::AcceptedDebugRow accepted_row;
    accepted_row.frame_idx = row.frame_idx;
    accepted_row.track_idx = row.track_idx;
    accepted_row.semantic_id = row.semantic_id;
    accepted_row.final_score = row.final_score;
    accepted_row.margin = row.margin;
    accepted_row.accepted = row.accepted;
    accepted_row.stage = row.stage;
    accepted_debug_rows.push_back(std::move(accepted_row));
  }

  std::unordered_set<int> occupied_semantic_ids;
  occupied_semantic_ids = runtime_state.identity_store.OccupiedSemanticIds();

  std::vector<int> raw_map_track_order;
  raw_map_track_order.reserve(track_idx_to_sid.size());
  for (const auto &[track_idx, sid] : track_idx_to_sid) {
    (void)sid;
    raw_map_track_order.push_back(track_idx);
  }

  const auto application_plan = AssignmentApplicationPlan::Build(
      application_candidates, accepted_debug_rows, runtime_state.frame_index, occupied_semantic_ids,
      &runtime_state.semantic_id_allocator, raw_map_track_order);

  std::vector<int> planned_track_indices;
  planned_track_indices.reserve(track_idx_to_sid.size());
  for (const auto &[planned_track_idx, unused_sid] : track_idx_to_sid) {
    (void)unused_sid;
    planned_track_indices.push_back(planned_track_idx);
  }

  std::unordered_map<int, std::size_t> feature_index_by_track_idx;
  feature_index_by_track_idx.reserve(person_track_indices.size());
  for (std::size_t rel = 0; rel < person_track_indices.size(); ++rel) {
    feature_index_by_track_idx[person_track_indices[rel]] = rel;
  }

  AssignmentApplicationExecutor<ScoreDebugRow>::Config executor_config;
  executor_config.frame_idx = runtime_state.frame_index;
  executor_config.occlusion_protect_frames = input.config.occlusion_protect_frames;
  executor_config.protect_unseen_active_people =
      runtime_state.occlusion_mode.mode == IdentityLifecycleMode::kMerged ||
      runtime_state.occlusion_mode.mode == IdentityLifecycleMode::kSplitRecovery || input.has_overlap;

  AssignmentApplicationExecutor<ScoreDebugRow>::Execute(
      application_plan, planned_track_indices, tracks, feature_index_by_track_idx, person_features,
      &runtime_state.last_score_debug_rows, &runtime_state.identity_store,
      &runtime_state.raw_semantic_bindings, executor_config,
      [&](const Track &track, const int track_idx, const int semantic_id,
          const IdentityRuntimeRecord *identity, const float assignment_cost,
          const float assignment_margin, const bool force_geometry_update) {
        (void)semantic_id;
        return adapter.EvaluateUpdatePolicy(track, tracks, track_idx, identity, true,
                                            assignment_cost, assignment_margin, force_geometry_update);
      },
      [&](const Track &track, const int semantic_id, const std::vector<float> &feature,
          const FeatureUpdatePolicy::Decision &update_policy, const float assignment_cost,
          const float assignment_margin) {
        adapter.UpsertIdentity(track, semantic_id, feature, update_policy, assignment_cost,
                               assignment_margin);
      });

  IdentityRuntimeLifecycleCoordinator::ApplyEndFrameAging(
      IdentityRuntimeLifecycleCoordinator::EndFrameInput{&runtime_state.identity_store});

  runtime_state.occlusion_mode.prev_visible_person_count = static_cast<int>(person_track_indices.size());
  runtime_state.occlusion_mode.prev_had_overlap = input.has_overlap;

  result.raw_to_semantic = runtime_state.raw_semantic_bindings.Current();
  result.score_debug_rows = runtime_state.last_score_debug_rows;
  result.pairwise_assignment_debug_rows = runtime_state.last_pairwise_assignment_debug_rows;
  return result;
}

}  // namespace vision_demo_host
