#include "birth_manager.hpp"

namespace vision_demo_host {

BirthManager::BirthManager(BirthCandidateDecision::Config config) : config_(std::move(config)) {}

void BirthManager::Reset() {
  pending_candidates_.Clear();
}

void BirthManager::Erase(const int raw_track_id) {
  pending_candidates_.Erase(raw_track_id);
}

BirthManager::Result BirthManager::Evaluate(const Input &input,
                                            const AllocateSemanticIdFn &allocate_semantic_id) {
  Result result;

  if (!input.phase5_birth_manager_enabled && input.small_person_requires_stability) {
    result.stable_observation_count =
        pending_candidates_.UpdateObservation(input.raw_track_id, input.frame_index);
  }

  BirthCandidateDecision::Input decision_input;
  decision_input.track_idx = input.track_idx;
  decision_input.raw_track_id = input.raw_track_id;
  decision_input.hold_for_ambiguous_recovery = input.hold_for_ambiguous_recovery;
  decision_input.duplicate_split = input.duplicate_split;
  decision_input.hide_reason = input.hide_reason;
  decision_input.phase5_birth_manager_enabled = input.phase5_birth_manager_enabled;
  decision_input.small_person_requires_stability = input.small_person_requires_stability;
  decision_input.stable_observation_count = result.stable_observation_count;
  result.decision = BirthCandidateDecision::Evaluate(decision_input, config_);

  if (result.decision.clear_pending_candidate) {
    pending_candidates_.Erase(input.raw_track_id);
  }

  switch (result.decision.action) {
    case BirthCandidateDecision::Action::kHideWithDebugRow:
    case BirthCandidateDecision::Action::kPhase5Pending:
      result.has_debug_row = true;
      result.debug_row = MakeDebugRow(result.decision, -1);
      return result;
    case BirthCandidateDecision::Action::kLegacyPendingWithoutDebugRow:
      return result;
    case BirthCandidateDecision::Action::kAllocateNewSemantic:
      result.semantic_id = allocate_semantic_id();
      result.allocated_semantic_id = true;
      result.has_debug_row = true;
      result.debug_row = MakeDebugRow(result.decision, result.semantic_id);
      return result;
  }

  return result;
}

BirthManager::DebugRow BirthManager::MakeDebugRow(const BirthCandidateDecision::Decision &decision,
                                                  const int semantic_id) {
  DebugRow row;
  row.track_idx = decision.track_idx;
  row.raw_track_id = decision.raw_track_id;
  row.semantic_id = semantic_id;
  row.final_score = decision.final_score;
  row.selected = decision.selected;
  row.stage = decision.stage;
  row.margin = decision.margin;
  row.accepted = decision.accepted;
  row.reject_reason = decision.reject_reason;
  return row;
}

}  // namespace vision_demo_host
