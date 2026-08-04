#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

#include "inactive_recovery_solver.hpp"
#include "identity_runtime_record.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class InactiveRecoveryInputCollector {
 public:
  struct ScoreEvidence {
    float app_cost{1.0F};
    float geo_cost{1.0F};
    float similarity{0.0F};
    float recover_threshold{0.85F};
    bool passes_missing_identity_gate{true};
  };

  struct Input {
    const std::vector<Track> *tracks{nullptr};
    const std::vector<int> *person_track_indices{nullptr};
    const std::vector<std::vector<float>> *person_features{nullptr};
    const std::unordered_map<int, int> *assigned_track_to_sid{nullptr};
    const std::vector<int> *inactive_semantic_ids{nullptr};
    std::function<bool(int)> semantic_id_used;
    std::function<const IdentityRuntimeRecord *(int)> find_identity;
    std::function<bool(const IdentityRuntimeRecord &)> can_recover_identity;
    std::function<ScoreEvidence(const Track &, const IdentityRuntimeRecord &, const std::vector<float> &)>
        score_evidence;
  };

  struct Result {
    std::vector<int> recovery_track_indices;
    std::vector<int> free_semantic_ids;
    std::vector<std::vector<float>> selected_features;
    std::vector<InactiveRecoverySolver::TrackInput> solver_tracks;
    std::vector<InactiveRecoverySolver::CandidateInput> solver_candidates;
    std::vector<InactiveRecoverySolver::CandidateScore> solver_scores;
  };

  static Result Collect(const Input &input);
};

}  // namespace dog_patrol_perception_tracking
