#pragma once

#include <string>
#include <vector>

#include "active_assignment_solver.hpp"
#include "inactive_recovery_solver.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class AssignmentCandidateBuilder {
 public:
  struct DebugRow {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float app_cost{0.0F};
    float geo_cost{0.0F};
    float time_cost{0.0F};
    float final_score{0.0F};
    bool selected{false};
    std::string stage;
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
  };

  struct ActiveTrackInput {
    int track_idx{-1};
    int raw_track_id{-1};
    AssociationEvidence association;
  };

  struct ActiveCandidateInput {
    int semantic_id{-1};
    int missing_frames{0};
  };

  struct CandidateScore {
    int track_row{-1};
    int candidate_col{-1};
    float app_cost{1.0F};
    float geo_cost{1.0F};
    float time_cost{1.0F};
    float final_score{1.0F};
    bool passes_missing_identity_gate{true};
    bool passes_missing_appearance_gate{true};
  };

  struct ActiveBuildResult {
    std::vector<DebugRow> debug_rows;
    std::vector<ActiveAssignmentSolver::TrackInput> solver_tracks;
    std::vector<ActiveAssignmentSolver::CandidateInput> solver_candidates;
    std::vector<std::vector<float>> cost_matrix;
    std::vector<std::vector<float>> appearance_cost_matrix;
  };

  static DebugRow MakeAssignCandidateRow(int track_idx, int raw_track_id, int semantic_id, float app_cost,
                                         float geo_cost, float time_cost, float final_score,
                                         const std::string &reject_reason);
  static ActiveBuildResult BuildActiveAssignments(const std::vector<ActiveTrackInput> &tracks,
                                                  const std::vector<ActiveCandidateInput> &candidates,
                                                  const std::vector<CandidateScore> &scores);
  static void ApplyActiveSolverResults(const std::vector<ActiveAssignmentSolver::Assignment> &assignments,
                                       bool phase4_pairwise_override_pending,
                                       std::vector<DebugRow> *rows);

  static std::vector<DebugRow> BuildInactiveRecoveryRows(
      const std::vector<InactiveRecoverySolver::CandidateDecision> &decisions);
  static void ApplyInactiveRecoveryAssignments(const std::vector<InactiveRecoverySolver::Assignment> &assignments,
                                               std::vector<DebugRow> *rows);
};

}  // namespace dog_patrol_perception_tracking
