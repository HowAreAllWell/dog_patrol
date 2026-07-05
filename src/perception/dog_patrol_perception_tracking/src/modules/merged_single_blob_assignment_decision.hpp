#pragma once

#include <string>
#include <vector>

#include "inactive_recovery_solver.hpp"

namespace vision_demo_host {

class MergedSingleBlobAssignmentDecision {
 public:
  static constexpr float kBigCost = 1e6F;

  struct Config {
    float min_assignment_margin{0.08F};
    int max_missing_frames{180};
  };

  struct CandidateRow {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    int missing_frames{0};
    float app_cost{0.0F};
    float geo_cost{0.0F};
    float time_cost{0.0F};
    float final_score{0.0F};
    bool selected{false};
    std::string stage{"merged_candidate"};
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
  };

  struct Input {
    int track_idx{-1};
    int raw_track_id{-1};
    int continuity_semantic_id{-1};
    std::vector<CandidateRow> active_candidates;
    std::vector<CandidateRow> inactive_recovery_rows;
    std::vector<InactiveRecoverySolver::Assignment> inactive_recovery_assignments;
  };

  struct Result {
    int semantic_id{-1};
    bool needs_new_semantic_id{false};
    bool used_inactive_recovery{false};
    std::vector<CandidateRow> active_candidates;
    std::vector<CandidateRow> inactive_recovery_rows;
  };

  static Result Decide(const Input &input, const Config &config);
};

}  // namespace vision_demo_host
