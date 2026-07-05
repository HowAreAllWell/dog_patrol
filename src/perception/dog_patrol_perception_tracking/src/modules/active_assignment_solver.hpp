#pragma once

#include <string>
#include <vector>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class ActiveAssignmentSolver {
 public:
  static constexpr float kBigCost = 1e6F;

  struct Config {
    int max_missing_frames{180};
    float active_assign_max_cost{0.55F};
    float min_assignment_margin{0.08F};
  };

  struct TrackInput {
    int track_idx{-1};
    int raw_track_id{-1};
    AssociationEvidence association;
  };

  struct CandidateInput {
    int semantic_id{-1};
    int missing_frames{0};
  };

  struct PairwiseDebug {
    int first_track_row{-1};
    int second_track_row{-1};
    int selected_first_col{-1};
    int selected_second_col{-1};
    int alternate_first_col{-1};
    int alternate_second_col{-1};
    float selected_final_cost{0.0F};
    float alternate_final_cost{0.0F};
    float selected_app_cost{0.0F};
    float alternate_app_cost{0.0F};
    float margin{0.0F};
    bool appearance_override{false};
  };

  struct Assignment {
    int track_row{-1};
    int candidate_col{-1};
    int track_idx{-1};
    int semantic_id{-1};
    float score{-1.0F};
    float cost{1.0F};
    float margin{0.0F};
    bool accepted{false};
    std::string reject_reason;
    bool pairwise_appearance_override{false};
  };

  struct Result {
    std::vector<Assignment> assignments;
    std::vector<PairwiseDebug> pairwise_debug_rows;
  };

  static Result Solve(const std::vector<TrackInput> &tracks,
                      const std::vector<CandidateInput> &candidates,
                      const std::vector<std::vector<float>> &cost,
                      const std::vector<std::vector<float>> &appearance_cost,
                      const Config &config);

  static std::vector<int> HungarianAssignment(const std::vector<std::vector<float>> &cost);
  static float AssignmentMargin(const std::vector<std::vector<float>> &cost, std::size_t row,
                                int selected_col);
};

}  // namespace vision_demo_host
