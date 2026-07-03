#pragma once

#include <string>
#include <vector>

namespace vision_demo_host {

class InactiveRecoverySolver {
 public:
  static constexpr float kBigCost = 1e6F;

  struct Config {
    float recovery_max_cost{0.45F};
  };

  struct TrackInput {
    int track_idx{-1};
    int raw_track_id{-1};
  };

  struct CandidateInput {
    int semantic_id{-1};
  };

  struct CandidateScore {
    int track_row{-1};
    int candidate_col{-1};
    float app_cost{1.0F};
    float geo_cost{1.0F};
    float similarity{0.0F};
    float recover_threshold{0.85F};
    bool passes_missing_identity_gate{true};
  };

  struct CandidateDecision {
    int track_row{-1};
    int candidate_col{-1};
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float app_cost{1.0F};
    float geo_cost{1.0F};
    float similarity{0.0F};
    float recovery_cost{1.0F};
    bool accepted{false};
    std::string reject_reason;
  };

  struct Assignment {
    int track_row{-1};
    int candidate_col{-1};
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float similarity{0.0F};
    float recovery_cost{1.0F};
    float margin{0.0F};
  };

  struct Result {
    std::vector<CandidateDecision> candidates;
    std::vector<Assignment> assignments;
  };

  static Result SelectBestSimilarity(const std::vector<TrackInput> &tracks,
                                     const std::vector<CandidateInput> &candidates,
                                     const std::vector<CandidateScore> &scores,
                                     const Config &config);

  static Result SolveHungarian(const std::vector<TrackInput> &tracks,
                               const std::vector<CandidateInput> &candidates,
                               const std::vector<CandidateScore> &scores,
                               const Config &config);
};

}  // namespace vision_demo_host
