#pragma once

#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "identity_assignment_engine_adapter.hpp"

namespace dog_patrol_perception_tracking {

// Owns the ordered decisions and mutations for one prepared identity frame.
// The adapter remains responsible for frame preparation and public accessors.
class IdentityAssignmentFrameTransaction {
 public:
  using ScoreDebugRow = IdentityAssignmentEngineAdapter::ScoreDebugRow;
  using PairwiseAssignmentDebugRow = IdentityAssignmentEngineAdapter::PairwiseAssignmentDebugRow;

  struct Input {
    const std::vector<Track> *tracks{nullptr};
    const std::vector<int> *person_track_indices{nullptr};
    const std::vector<std::vector<float>> *person_features{nullptr};
    const std::vector<int> *active_semantic_ids{nullptr};
    const std::vector<int> *inactive_semantic_ids{nullptr};
    const std::unordered_map<int, int> *prev_raw_to_semantic{nullptr};
    std::unordered_map<int, int> initial_track_idx_to_sid;
    std::unordered_map<int, bool> initial_sid_used;
    int bootstrap_track_idx{-1};
    bool resolve_assignments{true};
    bool has_overlap{false};
    const cv::Mat *frame{nullptr};
    IdentityAssignmentEngineAdapter::Config config;
    IdentityAssignmentEngineAdapter::RuntimeState *runtime_state{nullptr};
    IdentityAssignmentEngineAdapter *adapter{nullptr};
    AppearanceFeatureService *appearance_features{nullptr};
  };

  struct Result {
    std::unordered_map<int, int> track_idx_to_sid;
    std::unordered_map<int, bool> sid_used;
    std::unordered_map<int, int> raw_to_semantic;
    std::vector<ScoreDebugRow> score_debug_rows;
    std::vector<PairwiseAssignmentDebugRow> pairwise_assignment_debug_rows;
  };

  static Result Execute(const Input &input);
};

}  // namespace dog_patrol_perception_tracking
