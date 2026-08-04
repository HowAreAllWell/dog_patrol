#pragma once

#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"

namespace dog_patrol_perception_tracking {

class Phase5BirthCoordinator {
 public:
  struct ShadowState {
    cv::Rect2f bbox;
    float confidence{0.0F};
    int stable_frames{0};
    int last_update_frame{-1};
    bool requires_stability{false};
  };

  using ApplyAcceptedAllocationFn = std::function<void(int raw_track_id)>;
  using RefreshScoreRowsFn = std::function<void()>;

  struct ApplyInput {
    bool enabled{false};
    int current_frame_idx{-1};
    const std::unordered_map<int, TrackletObservation> *observations_by_raw_track_id{nullptr};
    std::vector<IdentityManager::ScoreDebugRow> *score_rows{nullptr};
    std::map<int, ShadowState> *shadow_by_raw_track_id{nullptr};
    std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
  };

  struct ShadowRowsInput {
    int current_frame_idx{-1};
    const std::vector<TrackletObservation> *observations{nullptr};
    const std::unordered_map<int, TrackletObservation> *observations_by_raw_track_id{nullptr};
    const std::unordered_map<int, int> *raw_to_semantic_id{nullptr};
    const std::vector<IdentityManager::ScoreDebugRow> *score_rows{nullptr};
    std::map<int, ShadowState> *shadow_by_raw_track_id{nullptr};
    std::vector<IdentityManager::Phase3ShadowDebugRow> *phase3_rows{nullptr};
    int *next_event_idx{nullptr};
  };

  static void ApplyAcceptedBirths(const ApplyInput &input,
                                  const ApplyAcceptedAllocationFn &apply_accepted_allocation,
                                  const RefreshScoreRowsFn &refresh_score_rows);

  static void AppendShadowLifecycleRows(const ShadowRowsInput &input);
};

}  // namespace dog_patrol_perception_tracking
