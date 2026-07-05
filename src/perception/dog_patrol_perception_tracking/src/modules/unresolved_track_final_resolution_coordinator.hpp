#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "birth_manager.hpp"
#include "legacy_identity_record.hpp"
#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

class UnresolvedTrackFinalResolutionCoordinator {
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

  struct ScoreEvidence {
    float app_cost{1.0F};
    float geo_cost{1.0F};
    float time_cost{1.0F};
    float final_score{1.0F};
  };

  struct Config {
    int max_missing_frames{180};
    bool defer_small_phase5_birth_allocation{false};
  };

  struct Input {
    int frame_index{-1};
    const std::vector<Track> *tracks{nullptr};
    const std::vector<int> *person_track_indices{nullptr};
    const std::vector<std::vector<float>> *person_features{nullptr};
    const std::unordered_map<int, int> *assigned_track_to_sid{nullptr};
    const std::unordered_map<int, bool> *sid_used{nullptr};
    const std::vector<int> *active_semantic_ids{nullptr};
    const std::unordered_map<int, int> *prev_raw_to_semantic{nullptr};
    const std::vector<DebugRow> *score_debug_rows{nullptr};
    Config config;
    std::function<const LegacyIdentityRecord *(int)> find_identity;
    std::function<float(const LegacyIdentityRecord &, const AssociationEvidence &)> active_assignment_max_cost;
    std::function<ScoreEvidence(const Track &, const LegacyIdentityRecord &, const std::vector<float> &)>
        score_evidence;
    std::function<bool(const Track &, const LegacyIdentityRecord &, const std::vector<Track> &, int,
                       const std::unordered_map<int, int> &, float)>
        looks_like_merged_side_reappearance;
    std::function<BirthManager::Result(const BirthManager::Input &)> evaluate_birth;
  };

  struct Result {
    std::unordered_map<int, int> assigned_track_to_sid;
    std::unordered_map<int, bool> sid_used;
    std::vector<DebugRow> debug_rows;
    std::vector<int> erase_pending_raw_track_ids;
  };

  static Result Resolve(const Input &input);
};

}  // namespace vision_demo_host
