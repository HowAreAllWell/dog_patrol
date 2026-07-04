#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "semantic_id_allocator.hpp"

namespace vision_demo_host {

class AssignmentApplicationPlan {
 public:
  struct TrackApplicationCandidate {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
  };

  struct AcceptedDebugRow {
    int frame_idx{-1};
    int track_idx{-1};
    int semantic_id{-1};
    float final_score{0.0F};
    float margin{0.0F};
    bool accepted{false};
    std::string stage;
  };

  struct Application {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float assignment_cost{0.0F};
    float assignment_margin{1.0F};
    bool found_accepted_row{false};
    std::string accepted_stage;
  };

  struct RawMapping {
    int raw_track_id{-1};
    int semantic_id{-1};
  };

  struct Result {
    std::vector<Application> applications;
    std::vector<RawMapping> next_raw_to_semantic_entries;
    std::unordered_map<int, int> next_raw_to_semantic;
  };

  static Result Build(const std::vector<TrackApplicationCandidate> &candidates,
                      const std::vector<AcceptedDebugRow> &debug_rows,
                      int frame_idx,
                      const std::unordered_set<int> &occupied_semantic_ids,
                      SemanticIdAllocator *semantic_id_allocator,
                      const std::vector<int> &raw_map_track_order = {});
};

}  // namespace vision_demo_host
