#pragma once

#include <unordered_set>

namespace dog_patrol_perception_tracking {

class SemanticIdAllocator {
 public:
  void Reset();
  int Allocate(const std::unordered_set<int> &occupied_semantic_ids);

 private:
  int next_non_primary_semantic_id_{2};
};

}  // namespace dog_patrol_perception_tracking
