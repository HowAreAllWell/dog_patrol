#include "semantic_id_allocator.hpp"

namespace dog_patrol_perception_tracking {

void SemanticIdAllocator::Reset() {
  next_non_primary_semantic_id_ = 2;
}

int SemanticIdAllocator::Allocate(const std::unordered_set<int> &occupied_semantic_ids) {
  while (next_non_primary_semantic_id_ == 1 ||
         occupied_semantic_ids.count(next_non_primary_semantic_id_) > 0) {
    ++next_non_primary_semantic_id_;
  }
  return next_non_primary_semantic_id_++;
}

}  // namespace dog_patrol_perception_tracking
