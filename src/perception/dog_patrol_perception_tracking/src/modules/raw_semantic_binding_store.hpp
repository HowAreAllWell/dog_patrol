#pragma once

#include <unordered_map>
#include <vector>

#include "assignment_application_plan.hpp"

namespace dog_patrol_perception_tracking {

class RawSemanticBindingStore {
 public:
  std::unordered_map<int, int> PreviousSnapshot() const;
  void ReplaceFromPlannedEntries(const std::vector<AssignmentApplicationPlan::RawMapping> &planned_entries);
  int SemanticIdForRawTrack(int raw_track_id) const;
  bool HasBinding(int raw_track_id) const;
  void Bind(int raw_track_id, int semantic_id);
  void Erase(int raw_track_id);
  void Clear();
  void Reset();
  const std::unordered_map<int, int> &Current() const;
  const std::vector<AssignmentApplicationPlan::RawMapping> &PlannedEntries() const;

 private:
  std::unordered_map<int, int> raw_to_semantic_id_;
  std::vector<AssignmentApplicationPlan::RawMapping> planned_entries_;
};

}  // namespace dog_patrol_perception_tracking
