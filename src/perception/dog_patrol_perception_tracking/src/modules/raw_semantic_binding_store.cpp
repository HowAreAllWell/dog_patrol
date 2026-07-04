#include "raw_semantic_binding_store.hpp"

#include <algorithm>

namespace vision_demo_host {

namespace {

auto FindPlannedEntry(std::vector<AssignmentApplicationPlan::RawMapping> *planned_entries, const int raw_track_id) {
  return std::find_if(planned_entries->begin(), planned_entries->end(), [&](const auto &entry) {
    return entry.raw_track_id == raw_track_id;
  });
}

}  // namespace

std::unordered_map<int, int> RawSemanticBindingStore::PreviousSnapshot() const {
  return raw_to_semantic_id_;
}

void RawSemanticBindingStore::ReplaceFromPlannedEntries(
    const std::vector<AssignmentApplicationPlan::RawMapping> &planned_entries) {
  planned_entries_ = planned_entries;
  raw_to_semantic_id_.clear();
  raw_to_semantic_id_.reserve(planned_entries_.size());
  for (const auto &entry : planned_entries_) {
    raw_to_semantic_id_[entry.raw_track_id] = entry.semantic_id;
  }
}

int RawSemanticBindingStore::SemanticIdForRawTrack(const int raw_track_id) const {
  const auto it = raw_to_semantic_id_.find(raw_track_id);
  if (it == raw_to_semantic_id_.end()) {
    return -1;
  }
  return it->second;
}

bool RawSemanticBindingStore::HasBinding(const int raw_track_id) const {
  return raw_to_semantic_id_.count(raw_track_id) > 0U;
}

void RawSemanticBindingStore::Bind(const int raw_track_id, const int semantic_id) {
  raw_to_semantic_id_[raw_track_id] = semantic_id;
  auto planned_it = FindPlannedEntry(&planned_entries_, raw_track_id);
  if (planned_it == planned_entries_.end()) {
    planned_entries_.push_back(AssignmentApplicationPlan::RawMapping{raw_track_id, semantic_id});
    return;
  }
  planned_it->semantic_id = semantic_id;
}

void RawSemanticBindingStore::Erase(const int raw_track_id) {
  raw_to_semantic_id_.erase(raw_track_id);
  const auto planned_it = FindPlannedEntry(&planned_entries_, raw_track_id);
  if (planned_it != planned_entries_.end()) {
    planned_entries_.erase(planned_it);
  }
}

void RawSemanticBindingStore::Clear() {
  raw_to_semantic_id_.clear();
  planned_entries_.clear();
}

void RawSemanticBindingStore::Reset() {
  Clear();
}

const std::unordered_map<int, int> &RawSemanticBindingStore::Current() const {
  return raw_to_semantic_id_;
}

const std::vector<AssignmentApplicationPlan::RawMapping> &RawSemanticBindingStore::PlannedEntries() const {
  return planned_entries_;
}

}  // namespace vision_demo_host
