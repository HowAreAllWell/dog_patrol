#include "identity_runtime_store.hpp"

#include <algorithm>

#include "identity_runtime_record_lifecycle.hpp"

namespace dog_patrol_perception_tracking {

void IdentityRuntimeStore::Reset() { identities_by_semantic_id_.clear(); }

bool IdentityRuntimeStore::Contains(const int semantic_id) const {
  return identities_by_semantic_id_.find(semantic_id) != identities_by_semantic_id_.end();
}

const IdentityRuntimeRecord *IdentityRuntimeStore::Find(const int semantic_id) const {
  const auto it = identities_by_semantic_id_.find(semantic_id);
  return it == identities_by_semantic_id_.end() ? nullptr : &it->second;
}

IdentityRuntimeRecord *IdentityRuntimeStore::FindMutable(const int semantic_id) {
  const auto it = identities_by_semantic_id_.find(semantic_id);
  return it == identities_by_semantic_id_.end() ? nullptr : &it->second;
}

IdentityRuntimeRecord &IdentityRuntimeStore::Upsert(const int semantic_id) {
  IdentityRuntimeRecord &record = identities_by_semantic_id_[semantic_id];
  record.semantic_id = semantic_id;
  return record;
}

std::size_t IdentityRuntimeStore::Size() const { return identities_by_semantic_id_.size(); }

std::unordered_set<int> IdentityRuntimeStore::OccupiedSemanticIds() const {
  std::unordered_set<int> occupied_semantic_ids;
  occupied_semantic_ids.reserve(identities_by_semantic_id_.size());
  for (const auto &[semantic_id, identity] : identities_by_semantic_id_) {
    (void)identity;
    occupied_semantic_ids.insert(semantic_id);
  }
  return occupied_semantic_ids;
}

std::vector<int> IdentityRuntimeStore::PersonSemanticIds(const bool active, const int max_missing_frames) const {
  std::vector<int> semantic_ids;
  semantic_ids.reserve(identities_by_semantic_id_.size());
  for (const auto &[semantic_id, identity] : identities_by_semantic_id_) {
    if (identity.class_id != ClassId::kPerson) {
      continue;
    }
    const bool is_active = identity.missing_frames <= max_missing_frames;
    if (is_active == active) {
      semantic_ids.push_back(semantic_id);
    }
  }
  return semantic_ids;
}

std::vector<int> IdentityRuntimeStore::SemanticIdsInStorageOrder() const {
  std::vector<int> semantic_ids;
  semantic_ids.reserve(identities_by_semantic_id_.size());
  for (const auto &[semantic_id, identity] : identities_by_semantic_id_) {
    (void)identity;
    semantic_ids.push_back(semantic_id);
  }
  return semantic_ids;
}

void IdentityRuntimeStore::BeginFrame() {
  for (auto &[semantic_id, identity] : identities_by_semantic_id_) {
    (void)semantic_id;
    IdentityRuntimeRecordLifecycle::BeginFrame(&identity);
  }
}

void IdentityRuntimeStore::AgeOneFrame() {
  for (auto &[semantic_id, identity] : identities_by_semantic_id_) {
    (void)semantic_id;
    IdentityRuntimeRecordLifecycle::AgeOneFrame(&identity);
  }
}

void IdentityRuntimeStore::ProtectUnseenActivePeople(const int occlusion_protect_frames) {
  for (auto &[semantic_id, identity] : identities_by_semantic_id_) {
    (void)semantic_id;
    IdentityRuntimeRecordLifecycle::ProtectIfUnseenActivePerson(occlusion_protect_frames, &identity);
  }
}

bool IdentityRuntimeStore::MarkCarrierMissingForHandoff(const int semantic_id) {
  auto *record = FindMutable(semantic_id);
  if (record == nullptr) {
    return false;
  }
  IdentityRuntimeRecordLifecycle::MarkCarrierMissingForHandoff(record);
  return true;
}

std::vector<IdentityRuntimeSnapshot> IdentityRuntimeStore::Snapshots() const {
  std::vector<IdentityRuntimeSnapshot> snapshots;
  snapshots.reserve(identities_by_semantic_id_.size());
  for (const auto &[semantic_id, identity] : identities_by_semantic_id_) {
    snapshots.push_back(IdentityRuntimeRecordLifecycle::BuildSnapshot(semantic_id, identity));
  }
  std::sort(snapshots.begin(), snapshots.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.semantic_id < rhs.semantic_id;
  });
  return snapshots;
}

}  // namespace dog_patrol_perception_tracking
