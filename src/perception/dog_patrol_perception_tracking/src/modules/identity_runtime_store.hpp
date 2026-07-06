#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "identity_runtime_record.hpp"
#include "identity_runtime_snapshot.hpp"

namespace vision_demo_host {

class IdentityRuntimeStore {
 public:
  void Reset();

  bool Contains(int semantic_id) const;
  const IdentityRuntimeRecord *Find(int semantic_id) const;
  IdentityRuntimeRecord *FindMutable(int semantic_id);
  IdentityRuntimeRecord &Upsert(int semantic_id);

  std::size_t Size() const;
  std::unordered_set<int> OccupiedSemanticIds() const;
  std::vector<int> PersonSemanticIds(bool active, int max_missing_frames) const;
  std::vector<int> SemanticIdsInStorageOrder() const;

  void BeginFrame();
  void AgeOneFrame();
  void ProtectUnseenActivePeople(int occlusion_protect_frames);
  bool MarkCarrierMissingForHandoff(int semantic_id);

  std::vector<IdentityRuntimeSnapshot> Snapshots() const;

 private:
  std::unordered_map<int, IdentityRuntimeRecord> identities_by_semantic_id_;
};

}  // namespace vision_demo_host
