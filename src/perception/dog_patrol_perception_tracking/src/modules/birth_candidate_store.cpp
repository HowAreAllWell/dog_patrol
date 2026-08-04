#include "birth_candidate_store.hpp"

namespace dog_patrol_perception_tracking {

int BirthCandidateStore::UpdateObservation(const int raw_track_id, const int frame_index) {
  Candidate &candidate = candidates_by_raw_id_[raw_track_id];
  if (candidate.last_seen_frame != frame_index - 1) {
    candidate.consecutive_hits = 0;
  }
  candidate.consecutive_hits += 1;
  candidate.last_seen_frame = frame_index;
  return candidate.consecutive_hits;
}

void BirthCandidateStore::Erase(const int raw_track_id) {
  candidates_by_raw_id_.erase(raw_track_id);
}

void BirthCandidateStore::Clear() {
  candidates_by_raw_id_.clear();
}

}  // namespace dog_patrol_perception_tracking
