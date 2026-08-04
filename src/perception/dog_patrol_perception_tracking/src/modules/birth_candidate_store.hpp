#pragma once

#include <unordered_map>

namespace dog_patrol_perception_tracking {

class BirthCandidateStore {
 public:
  int UpdateObservation(int raw_track_id, int frame_index);
  void Erase(int raw_track_id);
  void Clear();

 private:
  struct Candidate {
    int consecutive_hits{0};
    int last_seen_frame{-1};
  };

  std::unordered_map<int, Candidate> candidates_by_raw_id_;
};

}  // namespace dog_patrol_perception_tracking
