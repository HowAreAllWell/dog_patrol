#pragma once

#include <string>

namespace vision_demo_host {

class BirthCandidateDecision {
 public:
  enum class Action {
    kHideWithDebugRow,
    kPhase5Pending,
  };

  struct Config {
    int small_person_stable_frames_required{2};
  };

  struct Input {
    int track_idx{-1};
    int raw_track_id{-1};
    bool hold_for_ambiguous_recovery{false};
    bool duplicate_split{false};
    std::string hide_reason;
    bool small_person_requires_stability{false};
    int stable_observation_count{0};
  };

  struct Decision {
    Action action{Action::kPhase5Pending};
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float final_score{0.0F};
    float margin{0.0F};
    bool selected{false};
    bool accepted{false};
    std::string stage;
    std::string reject_reason;
    bool clear_pending_candidate{false};
  };

  static Decision Evaluate(const Input &input, const Config &config);
};

}  // namespace vision_demo_host
