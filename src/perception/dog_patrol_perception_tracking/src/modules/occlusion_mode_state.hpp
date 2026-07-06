#pragma once

#include "identity_lifecycle_mode.hpp"

namespace vision_demo_host {

class OcclusionModeState {
 public:
  struct Config {
    int split_stable_frames{3};
    int merge_hold_frames{2};
    bool merged_requires_overlap{true};
  };

  struct State {
    int prev_visible_person_count{0};
    bool prev_had_overlap{false};
    int merged_frames{0};
    int split_stable_count{0};
    IdentityLifecycleMode mode{IdentityLifecycleMode::kNormal};
    bool feature_update_frozen{false};
  };

  struct Input {
    int visible_person_count{0};
    bool has_overlap{false};
  };

  static State Advance(const Config &config, const State &previous, const Input &input);
};

}  // namespace vision_demo_host
