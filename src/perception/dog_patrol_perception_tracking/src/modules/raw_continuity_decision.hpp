#pragma once

#include <string>

namespace vision_demo_host {

class RawContinuityDecision {
 public:
  struct Config {
    float raw_continuity_max_cost{0.55F};
  };

  struct Input {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float app_cost{0.0F};
    float geo_cost{0.0F};
    float time_cost{0.0F};
    float final_cost{1.0F};
    bool identity_found{true};
    bool passes_missing_identity_gate{true};
    bool weak_mot_association{false};
  };

  struct Decision {
    int track_idx{-1};
    int raw_track_id{-1};
    int semantic_id{-1};
    float app_cost{0.0F};
    float geo_cost{0.0F};
    float time_cost{0.0F};
    float final_score{1.0F};
    float margin{0.0F};
    bool selected{false};
    bool accepted{false};
    std::string reject_reason;
    bool continuity_used{true};
  };

  static Decision Evaluate(const Input &input, const Config &config);
};

}  // namespace vision_demo_host
