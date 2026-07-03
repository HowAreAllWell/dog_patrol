#pragma once

#include <string>

namespace vision_demo_host {

class FeatureUpdatePolicy {
 public:
  struct Input {
    bool accepted{false};
    bool global_freeze{false};
    bool overlap_freeze{false};
    bool reliable_observation{false};
    bool force_geometry_update{false};
    bool has_existing_feature_bank{false};
    int stable_update_frames{0};
    int stable_frames_before_feature_update{1};
  };

  struct Decision {
    bool feature_update_allowed{false};
    bool geometry_update_allowed{false};
    std::string feature_update_reason;
    std::string geometry_update_reason;
  };

  static Decision Decide(const Input &input);
};

}  // namespace vision_demo_host
