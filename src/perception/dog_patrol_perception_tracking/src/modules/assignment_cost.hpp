#pragma once

#include <vector>

#include "identity_runtime_record.hpp"
#include "dog_patrol_perception_tracking/types.hpp"

namespace dog_patrol_perception_tracking {

class AssignmentCost {
 public:
  struct Config {
    int max_missing_frames{180};
    float app_weight{0.70F};
    float geometry_weight{0.20F};
    float time_weight{0.10F};
  };

  struct Result {
    float appearance{0.0F};
    float geometry{0.0F};
    float time{0.0F};
    float final{0.0F};
  };

  static Result Compute(const Track &track, const IdentityRuntimeRecord &identity,
                        const std::vector<float> &feature, const Config &config);
};

}  // namespace dog_patrol_perception_tracking
