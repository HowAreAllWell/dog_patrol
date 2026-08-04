#pragma once

#include <deque>
#include <vector>

namespace dog_patrol_perception_tracking {

class FeatureBankCost {
 public:
  static float CosineDistance(const std::vector<float> &query, const std::vector<float> &reference);
  static float AppearanceCost(const std::vector<float> &query,
                              const std::deque<std::vector<float>> &feature_bank);
};

}  // namespace dog_patrol_perception_tracking
