#pragma once

#include <deque>
#include <vector>

namespace vision_demo_host {

class FeatureBankCost {
 public:
  static float CosineDistance(const std::vector<float> &query, const std::vector<float> &reference);
  static float AppearanceCost(const std::vector<float> &query,
                              const std::deque<std::vector<float>> &feature_bank);
};

}  // namespace vision_demo_host
