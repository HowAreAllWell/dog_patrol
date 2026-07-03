#include "vision_demo_host/modules/feature_bank_cost.hpp"

#include <algorithm>
#include <cmath>

namespace vision_demo_host {

float FeatureBankCost::CosineDistance(const std::vector<float> &query,
                                      const std::vector<float> &reference) {
  if (query.empty() || reference.empty() || query.size() != reference.size()) {
    return 1.0F;
  }

  float dot = 0.0F;
  float query_norm = 0.0F;
  float reference_norm = 0.0F;
  for (std::size_t i = 0; i < query.size(); ++i) {
    dot += query[i] * reference[i];
    query_norm += query[i] * query[i];
    reference_norm += reference[i] * reference[i];
  }
  if (query_norm <= 1e-8F || reference_norm <= 1e-8F) {
    return 1.0F;
  }

  const float cosine = dot / (std::sqrt(query_norm) * std::sqrt(reference_norm));
  return 1.0F - std::clamp(cosine, 0.0F, 1.0F);
}

float FeatureBankCost::AppearanceCost(const std::vector<float> &query,
                                      const std::deque<std::vector<float>> &feature_bank) {
  if (query.empty() || feature_bank.empty()) {
    return 0.5F;
  }

  float best = 1.0F;
  for (const auto &reference : feature_bank) {
    best = std::min(best, CosineDistance(query, reference));
  }
  return std::clamp(best, 0.0F, 1.0F);
}

}  // namespace vision_demo_host
