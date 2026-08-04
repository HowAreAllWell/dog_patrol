#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "dog_patrol_perception_tracking/modules/feature_bank_cost.hpp"

namespace {

using dog_patrol_perception_tracking::FeatureBankCost;

}  // namespace

TEST(FeatureBankCostTest, EmptyQueryOrBankUsesNeutralAppearanceCost) {
  const std::deque<std::vector<float>> bank{{1.0F, 0.0F}};

  EXPECT_FLOAT_EQ(FeatureBankCost::AppearanceCost({}, bank), 0.5F);
  EXPECT_FLOAT_EQ(FeatureBankCost::AppearanceCost({1.0F, 0.0F}, {}), 0.5F);
}

TEST(FeatureBankCostTest, CosineDistanceMatchesLegacySemantics) {
  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({1.0F, 0.0F}, {1.0F, 0.0F}), 0.0F);
  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({1.0F, 0.0F}, {0.0F, 1.0F}), 1.0F);
  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({1.0F, 0.0F}, {-1.0F, 0.0F}), 1.0F);
  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({2.0F, 0.0F}, {4.0F, 0.0F}), 0.0F);
}

TEST(FeatureBankCostTest, InvalidReferenceVectorsDoNotBecomeBestMatches) {
  const std::deque<std::vector<float>> bank{{1.0F}, {}, {0.0F, 0.0F}, {0.0F, 1.0F}};

  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({1.0F, 0.0F}, {1.0F}), 1.0F);
  EXPECT_FLOAT_EQ(FeatureBankCost::CosineDistance({1.0F, 0.0F}, {0.0F, 0.0F}), 1.0F);
  EXPECT_FLOAT_EQ(FeatureBankCost::AppearanceCost({1.0F, 0.0F}, bank), 1.0F);
}

TEST(FeatureBankCostTest, AppearanceCostUsesBestBankMatchAndClampsOutput) {
  const std::deque<std::vector<float>> bank{{0.0F, 1.0F}, {1.0F, 0.0F}, {0.7F, 0.7F}};

  EXPECT_FLOAT_EQ(FeatureBankCost::AppearanceCost({1.0F, 0.0F}, bank), 0.0F);
}
