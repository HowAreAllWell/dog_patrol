#include <gtest/gtest.h>

#include <opencv2/core/types.hpp>

#include "vision_demo_host/modules/reliable_geometry_cost.hpp"

namespace {

using vision_demo_host::ReliableGeometryCost;

ReliableGeometryCost::State StateWithLatest(const cv::Rect2f &bbox) {
  ReliableGeometryCost::State state;
  state.latest_bbox = bbox;
  state.latest_center = cv::Point2f(bbox.x + bbox.width * 0.5F, bbox.y + bbox.height * 0.5F);
  return state;
}

}  // namespace

TEST(ReliableGeometryCostTest, FallsBackToLatestGeometryWhenReliableGeometryIsMissing) {
  const auto state = StateWithLatest(cv::Rect2f(10.0F, 20.0F, 30.0F, 40.0F));

  EXPECT_FLOAT_EQ(ReliableGeometryCost::ReferenceBBox(state).x, 10.0F);
  EXPECT_FLOAT_EQ(ReliableGeometryCost::PredictedCenter(state).x, 25.0F);
  EXPECT_FLOAT_EQ(ReliableGeometryCost::PredictedCenter(state).y, 40.0F);
}

TEST(ReliableGeometryCostTest, UsesReliableGeometryAndPredictsMissingCenterWithClamp) {
  auto state = StateWithLatest(cv::Rect2f(1000.0F, 1000.0F, 50.0F, 50.0F));
  state.has_reliable_geometry = true;
  state.reliable_bbox = cv::Rect2f(0.0F, 0.0F, 30.0F, 40.0F);
  state.reliable_center = cv::Point2f(15.0F, 20.0F);
  state.reliable_velocity = cv::Point2f(10.0F, 0.0F);
  state.missing_frames = 100;

  EXPECT_FLOAT_EQ(ReliableGeometryCost::ReferenceBBox(state).x, 0.0F);
  EXPECT_FLOAT_EQ(ReliableGeometryCost::PredictedCenter(state).x, 52.5F);
  EXPECT_FLOAT_EQ(ReliableGeometryCost::PredictedCenter(state).y, 20.0F);
}

TEST(ReliableGeometryCostTest, GeometryCostCombinesIouAndPredictedCenterDistance) {
  auto state = StateWithLatest(cv::Rect2f(0.0F, 0.0F, 100.0F, 100.0F));
  EXPECT_FLOAT_EQ(ReliableGeometryCost::GeometryCost(cv::Rect2f(0.0F, 0.0F, 100.0F, 100.0F), state),
                  0.0F);

  const float cost = ReliableGeometryCost::GeometryCost(cv::Rect2f(100.0F, 0.0F, 100.0F, 100.0F), state);
  EXPECT_NEAR(cost, 0.912132F, 1e-5F);
}

TEST(ReliableGeometryCostTest, MissingIdentityGateUsesReliableReferenceAndRelaxedStrongAppearanceArea) {
  auto state = StateWithLatest(cv::Rect2f(400.0F, 0.0F, 100.0F, 300.0F));
  state.has_reliable_geometry = true;
  state.reliable_bbox = cv::Rect2f(0.0F, 0.0F, 100.0F, 300.0F);
  state.reliable_center = cv::Point2f(50.0F, 150.0F);
  state.missing_frames = 3;

  ReliableGeometryCost::MissingGateConfig config;
  config.min_area_ratio = 0.50F;
  config.max_area_ratio = 2.0F;
  config.max_center_dist_norm = 0.30F;
  config.max_app_cost = 0.30F;
  config.active_max_cost = 0.50F;

  EXPECT_FALSE(ReliableGeometryCost::PassesMissingIdentityGate(
      cv::Rect2f(25.0F, 75.0F, 50.0F, 150.0F), state, 0.40F, 0.10F, config));
  EXPECT_TRUE(ReliableGeometryCost::PassesMissingIdentityGate(
      cv::Rect2f(25.0F, 75.0F, 50.0F, 150.0F), state, 0.20F, 0.10F, config));
  EXPECT_FALSE(ReliableGeometryCost::PassesMissingIdentityGate(
      cv::Rect2f(300.0F, 75.0F, 50.0F, 150.0F), state, 0.20F, 0.10F, config));
}

TEST(ReliableGeometryCostTest, ShortMissingAppearanceGateAllowsGeometryFirstRecovery) {
  auto state = StateWithLatest(cv::Rect2f(0.0F, 0.0F, 100.0F, 300.0F));
  state.missing_frames = 10;

  EXPECT_TRUE(ReliableGeometryCost::PassesShortMissingAppearanceGate(state, 0.74F, 0.25F, 0.30F));
  EXPECT_FALSE(ReliableGeometryCost::PassesShortMissingAppearanceGate(state, 0.76F, 0.25F, 0.30F));
  state.missing_frames = 46;
  EXPECT_TRUE(ReliableGeometryCost::PassesShortMissingAppearanceGate(state, 0.29F, 1.0F, 0.30F));
  EXPECT_FALSE(ReliableGeometryCost::PassesShortMissingAppearanceGate(state, 0.31F, 1.0F, 0.30F));
}
