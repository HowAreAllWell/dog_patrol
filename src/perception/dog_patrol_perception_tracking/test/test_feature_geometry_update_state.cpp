#include <gtest/gtest.h>

#include "vision_demo_host/modules/feature_geometry_update_state.hpp"

namespace vision_demo_host {
namespace {

FeatureUpdatePolicy::Decision AllowAllUpdates() {
  FeatureUpdatePolicy::Decision decision;
  decision.feature_update_allowed = true;
  decision.geometry_update_allowed = true;
  decision.feature_update_reason = "allowed_update";
  decision.geometry_update_reason = "allowed_update";
  return decision;
}

FeatureGeometryUpdateState::Observation ObservationAt(
    const int frame_index, const cv::Rect2f &bbox,
    const std::vector<float> &feature = {1.0F, 0.0F}) {
  FeatureGeometryUpdateState::Observation observation;
  observation.frame_index = frame_index;
  observation.bbox = bbox;
  observation.feature = feature;
  return observation;
}

TEST(FeatureGeometryUpdateStateTest, FirstReliableUpdateStoresGeometryAndFirstFeature) {
  FeatureGeometryUpdateState::State state;
  FeatureGeometryUpdateState::Config config;
  config.stable_frames_before_feature_update = 3;

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(5, cv::Rect2f(10, 20, 30, 40), {0.2F, 0.8F}),
                                    config, &state);

  EXPECT_TRUE(state.has_reliable_geometry);
  EXPECT_EQ(state.last_reliable_frame, 5);
  EXPECT_FLOAT_EQ(state.reliable_bbox.x, 10.0F);
  EXPECT_FLOAT_EQ(state.reliable_center.x, 25.0F);
  EXPECT_FLOAT_EQ(state.reliable_center.y, 40.0F);
  EXPECT_EQ(state.stable_update_frames, 1);
  ASSERT_EQ(state.feature_bank.size(), 1U);
  EXPECT_FLOAT_EQ(state.feature_bank.front()[0], 0.2F);
}

TEST(FeatureGeometryUpdateStateTest, ReliableVelocityUsesFrameDeltaAndSmoothing) {
  FeatureGeometryUpdateState::State state;
  state.has_reliable_geometry = true;
  state.last_reliable_frame = 4;
  state.reliable_center = cv::Point2f(10.0F, 10.0F);
  state.reliable_velocity = cv::Point2f(2.0F, -1.0F);
  state.feature_bank.push_back({1.0F, 0.0F});
  state.stable_update_frames = 3;
  FeatureGeometryUpdateState::Config config;
  config.stable_frames_before_feature_update = 1;

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(6, cv::Rect2f(16, 8, 4, 4), {0.0F, 1.0F}),
                                    config, &state);

  EXPECT_FLOAT_EQ(state.reliable_center.x, 18.0F);
  EXPECT_FLOAT_EQ(state.reliable_center.y, 10.0F);
  EXPECT_FLOAT_EQ(state.reliable_velocity.x, 2.7F);
  EXPECT_FLOAT_EQ(state.reliable_velocity.y, -0.65F);
  EXPECT_EQ(state.stable_update_frames, 4);
  ASSERT_EQ(state.feature_bank.size(), 2U);
  EXPECT_FLOAT_EQ(state.feature_bank.back()[1], 1.0F);
}

TEST(FeatureGeometryUpdateStateTest, BlockedGeometryResetsStableFramesAndDoesNotAppendFeature) {
  FeatureGeometryUpdateState::State state;
  state.has_reliable_geometry = true;
  state.last_reliable_frame = 9;
  state.reliable_bbox = cv::Rect2f(1, 2, 3, 4);
  state.reliable_center = cv::Point2f(2.5F, 4.0F);
  state.stable_update_frames = 5;
  state.feature_bank.push_back({1.0F, 0.0F});

  auto decision = AllowAllUpdates();
  decision.geometry_update_allowed = false;
  FeatureGeometryUpdateState::Config config;

  FeatureGeometryUpdateState::Apply(decision, ObservationAt(10, cv::Rect2f(100, 100, 20, 20), {0.0F, 1.0F}),
                                    config, &state);

  EXPECT_EQ(state.last_reliable_frame, 9);
  EXPECT_FLOAT_EQ(state.reliable_bbox.x, 1.0F);
  EXPECT_EQ(state.stable_update_frames, 0);
  ASSERT_EQ(state.feature_bank.size(), 1U);
  EXPECT_FLOAT_EQ(state.feature_bank.front()[0], 1.0F);
}

TEST(FeatureGeometryUpdateStateTest, ExistingFeatureBankWaitsForStableGeometryBeforeAppending) {
  FeatureGeometryUpdateState::State state;
  state.has_reliable_geometry = true;
  state.last_reliable_frame = 1;
  state.reliable_center = cv::Point2f(5.0F, 5.0F);
  state.stable_update_frames = 1;
  state.feature_bank.push_back({1.0F, 0.0F});
  FeatureGeometryUpdateState::Config config;
  config.stable_frames_before_feature_update = 3;

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(2, cv::Rect2f(6, 0, 10, 10), {0.0F, 1.0F}),
                                    config, &state);
  EXPECT_EQ(state.stable_update_frames, 2);
  EXPECT_EQ(state.feature_bank.size(), 1U);

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(3, cv::Rect2f(7, 0, 10, 10), {0.0F, 1.0F}),
                                    config, &state);
  EXPECT_EQ(state.stable_update_frames, 3);
  ASSERT_EQ(state.feature_bank.size(), 2U);
  EXPECT_FLOAT_EQ(state.feature_bank.back()[1], 1.0F);
}

TEST(FeatureGeometryUpdateStateTest, TrimsFeatureBankAndIgnoresEmptyFeatures) {
  FeatureGeometryUpdateState::State state;
  state.feature_bank.push_back({1.0F, 0.0F, 0.0F});
  state.feature_bank.push_back({0.0F, 1.0F, 0.0F});
  FeatureGeometryUpdateState::Config config;
  config.feature_bank_max_size = 2;
  config.stable_frames_before_feature_update = 1;

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(1, cv::Rect2f(0, 0, 10, 10), {0.0F, 0.0F, 1.0F}),
                                    config, &state);

  ASSERT_EQ(state.feature_bank.size(), 2U);
  EXPECT_FLOAT_EQ(state.feature_bank.front()[1], 1.0F);
  EXPECT_FLOAT_EQ(state.feature_bank.back()[2], 1.0F);

  FeatureGeometryUpdateState::Apply(AllowAllUpdates(), ObservationAt(2, cv::Rect2f(1, 0, 10, 10), {}), config,
                                    &state);

  ASSERT_EQ(state.feature_bank.size(), 2U);
  EXPECT_FLOAT_EQ(state.feature_bank.front()[1], 1.0F);
  EXPECT_FLOAT_EQ(state.feature_bank.back()[2], 1.0F);
}

TEST(FeatureGeometryUpdateStateTest, ForceGeometryStyleDecisionUpdatesGeometryWithoutFeatureAppend) {
  FeatureGeometryUpdateState::State state;
  state.feature_bank.push_back({1.0F, 0.0F});
  FeatureGeometryUpdateState::Config config;
  config.stable_frames_before_feature_update = 1;
  auto decision = AllowAllUpdates();
  decision.feature_update_allowed = false;
  decision.feature_update_reason = "overlapping_track_freeze";

  FeatureGeometryUpdateState::Apply(decision, ObservationAt(7, cv::Rect2f(20, 10, 10, 10), {0.0F, 1.0F}), config,
                                    &state);

  EXPECT_TRUE(state.has_reliable_geometry);
  EXPECT_EQ(state.last_reliable_frame, 7);
  EXPECT_EQ(state.stable_update_frames, 1);
  ASSERT_EQ(state.feature_bank.size(), 1U);
  EXPECT_FLOAT_EQ(state.feature_bank.front()[0], 1.0F);
}

}  // namespace
}  // namespace vision_demo_host
