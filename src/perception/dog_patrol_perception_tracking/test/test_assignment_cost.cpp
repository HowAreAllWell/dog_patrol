#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include <opencv2/core/types.hpp>

#include "assignment_cost.hpp"
#include "identity_runtime_record.hpp"
#include "vision_demo_host/types.hpp"

namespace {

using vision_demo_host::AssignmentCost;
using vision_demo_host::ClassId;
using vision_demo_host::IdentityRuntimeRecord;
using vision_demo_host::Track;

Track PersonTrack(const cv::Rect2f &bbox) {
  Track track;
  track.id = 10;
  track.class_id = ClassId::kPerson;
  track.bbox = bbox;
  return track;
}

IdentityRuntimeRecord IdentityRecord(const cv::Rect2f &bbox) {
  IdentityRuntimeRecord record;
  record.semantic_id = 1;
  record.class_id = ClassId::kPerson;
  record.last_bbox = bbox;
  record.last_center = cv::Point2f(bbox.x + bbox.width * 0.5F, bbox.y + bbox.height * 0.5F);
  record.feature_geometry.feature_bank.push_back({1.0F, 0.0F});
  return record;
}

}  // namespace

TEST(AssignmentCostTest, ComposesAppearanceGeometryTimeAndWeightedFinalScore) {
  const Track track = PersonTrack(cv::Rect2f(0.0F, 0.0F, 100.0F, 100.0F));
  auto record = IdentityRecord(cv::Rect2f(0.0F, 0.0F, 100.0F, 100.0F));
  record.missing_frames = 18;

  AssignmentCost::Config config;
  config.max_missing_frames = 180;
  config.app_weight = 0.7F;
  config.geometry_weight = 0.2F;
  config.time_weight = 0.1F;

  const auto cost = AssignmentCost::Compute(track, record, {0.0F, 1.0F}, config);

  EXPECT_FLOAT_EQ(cost.appearance, 1.0F);
  EXPECT_FLOAT_EQ(cost.geometry, 0.0F);
  EXPECT_FLOAT_EQ(cost.time, 0.1F);
  EXPECT_FLOAT_EQ(cost.final, 0.71F);
}

TEST(AssignmentCostTest, ReusesNeutralAppearanceAndReliableGeometryPrediction) {
  const Track track = PersonTrack(cv::Rect2f(55.0F, 0.0F, 30.0F, 40.0F));
  auto record = IdentityRecord(cv::Rect2f(1000.0F, 1000.0F, 50.0F, 50.0F));
  record.feature_geometry.feature_bank.clear();
  record.feature_geometry.has_reliable_geometry = true;
  record.feature_geometry.reliable_bbox = cv::Rect2f(0.0F, 0.0F, 30.0F, 40.0F);
  record.feature_geometry.reliable_center = cv::Point2f(15.0F, 20.0F);
  record.feature_geometry.reliable_velocity = cv::Point2f(10.0F, 0.0F);
  record.missing_frames = 4;

  AssignmentCost::Config config;
  config.max_missing_frames = 40;
  config.app_weight = 0.0F;
  config.geometry_weight = 1.0F;
  config.time_weight = 0.0F;

  const auto cost = AssignmentCost::Compute(track, record, {}, config);

  EXPECT_FLOAT_EQ(cost.appearance, 0.5F);
  EXPECT_NEAR(cost.geometry, 0.805F, 1e-5F);
  EXPECT_FLOAT_EQ(cost.time, 0.1F);
  EXPECT_FLOAT_EQ(cost.final, cost.geometry);
}

TEST(AssignmentCostTest, ZeroWeightGuardPreservesLegacyNeutralFinalScore) {
  const Track track = PersonTrack(cv::Rect2f(200.0F, 0.0F, 100.0F, 100.0F));
  auto record = IdentityRecord(cv::Rect2f(0.0F, 0.0F, 100.0F, 100.0F));
  record.missing_frames = 180;

  AssignmentCost::Config config;
  config.max_missing_frames = 180;
  config.app_weight = 0.0F;
  config.geometry_weight = 0.0F;
  config.time_weight = 0.0F;

  const auto cost = AssignmentCost::Compute(track, record, {0.0F, 1.0F}, config);

  EXPECT_FLOAT_EQ(cost.final, 0.0F);
}
