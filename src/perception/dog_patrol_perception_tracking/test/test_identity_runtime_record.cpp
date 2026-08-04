#include <gtest/gtest.h>

#include "identity_runtime_record.hpp"

TEST(IdentityRuntimeRecordTest, CarriesRuntimeIdentityStateShape) {
  dog_patrol_perception_tracking::IdentityRuntimeRecord record;

  record.semantic_id = 7;
  record.class_id = dog_patrol_perception_tracking::ClassId::kPerson;
  record.last_bbox = cv::Rect2f(10.0F, 20.0F, 30.0F, 40.0F);
  record.last_center = cv::Point2f(25.0F, 40.0F);
  record.feature_geometry.feature_bank.push_back({0.25F, 0.75F});
  record.feature_geometry.reliable_bbox = cv::Rect2f(11.0F, 21.0F, 31.0F, 41.0F);
  record.feature_geometry.has_reliable_geometry = true;
  record.last_assignment_cost = 0.2F;
  record.last_assignment_margin = 0.3F;
  record.last_seen_frame = 42;
  record.missing_frames = 3;
  record.seen_this_frame = true;
  record.occlusion_protect_remaining = 5;
  record.supporting_raw_track_id = 99;
  record.confidence = 0.9F;

  EXPECT_EQ(record.semantic_id, 7);
  EXPECT_EQ(record.class_id, dog_patrol_perception_tracking::ClassId::kPerson);
  EXPECT_FLOAT_EQ(record.last_bbox.x, 10.0F);
  EXPECT_FLOAT_EQ(record.last_center.y, 40.0F);
  EXPECT_EQ(record.feature_geometry.feature_bank.size(), 1U);
  EXPECT_TRUE(record.feature_geometry.has_reliable_geometry);
  EXPECT_FLOAT_EQ(record.last_assignment_cost, 0.2F);
  EXPECT_FLOAT_EQ(record.last_assignment_margin, 0.3F);
  EXPECT_EQ(record.last_seen_frame, 42);
  EXPECT_EQ(record.missing_frames, 3);
  EXPECT_TRUE(record.seen_this_frame);
  EXPECT_EQ(record.occlusion_protect_remaining, 5);
  EXPECT_EQ(record.supporting_raw_track_id, 99);
  EXPECT_FLOAT_EQ(record.confidence, 0.9F);
}
