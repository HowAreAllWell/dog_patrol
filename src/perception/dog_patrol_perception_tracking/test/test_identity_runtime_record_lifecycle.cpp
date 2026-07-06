#include <gtest/gtest.h>

#include "identity_runtime_record_lifecycle.hpp"

namespace vision_demo_host {
namespace {

Track MakeTrack(const int raw_id, const cv::Rect2f &bbox, const float confidence = 0.85F) {
  Track track;
  track.id = raw_id;
  track.class_id = ClassId::kPerson;
  track.confidence = confidence;
  track.bbox = bbox;
  return track;
}

TEST(IdentityRuntimeRecordLifecycleTest, AppliesObservationFieldsWithoutTouchingReliableGeometry) {
  IdentityRuntimeRecord record;
  record.semantic_id = 7;
  record.missing_frames = 4;
  record.occlusion_protect_remaining = 3;
  record.feature_geometry.has_reliable_geometry = true;
  record.feature_geometry.reliable_bbox = cv::Rect2f(1.0F, 2.0F, 3.0F, 4.0F);

  IdentityRuntimeRecordLifecycle::ApplyObservation(MakeTrack(42, cv::Rect2f(10.0F, 20.0F, 30.0F, 40.0F), 0.91F),
                                                  25, 0.22F, 0.33F, &record);

  EXPECT_EQ(record.class_id, ClassId::kPerson);
  EXPECT_FLOAT_EQ(record.last_bbox.x, 10.0F);
  EXPECT_FLOAT_EQ(record.last_center.x, 25.0F);
  EXPECT_FLOAT_EQ(record.last_center.y, 40.0F);
  EXPECT_FLOAT_EQ(record.last_assignment_cost, 0.22F);
  EXPECT_FLOAT_EQ(record.last_assignment_margin, 0.33F);
  EXPECT_EQ(record.last_seen_frame, 25);
  EXPECT_EQ(record.missing_frames, 0);
  EXPECT_TRUE(record.seen_this_frame);
  EXPECT_EQ(record.supporting_raw_track_id, 42);
  EXPECT_FLOAT_EQ(record.confidence, 0.91F);
  EXPECT_TRUE(record.feature_geometry.has_reliable_geometry);
  EXPECT_FLOAT_EQ(record.feature_geometry.reliable_bbox.x, 1.0F);
  EXPECT_EQ(record.occlusion_protect_remaining, 3);
}

TEST(IdentityRuntimeRecordLifecycleTest, AgesOnlyUnseenRecordsAndPreservesOcclusionProtectionSemantics) {
  IdentityRuntimeRecord observed;
  observed.seen_this_frame = true;
  observed.missing_frames = 0;
  observed.supporting_raw_track_id = 11;
  observed.confidence = 0.8F;
  observed.occlusion_protect_remaining = 2;

  IdentityRuntimeRecord unseen;
  unseen.seen_this_frame = false;
  unseen.missing_frames = 5;
  unseen.supporting_raw_track_id = 22;
  unseen.confidence = 0.7F;
  unseen.occlusion_protect_remaining = 2;

  IdentityRuntimeRecordLifecycle::AgeOneFrame(&observed);
  IdentityRuntimeRecordLifecycle::AgeOneFrame(&unseen);

  EXPECT_EQ(observed.occlusion_protect_remaining, 1);
  EXPECT_EQ(observed.missing_frames, 0);
  EXPECT_EQ(observed.supporting_raw_track_id, 11);
  EXPECT_FLOAT_EQ(observed.confidence, 0.8F);

  EXPECT_EQ(unseen.occlusion_protect_remaining, 1);
  EXPECT_EQ(unseen.missing_frames, 6);
  EXPECT_EQ(unseen.supporting_raw_track_id, -1);
  EXPECT_FLOAT_EQ(unseen.confidence, 0.0F);
}

TEST(IdentityRuntimeRecordLifecycleTest, ProtectsVisibleUnseenPersonAndMarksCarrierMissingForHandoff) {
  IdentityRuntimeRecord active_person;
  active_person.class_id = ClassId::kPerson;
  active_person.seen_this_frame = false;
  active_person.missing_frames = 0;
  active_person.occlusion_protect_remaining = 2;

  IdentityRuntimeRecord active_car;
  active_car.class_id = ClassId::kCar;
  active_car.seen_this_frame = false;
  active_car.missing_frames = 0;
  active_car.occlusion_protect_remaining = 0;

  IdentityRuntimeRecord already_missing;
  already_missing.class_id = ClassId::kPerson;
  already_missing.seen_this_frame = false;
  already_missing.missing_frames = 1;
  already_missing.occlusion_protect_remaining = 0;

  IdentityRuntimeRecordLifecycle::ProtectIfUnseenActivePerson(30, &active_person);
  IdentityRuntimeRecordLifecycle::ProtectIfUnseenActivePerson(30, &active_car);
  IdentityRuntimeRecordLifecycle::ProtectIfUnseenActivePerson(30, &already_missing);

  EXPECT_EQ(active_person.occlusion_protect_remaining, 30);
  EXPECT_EQ(active_car.occlusion_protect_remaining, 0);
  EXPECT_EQ(already_missing.occlusion_protect_remaining, 0);

  active_person.seen_this_frame = true;
  active_person.supporting_raw_track_id = 55;
  active_person.confidence = 0.75F;
  active_person.missing_frames = 0;

  IdentityRuntimeRecordLifecycle::MarkCarrierMissingForHandoff(&active_person);

  EXPECT_FALSE(active_person.seen_this_frame);
  EXPECT_EQ(active_person.supporting_raw_track_id, -1);
  EXPECT_FLOAT_EQ(active_person.confidence, 0.0F);
  EXPECT_EQ(active_person.missing_frames, 1);
}

TEST(IdentityRuntimeRecordLifecycleTest, BuildsSnapshotWithActiveBBoxAndReliableFallback) {
  IdentityRuntimeRecord reliable_record;
  reliable_record.semantic_id = 8;
  reliable_record.class_id = ClassId::kPerson;
  reliable_record.last_bbox = cv::Rect2f(10.0F, 20.0F, 30.0F, 40.0F);
  reliable_record.feature_geometry.has_reliable_geometry = true;
  reliable_record.feature_geometry.reliable_bbox = cv::Rect2f(11.0F, 21.0F, 31.0F, 41.0F);
  reliable_record.missing_frames = 2;
  reliable_record.occlusion_protect_remaining = 5;
  reliable_record.seen_this_frame = true;
  reliable_record.supporting_raw_track_id = 99;
  reliable_record.confidence = 0.65F;

  const auto reliable_snapshot = IdentityRuntimeRecordLifecycle::BuildSnapshot(8, reliable_record);

  EXPECT_EQ(reliable_snapshot.semantic_id, 8);
  EXPECT_EQ(reliable_snapshot.class_id, ClassId::kPerson);
  EXPECT_FLOAT_EQ(reliable_snapshot.bbox.x, 10.0F);
  EXPECT_FLOAT_EQ(reliable_snapshot.reliable_bbox.x, 11.0F);
  EXPECT_TRUE(reliable_snapshot.has_reliable_geometry);
  EXPECT_EQ(reliable_snapshot.missing_frames, 2);
  EXPECT_EQ(reliable_snapshot.occlusion_protect_remaining, 5);
  EXPECT_TRUE(reliable_snapshot.seen_this_frame);
  EXPECT_EQ(reliable_snapshot.supporting_raw_track_id, 99);
  EXPECT_FLOAT_EQ(reliable_snapshot.confidence, 0.65F);

  IdentityRuntimeRecord fallback_record;
  fallback_record.last_bbox = cv::Rect2f(50.0F, 60.0F, 70.0F, 80.0F);

  const auto fallback_snapshot = IdentityRuntimeRecordLifecycle::BuildSnapshot(9, fallback_record);

  EXPECT_EQ(fallback_snapshot.semantic_id, 9);
  EXPECT_FALSE(fallback_snapshot.has_reliable_geometry);
  EXPECT_FLOAT_EQ(fallback_snapshot.reliable_bbox.x, 50.0F);
}

}  // namespace
}  // namespace vision_demo_host
