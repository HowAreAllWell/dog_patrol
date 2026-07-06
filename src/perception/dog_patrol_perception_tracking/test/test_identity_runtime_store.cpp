#include <gtest/gtest.h>

#include <algorithm>

#include "identity_runtime_store.hpp"

namespace vision_demo_host {
namespace {

IdentityRuntimeRecord MakePersonRecord(const int semantic_id,
                                       const int missing_frames,
                                       const bool seen_this_frame = false) {
  IdentityRuntimeRecord record;
  record.semantic_id = semantic_id;
  record.class_id = ClassId::kPerson;
  record.last_bbox = cv::Rect2f(static_cast<float>(semantic_id), 10.0F, 20.0F, 30.0F);
  record.feature_geometry.reliable_bbox = cv::Rect2f(static_cast<float>(semantic_id + 100), 10.0F, 20.0F, 30.0F);
  record.feature_geometry.has_reliable_geometry = true;
  record.missing_frames = missing_frames;
  record.seen_this_frame = seen_this_frame;
  record.occlusion_protect_remaining = 2;
  record.supporting_raw_track_id = semantic_id * 10;
  record.confidence = 0.8F;
  return record;
}

TEST(IdentityRuntimeStoreTest, UpsertAndLookupExposeSameRecordContainer) {
  IdentityRuntimeStore store;

  auto &inserted = store.Upsert(7);
  inserted = MakePersonRecord(7, 0, true);
  inserted.last_assignment_cost = 0.25F;

  EXPECT_TRUE(store.Contains(7));
  ASSERT_NE(store.Find(7), nullptr);
  EXPECT_FLOAT_EQ(store.Find(7)->last_assignment_cost, 0.25F);

  auto *mutable_record = store.FindMutable(7);
  ASSERT_NE(mutable_record, nullptr);
  mutable_record->last_assignment_margin = 0.5F;

  auto &same_slot = store.Upsert(7);
  EXPECT_EQ(&same_slot, mutable_record);
  EXPECT_FLOAT_EQ(store.Find(7)->last_assignment_margin, 0.5F);
  EXPECT_EQ(store.Size(), 1U);
}

TEST(IdentityRuntimeStoreTest, EnumeratesOccupiedAndActiveInactivePersonIds) {
  IdentityRuntimeStore store;
  store.Upsert(3) = MakePersonRecord(3, 0);
  store.Upsert(9) = MakePersonRecord(9, 5);
  store.Upsert(5) = MakePersonRecord(5, 8);
  auto &car = store.Upsert(11);
  car.semantic_id = 11;
  car.class_id = ClassId::kCar;
  car.missing_frames = 0;

  const auto occupied = store.OccupiedSemanticIds();
  EXPECT_EQ(occupied.size(), 4U);
  EXPECT_TRUE(occupied.count(3));
  EXPECT_TRUE(occupied.count(5));
  EXPECT_TRUE(occupied.count(9));
  EXPECT_TRUE(occupied.count(11));

  auto active = store.PersonSemanticIds(true, 5);
  auto inactive = store.PersonSemanticIds(false, 5);
  std::sort(active.begin(), active.end());
  std::sort(inactive.begin(), inactive.end());
  EXPECT_EQ(active, (std::vector<int>{3, 9}));
  EXPECT_EQ(inactive, (std::vector<int>{5}));
}

TEST(IdentityRuntimeStoreTest, BeginFrameAgeAndOcclusionProtectionPreserveRuntimeSemantics) {
  IdentityRuntimeStore store;
  auto &visible = store.Upsert(1);
  visible = MakePersonRecord(1, 0, true);
  visible.occlusion_protect_remaining = 0;

  auto &missing = store.Upsert(2);
  missing = MakePersonRecord(2, 3, false);
  missing.occlusion_protect_remaining = 2;

  store.BeginFrame();
  EXPECT_FALSE(store.Find(1)->seen_this_frame);
  EXPECT_FALSE(store.Find(2)->seen_this_frame);

  store.ProtectUnseenActivePeople(30);
  EXPECT_EQ(store.Find(1)->occlusion_protect_remaining, 30);
  EXPECT_EQ(store.Find(2)->occlusion_protect_remaining, 2);

  store.AgeOneFrame();
  EXPECT_EQ(store.Find(1)->missing_frames, 1);
  EXPECT_EQ(store.Find(1)->supporting_raw_track_id, -1);
  EXPECT_EQ(store.Find(2)->missing_frames, 4);
  EXPECT_EQ(store.Find(2)->occlusion_protect_remaining, 1);
}

TEST(IdentityRuntimeStoreTest, MarksCarrierMissingAndBuildsSortedSnapshots) {
  IdentityRuntimeStore store;
  auto &later = store.Upsert(9);
  later = MakePersonRecord(9, 2, true);
  later.last_bbox = cv::Rect2f(90.0F, 0.0F, 10.0F, 10.0F);

  auto &earlier = store.Upsert(4);
  earlier = MakePersonRecord(4, 0, true);
  earlier.last_bbox = cv::Rect2f(40.0F, 0.0F, 10.0F, 10.0F);
  earlier.feature_geometry.has_reliable_geometry = false;

  ASSERT_TRUE(store.MarkCarrierMissingForHandoff(4));
  EXPECT_EQ(store.Find(4)->missing_frames, 1);
  EXPECT_FALSE(store.Find(4)->seen_this_frame);
  EXPECT_EQ(store.Find(4)->supporting_raw_track_id, -1);

  const auto snapshots = store.Snapshots();
  ASSERT_EQ(snapshots.size(), 2U);
  EXPECT_EQ(snapshots[0].semantic_id, 4);
  EXPECT_EQ(snapshots[1].semantic_id, 9);
  EXPECT_FLOAT_EQ(snapshots[0].reliable_bbox.x, 40.0F);
  EXPECT_FALSE(snapshots[0].has_reliable_geometry);
  EXPECT_FLOAT_EQ(snapshots[1].reliable_bbox.x, 109.0F);
  EXPECT_TRUE(snapshots[1].has_reliable_geometry);
}

}  // namespace
}  // namespace vision_demo_host
