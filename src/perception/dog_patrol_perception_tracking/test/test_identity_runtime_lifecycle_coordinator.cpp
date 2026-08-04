#include <gtest/gtest.h>

#include "identity_runtime_lifecycle_coordinator.hpp"

namespace dog_patrol_perception_tracking {
namespace {

IdentityRuntimeRecord MakePersonRecord(const int semantic_id,
                                       const bool seen_this_frame,
                                       const int missing_frames,
                                       const int occlusion_protect_remaining) {
  IdentityRuntimeRecord record;
  record.semantic_id = semantic_id;
  record.class_id = ClassId::kPerson;
  record.seen_this_frame = seen_this_frame;
  record.missing_frames = missing_frames;
  record.occlusion_protect_remaining = occlusion_protect_remaining;
  record.supporting_raw_track_id = semantic_id * 10;
  record.confidence = 0.75F;
  return record;
}

TEST(IdentityRuntimeLifecycleCoordinatorTest, AppliesEndFrameLostInactiveAgingToRuntimeStore) {
  IdentityRuntimeStore store;
  store.Upsert(1) = MakePersonRecord(1, true, 0, 2);
  store.Upsert(2) = MakePersonRecord(2, false, 4, 2);

  IdentityRuntimeLifecycleCoordinator::ApplyEndFrameAging(
      IdentityRuntimeLifecycleCoordinator::EndFrameInput{&store});

  const auto *visible = store.Find(1);
  ASSERT_NE(visible, nullptr);
  EXPECT_EQ(visible->missing_frames, 0);
  EXPECT_EQ(visible->supporting_raw_track_id, 10);
  EXPECT_FLOAT_EQ(visible->confidence, 0.75F);
  EXPECT_EQ(visible->occlusion_protect_remaining, 1);

  const auto *missing = store.Find(2);
  ASSERT_NE(missing, nullptr);
  EXPECT_EQ(missing->missing_frames, 5);
  EXPECT_EQ(missing->supporting_raw_track_id, -1);
  EXPECT_FLOAT_EQ(missing->confidence, 0.0F);
  EXPECT_EQ(missing->occlusion_protect_remaining, 1);
}

TEST(IdentityRuntimeLifecycleCoordinatorTest, IgnoresNullRuntimeStore) {
  IdentityRuntimeLifecycleCoordinator::ApplyEndFrameAging(
      IdentityRuntimeLifecycleCoordinator::EndFrameInput{nullptr});
}

}  // namespace
}  // namespace dog_patrol_perception_tracking
