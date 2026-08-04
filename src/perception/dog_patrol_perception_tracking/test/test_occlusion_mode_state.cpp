#include <gtest/gtest.h>

#include "occlusion_mode_state.hpp"

namespace dog_patrol_perception_tracking {
namespace {

OcclusionModeState::Config DefaultConfig() {
  OcclusionModeState::Config config;
  config.merge_hold_frames = 2;
  config.split_stable_frames = 3;
  config.merged_requires_overlap = true;
  return config;
}

TEST(OcclusionModeStateTest, EntersMergedModeFromTwoPersonToOnePersonOverlap) {
  OcclusionModeState::State state;
  state.prev_visible_person_count = 2;
  state.prev_had_overlap = true;

  OcclusionModeState::Input input;
  input.visible_person_count = 1;
  input.has_overlap = false;

  const auto next = OcclusionModeState::Advance(DefaultConfig(), state, input);
  EXPECT_EQ(next.mode, IdentityLifecycleMode::kMerged);
  EXPECT_EQ(next.merged_frames, 1);
  EXPECT_EQ(next.split_stable_count, 0);
  EXPECT_EQ(next.prev_visible_person_count, 1);
  EXPECT_FALSE(next.prev_had_overlap);
  EXPECT_TRUE(next.feature_update_frozen);
}

TEST(OcclusionModeStateTest, HoldsMergedModeForMergeHoldFrames) {
  OcclusionModeState::State state;
  state.prev_visible_person_count = 1;
  state.prev_had_overlap = true;
  state.mode = IdentityLifecycleMode::kMerged;
  state.merged_frames = 1;

  OcclusionModeState::Input input;
  input.visible_person_count = 1;
  input.has_overlap = false;

  const auto next = OcclusionModeState::Advance(DefaultConfig(), state, input);
  EXPECT_EQ(next.mode, IdentityLifecycleMode::kMerged);
  EXPECT_EQ(next.merged_frames, 2);
  EXPECT_TRUE(next.feature_update_frozen);
}

TEST(OcclusionModeStateTest, EntersSplitRecoveryAfterMergeHoldOnTwoVisiblePeople) {
  OcclusionModeState::State state;
  state.mode = IdentityLifecycleMode::kMerged;
  state.merged_frames = 2;

  OcclusionModeState::Input input;
  input.visible_person_count = 2;
  input.has_overlap = false;

  const auto next = OcclusionModeState::Advance(DefaultConfig(), state, input);
  EXPECT_EQ(next.mode, IdentityLifecycleMode::kSplitRecovery);
  EXPECT_EQ(next.split_stable_count, 0);
  EXPECT_TRUE(next.feature_update_frozen);
}

TEST(OcclusionModeStateTest, StableSplitRecoveryReturnsToNormal) {
  OcclusionModeState::State state;
  state.mode = IdentityLifecycleMode::kSplitRecovery;
  state.split_stable_count = 2;

  OcclusionModeState::Input input;
  input.visible_person_count = 2;
  input.has_overlap = false;

  const auto next = OcclusionModeState::Advance(DefaultConfig(), state, input);
  EXPECT_EQ(next.mode, IdentityLifecycleMode::kNormal);
  EXPECT_EQ(next.merged_frames, 0);
  EXPECT_EQ(next.split_stable_count, 0);
  EXPECT_FALSE(next.feature_update_frozen);
}

TEST(OcclusionModeStateTest, OverlapResetsSplitStability) {
  OcclusionModeState::State state;
  state.mode = IdentityLifecycleMode::kSplitRecovery;
  state.split_stable_count = 2;

  OcclusionModeState::Input input;
  input.visible_person_count = 2;
  input.has_overlap = true;

  const auto next = OcclusionModeState::Advance(DefaultConfig(), state, input);
  EXPECT_EQ(next.mode, IdentityLifecycleMode::kSplitRecovery);
  EXPECT_EQ(next.split_stable_count, 0);
  EXPECT_TRUE(next.feature_update_frozen);
}

TEST(OcclusionModeStateTest, FreezeTracksMergedSplitAndOverlapStates) {
  OcclusionModeState::State merged_seed;
  merged_seed.prev_visible_person_count = 2;
  merged_seed.prev_had_overlap = true;
  OcclusionModeState::Input merged_input;
  merged_input.visible_person_count = 1;
  merged_input.has_overlap = false;
  const auto merged = OcclusionModeState::Advance(DefaultConfig(), merged_seed, merged_input);
  EXPECT_TRUE(merged.feature_update_frozen);

  OcclusionModeState::State split_seed;
  split_seed.mode = IdentityLifecycleMode::kMerged;
  split_seed.merged_frames = 2;
  OcclusionModeState::Input split_input;
  split_input.visible_person_count = 2;
  split_input.has_overlap = false;
  const auto split = OcclusionModeState::Advance(DefaultConfig(), split_seed, split_input);
  EXPECT_EQ(split.mode, IdentityLifecycleMode::kSplitRecovery);
  EXPECT_TRUE(split.feature_update_frozen);

  OcclusionModeState::Input overlap_input;
  overlap_input.visible_person_count = 2;
  overlap_input.has_overlap = true;
  const auto overlapped = OcclusionModeState::Advance(DefaultConfig(), OcclusionModeState::State{}, overlap_input);
  EXPECT_EQ(overlapped.mode, IdentityLifecycleMode::kNormal);
  EXPECT_TRUE(overlapped.feature_update_frozen);
}

}  // namespace
}  // namespace dog_patrol_perception_tracking
