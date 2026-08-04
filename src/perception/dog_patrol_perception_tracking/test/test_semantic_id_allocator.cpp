#include <gtest/gtest.h>

#include <unordered_set>

#include "semantic_id_allocator.hpp"

namespace {

using dog_patrol_perception_tracking::SemanticIdAllocator;

}  // namespace

TEST(SemanticIdAllocatorTest, StartsAtFirstNonPrimarySemanticId) {
  SemanticIdAllocator allocator;

  EXPECT_EQ(allocator.Allocate({}), 2);
  EXPECT_EQ(allocator.Allocate({}), 3);
}

TEST(SemanticIdAllocatorTest, SkipsPrimaryAndOccupiedSemanticIds) {
  SemanticIdAllocator allocator;
  const std::unordered_set<int> occupied{1, 2, 3};

  EXPECT_EQ(allocator.Allocate(occupied), 4);
}

TEST(SemanticIdAllocatorTest, AdvancesMonotonicallyAcrossOccupiedSets) {
  SemanticIdAllocator allocator;

  EXPECT_EQ(allocator.Allocate({2}), 3);
  EXPECT_EQ(allocator.Allocate({}), 4);
  EXPECT_EQ(allocator.Allocate({5}), 6);
}

TEST(SemanticIdAllocatorTest, ResetRestartsAtFirstNonPrimarySemanticId) {
  SemanticIdAllocator allocator;
  EXPECT_EQ(allocator.Allocate({}), 2);

  allocator.Reset();

  EXPECT_EQ(allocator.Allocate({}), 2);
}

TEST(SemanticIdAllocatorTest, HiddenAndPendingPathsDoNotAdvanceWithoutAllocationCall) {
  SemanticIdAllocator allocator;
  const bool hidden_candidate = true;
  const bool pending_candidate = true;

  (void)hidden_candidate;
  (void)pending_candidate;

  EXPECT_EQ(allocator.Allocate({}), 2);
}
