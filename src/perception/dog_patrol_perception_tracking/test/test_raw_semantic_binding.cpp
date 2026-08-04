#include "raw_semantic_binding_store.hpp"

#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "assignment_application_plan.hpp"

using dog_patrol_perception_tracking::AssignmentApplicationPlan;
using dog_patrol_perception_tracking::RawSemanticBindingStore;

namespace {

AssignmentApplicationPlan::RawMapping RawMapping(const int raw_track_id, const int semantic_id) {
  AssignmentApplicationPlan::RawMapping mapping;
  mapping.raw_track_id = raw_track_id;
  mapping.semantic_id = semantic_id;
  return mapping;
}

}  // namespace

TEST(RawSemanticBindingStoreTest, PreviousSnapshotKeepsBindingsBeforeClear) {
  RawSemanticBindingStore store;
  store.Bind(10, 1);
  store.Bind(20, 2);

  const auto snapshot = store.PreviousSnapshot();
  store.Clear();

  const std::unordered_map<int, int> expected{{10, 1}, {20, 2}};
  EXPECT_EQ(snapshot, expected);
  EXPECT_TRUE(store.Current().empty());
}

TEST(RawSemanticBindingStoreTest, LookupReturnsMissWhenRawTrackIsUnknown) {
  RawSemanticBindingStore store;
  store.Bind(10, 1);

  EXPECT_EQ(store.SemanticIdForRawTrack(99), -1);
}

TEST(RawSemanticBindingStoreTest, ReplaceFromPlannedEntriesRebuildsCurrentBindings) {
  RawSemanticBindingStore store;
  store.Bind(10, 1);
  store.Bind(20, 2);

  store.ReplaceFromPlannedEntries({
      RawMapping(30, 3),
      RawMapping(40, 4),
  });

  const std::unordered_map<int, int> expected{{30, 3}, {40, 4}};
  EXPECT_EQ(store.Current(), expected);
  EXPECT_EQ(store.SemanticIdForRawTrack(10), -1);
}

TEST(RawSemanticBindingStoreTest, DirectBindingOverwriteKeepsLatestSemanticId) {
  RawSemanticBindingStore store;

  store.Bind(10, 1);
  store.Bind(10, 7);

  EXPECT_EQ(store.SemanticIdForRawTrack(10), 7);
}

TEST(RawSemanticBindingStoreTest, EraseRemovesOnlyRequestedBinding) {
  RawSemanticBindingStore store;
  store.Bind(10, 1);
  store.Bind(20, 2);

  store.Erase(10);

  EXPECT_EQ(store.SemanticIdForRawTrack(10), -1);
  EXPECT_EQ(store.SemanticIdForRawTrack(20), 2);
}

TEST(RawSemanticBindingStoreTest, ResetClearsAllBindings) {
  RawSemanticBindingStore store;
  store.Bind(10, 1);
  store.Bind(20, 2);

  store.Reset();

  EXPECT_TRUE(store.Current().empty());
  EXPECT_EQ(store.SemanticIdForRawTrack(10), -1);
  EXPECT_EQ(store.SemanticIdForRawTrack(20), -1);
}

TEST(RawSemanticBindingStoreTest, PlannedReplacementKeepsDeterministicRawMappingContent) {
  RawSemanticBindingStore store;

  const std::vector<AssignmentApplicationPlan::RawMapping> planned{
      RawMapping(103, 7),
      RawMapping(102, 4),
      RawMapping(101, 5),
  };
  store.ReplaceFromPlannedEntries(planned);

  ASSERT_EQ(store.PlannedEntries().size(), planned.size());
  for (std::size_t i = 0; i < planned.size(); ++i) {
    EXPECT_EQ(store.PlannedEntries()[i].raw_track_id, planned[i].raw_track_id);
    EXPECT_EQ(store.PlannedEntries()[i].semantic_id, planned[i].semantic_id);
  }
  const std::unordered_map<int, int> expected{{103, 7}, {102, 4}, {101, 5}};
  EXPECT_EQ(store.Current(), expected);
}
