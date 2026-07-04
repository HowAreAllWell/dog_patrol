#include "birth_candidate_store.hpp"

#include <gtest/gtest.h>

using vision_demo_host::BirthCandidateStore;

TEST(BirthCandidateStoreTest, ConsecutiveHitsIncrementForSameRawTrackOnAdjacentFrames) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  EXPECT_EQ(store.UpdateObservation(7, 11), 2);
  EXPECT_EQ(store.UpdateObservation(7, 12), 3);
}

TEST(BirthCandidateStoreTest, GapResetsConsecutiveHitsForSameRawTrack) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  EXPECT_EQ(store.UpdateObservation(7, 11), 2);
  EXPECT_EQ(store.UpdateObservation(7, 13), 1);
}

TEST(BirthCandidateStoreTest, RawIdChangeStartsIndependentCounter) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  EXPECT_EQ(store.UpdateObservation(7, 11), 2);
  EXPECT_EQ(store.UpdateObservation(8, 12), 1);
  EXPECT_EQ(store.UpdateObservation(7, 12), 3);
}

TEST(BirthCandidateStoreTest, ExplicitEraseRemovesOnlyThatRawTrack) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  EXPECT_EQ(store.UpdateObservation(8, 10), 1);
  EXPECT_EQ(store.UpdateObservation(8, 11), 2);

  store.Erase(8);

  EXPECT_EQ(store.UpdateObservation(7, 11), 2);
  EXPECT_EQ(store.UpdateObservation(8, 12), 1);
}

TEST(BirthCandidateStoreTest, ClearResetsAllPendingCandidates) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  EXPECT_EQ(store.UpdateObservation(8, 10), 1);
  store.Clear();

  EXPECT_EQ(store.UpdateObservation(7, 11), 1);
  EXPECT_EQ(store.UpdateObservation(8, 11), 1);
}

TEST(BirthCandidateStoreTest, StoreHasNoSemanticIdAllocationResponsibility) {
  BirthCandidateStore store;

  EXPECT_EQ(store.UpdateObservation(7, 10), 1);
  store.Erase(7);
  EXPECT_EQ(store.UpdateObservation(7, 11), 1);
}
