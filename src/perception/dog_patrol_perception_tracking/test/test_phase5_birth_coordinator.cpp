#include <gtest/gtest.h>

#include <map>
#include <unordered_map>
#include <vector>

#include "phase5_birth_coordinator.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityManager;
using dog_patrol_perception_tracking::Phase5BirthCoordinator;
using dog_patrol_perception_tracking::TrackletObservation;

TrackletObservation MakeObservation(const int raw_track_id,
                                   const cv::Rect2f &bbox,
                                   const float confidence = 0.9F) {
  TrackletObservation observation;
  observation.raw_track_id = raw_track_id;
  observation.class_id = ClassId::kPerson;
  observation.bbox = bbox;
  observation.confidence = confidence;
  return observation;
}

IdentityManager::ScoreDebugRow MakePhase5Score(const int raw_track_id, const std::string &reject_reason) {
  IdentityManager::ScoreDebugRow row;
  row.raw_track_id = raw_track_id;
  row.semantic_id = -1;
  row.stage = "phase5_birth_candidate";
  row.reject_reason = reject_reason;
  row.selected = false;
  row.accepted = false;
  return row;
}

}  // namespace

TEST(Phase5BirthCoordinatorTest, ApplyAcceptedBirthsKeepsSmallPendingThenAllocatesAfterStableFrame) {
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {8, MakeObservation(8, cv::Rect2f(10, 20, 50, 170))}};
  std::vector<IdentityManager::ScoreDebugRow> score_rows{MakePhase5Score(8, "phase5_birth_manager_pending")};
  std::map<int, Phase5BirthCoordinator::ShadowState> shadow_by_raw_track_id;
  std::unordered_map<int, int> raw_to_semantic_id;

  int allocation_calls = 0;
  int refresh_calls = 0;
  Phase5BirthCoordinator::ApplyInput first_input;
  first_input.enabled = true;
  first_input.current_frame_idx = 0;
  first_input.observations_by_raw_track_id = &observations_by_raw_track_id;
  first_input.score_rows = &score_rows;
  first_input.shadow_by_raw_track_id = &shadow_by_raw_track_id;
  first_input.raw_to_semantic_id = &raw_to_semantic_id;
  Phase5BirthCoordinator::ApplyAcceptedBirths(
      first_input,
      [&](const int raw_track_id) {
        ++allocation_calls;
        EXPECT_EQ(raw_track_id, 8);
      },
      [&]() { ++refresh_calls; });

  ASSERT_EQ(score_rows.size(), 1U);
  EXPECT_EQ(score_rows[0].reject_reason, "small_new_person_pending");
  EXPECT_TRUE(score_rows[0].selected);
  EXPECT_FALSE(score_rows[0].accepted);
  ASSERT_EQ(shadow_by_raw_track_id.size(), 1U);
  EXPECT_EQ(shadow_by_raw_track_id.at(8).stable_frames, 1);
  EXPECT_TRUE(shadow_by_raw_track_id.at(8).requires_stability);
  EXPECT_EQ(allocation_calls, 0);
  EXPECT_EQ(refresh_calls, 0);
  EXPECT_TRUE(raw_to_semantic_id.empty());

  score_rows[0] = MakePhase5Score(8, "phase5_birth_manager_pending");
  Phase5BirthCoordinator::ApplyInput second_input = first_input;
  second_input.current_frame_idx = 1;
  Phase5BirthCoordinator::ApplyAcceptedBirths(
      second_input,
      [&](const int raw_track_id) {
        ++allocation_calls;
        EXPECT_EQ(raw_track_id, 8);
      },
      [&]() {
        ++refresh_calls;
        IdentityManager::ScoreDebugRow allocated;
        allocated.raw_track_id = 8;
        allocated.semantic_id = 3;
        allocated.stage = "phase5_new_semantic";
        allocated.selected = true;
        allocated.accepted = true;
        score_rows.assign({allocated});
      });

  EXPECT_EQ(allocation_calls, 1);
  EXPECT_EQ(refresh_calls, 1);
  ASSERT_EQ(score_rows.size(), 1U);
  EXPECT_EQ(score_rows[0].stage, "phase5_new_semantic");
  EXPECT_EQ(score_rows[0].semantic_id, 3);
  ASSERT_EQ(raw_to_semantic_id.size(), 1U);
  EXPECT_EQ(raw_to_semantic_id.at(8), 3);
}

TEST(Phase5BirthCoordinatorTest, ApplyAcceptedBirthsClearsPendingShadowForHiddenRows) {
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {9, MakeObservation(9, cv::Rect2f(10, 20, 142, 762))}};
  std::vector<IdentityManager::ScoreDebugRow> score_rows{MakePhase5Score(9, "skinny_partial_hidden")};
  std::map<int, Phase5BirthCoordinator::ShadowState> shadow_by_raw_track_id{
      {9, Phase5BirthCoordinator::ShadowState{cv::Rect2f(10, 20, 142, 762), 0.8F, 2, 4, true}}};
  std::unordered_map<int, int> raw_to_semantic_id;

  Phase5BirthCoordinator::ApplyInput input;
  input.enabled = true;
  input.current_frame_idx = 5;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.score_rows = &score_rows;
  input.shadow_by_raw_track_id = &shadow_by_raw_track_id;
  input.raw_to_semantic_id = &raw_to_semantic_id;

  int allocation_calls = 0;
  int refresh_calls = 0;
  Phase5BirthCoordinator::ApplyAcceptedBirths(
      input,
      [&](int) { ++allocation_calls; },
      [&]() { ++refresh_calls; });

  EXPECT_TRUE(shadow_by_raw_track_id.empty());
  EXPECT_EQ(allocation_calls, 0);
  EXPECT_EQ(refresh_calls, 0);
  EXPECT_TRUE(raw_to_semantic_id.empty());
}

TEST(Phase5BirthCoordinatorTest, AppendShadowLifecycleRowsEmitsHiddenPendingAndAllocatedRows) {
  const std::vector<TrackletObservation> observations{
      MakeObservation(8, cv::Rect2f(10, 20, 50, 170)),
      MakeObservation(14, cv::Rect2f(20, 30, 178, 1270)),
  };
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {8, observations[0]},
      {14, observations[1]},
  };
  std::unordered_map<int, int> raw_to_semantic_id{{14, 4}};
  std::vector<IdentityManager::ScoreDebugRow> score_rows;
  auto pending = MakePhase5Score(8, "small_new_person_pending");
  pending.selected = true;
  score_rows.push_back(pending);
  auto hidden = MakePhase5Score(9, "duplicate_split_hidden");
  hidden.selected = true;
  score_rows.push_back(hidden);
  IdentityManager::ScoreDebugRow allocated;
  allocated.raw_track_id = 14;
  allocated.semantic_id = 4;
  allocated.stage = "phase5_new_semantic";
  allocated.selected = true;
  allocated.accepted = true;
  score_rows.push_back(allocated);

  std::map<int, Phase5BirthCoordinator::ShadowState> shadow_by_raw_track_id{
      {8, Phase5BirthCoordinator::ShadowState{cv::Rect2f(10, 20, 50, 170), 0.9F, 2, 1, true}},
      {9, Phase5BirthCoordinator::ShadowState{cv::Rect2f(11, 21, 51, 171), 0.8F, 1, 0, true}},
      {14, Phase5BirthCoordinator::ShadowState{cv::Rect2f(20, 30, 178, 1270), 0.95F, 0, 0, false}},
  };
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  int event_idx = 0;

  Phase5BirthCoordinator::ShadowRowsInput input;
  input.current_frame_idx = 1;
  input.observations = &observations;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.score_rows = &score_rows;
  input.shadow_by_raw_track_id = &shadow_by_raw_track_id;
  input.phase3_rows = &phase3_rows;
  input.next_event_idx = &event_idx;
  Phase5BirthCoordinator::AppendShadowLifecycleRows(input);

  ASSERT_EQ(phase3_rows.size(), 3U);
  EXPECT_EQ(phase3_rows[0].event_type, "new_birth_candidate_pending");
  EXPECT_EQ(phase3_rows[0].candidate_raw_track_id, 8);
  EXPECT_EQ(phase3_rows[0].reason, "small_new_person_pending");
  EXPECT_EQ(phase3_rows[0].candidate_stable_frames, 2);
  EXPECT_EQ(phase3_rows[0].hypothesis_status, "pending_stability");

  EXPECT_EQ(phase3_rows[1].event_type, "new_birth_candidate_hidden");
  EXPECT_EQ(phase3_rows[1].candidate_raw_track_id, 9);
  EXPECT_EQ(phase3_rows[1].reason, "duplicate_split_hidden");
  EXPECT_EQ(phase3_rows[1].hypothesis_status, "hidden_duplicate_split");

  EXPECT_EQ(phase3_rows[2].event_type, "new_birth_candidate_allocated");
  EXPECT_EQ(phase3_rows[2].candidate_raw_track_id, 14);
  EXPECT_EQ(phase3_rows[2].candidate_semantic_id, 4);
  EXPECT_EQ(phase3_rows[2].reason, "phase5_birth_manager_allocated");
  EXPECT_EQ(phase3_rows[2].hypothesis_status, "allocated");

  EXPECT_EQ(shadow_by_raw_track_id.size(), 1U);
  EXPECT_TRUE(shadow_by_raw_track_id.find(8) != shadow_by_raw_track_id.end());
}

TEST(Phase5BirthCoordinatorTest, AppendShadowLifecycleRowsSeedsAndCleansUnscoredCandidates) {
  const std::vector<TrackletObservation> observations{MakeObservation(21, cv::Rect2f(100, 120, 55, 160))};
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{{21, observations[0]}};
  std::unordered_map<int, int> raw_to_semantic_id;
  std::vector<IdentityManager::ScoreDebugRow> score_rows;
  std::map<int, Phase5BirthCoordinator::ShadowState> shadow_by_raw_track_id{
      {13, Phase5BirthCoordinator::ShadowState{cv::Rect2f(0, 0, 40, 140), 0.6F, 1, 0, true}}};
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  int event_idx = 0;

  Phase5BirthCoordinator::ShadowRowsInput input;
  input.current_frame_idx = 1;
  input.observations = &observations;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.score_rows = &score_rows;
  input.shadow_by_raw_track_id = &shadow_by_raw_track_id;
  input.phase3_rows = &phase3_rows;
  input.next_event_idx = &event_idx;
  Phase5BirthCoordinator::AppendShadowLifecycleRows(input);

  ASSERT_EQ(phase3_rows.size(), 1U);
  EXPECT_EQ(phase3_rows[0].event_type, "new_birth_candidate_pending");
  EXPECT_EQ(phase3_rows[0].candidate_raw_track_id, 21);
  EXPECT_EQ(phase3_rows[0].candidate_stable_frames, 1);
  ASSERT_EQ(shadow_by_raw_track_id.size(), 1U);
  EXPECT_TRUE(shadow_by_raw_track_id.find(21) != shadow_by_raw_track_id.end());
  EXPECT_EQ(shadow_by_raw_track_id.at(21).stable_frames, 1);
}
