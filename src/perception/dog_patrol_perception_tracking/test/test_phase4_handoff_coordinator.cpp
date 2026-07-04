#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "phase4_handoff_coordinator.hpp"

namespace {

using vision_demo_host::ClassId;
using vision_demo_host::IdentityManager;
using vision_demo_host::OcclusionGroupShadowLifecycle;
using vision_demo_host::Phase4HandoffCoordinator;
using vision_demo_host::TrackletHypothesis;
using vision_demo_host::TrackletHypothesisStatus;
using vision_demo_host::TrackletObservation;

TrackletObservation MakeObservation(const int raw_track_id, const cv::Rect2f &bbox, const float confidence = 0.9F) {
  TrackletObservation observation;
  observation.raw_track_id = raw_track_id;
  observation.class_id = ClassId::kPerson;
  observation.bbox = bbox;
  observation.confidence = confidence;
  return observation;
}

TrackletHypothesis MakeHypothesis(const int raw_track_id,
                                  const cv::Rect2f &bbox,
                                  const float confidence = 0.9F,
                                  const TrackletHypothesisStatus status =
                                      TrackletHypothesisStatus::kTracked) {
  TrackletHypothesis hypothesis;
  hypothesis.raw_track_id = raw_track_id;
  hypothesis.class_id = ClassId::kPerson;
  hypothesis.bbox = bbox;
  hypothesis.confidence = confidence;
  hypothesis.status = status;
  hypothesis.candidate_reason = "final_track_output";
  return hypothesis;
}

IdentityManager::Phase3ShadowDebugRow MakePairwiseMatrixRow() {
  IdentityManager::Phase3ShadowDebugRow row;
  row.event_type = "pairwise_assignment_matrix";
  row.reason = "pairwise_appearance_override";
  row.pairwise_selected_pairs = "30->1|40->2";
  row.pairwise_alternate_pairs = "30->2|40->1";
  row.pairwise_appearance_override = true;
  return row;
}

IdentityManager::Phase3ShadowDebugRow MakeSingleBlobDecisionRow(const char *reason,
                                                                const int candidate_semantic_id,
                                                                const float app_cost,
                                                                const float geo_cost,
                                                                const float final_score) {
  IdentityManager::Phase3ShadowDebugRow row;
  row.event_type = "single_blob_handoff_decision";
  row.reason = reason;
  row.carrier_raw_track_id = 2;
  row.carrier_semantic_id = 2;
  row.candidate_raw_track_id = 2;
  row.candidate_semantic_id = candidate_semantic_id;
  row.decision_app_cost = app_cost;
  row.decision_geo_cost = geo_cost;
  row.decision_final_score = final_score;
  return row;
}

const IdentityManager::Phase3ShadowDebugRow *FindEvent(
    const std::vector<IdentityManager::Phase3ShadowDebugRow> &rows,
    const std::string &event_type,
    const int candidate_raw_track_id) {
  for (const auto &row : rows) {
    if (row.event_type == event_type && row.candidate_raw_track_id == candidate_raw_track_id) {
      return &row;
    }
  }
  return nullptr;
}

}  // namespace

TEST(Phase4HandoffCoordinatorTest, PairwiseAssignmentAppendsMigratedRowsOnAcceptedApply) {
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows{MakePairwiseMatrixRow()};
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 1;

  Phase4HandoffCoordinator::PairwiseInput input;
  input.enabled = true;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  int refresh_calls = 0;
  EXPECT_TRUE(Phase4HandoffCoordinator::ApplyPairwiseAssignment(
      input,
      [&](const int first_raw_track_id, const int first_semantic_id,
          const int second_raw_track_id, const int second_semantic_id) {
        ++apply_calls;
        EXPECT_EQ(first_raw_track_id, 30);
        EXPECT_EQ(first_semantic_id, 2);
        EXPECT_EQ(second_raw_track_id, 40);
        EXPECT_EQ(second_semantic_id, 1);
        return true;
      },
      [&]() { ++refresh_calls; }));

  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(refresh_calls, 1);
  ASSERT_EQ(raw_to_semantic_id.size(), 2U);
  EXPECT_EQ(raw_to_semantic_id.at(30), 2);
  EXPECT_EQ(raw_to_semantic_id.at(40), 1);
  ASSERT_EQ(phase3_rows.size(), 3U);
  EXPECT_EQ(phase3_rows[1].event_type, "phase4_pairwise_assignment");
  EXPECT_EQ(phase3_rows[1].candidate_raw_track_id, 30);
  EXPECT_EQ(phase3_rows[1].candidate_semantic_id, 2);
  EXPECT_EQ(phase3_rows[1].related_raw_track_id, 40);
  EXPECT_EQ(phase3_rows[2].candidate_raw_track_id, 40);
  EXPECT_EQ(phase3_rows[2].candidate_semantic_id, 1);
}

TEST(Phase4HandoffCoordinatorTest, PairwiseAssignmentSkipsNoOpPaths) {
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  phase3_rows.push_back(MakePairwiseMatrixRow());
  phase3_rows.back().pairwise_appearance_override = false;
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 1;

  Phase4HandoffCoordinator::PairwiseInput input;
  input.enabled = true;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  EXPECT_FALSE(Phase4HandoffCoordinator::ApplyPairwiseAssignment(
      input,
      [&](int, int, int, int) {
        ++apply_calls;
        return true;
      },
      [&]() { FAIL() << "refresh should not run"; }));
  EXPECT_EQ(apply_calls, 0);
  EXPECT_EQ(phase3_rows.size(), 1U);
  EXPECT_TRUE(raw_to_semantic_id.empty());
}

TEST(Phase4HandoffCoordinatorTest, SingleBlobHandoffMarksAcceptedDecisionAndAppendsMigratedRow) {
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows{
      MakeSingleBlobDecisionRow("single_blob_continuity_kept", 2, 0.30F, 0.30F, 0.40F),
      MakeSingleBlobDecisionRow("single_blob_handoff_eligible", 1, 0.10F, 0.50F, 0.35F),
  };
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 2;

  Phase4HandoffCoordinator::SingleBlobInput input;
  input.enabled = true;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  int refresh_calls = 0;
  EXPECT_TRUE(Phase4HandoffCoordinator::ApplyMergedSingleBlobHandoff(
      input,
      [&](const int carrier_raw_track_id, const int carrier_semantic_id, const int candidate_semantic_id) {
        ++apply_calls;
        EXPECT_EQ(carrier_raw_track_id, 2);
        EXPECT_EQ(carrier_semantic_id, 2);
        EXPECT_EQ(candidate_semantic_id, 1);
        return true;
      },
      [&]() { ++refresh_calls; }));

  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(refresh_calls, 1);
  ASSERT_EQ(raw_to_semantic_id.size(), 1U);
  EXPECT_EQ(raw_to_semantic_id.at(2), 1);
  ASSERT_EQ(phase3_rows.size(), 3U);
  EXPECT_EQ(phase3_rows[1].reason, "single_blob_handoff_accepted");
  EXPECT_TRUE(phase3_rows[1].decision_selected);
  EXPECT_TRUE(phase3_rows[1].decision_accepted);
  EXPECT_EQ(phase3_rows[2].event_type, "phase4_merged_single_blob_handoff");
  EXPECT_EQ(phase3_rows[2].reason, "merged_single_blob_handoff");
}

TEST(Phase4HandoffCoordinatorTest, SingleBlobHandoffRejectsEligibleRowWhenMarginGateFails) {
  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows{
      MakeSingleBlobDecisionRow("single_blob_continuity_kept", 2, 0.10F, 0.30F, 0.20F),
      MakeSingleBlobDecisionRow("single_blob_handoff_eligible", 1, 0.10F, 0.90F, 0.40F),
  };
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 2;

  Phase4HandoffCoordinator::SingleBlobInput input;
  input.enabled = true;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  EXPECT_FALSE(Phase4HandoffCoordinator::ApplyMergedSingleBlobHandoff(
      input,
      [&](int, int, int) {
        ++apply_calls;
        return true;
      },
      [&]() { FAIL() << "refresh should not run"; }));
  EXPECT_EQ(apply_calls, 0);
  EXPECT_TRUE(raw_to_semantic_id.empty());
  ASSERT_EQ(phase3_rows.size(), 2U);
  EXPECT_EQ(FindEvent(phase3_rows, "phase4_merged_single_blob_handoff", 2), nullptr);
}

TEST(Phase4HandoffCoordinatorTest, SideRecoveryAppendsMigratedRowOnAcceptedApply) {
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {4, MakeObservation(4, cv::Rect2f(310.0F, 0.0F, 298.0F, 928.0F))},
  };
  std::vector<TrackletHypothesis> shadow_hypotheses{
      MakeHypothesis(6, cv::Rect2f(179.0F, 228.0F, 194.0F, 514.0F)),
  };
  std::vector<IdentityManager::ScoreDebugRow> score_rows(1);
  score_rows[0].raw_track_id = 4;
  score_rows[0].semantic_id = 1;
  score_rows[0].stage = "raw_continuity";
  score_rows[0].accepted = true;

  OcclusionGroupShadowLifecycle::State occlusion_state;
  occlusion_state.merged_group.active = true;
  occlusion_state.merged_group.group_id = 1;
  occlusion_state.merged_group.semantic_ids = {1, 2};
  occlusion_state.merged_group.carrier_raw_track_id = 4;
  occlusion_state.merged_group.carrier_semantic_id = 1;
  occlusion_state.merged_group.age_frames = 3;
  occlusion_state.merged_group.last_update_frame = 30;

  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 0;

  Phase4HandoffCoordinator::SideRecoveryInput input;
  input.enabled = true;
  input.current_frame_idx = 31;
  input.shadow_hypotheses = &shadow_hypotheses;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.score_rows = &score_rows;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.occlusion_state = &occlusion_state;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  int refresh_calls = 0;
  EXPECT_TRUE(Phase4HandoffCoordinator::ApplyMergedSideRecovery(
      input,
      [&](const int carrier_raw_track_id, const int carrier_semantic_id,
          const int candidate_raw_track_id, const int candidate_semantic_id) {
        ++apply_calls;
        EXPECT_EQ(carrier_raw_track_id, 4);
        EXPECT_EQ(carrier_semantic_id, 1);
        EXPECT_EQ(candidate_raw_track_id, 6);
        EXPECT_EQ(candidate_semantic_id, 2);
        return true;
      },
      [&]() { ++refresh_calls; }));

  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(refresh_calls, 1);
  ASSERT_EQ(raw_to_semantic_id.size(), 1U);
  EXPECT_EQ(raw_to_semantic_id.at(6), 2);
  ASSERT_EQ(phase3_rows.size(), 1U);
  EXPECT_EQ(phase3_rows[0].event_type, "phase4_merged_side_recovery");
  EXPECT_EQ(phase3_rows[0].reason, "merged_side_recovery");
  EXPECT_EQ(phase3_rows[0].candidate_raw_track_id, 6);
  EXPECT_EQ(phase3_rows[0].related_raw_track_id, 4);
}

TEST(Phase4HandoffCoordinatorTest, SideRecoverySkipsCandidateWithoutMergedSideShape) {
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {4, MakeObservation(4, cv::Rect2f(310.0F, 0.0F, 298.0F, 928.0F))},
  };
  std::vector<TrackletHypothesis> shadow_hypotheses{
      MakeHypothesis(6, cv::Rect2f(900.0F, 50.0F, 80.0F, 120.0F)),
  };
  std::vector<IdentityManager::ScoreDebugRow> score_rows(1);
  score_rows[0].raw_track_id = 4;
  score_rows[0].semantic_id = 1;
  score_rows[0].stage = "raw_continuity";
  score_rows[0].accepted = true;

  OcclusionGroupShadowLifecycle::State occlusion_state;
  occlusion_state.merged_group.active = true;
  occlusion_state.merged_group.group_id = 1;
  occlusion_state.merged_group.semantic_ids = {1, 2};
  occlusion_state.merged_group.carrier_raw_track_id = 4;
  occlusion_state.merged_group.carrier_semantic_id = 1;

  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 0;

  Phase4HandoffCoordinator::SideRecoveryInput input;
  input.enabled = true;
  input.current_frame_idx = 31;
  input.shadow_hypotheses = &shadow_hypotheses;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.score_rows = &score_rows;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.occlusion_state = &occlusion_state;
  input.next_event_idx = &next_event_idx;

  EXPECT_FALSE(Phase4HandoffCoordinator::ApplyMergedSideRecovery(
      input,
      [&](int, int, int, int) { return true; },
      [&]() { FAIL() << "refresh should not run"; }));
  EXPECT_TRUE(raw_to_semantic_id.empty());
  EXPECT_TRUE(phase3_rows.empty());
}

TEST(Phase4HandoffCoordinatorTest, MergedSplitHandoffAppendsMigratedRowOnAcceptedApply) {
  std::unordered_map<int, int> prev_raw_to_semantic{{2, 2}};
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {2, MakeObservation(2, cv::Rect2f(646.0F, 250.0F, 139.0F, 463.0F))},
  };

  OcclusionGroupShadowLifecycle::State occlusion_state;
  occlusion_state.merged_group.active = true;
  occlusion_state.merged_group.group_id = 1;
  occlusion_state.merged_group.semantic_ids = {1, 2};
  occlusion_state.merged_group.carrier_raw_track_id = 2;
  occlusion_state.merged_group.carrier_semantic_id = 2;
  occlusion_state.merged_group.age_frames = 5;
  occlusion_state.merged_group.last_update_frame = 21;
  OcclusionGroupShadowLifecycle::SplitCandidateShadowState candidate;
  candidate.group_id = 1;
  candidate.candidate_raw_track_id = 7;
  candidate.candidate_semantic_id = 1;
  candidate.bbox = cv::Rect2f(736.0F, 204.0F, 177.0F, 555.0F);
  candidate.confidence = 0.90F;
  candidate.reason = "final_track_output";
  candidate.hypothesis_status = "tracked";
  candidate.stable_frames = 1;
  candidate.seen_this_frame = true;
  occlusion_state.split_candidates_by_raw_id.emplace(7, candidate);

  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 0;

  Phase4HandoffCoordinator::MergedSplitInput input;
  input.enabled = true;
  input.current_frame_idx = 22;
  input.phase4_continuity_raw = 2;
  input.phase4_continuity_sid = 2;
  input.prev_raw_to_semantic = &prev_raw_to_semantic;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.occlusion_state = &occlusion_state;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  int refresh_calls = 0;
  EXPECT_TRUE(Phase4HandoffCoordinator::ApplyMergedSplitHandoff(
      input,
      [&](const int continuity_raw_track_id, const int exposed_partial_sid,
          const int candidate_raw_track_id, const int continuity_sid) {
        ++apply_calls;
        EXPECT_EQ(continuity_raw_track_id, 2);
        EXPECT_EQ(exposed_partial_sid, 1);
        EXPECT_EQ(candidate_raw_track_id, 7);
        EXPECT_EQ(continuity_sid, 2);
        return true;
      },
      [&]() { ++refresh_calls; }));

  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(refresh_calls, 1);
  ASSERT_EQ(raw_to_semantic_id.size(), 2U);
  EXPECT_EQ(raw_to_semantic_id.at(2), 1);
  EXPECT_EQ(raw_to_semantic_id.at(7), 2);
  EXPECT_EQ(occlusion_state.merged_group.carrier_semantic_id, 1);
  ASSERT_EQ(phase3_rows.size(), 1U);
  EXPECT_EQ(phase3_rows[0].event_type, "phase4_merged_split_handoff");
  EXPECT_EQ(phase3_rows[0].reason, "merged_split_handoff");
  EXPECT_EQ(phase3_rows[0].candidate_raw_track_id, 7);
  EXPECT_EQ(phase3_rows[0].candidate_semantic_id, 2);
}

TEST(Phase4HandoffCoordinatorTest, MergedSplitHandoffSkipsNoOpPaths) {
  std::unordered_map<int, int> prev_raw_to_semantic{{7, 2}};
  std::unordered_map<int, TrackletObservation> observations_by_raw_track_id{
      {2, MakeObservation(2, cv::Rect2f(646.0F, 250.0F, 139.0F, 463.0F))},
  };

  OcclusionGroupShadowLifecycle::State occlusion_state;
  occlusion_state.merged_group.active = true;
  occlusion_state.merged_group.group_id = 1;
  occlusion_state.merged_group.semantic_ids = {1, 2};
  OcclusionGroupShadowLifecycle::SplitCandidateShadowState candidate;
  candidate.group_id = 1;
  candidate.candidate_raw_track_id = 7;
  candidate.candidate_semantic_id = 1;
  candidate.bbox = cv::Rect2f(736.0F, 204.0F, 177.0F, 555.0F);
  candidate.confidence = 0.90F;
  candidate.hypothesis_status = "tracked";
  candidate.stable_frames = 1;
  candidate.seen_this_frame = true;
  occlusion_state.split_candidates_by_raw_id.emplace(7, candidate);

  std::vector<IdentityManager::Phase3ShadowDebugRow> phase3_rows;
  std::unordered_map<int, int> raw_to_semantic_id;
  int next_event_idx = 0;

  Phase4HandoffCoordinator::MergedSplitInput input;
  input.enabled = true;
  input.current_frame_idx = 22;
  input.phase4_continuity_raw = 2;
  input.phase4_continuity_sid = 2;
  input.prev_raw_to_semantic = &prev_raw_to_semantic;
  input.observations_by_raw_track_id = &observations_by_raw_track_id;
  input.phase3_rows = &phase3_rows;
  input.raw_to_semantic_id = &raw_to_semantic_id;
  input.occlusion_state = &occlusion_state;
  input.next_event_idx = &next_event_idx;

  int apply_calls = 0;
  EXPECT_FALSE(Phase4HandoffCoordinator::ApplyMergedSplitHandoff(
      input,
      [&](int, int, int, int) {
        ++apply_calls;
        return true;
      },
      [&]() { FAIL() << "refresh should not run"; }));
  EXPECT_EQ(apply_calls, 0);
  EXPECT_TRUE(raw_to_semantic_id.empty());
  EXPECT_TRUE(phase3_rows.empty());
}
