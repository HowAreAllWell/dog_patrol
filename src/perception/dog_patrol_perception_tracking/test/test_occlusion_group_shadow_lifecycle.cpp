#include <gtest/gtest.h>

#include "occlusion_group_shadow_lifecycle.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityManager;
using dog_patrol_perception_tracking::OcclusionGroupShadowLifecycle;
using dog_patrol_perception_tracking::TrackletHypothesis;
using dog_patrol_perception_tracking::TrackletHypothesisStatus;

TrackletHypothesis MakeHypothesis(const int raw_id, const TrackletHypothesisStatus status,
                                  const char *reason, const int related_raw_track_id = -1) {
  TrackletHypothesis hypothesis;
  hypothesis.raw_track_id = raw_id;
  hypothesis.class_id = ClassId::kPerson;
  hypothesis.confidence = 0.75F;
  hypothesis.bbox = cv::Rect2f(10.0F * raw_id, 1.0F, 40.0F, 80.0F);
  hypothesis.status = status;
  hypothesis.candidate_reason = reason;
  if (related_raw_track_id > 0) {
    hypothesis.related_raw_track_id = related_raw_track_id;
  }
  return hypothesis;
}

}  // namespace

TEST(OcclusionGroupShadowLifecycleTest, CoversGroupLifecycleAndRecentCarryover) {
  OcclusionGroupShadowLifecycle::State state;
  std::vector<IdentityManager::Phase3ShadowDebugRow> rows;
  int next_event_idx = 0;
  OcclusionGroupShadowLifecycle::SyncGroupModeInput sync_input;
  OcclusionGroupShadowLifecycle::ShadowRowsContext context;
  context.next_event_idx = &next_event_idx;
  context.phase3_rows = &rows;

  sync_input.mode = IdentityManager::Mode::kMerged;
  sync_input.person_semantic_ids = {1, 2};
  sync_input.carrier_semantic_id = 1;
  sync_input.carrier_raw_track_id = 10;
  sync_input.phase4_continuity_raw = -1;
  context.current_frame_idx = 3;
  OcclusionGroupShadowLifecycle::SyncGroupMode(&state, sync_input, context);
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].event_type, "merged_group_enter");
  EXPECT_EQ(rows[0].group_id, 1);
  EXPECT_EQ(rows[0].carrier_semantic_id, 1);
  EXPECT_EQ(rows[0].carrier_raw_track_id, 10);
  EXPECT_EQ(rows[0].semantic_ids, "1|2");

  sync_input.mode = IdentityManager::Mode::kSplitRecovery;
  sync_input.person_semantic_ids = {1, 2};
  sync_input.carrier_semantic_id = 2;
  sync_input.carrier_raw_track_id = 20;
  sync_input.phase4_continuity_raw = -1;
  context.current_frame_idx = 4;
  OcclusionGroupShadowLifecycle::SyncGroupMode(&state, sync_input, context);
  ASSERT_EQ(rows.size(), 2U);
  EXPECT_EQ(rows[1].event_type, "merged_group_update");
  EXPECT_EQ(rows[1].reason, "legacy_mode_split_recovery_hold");
  EXPECT_EQ(rows[1].group_age_frames, 2);
  EXPECT_EQ(rows[1].carrier_semantic_id, 2);
  EXPECT_EQ(rows[1].carrier_raw_track_id, 20);

  sync_input.mode = IdentityManager::Mode::kNormalResumed;
  sync_input.person_semantic_ids.clear();
  sync_input.carrier_semantic_id = -1;
  sync_input.carrier_raw_track_id = -1;
  sync_input.phase4_continuity_raw = -1;
  context.current_frame_idx = 5;
  OcclusionGroupShadowLifecycle::SyncGroupMode(&state, sync_input, context);
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows[2].event_type, "merged_group_end");
  EXPECT_EQ(rows[2].reason, "legacy_mode_normal_resumed");
  ASSERT_NE(OcclusionGroupShadowLifecycle::RecoveryGroup(state, 6), nullptr);
  EXPECT_EQ(OcclusionGroupShadowLifecycle::RecoveryGroup(state, 8), nullptr);
}

TEST(OcclusionGroupShadowLifecycleTest, CoversSplitCandidateLifecycleAndGroupEndCleanup) {
  OcclusionGroupShadowLifecycle::State state;
  std::vector<IdentityManager::Phase3ShadowDebugRow> rows;
  int next_event_idx = 0;
  const auto context = [&](const int frame_idx) {
    OcclusionGroupShadowLifecycle::ShadowRowsContext shadow_context;
    shadow_context.current_frame_idx = frame_idx;
    shadow_context.next_event_idx = &next_event_idx;
    shadow_context.phase3_rows = &rows;
    return shadow_context;
  };
  OcclusionGroupShadowLifecycle::SyncGroupModeInput sync_input;
  OcclusionGroupShadowLifecycle::ObserveSplitCandidateInput candidate_input;

  sync_input.mode = IdentityManager::Mode::kMerged;
  sync_input.person_semantic_ids = {1, 2};
  sync_input.carrier_semantic_id = 1;
  sync_input.carrier_raw_track_id = 10;
  sync_input.phase4_continuity_raw = -1;
  OcclusionGroupShadowLifecycle::SyncGroupMode(&state, sync_input, context(1));
  rows.clear();
  next_event_idx = 0;

  candidate_input.hypothesis =
      MakeHypothesis(30, TrackletHypothesisStatus::kSuppressedDuplicateCandidate, "duplicate_output_hidden", 10);
  candidate_input.related_raw_track_id = 10;
  candidate_input.candidate_semantic_id = 2;
  OcclusionGroupShadowLifecycle::ObserveSplitCandidate(&state, candidate_input, context(2));
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].event_type, "split_candidate_enter");
  EXPECT_EQ(rows[0].candidate_raw_track_id, 30);
  EXPECT_EQ(rows[0].candidate_semantic_id, 2);
  EXPECT_EQ(rows[0].candidate_stable_frames, 1);

  rows.clear();
  next_event_idx = 0;
  OcclusionGroupShadowLifecycle::MarkSplitCandidatesUnseen(&state);
  candidate_input.hypothesis = MakeHypothesis(30, TrackletHypothesisStatus::kTracked, "final_track_output", 10);
  candidate_input.related_raw_track_id = 10;
  candidate_input.candidate_semantic_id = 2;
  OcclusionGroupShadowLifecycle::ObserveSplitCandidate(&state, candidate_input, context(3));
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].event_type, "split_candidate_update");
  EXPECT_EQ(rows[0].candidate_stable_frames, 2);

  rows.clear();
  next_event_idx = 0;
  OcclusionGroupShadowLifecycle::MarkSplitCandidatesUnseen(&state);
  OcclusionGroupShadowLifecycle::EndMissingSplitCandidates(&state, context(4));
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].event_type, "split_candidate_end");
  EXPECT_EQ(rows[0].reason, "candidate_missing");
  EXPECT_TRUE(state.split_candidates_by_raw_id.empty());

  rows.clear();
  next_event_idx = 0;
  candidate_input.hypothesis = MakeHypothesis(31, TrackletHypothesisStatus::kTracked, "final_track_output", 10);
  candidate_input.related_raw_track_id = 10;
  candidate_input.candidate_semantic_id = 2;
  OcclusionGroupShadowLifecycle::ObserveSplitCandidate(&state, candidate_input, context(5));
  rows.clear();
  next_event_idx = 0;
  sync_input.mode = IdentityManager::Mode::kNormal;
  sync_input.person_semantic_ids.clear();
  sync_input.carrier_semantic_id = -1;
  sync_input.carrier_raw_track_id = -1;
  sync_input.phase4_continuity_raw = -1;
  OcclusionGroupShadowLifecycle::SyncGroupMode(&state, sync_input, context(6));
  ASSERT_EQ(rows.size(), 2U);
  EXPECT_EQ(rows[0].event_type, "split_candidate_end");
  EXPECT_EQ(rows[0].reason, "group_end");
  EXPECT_EQ(rows[1].event_type, "merged_group_end");
}
