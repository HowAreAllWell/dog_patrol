#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "vision_demo_host/modules/identity_manager.hpp"

namespace {

vision_demo_host::Track MakePersonTrack(const int raw_id, const cv::Rect2f &bbox,
                                        const std::vector<float> &feature = {1.0F, 0.0F, 0.0F}) {
  vision_demo_host::Track track;
  track.id = raw_id;
  track.class_id = vision_demo_host::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = bbox;
  track.is_confirmed = true;
  track.appearance_feature = feature;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

vision_demo_host::PrimaryTargetResult IdlePrimary() {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kIdle;
  return primary;
}

vision_demo_host::PrimaryTargetResult LockedPrimary(const int semantic_id, const int raw_track_id = -1) {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kLocked;
  primary.primary_target_id = semantic_id;
  primary.raw_track_id = raw_track_id;
  return primary;
}

const vision_demo_host::IdentityObservation *FindIdentity(
    const std::vector<vision_demo_host::IdentityObservation> &identities, const int semantic_id) {
  const auto it = std::find_if(identities.begin(), identities.end(), [&](const auto &identity) {
    return identity.semantic_id == semantic_id;
  });
  return it == identities.end() ? nullptr : &(*it);
}

std::vector<std::string> EventTypes(const std::vector<vision_demo_host::IdentityManager::Phase3ShadowDebugRow> &rows) {
  std::vector<std::string> out;
  out.reserve(rows.size());
  for (const auto &row : rows) {
    out.push_back(row.event_type);
  }
  return out;
}

const vision_demo_host::IdentityManager::Phase3ShadowDebugRow *FindEvent(
    const std::vector<vision_demo_host::IdentityManager::Phase3ShadowDebugRow> &rows,
    const std::string &event_type, const int candidate_raw_track_id) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.event_type == event_type && row.candidate_raw_track_id == candidate_raw_track_id;
  });
  return it == rows.end() ? nullptr : &(*it);
}

const vision_demo_host::IdentityManager::ScoreDebugRow *FindScoreStage(
    const std::vector<vision_demo_host::IdentityManager::ScoreDebugRow> &rows,
    const int raw_track_id,
    const std::string &stage) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.raw_track_id == raw_track_id && row.stage == stage;
  });
  return it == rows.end() ? nullptr : &(*it);
}

std::vector<const vision_demo_host::IdentityManager::Phase3ShadowDebugRow *> FindEvents(
    const std::vector<vision_demo_host::IdentityManager::Phase3ShadowDebugRow> &rows,
    const std::string &event_type) {
  std::vector<const vision_demo_host::IdentityManager::Phase3ShadowDebugRow *> out;
  for (const auto &row : rows) {
    if (row.event_type == event_type) {
      out.push_back(&row);
    }
  }
  return out;
}

}  // namespace

TEST(IdentityManagerTest, ProducesVisibleIdentityObservations) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;

  vision_demo_host::IdentityManager manager(cfg);

  const std::vector<vision_demo_host::Track> tracks{
      MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50)),
      MakePersonTrack(20, cv::Rect2f(200, 0, 50, 50), {0.0F, 1.0F, 0.0F}),
  };
  const auto primary = IdlePrimary();

  const auto result = manager.Update(vision_demo_host::TrackletObservationsFromTracks(tracks), primary);

  ASSERT_EQ(result.identities.size(), tracks.size());
  EXPECT_EQ(result.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(result.SemanticIdForRawTrack(20), 2);

  const auto *identity1 = FindIdentity(result.identities, 1);
  ASSERT_NE(identity1, nullptr);
  ASSERT_TRUE(identity1->supporting_raw_track_id.has_value());
  EXPECT_EQ(*identity1->supporting_raw_track_id, 10);
  ASSERT_TRUE(identity1->supporting_tracklet.has_value());
  EXPECT_EQ(identity1->supporting_tracklet->raw_track_id, 10);
  EXPECT_TRUE(identity1->visible);
  EXPECT_EQ(identity1->missing_frames, 0);
  EXPECT_EQ(identity1->state, vision_demo_host::IdentityState::kActive);

  const auto *identity2 = FindIdentity(result.identities, 2);
  ASSERT_NE(identity2, nullptr);
  ASSERT_TRUE(identity2->supporting_raw_track_id.has_value());
  EXPECT_EQ(*identity2->supporting_raw_track_id, 20);
  EXPECT_TRUE(identity2->visible);
  EXPECT_EQ(identity2->state, vision_demo_host::IdentityState::kActive);
}

TEST(IdentityManagerTest, CarriesAssignmentEvidenceAndPrimaryFlag) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.raw_continuity_max_cost = 0.10F;
  cfg.active_assign_max_cost = 0.90F;
  vision_demo_host::IdentityManager manager(cfg);

  auto first = MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50));
  vision_demo_host::PrimaryTargetResult primary = IdlePrimary();
  primary.primary_target_id = 1;

  auto first_result = manager.Update(vision_demo_host::TrackletObservationsFromTracks({first}), primary);
  ASSERT_EQ(first_result.identities.size(), 1U);
  EXPECT_EQ(first_result.primary_semantic_id, 1);
  EXPECT_TRUE(first_result.identities.front().primary);

  auto jumped = MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50));
  const auto second_result = manager.Update(vision_demo_host::TrackletObservationsFromTracks({jumped}), primary);

  ASSERT_EQ(second_result.identities.size(), 1U);
  const auto &identity = second_result.identities.front();
  EXPECT_EQ(identity.semantic_id, 1);
  EXPECT_EQ(identity.assignment.stage, "assign_candidate");
  EXPECT_TRUE(identity.assignment.accepted);

  const auto &debug_rows = manager.LastScoreDebugRows();
  const auto raw_reject = std::find_if(debug_rows.begin(), debug_rows.end(), [](const auto &row) {
    return row.stage == "raw_continuity" && row.reject_reason == "raw_continuity_max_cost_reject";
  });
  ASSERT_NE(raw_reject, debug_rows.end());
  EXPECT_TRUE(raw_reject->continuity_used);
  EXPECT_FALSE(raw_reject->accepted);
}

TEST(IdentityManagerTest, ShadowHypothesesInputDoesNotChangeLegacyIdentityResult) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;

  vision_demo_host::IdentityManager baseline_manager(cfg);
  vision_demo_host::IdentityManager shadow_manager(cfg);

  const std::vector<vision_demo_host::Track> tracks{
      MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50)),
      MakePersonTrack(20, cv::Rect2f(200, 0, 50, 50), {0.0F, 1.0F, 0.0F}),
  };
  const auto observations = vision_demo_host::TrackletObservationsFromTracks(tracks);

  vision_demo_host::TrackletHypothesis tracked_hypothesis;
  tracked_hypothesis.raw_track_id = 10;
  tracked_hypothesis.class_id = vision_demo_host::ClassId::kPerson;
  tracked_hypothesis.confidence = 0.9F;
  tracked_hypothesis.bbox = cv::Rect2f(0, 0, 50, 50);
  tracked_hypothesis.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
  tracked_hypothesis.candidate_reason = "final_track_output";

  vision_demo_host::TrackletHypothesis hidden_hypothesis;
  hidden_hypothesis.raw_track_id = 30;
  hidden_hypothesis.class_id = vision_demo_host::ClassId::kPerson;
  hidden_hypothesis.confidence = 0.7F;
  hidden_hypothesis.bbox = cv::Rect2f(5, 0, 48, 50);
  hidden_hypothesis.status = vision_demo_host::TrackletHypothesisStatus::kSuppressedDuplicateCandidate;
  hidden_hypothesis.candidate_reason = "duplicate_output_hidden";
  hidden_hypothesis.related_raw_track_id = 10;

  const auto baseline_result = baseline_manager.Update(observations, IdlePrimary());
  const auto shadow_result =
      shadow_manager.Update(observations, {tracked_hypothesis, hidden_hypothesis}, IdlePrimary());

  ASSERT_EQ(shadow_result.identities.size(), baseline_result.identities.size());
  EXPECT_EQ(shadow_result.SemanticIdForRawTrack(10), baseline_result.SemanticIdForRawTrack(10));
  EXPECT_EQ(shadow_result.SemanticIdForRawTrack(20), baseline_result.SemanticIdForRawTrack(20));
  EXPECT_EQ(shadow_result.primary_semantic_id, baseline_result.primary_semantic_id);
  EXPECT_EQ(shadow_result.feature_update_frozen, baseline_result.feature_update_frozen);

  const auto &shadow_rows = shadow_manager.LastPhase3ShadowDebugRows();
  ASSERT_EQ(shadow_rows.size(), 2U);
  EXPECT_EQ(shadow_rows[0].event_type, "hypothesis_input");
  EXPECT_EQ(shadow_rows[0].candidate_raw_track_id, 10);
  EXPECT_EQ(shadow_rows[0].reason, "final_track_output");
  EXPECT_EQ(shadow_rows[0].related_raw_track_id, -1);
  EXPECT_EQ(shadow_rows[0].hypothesis_status, "tracked");
  EXPECT_EQ(shadow_rows[1].event_type, "hypothesis_input");
  EXPECT_EQ(shadow_rows[1].candidate_raw_track_id, 30);
  EXPECT_EQ(shadow_rows[1].reason, "duplicate_output_hidden");
  EXPECT_EQ(shadow_rows[1].related_raw_track_id, 10);
  EXPECT_EQ(shadow_rows[1].hypothesis_status, "suppressed_duplicate_candidate");
}

TEST(IdentityManagerTest, Phase3ShadowFrameIdxUsesZeroBasedUpdateFrameForHypothesesLinking) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityManager manager(cfg);

  vision_demo_host::TrackletHypothesis tracked_hypothesis;
  tracked_hypothesis.raw_track_id = 10;
  tracked_hypothesis.class_id = vision_demo_host::ClassId::kPerson;
  tracked_hypothesis.confidence = 0.9F;
  tracked_hypothesis.bbox = cv::Rect2f(0, 0, 50, 50);
  tracked_hypothesis.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
  tracked_hypothesis.candidate_reason = "final_track_output";

  const auto track = MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50));

  manager.Update(vision_demo_host::TrackletObservationsFromTracks({track}), {tracked_hypothesis}, IdlePrimary());
  ASSERT_FALSE(manager.LastPhase3ShadowDebugRows().empty());
  EXPECT_EQ(manager.LastPhase3ShadowDebugRows().front().frame_idx, 0);

  manager.Update(vision_demo_host::TrackletObservationsFromTracks({track}), {tracked_hypothesis}, IdlePrimary());
  ASSERT_FALSE(manager.LastPhase3ShadowDebugRows().empty());
  EXPECT_EQ(manager.LastPhase3ShadowDebugRows().front().frame_idx, 1);
}

TEST(IdentityManagerTest, EmitsShadowMergedGroupLifecycleWithoutChangingAssignments) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  cfg.merge_hold_frames = 1;
  cfg.split_stable_frames = 1;
  cfg.merged_requires_overlap = false;
  vision_demo_host::IdentityManager manager(cfg);

  const auto left = MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50));
  const auto right = MakePersonTrack(20, cv::Rect2f(100, 0, 50, 50), {0.0F, 1.0F, 0.0F});
  const auto initial = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  ASSERT_EQ(initial.SemanticIdForRawTrack(10), 1);
  ASSERT_EQ(initial.SemanticIdForRawTrack(20), 2);

  auto carrier = MakePersonTrack(10, cv::Rect2f(20, 0, 110, 50));
  const auto merged = manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), IdlePrimary());
  EXPECT_EQ(merged.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);

  const auto &enter_rows = manager.LastPhase3ShadowDebugRows();
  ASSERT_GE(enter_rows.size(), 1U);
  EXPECT_EQ(enter_rows[0].event_type, "merged_group_enter");
  EXPECT_EQ(enter_rows[0].group_id, 1);
  EXPECT_NE(enter_rows[0].semantic_ids.find("1"), std::string::npos);
  EXPECT_NE(enter_rows[0].semantic_ids.find("2"), std::string::npos);
  EXPECT_EQ(enter_rows[0].carrier_semantic_id, 1);
  EXPECT_EQ(enter_rows[0].carrier_raw_track_id, 10);
  EXPECT_EQ(enter_rows[0].candidate_raw_track_id, -1);
  EXPECT_EQ(enter_rows[0].reason, "legacy_mode_merged_enter");
  EXPECT_EQ(enter_rows[0].group_age_frames, 1);
  EXPECT_EQ(enter_rows[0].group_last_update_frame, enter_rows[0].frame_idx);

  carrier.bbox = cv::Rect2f(24, 0, 108, 50);
  const auto held = manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), IdlePrimary());
  EXPECT_EQ(held.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);

  const auto hold_events = EventTypes(manager.LastPhase3ShadowDebugRows());
  EXPECT_NE(std::find(hold_events.begin(), hold_events.end(), "merged_group_update"), hold_events.end());

  const auto split_recovery =
      manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  EXPECT_EQ(split_recovery.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(split_recovery.SemanticIdForRawTrack(20), 2);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kSplitRecovery);

  const auto split_events = EventTypes(manager.LastPhase3ShadowDebugRows());
  EXPECT_NE(std::find(split_events.begin(), split_events.end(), "merged_group_update"), split_events.end());

  const auto separated = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  EXPECT_EQ(separated.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(separated.SemanticIdForRawTrack(20), 2);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kNormal);

  const auto end_events = EventTypes(manager.LastPhase3ShadowDebugRows());
  EXPECT_NE(std::find(end_events.begin(), end_events.end(), "merged_group_end"), end_events.end());
}

TEST(IdentityManagerTest, EmitsShadowSplitCandidateLifecycleLinkedToMergedGroup) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  cfg.merge_hold_frames = 1;
  cfg.split_stable_frames = 1;
  cfg.merged_requires_overlap = false;
  vision_demo_host::IdentityManager manager(cfg);

  const auto left = MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50));
  const auto right = MakePersonTrack(20, cv::Rect2f(100, 0, 50, 50), {0.0F, 1.0F, 0.0F});
  const auto initial = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  ASSERT_EQ(initial.SemanticIdForRawTrack(10), 1);
  ASSERT_EQ(initial.SemanticIdForRawTrack(20), 2);

  auto carrier = MakePersonTrack(10, cv::Rect2f(20, 0, 110, 50));
  const auto merged = manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), IdlePrimary());
  ASSERT_EQ(merged.SemanticIdForRawTrack(10), 1);
  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);

  vision_demo_host::TrackletHypothesis hidden_candidate;
  hidden_candidate.raw_track_id = 30;
  hidden_candidate.class_id = vision_demo_host::ClassId::kPerson;
  hidden_candidate.confidence = 0.72F;
  hidden_candidate.bbox = cv::Rect2f(82, 2, 42, 48);
  hidden_candidate.status = vision_demo_host::TrackletHypothesisStatus::kSuppressedDuplicateCandidate;
  hidden_candidate.candidate_reason = "duplicate_output_hidden";
  hidden_candidate.related_raw_track_id = 10;

  vision_demo_host::TrackletHypothesis suppressed_candidate;
  suppressed_candidate.raw_track_id = 31;
  suppressed_candidate.class_id = vision_demo_host::ClassId::kPerson;
  suppressed_candidate.confidence = 0.68F;
  suppressed_candidate.bbox = cv::Rect2f(78, 5, 44, 45);
  suppressed_candidate.status = vision_demo_host::TrackletHypothesisStatus::kSuppressedDuplicateCandidate;
  suppressed_candidate.candidate_reason = "new_track_suppressed_duplicate_tracked";
  suppressed_candidate.related_raw_track_id = 10;

  carrier.bbox = cv::Rect2f(22, 0, 108, 50);
  const auto candidate_first = manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}),
                                              {hidden_candidate, suppressed_candidate}, IdlePrimary());
  EXPECT_EQ(candidate_first.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);

  const auto *first_row = FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_enter", 30);
  ASSERT_NE(first_row, nullptr);
  EXPECT_EQ(first_row->group_id, 1);
  EXPECT_EQ(first_row->carrier_raw_track_id, 10);
  EXPECT_EQ(first_row->candidate_raw_track_id, 30);
  EXPECT_EQ(first_row->candidate_semantic_id, 2);
  EXPECT_EQ(first_row->reason, "duplicate_output_hidden");
  EXPECT_EQ(first_row->related_raw_track_id, 10);
  EXPECT_EQ(first_row->hypothesis_status, "suppressed_duplicate_candidate");
  EXPECT_EQ(first_row->candidate_stable_frames, 1);
  EXPECT_FLOAT_EQ(first_row->candidate_confidence, 0.72F);
  EXPECT_FLOAT_EQ(first_row->candidate_bbox.x, 82.0F);
  EXPECT_FLOAT_EQ(first_row->candidate_bbox.y, 2.0F);
  EXPECT_FLOAT_EQ(first_row->candidate_bbox.width, 42.0F);
  EXPECT_FLOAT_EQ(first_row->candidate_bbox.height, 48.0F);

  const auto *suppressed_row = FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_enter", 31);
  ASSERT_NE(suppressed_row, nullptr);
  EXPECT_EQ(suppressed_row->group_id, 1);
  EXPECT_EQ(suppressed_row->candidate_semantic_id, 2);
  EXPECT_EQ(suppressed_row->reason, "new_track_suppressed_duplicate_tracked");
  EXPECT_EQ(suppressed_row->related_raw_track_id, 10);
  EXPECT_EQ(suppressed_row->hypothesis_status, "suppressed_duplicate_candidate");
  EXPECT_EQ(suppressed_row->candidate_stable_frames, 1);

  hidden_candidate.confidence = 0.74F;
  hidden_candidate.bbox = cv::Rect2f(84, 2, 42, 48);
  const auto candidate_second =
      manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), {hidden_candidate}, IdlePrimary());
  EXPECT_EQ(candidate_second.SemanticIdForRawTrack(10), 1);
  const auto *second_row = FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_update", 30);
  ASSERT_NE(second_row, nullptr);
  EXPECT_EQ(second_row->candidate_stable_frames, 2);

  const auto no_candidate = manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), {}, IdlePrimary());
  EXPECT_EQ(no_candidate.SemanticIdForRawTrack(10), 1);
  const auto *end_row = FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_end", 30);
  ASSERT_NE(end_row, nullptr);
  EXPECT_EQ(end_row->group_id, 1);
  EXPECT_EQ(end_row->candidate_stable_frames, 2);
  EXPECT_EQ(end_row->reason, "candidate_missing");

  const auto separated = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  EXPECT_EQ(separated.SemanticIdForRawTrack(10), 1);
  EXPECT_EQ(separated.SemanticIdForRawTrack(20), 2);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kSplitRecovery);
  EXPECT_EQ(FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_update", 30), nullptr);
}

TEST(IdentityManagerTest, EndsShadowSplitCandidateWhenMergedGroupEnds) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  cfg.merge_hold_frames = 1;
  cfg.split_stable_frames = 1;
  cfg.merged_requires_overlap = false;
  vision_demo_host::IdentityManager manager(cfg);

  const auto left = MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50));
  const auto right = MakePersonTrack(20, cv::Rect2f(100, 0, 50, 50), {0.0F, 1.0F, 0.0F});
  ASSERT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary())
                .SemanticIdForRawTrack(20),
            2);

  auto carrier = MakePersonTrack(10, cv::Rect2f(20, 0, 110, 50));
  ASSERT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({carrier}), IdlePrimary())
                .SemanticIdForRawTrack(10),
            1);
  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);

  vision_demo_host::TrackletHypothesis tracked_candidate;
  tracked_candidate.raw_track_id = 20;
  tracked_candidate.class_id = vision_demo_host::ClassId::kPerson;
  tracked_candidate.confidence = 0.88F;
  tracked_candidate.bbox = right.bbox;
  tracked_candidate.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
  tracked_candidate.candidate_reason = "final_track_output";

  manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), {tracked_candidate}, IdlePrimary());
  ASSERT_NE(FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_enter", 20), nullptr);
  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kSplitRecovery);

  manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), {tracked_candidate}, IdlePrimary());
  const auto *end_row = FindEvent(manager.LastPhase3ShadowDebugRows(), "split_candidate_end", 20);
  ASSERT_NE(end_row, nullptr);
  EXPECT_EQ(end_row->reason, "group_end");
  EXPECT_EQ(end_row->candidate_stable_frames, 1);
  EXPECT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kNormal);
}

TEST(IdentityManagerTest, EmitsSingleBlobContinuityAndMissingAgeDecisionRowsWithoutChangingAssignments) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.app_w = 0.0F;
  cfg.geo_w = 1.0F;
  cfg.time_w = 0.0F;
  cfg.min_assignment_margin = 0.08F;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityManager manager(cfg);

  const auto left = MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100));
  const auto right = MakePersonTrack(2, cv::Rect2f(20, 0, 100, 100), {0.0F, 1.0F, 0.0F});
  const auto first = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  ASSERT_EQ(first.SemanticIdForRawTrack(1), 1);
  ASSERT_EQ(first.SemanticIdForRawTrack(2), 2);

  const auto merged = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(11, 0, 100, 100), {0.0F, 1.0F, 0.0F}),
      }),
      IdlePrimary());

  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);
  ASSERT_EQ(merged.SemanticIdForRawTrack(1), 1);

  const auto rows = FindEvents(manager.LastPhase3ShadowDebugRows(), "single_blob_handoff_decision");
  ASSERT_EQ(rows.size(), 2U);

  const auto continuity_it = std::find_if(rows.begin(), rows.end(), [](const auto *row) {
    return row->candidate_semantic_id == 1;
  });
  ASSERT_NE(continuity_it, rows.end());
  const auto *continuity_row = *continuity_it;
  EXPECT_EQ(continuity_row->group_id, 1);
  EXPECT_EQ(continuity_row->carrier_raw_track_id, 1);
  EXPECT_EQ(continuity_row->carrier_semantic_id, 1);
  EXPECT_EQ(continuity_row->candidate_raw_track_id, 1);
  EXPECT_EQ(continuity_row->reason, "single_blob_continuity_kept");
  EXPECT_EQ(continuity_row->related_raw_track_id, 1);
  EXPECT_TRUE(continuity_row->decision_selected);
  EXPECT_TRUE(continuity_row->decision_accepted);
  EXPECT_GT(continuity_row->decision_final_score, 0.0F);

  const auto missing_age_it = std::find_if(rows.begin(), rows.end(), [](const auto *row) {
    return row->candidate_semantic_id == 2;
  });
  ASSERT_NE(missing_age_it, rows.end());
  const auto *missing_age_row = *missing_age_it;
  EXPECT_EQ(missing_age_row->reason, "single_blob_rejected_by_missing_age");
  EXPECT_EQ(missing_age_row->related_raw_track_id, 1);
  EXPECT_FALSE(missing_age_row->decision_selected);
  EXPECT_FALSE(missing_age_row->decision_accepted);
}

TEST(IdentityManagerTest, EmitsSingleBlobHandoffRejectReasonsWithoutChangingAssignments) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_min_area_ratio = 0.40F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityManager manager(cfg);

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F}),
      }),
      LockedPrimary(1, 1));
  ASSERT_EQ(first.SemanticIdForRawTrack(1), 1);

  const auto second = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F}),
          MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F}),
      }),
      LockedPrimary(1, 1));
  ASSERT_EQ(second.SemanticIdForRawTrack(1), 1);
  ASSERT_EQ(second.SemanticIdForRawTrack(2), 2);

  manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(1200, 0, 500, 1000), {1.0F, 0.0F, 0.0F}),
          MakePersonTrack(2, cv::Rect2f(1250, 0, 500, 1000), {0.0F, 1.0F, 0.0F}),
      }),
      LockedPrimary(1, 1));

  vision_demo_host::IdentityManagerResult only_other;
  for (int i = 0; i < 18; ++i) {
    only_other = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F}),
        }),
        LockedPrimary(1, 1));
    ASSERT_EQ(only_other.SemanticIdForRawTrack(2), 2);
  }

  const auto reject_rows = FindEvents(manager.LastPhase3ShadowDebugRows(), "single_blob_handoff_decision");
  ASSERT_FALSE(reject_rows.empty());
  EXPECT_EQ(only_other.SemanticIdForRawTrack(2), 2);
  EXPECT_NE(std::find_if(reject_rows.begin(), reject_rows.end(), [](const auto *row) {
              return row->candidate_semantic_id == 1 &&
                     row->reason == "single_blob_rejected_by_appearance_or_geometry_margin";
            }),
            reject_rows.end());
}

TEST(IdentityManagerTest, EmitsSingleBlobHandoffAcceptedDecisionWithoutChangingLegacyBehavior) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityManager manager(cfg);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  ASSERT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                               MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
                               MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature),
                           }),
                           LockedPrimary(1, 1))
                .SemanticIdForRawTrack(1),
            1);
  ASSERT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                               MakePersonTrack(1, cv::Rect2f(320, 220, 240, 620), primary_feature),
                               MakePersonTrack(2, cv::Rect2f(680, 160, 180, 560), secondary_feature),
                           }),
                           LockedPrimary(1, 1))
                .SemanticIdForRawTrack(2),
            2);
  manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                     MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
                     MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature),
                 }),
                 LockedPrimary(1, 1));
  for (int i = 0; i < 18; ++i) {
    ASSERT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                                 MakePersonTrack(2, cv::Rect2f(590, 170, 220, 600), secondary_feature),
                             }),
                             LockedPrimary(1, 1))
                  .SemanticIdForRawTrack(2),
              2);
  }

  const auto handoff = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(2, cv::Rect2f(560, 240, 200, 560), primary_feature),
      }),
      LockedPrimary(1, 1));

  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);
  EXPECT_EQ(handoff.SemanticIdForRawTrack(2), 1);
  const auto rows = FindEvents(manager.LastPhase3ShadowDebugRows(), "single_blob_handoff_decision");
  ASSERT_FALSE(rows.empty());
  EXPECT_NE(std::find_if(rows.begin(), rows.end(), [](const auto *row) {
              return row->candidate_semantic_id == 1 && row->reason == "single_blob_handoff_accepted" &&
                     row->decision_selected && row->decision_accepted;
            }),
            rows.end());
}

TEST(IdentityManagerTest, Phase4MergedSingleBlobHandoffFlagKeepsDefaultOffAndEmitsMigratedDecision) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};

  const auto run_sequence = [&](vision_demo_host::IdentityManager manager) {
    EXPECT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                                 MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
                                 MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature),
                             }),
                             LockedPrimary(1, 1))
                  .SemanticIdForRawTrack(1),
              1);
    EXPECT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                                 MakePersonTrack(1, cv::Rect2f(320, 220, 240, 620), primary_feature),
                                 MakePersonTrack(2, cv::Rect2f(680, 160, 180, 560), secondary_feature),
                             }),
                             LockedPrimary(1, 1))
                  .SemanticIdForRawTrack(2),
              2);
    manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                       MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
                       MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature),
                   }),
                   LockedPrimary(1, 1));
    for (int i = 0; i < 18; ++i) {
      EXPECT_EQ(manager.Update(vision_demo_host::TrackletObservationsFromTracks({
                                   MakePersonTrack(2, cv::Rect2f(590, 170, 220, 600), secondary_feature),
                               }),
                               LockedPrimary(1, 1))
                    .SemanticIdForRawTrack(2),
                2);
    }

    const auto handoff = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(2, cv::Rect2f(560, 240, 200, 560), primary_feature),
        }),
        LockedPrimary(1, 1));
    return std::make_pair(handoff, manager);
  };

  auto [default_result, default_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  ASSERT_EQ(default_result.SemanticIdForRawTrack(2), 1);
  EXPECT_NE(FindScoreStage(default_manager.LastScoreDebugRows(), 2, "merged_candidate"), nullptr);
  EXPECT_EQ(FindScoreStage(default_manager.LastScoreDebugRows(), 2, "phase4_merged_single_blob_handoff"), nullptr);
  EXPECT_EQ(FindEvent(default_manager.LastPhase3ShadowDebugRows(), "phase4_merged_single_blob_handoff", 2), nullptr);

  cfg.enable_phase4_merged_single_blob_handoff = true;
  auto [migrated_result, migrated_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  EXPECT_EQ(migrated_result.SemanticIdForRawTrack(2), default_result.SemanticIdForRawTrack(2));
  EXPECT_FALSE(std::any_of(migrated_manager.LastScoreDebugRows().begin(),
                           migrated_manager.LastScoreDebugRows().end(), [](const auto &row) {
                             return row.raw_track_id == 2 && row.semantic_id == 1 &&
                                    row.stage == "merged_candidate" && row.accepted;
                           }));
  EXPECT_NE(FindScoreStage(migrated_manager.LastScoreDebugRows(), 2, "phase4_merged_single_blob_handoff"), nullptr);

  const auto *decision_row =
      FindEvent(migrated_manager.LastPhase3ShadowDebugRows(), "single_blob_handoff_decision", 2);
  ASSERT_NE(decision_row, nullptr);
  EXPECT_EQ(decision_row->candidate_semantic_id, 1);
  EXPECT_EQ(decision_row->reason, "single_blob_handoff_accepted");
  EXPECT_TRUE(decision_row->decision_selected);
  EXPECT_TRUE(decision_row->decision_accepted);

  const auto *migrated_row =
      FindEvent(migrated_manager.LastPhase3ShadowDebugRows(), "phase4_merged_single_blob_handoff", 2);
  ASSERT_NE(migrated_row, nullptr);
  EXPECT_EQ(migrated_row->candidate_semantic_id, 1);
  EXPECT_EQ(migrated_row->carrier_raw_track_id, 2);
  EXPECT_EQ(migrated_row->carrier_semantic_id, 2);
  EXPECT_EQ(migrated_row->reason, "merged_single_blob_handoff");
  EXPECT_TRUE(migrated_row->decision_selected);
  EXPECT_TRUE(migrated_row->decision_accepted);
  EXPECT_NEAR(migrated_row->decision_app_cost, decision_row->decision_app_cost, 1e-5F);
  EXPECT_NEAR(migrated_row->decision_final_score, decision_row->decision_final_score, 1e-5F);
}

TEST(IdentityManagerTest, Phase4MergedSingleBlobHandoffFlagDoesNotAcceptRejectedDecisionRows) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.enable_phase4_merged_single_blob_handoff = true;
  cfg.app_w = 0.0F;
  cfg.geo_w = 1.0F;
  cfg.time_w = 0.0F;
  cfg.min_assignment_margin = 0.08F;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityManager manager(cfg);

  const auto left = MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100));
  const auto right = MakePersonTrack(2, cv::Rect2f(20, 0, 100, 100), {0.0F, 1.0F, 0.0F});
  const auto first = manager.Update(vision_demo_host::TrackletObservationsFromTracks({left, right}), IdlePrimary());
  ASSERT_EQ(first.SemanticIdForRawTrack(1), 1);
  ASSERT_EQ(first.SemanticIdForRawTrack(2), 2);

  const auto merged = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(11, 0, 100, 100), {0.0F, 1.0F, 0.0F}),
      }),
      IdlePrimary());

  ASSERT_EQ(manager.CurrentMode(), vision_demo_host::IdentityManager::Mode::kMerged);
  EXPECT_EQ(merged.SemanticIdForRawTrack(1), 1);
  EXPECT_NE(FindEvent(manager.LastPhase3ShadowDebugRows(), "single_blob_handoff_decision", 1), nullptr);
  EXPECT_EQ(FindEvent(manager.LastPhase3ShadowDebugRows(), "phase4_merged_single_blob_handoff", 1), nullptr);
  EXPECT_EQ(FindScoreStage(manager.LastScoreDebugRows(), 1, "phase4_merged_single_blob_handoff"), nullptr);
}

TEST(IdentityManagerTest, EmitsPairwiseAssignmentMatrixShadowRowsWithoutChangingAssignments) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.stable_frames_before_feature_update = 1;
  vision_demo_host::IdentityManager manager(cfg);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.96F, 0.28F};
  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100), primary_feature),
          MakePersonTrack(2, cv::Rect2f(200, 0, 100, 100), secondary_feature),
      }),
      IdlePrimary());
  ASSERT_EQ(first.SemanticIdForRawTrack(1), 1);
  ASSERT_EQ(first.SemanticIdForRawTrack(2), 2);

  for (int i = 0; i < 10; ++i) {
    manager.Update({}, IdlePrimary());
  }

  const auto recovered = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(30, cv::Rect2f(80, 0, 100, 100), secondary_feature),
          MakePersonTrack(40, cv::Rect2f(120, 0, 100, 100), primary_feature),
      }),
      IdlePrimary());

  EXPECT_EQ(recovered.SemanticIdForRawTrack(30), 2);
  EXPECT_EQ(recovered.SemanticIdForRawTrack(40), 1);

  const auto pairwise_rows = FindEvents(manager.LastPhase3ShadowDebugRows(), "pairwise_assignment_matrix");
  ASSERT_EQ(pairwise_rows.size(), 1U);
  const auto &row = *pairwise_rows.front();
  EXPECT_EQ(row.reason, "pairwise_appearance_override");
  EXPECT_EQ(row.pairwise_selected_pairs, "30->1|40->2");
  EXPECT_EQ(row.pairwise_alternate_pairs, "30->2|40->1");
  EXPECT_LT(row.pairwise_alternate_final_cost, row.pairwise_selected_final_cost + 0.08F);
  EXPECT_LT(row.pairwise_alternate_app_cost + 0.035F, row.pairwise_selected_app_cost);
  EXPECT_GT(row.pairwise_margin, 0.0F);
  EXPECT_TRUE(row.pairwise_appearance_override);
}

TEST(IdentityManagerTest, EmitsSideReappearanceCandidateLinkedToMergedGroupWithoutChangingAssignments) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  cfg.merge_hold_frames = 1;
  cfg.split_stable_frames = 1;
  cfg.merged_requires_overlap = false;
  vision_demo_host::IdentityManager manager(cfg);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const std::vector<float> side_reappear_feature{0.7F, 0.3F};

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(4, cv::Rect2f(330, 0, 300, 900), primary_feature),
          MakePersonTrack(5, cv::Rect2f(610, 70, 180, 680), secondary_feature),
      }),
      IdlePrimary());
  ASSERT_EQ(first.SemanticIdForRawTrack(4), 1);
  ASSERT_EQ(first.SemanticIdForRawTrack(5), 2);

  const auto overlap = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(4, cv::Rect2f(330, 0, 310, 900), primary_feature),
          MakePersonTrack(5, cv::Rect2f(590, 170, 125, 555), secondary_feature),
      }),
      IdlePrimary());
  ASSERT_EQ(overlap.SemanticIdForRawTrack(4), 1);
  ASSERT_EQ(overlap.SemanticIdForRawTrack(5), 2);

  for (int i = 0; i < 30; ++i) {
    const auto merged = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(315, 0, 300, 930), primary_feature),
        }),
        IdlePrimary());
    ASSERT_EQ(merged.SemanticIdForRawTrack(4), 1);
  }

  vision_demo_host::TrackletHypothesis side_reappearance;
  side_reappearance.raw_track_id = 6;
  side_reappearance.class_id = vision_demo_host::ClassId::kPerson;
  side_reappearance.confidence = 0.90F;
  side_reappearance.bbox = cv::Rect2f(179, 228, 194, 514);
  side_reappearance.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
  side_reappearance.candidate_reason = "final_track_output";

  const auto recovered = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({
          MakePersonTrack(4, cv::Rect2f(310, 0, 298, 928), primary_feature),
          MakePersonTrack(6, cv::Rect2f(179, 228, 194, 514), side_reappear_feature),
      }),
      {side_reappearance},
      IdlePrimary());

  ASSERT_EQ(recovered.SemanticIdForRawTrack(4), 1);
  ASSERT_EQ(recovered.SemanticIdForRawTrack(6), 2);
  EXPECT_NE(FindScoreStage(manager.LastScoreDebugRows(), 6, "merged_side_recovery"), nullptr);

  const auto *row = FindEvent(manager.LastPhase3ShadowDebugRows(), "side_reappearance_candidate", 6);
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->group_id, 1);
  EXPECT_EQ(row->carrier_raw_track_id, 4);
  EXPECT_EQ(row->carrier_semantic_id, 1);
  EXPECT_EQ(row->candidate_raw_track_id, 6);
  EXPECT_EQ(row->candidate_semantic_id, 2);
  EXPECT_EQ(row->reason, "side_reappearance_candidate");
  EXPECT_EQ(row->related_raw_track_id, 4);
  EXPECT_EQ(row->hypothesis_status, "tracked");
  EXPECT_EQ(row->candidate_stable_frames, 1);
  EXPECT_FLOAT_EQ(row->candidate_confidence, 0.90F);
  EXPECT_FLOAT_EQ(row->candidate_bbox.x, 179.0F);
  EXPECT_FLOAT_EQ(row->candidate_bbox.y, 228.0F);
  EXPECT_FLOAT_EQ(row->candidate_bbox.width, 194.0F);
  EXPECT_FLOAT_EQ(row->candidate_bbox.height, 514.0F);
}

TEST(IdentityManagerTest, Phase4MergedSideRecoveryFlagKeepsDefaultOffAndEmitsMigratedDecision) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  cfg.merge_hold_frames = 1;
  cfg.split_stable_frames = 1;
  cfg.merged_requires_overlap = false;

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const std::vector<float> side_reappear_feature{0.7F, 0.3F};

  auto run_sequence = [&](vision_demo_host::IdentityManager manager) {
    const auto first = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(330, 0, 300, 900), primary_feature),
            MakePersonTrack(5, cv::Rect2f(610, 70, 180, 680), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(first.SemanticIdForRawTrack(4), 1);
    EXPECT_EQ(first.SemanticIdForRawTrack(5), 2);

    const auto overlap = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(330, 0, 310, 900), primary_feature),
            MakePersonTrack(5, cv::Rect2f(590, 170, 125, 555), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(overlap.SemanticIdForRawTrack(4), 1);
    EXPECT_EQ(overlap.SemanticIdForRawTrack(5), 2);

    for (int i = 0; i < 30; ++i) {
      const auto merged = manager.Update(
          vision_demo_host::TrackletObservationsFromTracks({
              MakePersonTrack(4, cv::Rect2f(315, 0, 300, 930), primary_feature),
          }),
          IdlePrimary());
      EXPECT_EQ(merged.SemanticIdForRawTrack(4), 1);
    }

    vision_demo_host::TrackletHypothesis side_reappearance;
    side_reappearance.raw_track_id = 6;
    side_reappearance.class_id = vision_demo_host::ClassId::kPerson;
    side_reappearance.confidence = 0.90F;
    side_reappearance.bbox = cv::Rect2f(179, 228, 194, 514);
    side_reappearance.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
    side_reappearance.candidate_reason = "final_track_output";

    const auto recovered = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(310, 0, 298, 928), primary_feature),
            MakePersonTrack(6, cv::Rect2f(179, 228, 194, 514), side_reappear_feature),
        }),
        {side_reappearance},
        IdlePrimary());
    return std::make_pair(recovered, manager);
  };

  auto run_sequence_without_shadow_candidate = [&](vision_demo_host::IdentityManager manager) {
    const auto first = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(330, 0, 300, 900), primary_feature),
            MakePersonTrack(5, cv::Rect2f(610, 70, 180, 680), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(first.SemanticIdForRawTrack(4), 1);
    EXPECT_EQ(first.SemanticIdForRawTrack(5), 2);

    const auto overlap = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(330, 0, 310, 900), primary_feature),
            MakePersonTrack(5, cv::Rect2f(590, 170, 125, 555), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(overlap.SemanticIdForRawTrack(4), 1);
    EXPECT_EQ(overlap.SemanticIdForRawTrack(5), 2);

    for (int i = 0; i < 30; ++i) {
      const auto merged = manager.Update(
          vision_demo_host::TrackletObservationsFromTracks({
              MakePersonTrack(4, cv::Rect2f(315, 0, 300, 930), primary_feature),
          }),
          IdlePrimary());
      EXPECT_EQ(merged.SemanticIdForRawTrack(4), 1);
    }

    const auto recovered = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(4, cv::Rect2f(310, 0, 298, 928), primary_feature),
            MakePersonTrack(6, cv::Rect2f(179, 228, 194, 514), side_reappear_feature),
        }),
        {},
        IdlePrimary());
    return std::make_pair(recovered, manager);
  };

  auto [default_result, default_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  ASSERT_EQ(default_result.SemanticIdForRawTrack(4), 1);
  ASSERT_EQ(default_result.SemanticIdForRawTrack(6), 2);
  EXPECT_NE(FindScoreStage(default_manager.LastScoreDebugRows(), 6, "merged_side_recovery"), nullptr);
  EXPECT_EQ(FindEvent(default_manager.LastPhase3ShadowDebugRows(), "phase4_merged_side_recovery", 6), nullptr);

  cfg.enable_phase4_merged_side_recovery = true;
  auto [migrated_result, migrated_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  EXPECT_EQ(migrated_result.SemanticIdForRawTrack(4), default_result.SemanticIdForRawTrack(4));
  EXPECT_EQ(migrated_result.SemanticIdForRawTrack(6), default_result.SemanticIdForRawTrack(6));
  EXPECT_EQ(FindScoreStage(migrated_manager.LastScoreDebugRows(), 6, "merged_side_recovery"), nullptr);

  const auto *migrated_row =
      FindEvent(migrated_manager.LastPhase3ShadowDebugRows(), "phase4_merged_side_recovery", 6);
  ASSERT_NE(migrated_row, nullptr);
  EXPECT_EQ(migrated_row->group_id, 1);
  EXPECT_EQ(migrated_row->carrier_raw_track_id, 4);
  EXPECT_EQ(migrated_row->carrier_semantic_id, 1);
  EXPECT_EQ(migrated_row->candidate_raw_track_id, 6);
  EXPECT_EQ(migrated_row->candidate_semantic_id, 2);
  EXPECT_EQ(migrated_row->reason, "merged_side_recovery");
  EXPECT_EQ(migrated_row->related_raw_track_id, 4);
  EXPECT_EQ(migrated_row->hypothesis_status, "tracked");
  EXPECT_EQ(migrated_row->candidate_stable_frames, 1);

  auto [no_shadow_result, no_shadow_manager] =
      run_sequence_without_shadow_candidate(vision_demo_host::IdentityManager(cfg));
  (void)no_shadow_result;
  EXPECT_EQ(FindEvent(no_shadow_manager.LastPhase3ShadowDebugRows(), "phase4_merged_side_recovery", 6), nullptr);
  EXPECT_EQ(FindScoreStage(no_shadow_manager.LastScoreDebugRows(), 6, "phase4_merged_side_recovery"), nullptr);
}

TEST(IdentityManagerTest, Phase4MergedSplitHandoffFlagKeepsDefaultOffAndEmitsMigratedDecision) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};

  auto run_sequence = [&](vision_demo_host::IdentityManager manager) {
    const auto first = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
            MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(first.SemanticIdForRawTrack(1), 1);
    EXPECT_EQ(first.SemanticIdForRawTrack(2), 2);

    const auto overlap = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
            MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(overlap.SemanticIdForRawTrack(1), 1);
    EXPECT_EQ(overlap.SemanticIdForRawTrack(2), 2);

    for (int i = 0; i < 18; ++i) {
      const auto merged = manager.Update(
          vision_demo_host::TrackletObservationsFromTracks({
              MakePersonTrack(2, cv::Rect2f(585, 210, 270, 550), secondary_feature),
          }),
          IdlePrimary());
      EXPECT_EQ(merged.SemanticIdForRawTrack(2), 2);
    }

    vision_demo_host::TrackletHypothesis split_hypothesis;
    split_hypothesis.raw_track_id = 7;
    split_hypothesis.class_id = vision_demo_host::ClassId::kPerson;
    split_hypothesis.confidence = 0.90F;
    split_hypothesis.bbox = cv::Rect2f(736, 204, 177, 555);
    split_hypothesis.status = vision_demo_host::TrackletHypothesisStatus::kTracked;
    split_hypothesis.candidate_reason = "final_track_output";

    const auto split = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(2, cv::Rect2f(646, 250, 139, 463), primary_feature),
            MakePersonTrack(7, cv::Rect2f(736, 204, 177, 555), secondary_feature),
        }),
        {split_hypothesis},
        IdlePrimary());
    return std::make_pair(split, manager);
  };

  auto run_sequence_without_shadow_candidate = [&](vision_demo_host::IdentityManager manager) {
    const auto first = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
            MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(first.SemanticIdForRawTrack(1), 1);
    EXPECT_EQ(first.SemanticIdForRawTrack(2), 2);

    const auto overlap = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
            MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature),
        }),
        IdlePrimary());
    EXPECT_EQ(overlap.SemanticIdForRawTrack(1), 1);
    EXPECT_EQ(overlap.SemanticIdForRawTrack(2), 2);

    for (int i = 0; i < 18; ++i) {
      const auto merged = manager.Update(
          vision_demo_host::TrackletObservationsFromTracks({
              MakePersonTrack(2, cv::Rect2f(585, 210, 270, 550), secondary_feature),
          }),
          IdlePrimary());
      EXPECT_EQ(merged.SemanticIdForRawTrack(2), 2);
    }

    const auto split = manager.Update(
        vision_demo_host::TrackletObservationsFromTracks({
            MakePersonTrack(2, cv::Rect2f(646, 250, 139, 463), primary_feature),
            MakePersonTrack(7, cv::Rect2f(736, 204, 177, 555), secondary_feature),
        }),
        {},
        IdlePrimary());
    return std::make_pair(split, manager);
  };

  auto [default_result, default_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  ASSERT_EQ(default_result.SemanticIdForRawTrack(2), 1);
  ASSERT_EQ(default_result.SemanticIdForRawTrack(7), 2);
  EXPECT_NE(FindScoreStage(default_manager.LastScoreDebugRows(), 2, "merged_split_handoff"), nullptr);
  EXPECT_EQ(FindEvent(default_manager.LastPhase3ShadowDebugRows(), "phase4_merged_split_handoff", 7), nullptr);

  cfg.enable_phase4_merged_split_handoff = true;
  auto [migrated_result, migrated_manager] = run_sequence(vision_demo_host::IdentityManager(cfg));
  EXPECT_EQ(migrated_result.SemanticIdForRawTrack(2), default_result.SemanticIdForRawTrack(2));
  EXPECT_EQ(migrated_result.SemanticIdForRawTrack(7), default_result.SemanticIdForRawTrack(7));
  EXPECT_EQ(FindScoreStage(migrated_manager.LastScoreDebugRows(), 2, "merged_split_handoff"), nullptr);

  const auto *migrated_row =
      FindEvent(migrated_manager.LastPhase3ShadowDebugRows(), "phase4_merged_split_handoff", 7);
  ASSERT_NE(migrated_row, nullptr);
  EXPECT_EQ(migrated_row->candidate_semantic_id, 2);
  EXPECT_EQ(migrated_row->carrier_raw_track_id, 2);
  EXPECT_EQ(migrated_row->carrier_semantic_id, 1);
  EXPECT_EQ(migrated_row->reason, "merged_split_handoff");
  EXPECT_EQ(migrated_row->hypothesis_status, "tracked");
  EXPECT_EQ(migrated_row->candidate_stable_frames, 1);

  auto [no_shadow_result, no_shadow_manager] =
      run_sequence_without_shadow_candidate(vision_demo_host::IdentityManager(cfg));
  (void)no_shadow_result;
  EXPECT_EQ(FindEvent(no_shadow_manager.LastPhase3ShadowDebugRows(), "phase4_merged_split_handoff", 7), nullptr);
  EXPECT_EQ(FindScoreStage(no_shadow_manager.LastScoreDebugRows(), 2, "phase4_merged_split_handoff"), nullptr);
}

TEST(IdentityManagerTest, DefaultMissingWindowKeepsIdentityOccludedAcrossFourSecondAbsence) {
  vision_demo_host::IdentityManager manager;

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}),
      IdlePrimary());
  ASSERT_EQ(first.identities.size(), 1U);
  EXPECT_EQ(first.identities.front().state, vision_demo_host::IdentityState::kActive);

  vision_demo_host::IdentityManagerResult missing_result;
  for (int i = 0; i < 120; ++i) {
    missing_result = manager.Update({}, IdlePrimary());
  }

  ASSERT_EQ(missing_result.identities.size(), 1U);
  EXPECT_EQ(missing_result.identities.front().semantic_id, 1);
  EXPECT_EQ(missing_result.identities.front().state, vision_demo_host::IdentityState::kOccluded);
  EXPECT_EQ(missing_result.identities.front().missing_frames, 120);
}

TEST(IdentityManagerTest, EmitsOccludedAndLostIdentityLifecycleWithoutVisibleTracklet) {
  vision_demo_host::IdentityManager::Config cfg;
  cfg.max_missing_frames = 1;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityManager manager(cfg);

  const auto first = manager.Update(
      vision_demo_host::TrackletObservationsFromTracks({MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}),
      IdlePrimary());
  ASSERT_EQ(first.identities.size(), 1U);
  EXPECT_EQ(first.identities.front().state, vision_demo_host::IdentityState::kActive);

  const auto second = manager.Update({}, IdlePrimary());
  ASSERT_EQ(second.identities.size(), 1U);
  EXPECT_EQ(second.identities.front().semantic_id, 1);
  EXPECT_EQ(second.identities.front().state, vision_demo_host::IdentityState::kOccluded);
  EXPECT_FALSE(second.identities.front().visible);
  EXPECT_FALSE(second.identities.front().supporting_raw_track_id.has_value());
  EXPECT_EQ(second.identities.front().missing_frames, 1);

  const auto third = manager.Update({}, IdlePrimary());
  ASSERT_EQ(third.identities.size(), 1U);
  EXPECT_EQ(third.identities.front().semantic_id, 1);
  EXPECT_EQ(third.identities.front().state, vision_demo_host::IdentityState::kLost);
  EXPECT_EQ(third.identities.front().missing_frames, 2);
}
