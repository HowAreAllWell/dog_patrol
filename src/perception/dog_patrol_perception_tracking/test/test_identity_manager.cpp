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
