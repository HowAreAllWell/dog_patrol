#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "identity_runtime_mutation_applier.hpp"
#include "identity_assignment_engine_adapter.hpp"

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

vision_demo_host::PrimaryTargetResult LockedPrimary(const int semantic_id) {
  vision_demo_host::PrimaryTargetResult primary;
  primary.state = vision_demo_host::PrimaryState::kLocked;
  primary.primary_target_id = semantic_id;
  return primary;
}

const vision_demo_host::IdentityAssignmentEngineAdapter::ScoreDebugRow *FindDebugRow(
    const std::vector<vision_demo_host::IdentityAssignmentEngineAdapter::ScoreDebugRow> &rows,
    const int raw_id, const std::string &stage) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.raw_track_id == raw_id && row.stage == stage;
  });
  return it == rows.end() ? nullptr : &(*it);
}

bool HasSelectedRejectReason(
    const std::vector<vision_demo_host::IdentityAssignmentEngineAdapter::ScoreDebugRow> &rows,
    const std::string &stage, const std::string &reason) {
  return std::any_of(rows.begin(), rows.end(), [&](const auto &row) {
    return row.stage == stage && row.selected && !row.accepted && row.reject_reason == reason;
  });
}

std::vector<vision_demo_host::IdentityRuntimeSnapshot> MakeTwoIdentitySeed(
    vision_demo_host::IdentityAssignmentEngineAdapter *assigner) {
  const auto initial = assigner->Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(300, 0, 100, 300), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  EXPECT_EQ(initial.at(1), 1);
  EXPECT_EQ(initial.at(2), 2);
  return assigner->IdentitySnapshots();
}

vision_demo_host::IdentityRuntimeMutationApplier MakeMutationApplier(
    vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState *runtime_state,
    vision_demo_host::AppearanceFeatureService *appearance_features) {
  return vision_demo_host::IdentityRuntimeMutationApplier(
      vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, runtime_state, appearance_features);
}

}  // namespace

TEST(IdentityAssignmentEngineAdapterTest, RawContinuityCostOverThresholdRejectsDirectInheritance) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.raw_continuity_max_cost = 0.10F;
  cfg.active_assign_max_cost = 0.90F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, IdlePrimary());
  ASSERT_EQ(first.at(7), 1);

  const auto second = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50))}, IdlePrimary());
  EXPECT_EQ(second.at(7), 1);

  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 7, "raw_continuity");
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->accepted);
  EXPECT_FALSE(row->selected);
  EXPECT_TRUE(row->continuity_used);
  EXPECT_EQ(row->reject_reason, "raw_continuity_max_cost_reject");
}

TEST(IdentityAssignmentEngineAdapterTest, AdapterCanOperateOnExternallyOwnedRuntimeState) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter writer(
      vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &runtime_state);
  vision_demo_host::IdentityAssignmentEngineAdapter reader(
      vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &runtime_state);

  const auto first = writer.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, LockedPrimary(1));
  ASSERT_EQ(first.at(7), 1);

  EXPECT_EQ(reader.SemanticIdForRawTrack(7), 1);
  EXPECT_EQ(reader.CurrentPrimarySemanticId(), 1);
  ASSERT_EQ(reader.IdentitySnapshots().size(), 1U);
  EXPECT_EQ(reader.LastScoreDebugRows().size(), writer.LastScoreDebugRows().size());

  vision_demo_host::IdentityAssignmentEngineAdapter::ResetRuntimeState(&runtime_state);

  EXPECT_EQ(reader.SemanticIdForRawTrack(7), -1);
  EXPECT_EQ(reader.CurrentPrimarySemanticId(), -1);
  EXPECT_TRUE(reader.IdentitySnapshots().empty());
  EXPECT_TRUE(reader.LastScoreDebugRows().empty());
}

TEST(IdentityAssignmentEngineAdapterTest, EndFrameLifecycleAgingUpdatesRuntimeSnapshots) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 1;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, IdlePrimary());
  ASSERT_EQ(first.at(7), 1);
  ASSERT_EQ(assigner.IdentitySnapshots().size(), 1U);
  EXPECT_EQ(assigner.IdentitySnapshots().front().missing_frames, 0);
  EXPECT_TRUE(assigner.IdentitySnapshots().front().seen_this_frame);

  const auto second = assigner.Update({}, IdlePrimary());
  EXPECT_TRUE(second.empty());
  ASSERT_EQ(assigner.IdentitySnapshots().size(), 1U);
  EXPECT_EQ(assigner.IdentitySnapshots().front().semantic_id, 1);
  EXPECT_EQ(assigner.IdentitySnapshots().front().missing_frames, 1);
  EXPECT_FALSE(assigner.IdentitySnapshots().front().seen_this_frame);
  EXPECT_EQ(assigner.IdentitySnapshots().front().supporting_raw_track_id, -1);

  assigner.Update({}, IdlePrimary());
  ASSERT_EQ(assigner.IdentitySnapshots().size(), 1U);
  EXPECT_EQ(assigner.IdentitySnapshots().front().missing_frames, 2);
}

TEST(IdentityAssignmentEngineAdapterTest, AssignmentCostOverMaxRejectsAndAllocatesNewSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.active_assign_max_cost = 0.10F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, IdlePrimary());
  ASSERT_EQ(first.at(7), 1);

  const auto second = assigner.Update(
      {MakePersonTrack(8, cv::Rect2f(500, 0, 50, 50))}, IdlePrimary());
  ASSERT_EQ(second.count(8), 1U);
  EXPECT_EQ(second.at(8), 2);
  EXPECT_TRUE(HasSelectedRejectReason(assigner.LastScoreDebugRows(), "assign_candidate",
                                      "active_assign_max_cost_reject"));
}

TEST(IdentityAssignmentEngineAdapterTest, RecentlyMissingActiveIdentityUsesAssignmentMarginCushion) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.max_missing_frames = 100;
  cfg.app_w = 0.70F;
  cfg.geo_w = 0.20F;
  cfg.time_w = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto candidate = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 200, 200)),
       MakePersonTrack(7, cv::Rect2f(1349, 872, 40, 116), {})},
      IdlePrimary());
  ASSERT_EQ(candidate.at(1), 1);
  ASSERT_EQ(candidate.count(7), 0U);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 200, 200)),
       MakePersonTrack(7, cv::Rect2f(1350, 873, 40, 116), {})},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(7), 2);

  for (int i = 0; i < 74; ++i) {
    const auto keep_primary = assigner.Update({MakePersonTrack(1, cv::Rect2f(0, 0, 200, 200))}, IdlePrimary());
    ASSERT_EQ(keep_primary.at(1), 1);
  }

  const auto recovered = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 200, 200)),
       MakePersonTrack(9, cv::Rect2f(1298, 827, 40, 125), {})},
      IdlePrimary());

  ASSERT_EQ(recovered.count(9), 1U);
  EXPECT_EQ(recovered.at(9), 2);
}

TEST(IdentityAssignmentEngineAdapterTest, ShortMissingUsesGeometryWhenAppearanceIsWeak) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(12, cv::Rect2f(1298, 960, 70, 196), {1.0F, 0.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(12), 1);

  for (int i = 0; i < 30; ++i) {
    assigner.Update({}, IdlePrimary());
  }

  const auto recovered = assigner.Update(
      {MakePersonTrack(13, cv::Rect2f(1305, 970, 70, 196), {0.35F, 0.93675F, 0.0F})},
      IdlePrimary());

  ASSERT_EQ(recovered.count(13), 1U);
  EXPECT_EQ(recovered.at(13), 1);
  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 13, "assign_candidate");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->accepted);
  EXPECT_EQ(row->semantic_id, 1);
}

TEST(IdentityAssignmentEngineAdapterTest, RecentlyMissingActiveIdentityAllowsStrongAppearanceSmallAreaRecovery) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_min_area_ratio = 0.40F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F})},
      LockedPrimary(1));
  ASSERT_EQ(first.at(1), 1);

  for (int i = 0; i < 50; ++i) {
    assigner.Update({}, LockedPrimary(1));
  }

  const auto recovered = assigner.Update(
      {MakePersonTrack(2, cv::Rect2f(0, 0, 150, 1500), {1.0F, 0.0F, 0.0F})},
      LockedPrimary(1));

  ASSERT_EQ(recovered.count(2), 1U);
  EXPECT_EQ(recovered.at(2), 1);
  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 2, "assign_candidate");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->accepted);
  EXPECT_EQ(row->semantic_id, 1);
}

TEST(IdentityAssignmentEngineAdapterTest, InsufficientAssignmentMarginRejectsAndAllocatesNewSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.app_w = 1.0F;
  cfg.geo_w = 0.0F;
  cfg.time_w = 0.0F;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.20F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50), {}),
       MakePersonTrack(20, cv::Rect2f(200, 0, 50, 50), {})},
      IdlePrimary());
  ASSERT_EQ(first.at(10), 1);
  ASSERT_EQ(first.at(20), 2);

  const auto second = assigner.Update(
      {MakePersonTrack(30, cv::Rect2f(100, 0, 50, 50), {})}, IdlePrimary());
  ASSERT_EQ(second.count(30), 1U);
  EXPECT_EQ(second.at(30), 3);
  EXPECT_TRUE(HasSelectedRejectReason(assigner.LastScoreDebugRows(), "assign_candidate",
                                      "assignment_margin_reject"));
}

TEST(IdentityAssignmentEngineAdapterTest, AmbiguousRecentlyMissingAssignmentDoesNotBirthNewSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.app_w = 1.0F;
  cfg.geo_w = 0.0F;
  cfg.time_w = 0.0F;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.20F;
  cfg.max_missing_frames = 100;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(10, cv::Rect2f(0, 0, 50, 50), {}),
       MakePersonTrack(20, cv::Rect2f(200, 0, 50, 50), {})},
      IdlePrimary());
  ASSERT_EQ(first.at(10), 1);
  ASSERT_EQ(first.at(20), 2);

  for (int i = 0; i < 10; ++i) {
    assigner.Update({}, IdlePrimary());
  }

  const auto ambiguous = assigner.Update(
      {MakePersonTrack(30, cv::Rect2f(100, 0, 50, 50), {})}, IdlePrimary());
  EXPECT_EQ(ambiguous.count(30), 0U);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(30), -1);
  EXPECT_TRUE(HasSelectedRejectReason(assigner.LastScoreDebugRows(), "assign_candidate",
                                      "assignment_margin_reject"));

  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 30, "phase5_birth_candidate");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->semantic_id, -1);
  EXPECT_EQ(row->reject_reason, "ambiguous_recovery_pending");
}

TEST(IdentityAssignmentEngineAdapterTest, PairwiseAppearanceEmitsEvidenceWithoutLegacyApply) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.stable_frames_before_feature_update = 1;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.96F, 0.28F};
  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100), primary_feature),
       MakePersonTrack(2, cv::Rect2f(200, 0, 100, 100), secondary_feature)},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);

  for (int i = 0; i < 10; ++i) {
    assigner.Update({}, IdlePrimary());
  }

  const auto recovered = assigner.Update(
      {MakePersonTrack(30, cv::Rect2f(80, 0, 100, 100), secondary_feature),
       MakePersonTrack(40, cv::Rect2f(120, 0, 100, 100), primary_feature)},
      IdlePrimary());

  ASSERT_EQ(recovered.count(30), 1U);
  ASSERT_EQ(recovered.count(40), 1U);
  EXPECT_NE(recovered.at(30), 2);
  EXPECT_NE(recovered.at(40), 1);
  ASSERT_EQ(assigner.LastPairwiseAssignmentDebugRows().size(), 1U);
  const auto &row = assigner.LastPairwiseAssignmentDebugRows().front();
  EXPECT_EQ(row.selected_pairs, "30->1|40->2");
  EXPECT_EQ(row.alternate_pairs, "30->2|40->1");
  EXPECT_TRUE(row.appearance_override);
}

TEST(IdentityAssignmentEngineAdapterTest, SkinnyPartialNewTrackIsHiddenInsteadOfAllocatingSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(800, 0, 700, 1500))}, IdlePrimary());
  ASSERT_EQ(first.at(1), 1);

  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(800, 0, 700, 1500)),
       MakePersonTrack(9, cv::Rect2f(340, 541, 142, 762))},
      IdlePrimary());

  ASSERT_EQ(second.count(1), 1U);
  EXPECT_EQ(second.at(1), 1);
  EXPECT_EQ(second.count(9), 0U);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(9), -1);

  const auto snapshots = assigner.IdentitySnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].semantic_id, 1);

  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 9, "phase5_birth_candidate");
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->accepted);
  EXPECT_EQ(row->semantic_id, -1);
  EXPECT_EQ(row->reject_reason, "skinny_partial_hidden");
}

TEST(IdentityAssignmentEngineAdapterTest, WideLowHeightFragmentNewTrackIsHiddenInsteadOfAllocatingSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(14, cv::Rect2f(2300, 180, 300, 1210))}, IdlePrimary());
  ASSERT_EQ(first.at(14), 1);

  const auto second = assigner.Update(
      {MakePersonTrack(14, cv::Rect2f(2300, 180, 300, 1210)),
       MakePersonTrack(15, cv::Rect2f(1589, 1278, 447, 240))},
      IdlePrimary());

  ASSERT_EQ(second.count(14), 1U);
  EXPECT_EQ(second.at(14), 1);
  EXPECT_EQ(second.count(15), 0U);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(15), -1);

  const auto snapshots = assigner.IdentitySnapshots();
  ASSERT_EQ(snapshots.size(), 1U);
  EXPECT_EQ(snapshots[0].semantic_id, 1);
}

TEST(IdentityAssignmentEngineAdapterTest, ActiveDuplicateSplitTrackIsHiddenInsteadOfAllocatingSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(10, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(10), 2);

  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(10, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(13, cv::Rect2f(1331, 1065, 506, 451), {0.0F, 0.9F, 0.1F})},
      IdlePrimary());

  ASSERT_EQ(second.count(1), 1U);
  ASSERT_EQ(second.count(10), 1U);
  EXPECT_EQ(second.at(1), 1);
  EXPECT_EQ(second.at(10), 2);
  EXPECT_EQ(second.count(13), 0U);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(13), -1);

  const auto snapshots = assigner.IdentitySnapshots();
  ASSERT_EQ(snapshots.size(), 2U);
  EXPECT_EQ(snapshots[0].semantic_id, 1);
  EXPECT_EQ(snapshots[1].semantic_id, 2);
}

TEST(IdentityAssignmentEngineAdapterTest, DuplicateSplitTrackCanContinueExistingIdentityWhenMainRawDisappears) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(10, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(10), 2);

  const auto duplicate_hidden = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(10, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(13, cv::Rect2f(1331, 1065, 506, 451), {0.0F, 0.9F, 0.1F})},
      IdlePrimary());
  ASSERT_EQ(duplicate_hidden.count(13), 0U);

  const auto continuation = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(13, cv::Rect2f(1331, 1065, 506, 451), {0.0F, 0.9F, 0.1F})},
      IdlePrimary());

  ASSERT_EQ(continuation.count(13), 1U);
  EXPECT_EQ(continuation.at(13), 2);
}

TEST(IdentityAssignmentEngineAdapterTest, SmallStableNewTrackPromotesAfterQuickConfirmation) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500))}, IdlePrimary());
  ASSERT_EQ(first.at(1), 1);

  const auto hidden = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(12, cv::Rect2f(1329, 974, 58, 146), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(hidden.count(1), 1U);
  EXPECT_EQ(hidden.count(12), 0U);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(12), -1);

  const auto promoted = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(12, cv::Rect2f(1332, 976, 58, 146), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());

  ASSERT_EQ(promoted.count(12), 1U);
  EXPECT_EQ(promoted.at(12), 2);
}

TEST(IdentityAssignmentEngineAdapterTest, HiddenCandidatesDoNotConsumeSemanticIdsBeforeFullBodyEdgeNewcomer) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto initial = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(initial.at(1), 1);
  ASSERT_EQ(initial.at(2), 2);

  const auto small_hidden = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(1664, 805, 50, 170), {0.0F, 0.0F, 1.0F})},
      IdlePrimary());
  ASSERT_EQ(small_hidden.count(8), 0U);

  const auto small_promoted = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(1666, 805, 50, 170), {0.0F, 0.0F, 1.0F})},
      IdlePrimary());
  ASSERT_EQ(small_promoted.at(8), 3);

  const auto reflection_hidden = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(1666, 805, 50, 170), {0.0F, 0.0F, 1.0F}),
       MakePersonTrack(9, cv::Rect2f(340, 541, 142, 762))},
      IdlePrimary());
  ASSERT_EQ(reflection_hidden.count(9), 0U);

  const auto duplicate_hidden = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(1666, 805, 50, 170), {0.0F, 0.0F, 1.0F}),
       MakePersonTrack(13, cv::Rect2f(1331, 1065, 506, 451), {0.0F, 0.9F, 0.1F})},
      IdlePrimary());
  ASSERT_EQ(duplicate_hidden.count(13), 0U);

  const auto newcomer = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500)),
       MakePersonTrack(2, cv::Rect2f(824, 1067, 1013, 447), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(1666, 805, 50, 170), {0.0F, 0.0F, 1.0F}),
       MakePersonTrack(14, cv::Rect2f(2509, 150, 178, 1270), {0.5F, 0.5F, 0.707F})},
      IdlePrimary());

  ASSERT_EQ(newcomer.count(14), 1U);
  EXPECT_EQ(newcomer.at(14), 4);
}

TEST(IdentityAssignmentEngineAdapterTest, OverlapFreezesFeatureAndGeometryUpdates) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);

  const auto mapping = assigner.Update(
      {MakePersonTrack(10, cv::Rect2f(0, 0, 80, 80)),
       MakePersonTrack(20, cv::Rect2f(20, 20, 80, 80))},
      IdlePrimary());
  ASSERT_EQ(mapping.size(), 2U);
  EXPECT_TRUE(assigner.IsFeatureUpdateFrozen());

  const auto *new_row = FindDebugRow(assigner.LastScoreDebugRows(), 20, "phase5_new_semantic");
  ASSERT_NE(new_row, nullptr);
  EXPECT_TRUE(new_row->accepted);
  EXPECT_FALSE(new_row->feature_update_allowed);
  EXPECT_FALSE(new_row->geometry_update_allowed);
  EXPECT_EQ(new_row->feature_update_reason, "overlapping_track_freeze");
  EXPECT_EQ(new_row->geometry_update_reason, "overlapping_track_freeze");
}

TEST(IdentityAssignmentEngineAdapterTest, UpdatePolicyReasonsExplainAllowedStableAndRejectedUpdates) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.active_assign_max_cost = 0.90F;
  cfg.raw_continuity_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  cfg.stable_frames_before_feature_update = 3;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(300, 0, 100, 300), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(7), 1);
  ASSERT_EQ(first.at(8), 2);
  const auto *new_row = FindDebugRow(assigner.LastScoreDebugRows(), 8, "phase5_new_semantic");
  ASSERT_NE(new_row, nullptr);
  EXPECT_TRUE(new_row->feature_update_allowed);
  EXPECT_TRUE(new_row->geometry_update_allowed);
  EXPECT_EQ(new_row->feature_update_reason, "insufficient_stable_frames");
  EXPECT_EQ(new_row->geometry_update_reason, "allowed_update");

  const auto second = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(1, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(301, 0, 100, 300), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(second.at(8), 2);
  const auto *stable_wait_row = FindDebugRow(assigner.LastScoreDebugRows(), 8, "raw_continuity");
  ASSERT_NE(stable_wait_row, nullptr);
  EXPECT_TRUE(stable_wait_row->feature_update_allowed);
  EXPECT_TRUE(stable_wait_row->geometry_update_allowed);
  EXPECT_EQ(stable_wait_row->feature_update_reason, "allowed_update");
  EXPECT_EQ(stable_wait_row->geometry_update_reason, "allowed_update");

  auto weak_newcomer = MakePersonTrack(10, cv::Rect2f(500, 0, 100, 400), {0.0F, 0.0F, 1.0F});
  weak_newcomer.low_score_update = true;
  weak_newcomer.association.low_score_detection = true;
  const auto third = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(2, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(8, cv::Rect2f(302, 0, 100, 300), {0.0F, 1.0F, 0.0F}), weak_newcomer},
      IdlePrimary());
  ASSERT_EQ(third.at(8), 2);
  ASSERT_EQ(third.at(10), 3);
  const auto *weak_row = FindDebugRow(assigner.LastScoreDebugRows(), 10, "phase5_new_semantic");
  ASSERT_NE(weak_row, nullptr);
  EXPECT_TRUE(weak_row->accepted);
  EXPECT_FALSE(weak_row->feature_update_allowed);
  EXPECT_FALSE(weak_row->geometry_update_allowed);
  EXPECT_EQ(weak_row->feature_update_reason, "unreliable_low_quality_observation");
  EXPECT_EQ(weak_row->geometry_update_reason, "unreliable_low_quality_observation");

  vision_demo_host::IdentityAssignmentEngineAdapter::Config reject_cfg;
  reject_cfg.active_assign_max_cost = 0.10F;
  reject_cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState rejecter_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter rejecter(reject_cfg, &rejecter_runtime_state);
  ASSERT_EQ(rejecter.Update({MakePersonTrack(1, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary()).at(1), 1);
  const auto rejected = rejecter.Update(
      {MakePersonTrack(9, cv::Rect2f(900, 0, 100, 300), {0.0F, 0.0F, 1.0F})},
      IdlePrimary());
  ASSERT_EQ(rejected.at(9), 2);
  const auto *rejected_row = FindDebugRow(rejecter.LastScoreDebugRows(), 9, "assign_candidate");
  ASSERT_NE(rejected_row, nullptr);
  ASSERT_FALSE(rejected_row->accepted);
  EXPECT_EQ(rejected_row->feature_update_reason, "update_blocked_by_rejected_assignment");
  EXPECT_EQ(rejected_row->geometry_update_reason, "update_blocked_by_rejected_assignment");
}

TEST(IdentityAssignmentEngineAdapterTest, UnreliableObservationDoesNotOverwriteReliableGeometry) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.raw_continuity_max_cost = 0.15F;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(0, 0, 50, 50))}, IdlePrimary());
  ASSERT_EQ(first.at(7), 1);

  auto weak = MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50));
  weak.low_score_update = true;
  weak.association.low_score_detection = true;
  const auto second = assigner.Update({weak}, IdlePrimary());
  ASSERT_EQ(second.at(7), 1);

  const auto third = assigner.Update(
      {MakePersonTrack(7, cv::Rect2f(500, 0, 50, 50))}, IdlePrimary());
  EXPECT_EQ(third.at(7), 1);

  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 7, "raw_continuity");
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->accepted);
  EXPECT_EQ(row->reject_reason, "raw_continuity_max_cost_reject");
}

TEST(IdentityAssignmentEngineAdapterTest, SnapshotKeepsReliableBBoxSeparateFromFrozenActiveBBox) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.raw_continuity_max_cost = 0.90F;
  cfg.active_assign_max_cost = 0.90F;
  cfg.min_assignment_margin = 0.0F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update({MakePersonTrack(7, cv::Rect2f(0, 0, 100, 300))}, IdlePrimary());
  ASSERT_EQ(first.at(7), 1);

  auto weak = MakePersonTrack(7, cv::Rect2f(10, 0, 100, 300));
  weak.low_score_update = true;
  weak.association.low_score_detection = true;
  const auto frozen = assigner.Update({weak}, IdlePrimary());
  ASSERT_EQ(frozen.at(7), 1);

  const auto snapshots = assigner.IdentitySnapshots();
  const auto primary_it = std::find_if(snapshots.begin(), snapshots.end(), [](const auto &snapshot) {
    return snapshot.semantic_id == 1;
  });
  ASSERT_NE(primary_it, snapshots.end());
  EXPECT_FLOAT_EQ(primary_it->bbox.x, 10.0F);
  EXPECT_FLOAT_EQ(primary_it->reliable_bbox.x, 0.0F);
  EXPECT_TRUE(primary_it->has_reliable_geometry);
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobKeepsRawContinuityWhenAlternativeOnlySlightlyBetter) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.app_w = 0.0F;
  cfg.geo_w = 1.0F;
  cfg.time_w = 0.0F;
  cfg.min_assignment_margin = 0.08F;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100)),
       MakePersonTrack(2, cv::Rect2f(20, 0, 100, 100), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);
  ASSERT_TRUE(assigner.IsFeatureUpdateFrozen());

  const auto merged = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(11, 0, 100, 100), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));

  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);
  ASSERT_EQ(merged.count(1), 1U);
  EXPECT_EQ(merged.at(1), 1);
  EXPECT_TRUE(std::any_of(assigner.LastScoreDebugRows().begin(), assigner.LastScoreDebugRows().end(),
                          [](const auto &row) {
                            return row.accepted && row.feature_update_reason == "global_merge_split_freeze" &&
                                   row.geometry_update_reason == "global_merge_split_freeze";
                          }));
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobRejectsMissingPrimaryWhenAppearanceIsPoor) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F})},
      LockedPrimary(1));
  ASSERT_EQ(first.at(1), 1);

  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));
  ASSERT_EQ(second.at(1), 1);
  ASSERT_EQ(second.at(2), 2);

  const auto overlapped = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(1200, 0, 500, 1000), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(1250, 0, 500, 1000), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));
  ASSERT_EQ(overlapped.at(1), 1);
  ASSERT_EQ(overlapped.at(2), 2);
  ASSERT_TRUE(assigner.IsFeatureUpdateFrozen());

  for (int i = 0; i < 120; ++i) {
    const auto only_other = assigner.Update(
        {MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F})},
        LockedPrimary(1));
    ASSERT_EQ(only_other.at(2), 2);
  }
  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);

  const auto newcomer = assigner.Update(
      {MakePersonTrack(14, cv::Rect2f(2509, 150, 178, 1270), {0.4F, 0.0F, 0.916515F})},
      LockedPrimary(1));

  ASSERT_EQ(newcomer.count(14), 1U);
  EXPECT_GT(newcomer.at(14), 2);
  const auto &rows = assigner.LastScoreDebugRows();
  EXPECT_TRUE(std::any_of(rows.begin(), rows.end(), [](const auto &row) {
    return row.semantic_id == 1 && row.reject_reason == "missing_appearance_gate_reject";
  }));
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobRejectsMissingPrimaryWhenAreaShrinksTooMuch) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_min_area_ratio = 0.40F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  ASSERT_EQ(assigner.Update(
                {MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F})},
                LockedPrimary(1))
                .at(1),
            1);
  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(2469, 5, 215, 1503), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));
  ASSERT_EQ(second.at(1), 1);
  ASSERT_EQ(second.at(2), 2);
  assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(1200, 0, 500, 1000), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(1250, 0, 500, 1000), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));

  for (int i = 0; i < 120; ++i) {
    ASSERT_EQ(assigner.Update(
                  {MakePersonTrack(2, cv::Rect2f(826, 1064, 1018, 449), {0.0F, 1.0F, 0.0F})},
                  LockedPrimary(1))
                  .at(2),
              2);
  }

  const auto fragment = assigner.Update(
      {MakePersonTrack(15, cv::Rect2f(1589, 1278, 447, 240), {0.6F, 0.0F, 0.8F})},
      LockedPrimary(1));

  ASSERT_EQ(fragment.count(15), 1U);
  EXPECT_GT(fragment.at(15), 2);
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobKeepsNonPrimaryRawContinuityWhenPrimaryIsOnlySlightlyWorse) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.app_w = 0.0F;
  cfg.geo_w = 1.0F;
  cfg.time_w = 0.0F;
  cfg.min_assignment_margin = 0.08F;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 100, 100)),
       MakePersonTrack(2, cv::Rect2f(20, 0, 100, 100), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);

  const auto merged = assigner.Update(
      {MakePersonTrack(2, cv::Rect2f(11, 0, 100, 100), {0.0F, 1.0F, 0.0F})},
      LockedPrimary(1));

  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);
  ASSERT_EQ(merged.count(2), 1U);
  EXPECT_EQ(merged.at(2), 2);
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobKeepsContinuityForPhase4HandoffCoordinator) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
       MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);

  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(320, 220, 240, 620), primary_feature),
       MakePersonTrack(2, cv::Rect2f(680, 160, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(second.at(1), 1);
  ASSERT_EQ(second.at(2), 2);

  const auto overlap = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
       MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(overlap.at(1), 1);
  ASSERT_EQ(overlap.at(2), 2);

  for (int i = 0; i < 18; ++i) {
    const auto merged_other = assigner.Update(
        {MakePersonTrack(2, cv::Rect2f(590, 170, 220, 600), secondary_feature)},
        LockedPrimary(1));
    ASSERT_EQ(merged_other.at(2), 2);
  }

  const auto handoff = assigner.Update(
      {MakePersonTrack(2, cv::Rect2f(560, 240, 200, 560), primary_feature)},
      LockedPrimary(1));

  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);
  ASSERT_EQ(handoff.count(2), 1U);
  EXPECT_EQ(handoff.at(2), 2);
  EXPECT_NE(std::find_if(assigner.LastScoreDebugRows().begin(), assigner.LastScoreDebugRows().end(),
                         [](const auto &row) {
                           return row.raw_track_id == 2 && row.semantic_id == 2 &&
                                  row.stage == "merged_candidate" && row.accepted;
                         }),
            assigner.LastScoreDebugRows().end());
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSplitHandoffNoLongerAppliesInsideLegacyUpdate) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
       MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);

  const auto overlap = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
       MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(overlap.at(1), 1);
  ASSERT_EQ(overlap.at(2), 2);

  for (int i = 0; i < 18; ++i) {
    const auto merged_other = assigner.Update(
        {MakePersonTrack(2, cv::Rect2f(585, 210, 270, 550), secondary_feature)},
        LockedPrimary(1));
    ASSERT_EQ(merged_other.at(2), 2);
  }

  const auto split = assigner.Update(
      {MakePersonTrack(2, cv::Rect2f(646, 250, 139, 463), primary_feature),
       MakePersonTrack(7, cv::Rect2f(736, 204, 177, 555), secondary_feature)},
      LockedPrimary(1));

  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);
  ASSERT_EQ(split.count(2), 1U);
  ASSERT_EQ(split.count(7), 1U);
  EXPECT_EQ(split.at(2), 1);
  EXPECT_EQ(split.at(7), 2);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 2, "merged_split_handoff"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, EarlyMergedSplitHandoffNoLongerAppliesInsideLegacyUpdate) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const auto first = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
       MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(first.at(1), 1);
  ASSERT_EQ(first.at(2), 2);

  const auto overlap = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
       MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(overlap.at(1), 1);
  ASSERT_EQ(overlap.at(2), 2);

  for (int i = 0; i < 17; ++i) {
    const auto merged_other = assigner.Update(
        {MakePersonTrack(2, cv::Rect2f(585, 210, 270, 550), secondary_feature)},
        LockedPrimary(1));
    ASSERT_EQ(merged_other.at(2), 2);
  }

  const auto split = assigner.Update(
      {MakePersonTrack(2, cv::Rect2f(629, 247, 176, 498), primary_feature),
       MakePersonTrack(7, cv::Rect2f(717, 210, 164, 551), secondary_feature)},
      LockedPrimary(1));

  ASSERT_EQ(assigner.CurrentMode(), vision_demo_host::IdentityLifecycleMode::kMerged);
  ASSERT_EQ(split.count(2), 1U);
  ASSERT_EQ(split.count(7), 1U);
  EXPECT_EQ(split.at(2), 1);
  EXPECT_EQ(split.at(7), 2);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 2, "merged_split_handoff"), nullptr);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 7, "merged_split_handoff"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSideReappearanceNoLongerAppliesInsideLegacyUpdate) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const std::vector<float> side_reappear_feature{0.7F, 0.3F};
  const auto first = assigner.Update(
      {MakePersonTrack(4, cv::Rect2f(330, 0, 300, 900), primary_feature),
       MakePersonTrack(5, cv::Rect2f(610, 70, 180, 680), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(first.at(4), 1);
  ASSERT_EQ(first.at(5), 2);

  const auto overlap = assigner.Update(
      {MakePersonTrack(4, cv::Rect2f(330, 0, 310, 900), primary_feature),
       MakePersonTrack(5, cv::Rect2f(590, 170, 125, 555), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(overlap.at(4), 1);
  ASSERT_EQ(overlap.at(5), 2);

  for (int i = 0; i < 30; ++i) {
    const auto merged = assigner.Update({MakePersonTrack(4, cv::Rect2f(315, 0, 300, 930), primary_feature)},
                                        LockedPrimary(1));
    ASSERT_EQ(merged.at(4), 1);
  }

  const auto recovered = assigner.Update(
      {MakePersonTrack(4, cv::Rect2f(310, 0, 298, 928), primary_feature),
       MakePersonTrack(6, cv::Rect2f(179, 228, 194, 514), side_reappear_feature)},
      LockedPrimary(1));

  ASSERT_EQ(recovered.count(4), 1U);
  EXPECT_EQ(recovered.count(6), 0U);
  EXPECT_EQ(recovered.at(4), 1);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 6, "merged_side_recovery"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4MergedSplitDirectApplyUpdatesTwoTracks) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);

  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(11, cv::Rect2f(10, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
      MakePersonTrack(12, cv::Rect2f(310, 0, 100, 300), {0.0F, 1.0F, 0.0F})};

  ASSERT_TRUE(applier.ApplyPhase4MergedSplitHandoff(tracks, 11, 1, 12, 2));

  EXPECT_EQ(assigner.SemanticIdForRawTrack(11), 1);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(12), 2);
  const auto *first = FindDebugRow(assigner.LastScoreDebugRows(), 11, "phase4_merged_split_handoff");
  const auto *second = FindDebugRow(assigner.LastScoreDebugRows(), 12, "phase4_merged_split_handoff");
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(first->accepted);
  EXPECT_TRUE(first->selected);
  EXPECT_EQ(first->semantic_id, 1);
  EXPECT_TRUE(second->accepted);
  EXPECT_TRUE(second->selected);
  EXPECT_EQ(second->semantic_id, 2);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4MergedSideRecoveryDirectApplyForcesGeometryUpdate) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);

  auto candidate = MakePersonTrack(12, cv::Rect2f(310, 0, 100, 300), {0.0F, 1.0F, 0.0F});
  candidate.low_score_update = true;
  candidate.association.low_score_detection = true;
  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(11, cv::Rect2f(10, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
      candidate};

  ASSERT_TRUE(applier.ApplyPhase4MergedSideRecovery(tracks, 11, 1, 12, 2));

  EXPECT_EQ(assigner.SemanticIdForRawTrack(12), 2);
  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 12, "phase4_merged_side_recovery");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->accepted);
  EXPECT_TRUE(row->selected);
  EXPECT_EQ(row->semantic_id, 2);
  EXPECT_TRUE(row->geometry_update_allowed);
  EXPECT_EQ(row->geometry_update_reason, "allowed_update");
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4MergedSingleBlobDirectApplyMarksCarrierMissing) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);

  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(11, cv::Rect2f(10, 0, 100, 300), {0.0F, 1.0F, 0.0F})};

  ASSERT_TRUE(applier.ApplyPhase4MergedSingleBlobHandoff(tracks, 11, 1, 2));

  EXPECT_EQ(assigner.SemanticIdForRawTrack(11), 2);
  const auto *row = FindDebugRow(assigner.LastScoreDebugRows(), 11,
                                 "phase4_merged_single_blob_handoff");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->accepted);
  EXPECT_EQ(row->semantic_id, 2);

  const auto snapshots = assigner.IdentitySnapshots();
  const auto carrier = std::find_if(snapshots.begin(), snapshots.end(), [](const auto &snapshot) {
    return snapshot.semantic_id == 1;
  });
  ASSERT_NE(carrier, snapshots.end());
  EXPECT_EQ(carrier->missing_frames, 1);
  EXPECT_FALSE(carrier->seen_this_frame);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4PairwiseDirectApplyUpdatesTwoTracks) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);

  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(21, cv::Rect2f(10, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
      MakePersonTrack(22, cv::Rect2f(310, 0, 100, 300), {0.0F, 1.0F, 0.0F})};

  ASSERT_TRUE(applier.ApplyPhase4PairwiseAssignment(tracks, 21, 1, 22, 2));

  EXPECT_EQ(assigner.SemanticIdForRawTrack(21), 1);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(22), 2);
  EXPECT_NE(FindDebugRow(assigner.LastScoreDebugRows(), 21, "phase4_pairwise_assignment"), nullptr);
  EXPECT_NE(FindDebugRow(assigner.LastScoreDebugRows(), 22, "phase4_pairwise_assignment"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4DirectApplyRejectsInvalidRawId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);
  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(21, cv::Rect2f(10, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
      MakePersonTrack(22, cv::Rect2f(310, 0, 100, 300), {0.0F, 1.0F, 0.0F})};

  EXPECT_FALSE(applier.ApplyPhase4PairwiseAssignment(tracks, 99, 1, 22, 2));
  EXPECT_EQ(assigner.SemanticIdForRawTrack(99), -1);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 99, "phase4_pairwise_assignment"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4DirectApplyRejectsInvalidSemanticId) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  MakeTwoIdentitySeed(&assigner);
  const std::vector<vision_demo_host::Track> tracks = {
      MakePersonTrack(21, cv::Rect2f(10, 0, 100, 300), {1.0F, 0.0F, 0.0F}),
      MakePersonTrack(22, cv::Rect2f(310, 0, 100, 300), {0.0F, 1.0F, 0.0F})};

  EXPECT_FALSE(applier.ApplyPhase4PairwiseAssignment(tracks, 21, 1, 22, 99));
  EXPECT_EQ(assigner.SemanticIdForRawTrack(21), -1);
  EXPECT_EQ(assigner.SemanticIdForRawTrack(22), -1);
  EXPECT_EQ(FindDebugRow(assigner.LastScoreDebugRows(), 22, "phase4_pairwise_assignment"), nullptr);
}

TEST(IdentityAssignmentEngineAdapterTest, Phase4DirectApplyErasesAcceptedBirthCandidate) {
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(vision_demo_host::IdentityAssignmentEngineAdapter::Config{}, &assigner_runtime_state);
  vision_demo_host::AppearanceFeatureService mutation_features;
  auto applier = MakeMutationApplier(&assigner_runtime_state, &mutation_features);
  const auto initial = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(initial.at(1), 1);

  const auto pending = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(30, cv::Rect2f(1329, 974, 58, 146), {0.0F, 0.0F, 1.0F})},
      IdlePrimary());
  EXPECT_EQ(pending.count(30), 0U);

  const auto second_identity = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(700, 0, 500, 1500), {0.0F, 1.0F, 0.0F})},
      IdlePrimary());
  ASSERT_EQ(second_identity.at(2), 2);

  const std::vector<vision_demo_host::Track> applied_tracks = {
      MakePersonTrack(31, cv::Rect2f(10, 0, 500, 1500), {1.0F, 0.0F, 0.0F}),
      MakePersonTrack(30, cv::Rect2f(1329, 974, 58, 146), {0.0F, 1.0F, 0.0F})};
  ASSERT_TRUE(applier.ApplyPhase4MergedSplitHandoff(applied_tracks, 31, 1, 30, 2));

  ASSERT_TRUE(assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(700, 0, 500, 1500), {0.0F, 1.0F, 0.0F})},
      IdlePrimary()).count(30) == 0U);

  const auto small_again = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(0, 0, 500, 1500), {1.0F, 0.0F, 0.0F}),
       MakePersonTrack(2, cv::Rect2f(700, 0, 500, 1500), {0.0F, 1.0F, 0.0F}),
       MakePersonTrack(30, cv::Rect2f(1332, 976, 58, 146), {0.0F, 0.0F, 1.0F})},
      IdlePrimary());
  EXPECT_EQ(small_again.count(30), 0U);
}

TEST(IdentityAssignmentEngineAdapterTest, MergedSingleBlobKeepsContinuityBeforeMissingIdentityIsStableEnough) {
  vision_demo_host::IdentityAssignmentEngineAdapter::Config cfg;
  cfg.max_missing_frames = 180;
  cfg.active_assign_max_cost = 0.55F;
  cfg.min_assignment_margin = 0.08F;
  cfg.missing_assign_max_app_cost = 0.50F;
  cfg.stable_frames_before_feature_update = 1;
  cfg.overlap_iou_freeze = 0.10F;
  vision_demo_host::IdentityAssignmentEngineAdapter::RuntimeState assigner_runtime_state;
  vision_demo_host::IdentityAssignmentEngineAdapter assigner(cfg, &assigner_runtime_state);

  const std::vector<float> primary_feature{1.0F, 0.0F};
  const std::vector<float> secondary_feature{0.0F, 1.0F};
  const std::vector<float> ambiguous_merged_feature{0.8F, 0.6F};
  ASSERT_EQ(assigner.Update(
                {MakePersonTrack(1, cv::Rect2f(300, 220, 240, 620), primary_feature),
                 MakePersonTrack(2, cv::Rect2f(700, 160, 180, 560), secondary_feature)},
                LockedPrimary(1))
                .at(1),
            1);
  const auto second = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(320, 220, 240, 620), primary_feature),
       MakePersonTrack(2, cv::Rect2f(680, 160, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(second.at(1), 1);
  ASSERT_EQ(second.at(2), 2);
  const auto overlap = assigner.Update(
      {MakePersonTrack(1, cv::Rect2f(560, 240, 200, 560), primary_feature),
       MakePersonTrack(2, cv::Rect2f(590, 170, 180, 560), secondary_feature)},
      LockedPrimary(1));
  ASSERT_EQ(overlap.at(1), 1);
  ASSERT_EQ(overlap.at(2), 2);

  for (int i = 0; i < 4; ++i) {
    const auto merged_other = assigner.Update(
        {MakePersonTrack(2, cv::Rect2f(560, 240, 200, 560), ambiguous_merged_feature)},
        LockedPrimary(1));
    ASSERT_EQ(merged_other.count(2), 1U);
    EXPECT_EQ(merged_other.at(2), 2);
  }
}
