#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "unresolved_track_final_resolution_coordinator.hpp"

namespace {

using dog_patrol_perception_tracking::BirthCandidateDecision;
using dog_patrol_perception_tracking::BirthManager;
using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityRuntimeRecord;
using dog_patrol_perception_tracking::Track;
using dog_patrol_perception_tracking::UnresolvedTrackFinalResolutionCoordinator;

Track PersonTrack(const int raw_track_id, const cv::Rect2f &bbox, const bool occlusion_suspect = false) {
  Track track;
  track.id = raw_track_id;
  track.class_id = ClassId::kPerson;
  track.confidence = 0.90F;
  track.bbox = bbox;
  track.occlusion_suspect = occlusion_suspect;
  track.association.passed_final_cost_gate = true;
  return track;
}

IdentityRuntimeRecord Identity(const int semantic_id, const int missing_frames = 0) {
  IdentityRuntimeRecord identity;
  identity.semantic_id = semantic_id;
  identity.class_id = ClassId::kPerson;
  identity.missing_frames = missing_frames;
  return identity;
}

BirthManager::Result HiddenBirthResult(const BirthManager::Input &input, const std::string &reason) {
  BirthManager::Result result;
  result.decision.action = BirthCandidateDecision::Action::kHideWithDebugRow;
  result.has_debug_row = true;
  result.debug_row.track_idx = input.track_idx;
  result.debug_row.raw_track_id = input.raw_track_id;
  result.debug_row.semantic_id = -1;
  result.debug_row.stage = "phase5_birth_candidate";
  result.debug_row.reject_reason = reason;
  return result;
}

BirthManager::Result Phase5PendingBirthResult(const BirthManager::Input &input) {
  BirthManager::Result result;
  result.decision.action = BirthCandidateDecision::Action::kPhase5Pending;
  result.has_debug_row = true;
  result.debug_row.track_idx = input.track_idx;
  result.debug_row.raw_track_id = input.raw_track_id;
  result.debug_row.semantic_id = -1;
  result.debug_row.stage = "phase5_birth_candidate";
  result.debug_row.reject_reason = "phase5_birth_manager_pending";
  result.debug_row.selected = true;
  result.debug_row.accepted = false;
  result.debug_row.margin = 1.0F;
  return result;
}

BirthManager::Result AcceptedBirthResult(const BirthManager::Input &input, const int semantic_id) {
  BirthManager::Result result;
  result.decision.action = BirthCandidateDecision::Action::kPhase5Pending;
  result.has_debug_row = true;
  result.allocated_semantic_id = true;
  result.semantic_id = semantic_id;
  result.debug_row.track_idx = input.track_idx;
  result.debug_row.raw_track_id = input.raw_track_id;
  result.debug_row.semantic_id = semantic_id;
  result.debug_row.stage = "new_semantic";
  result.debug_row.selected = true;
  result.debug_row.accepted = true;
  result.debug_row.margin = 1.0F;
  return result;
}

UnresolvedTrackFinalResolutionCoordinator::Input BaseInput(
    const std::vector<Track> &tracks,
    const std::vector<int> &person_track_indices,
    const std::vector<std::vector<float>> &person_features,
    const std::unordered_map<int, int> &assigned_track_to_sid,
    const std::unordered_map<int, bool> &sid_used,
    const std::vector<int> &active_semantic_ids,
    const std::unordered_map<int, int> &prev_raw_to_semantic,
    const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> &score_debug_rows,
    const std::unordered_map<int, IdentityRuntimeRecord> &identities) {
  UnresolvedTrackFinalResolutionCoordinator::Input input;
  input.frame_index = 17;
  input.tracks = &tracks;
  input.person_track_indices = &person_track_indices;
  input.person_features = &person_features;
  input.assigned_track_to_sid = &assigned_track_to_sid;
  input.sid_used = &sid_used;
  input.active_semantic_ids = &active_semantic_ids;
  input.prev_raw_to_semantic = &prev_raw_to_semantic;
  input.score_debug_rows = &score_debug_rows;
  input.find_identity = [&](const int semantic_id) -> const IdentityRuntimeRecord * {
    const auto it = identities.find(semantic_id);
    return it == identities.end() ? nullptr : &it->second;
  };
  input.active_assignment_max_cost = [](const IdentityRuntimeRecord &, const dog_patrol_perception_tracking::AssociationEvidence &) {
    return 0.55F;
  };
  input.score_evidence = [](const Track &, const IdentityRuntimeRecord &identity, const std::vector<float> &feature) {
    UnresolvedTrackFinalResolutionCoordinator::ScoreEvidence evidence;
    evidence.app_cost = feature.empty() ? 0.20F : feature[0];
    evidence.geo_cost = feature.size() < 2U ? 0.10F : feature[1];
    evidence.time_cost = feature.size() < 3U ? 0.05F : feature[2];
    evidence.final_score = feature.size() < 4U ? evidence.app_cost : feature[3];
    if (identity.semantic_id == 12) {
      evidence.app_cost = 0.05F;
      evidence.final_score = 0.20F;
    }
    return evidence;
  };
  input.looks_like_merged_side_reappearance =
      [](const Track &, const IdentityRuntimeRecord &, const std::vector<Track> &, int,
         const std::unordered_map<int, int> &, float app_cost) {
        return app_cost <= 0.20F;
      };
  input.evaluate_birth = [](const BirthManager::Input &birth_input) {
    if (birth_input.hold_for_ambiguous_recovery) {
      return HiddenBirthResult(birth_input, "ambiguous_recovery_pending");
    }
    if (birth_input.duplicate_split) {
      return HiddenBirthResult(birth_input, "duplicate_split_hidden");
    }
    if (!birth_input.hide_reason.empty()) {
      return HiddenBirthResult(birth_input, birth_input.hide_reason);
    }
    return Phase5PendingBirthResult(birth_input);
  };
  return input;
}

const UnresolvedTrackFinalResolutionCoordinator::DebugRow *FindRow(
    const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> &rows,
    const std::string &stage,
    const std::string &reason = {}) {
  const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &row) {
    return row.stage == stage && (reason.empty() || row.reject_reason == reason);
  });
  return it == rows.end() ? nullptr : &(*it);
}

}  // namespace

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, OcclusionSuspectUnresolvedTrackIsSkipped) {
  const std::vector<Track> tracks{PersonTrack(101, cv::Rect2f(0, 0, 50, 100), true)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.10F, 0.20F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;
  int birth_calls = 0;
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used,
                         active_semantic_ids, prev_raw_to_semantic, score_debug_rows, identities);
  input.evaluate_birth = [&](const BirthManager::Input &birth_input) {
    ++birth_calls;
    return AcceptedBirthResult(birth_input, 90);
  };

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(input);

  EXPECT_TRUE(result.assigned_track_to_sid.empty());
  EXPECT_TRUE(result.debug_rows.empty());
  EXPECT_EQ(birth_calls, 0);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, AmbiguousRecoverySelectedMarginRejectHoldsBirthPending) {
  const std::vector<Track> tracks{PersonTrack(101, cv::Rect2f(0, 0, 50, 100))};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.80F, 0.10F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used{{7, false}};
  const std::vector<int> active_semantic_ids{7};
  const std::unordered_map<int, int> prev_raw_to_semantic;
  UnresolvedTrackFinalResolutionCoordinator::DebugRow reject_row;
  reject_row.track_idx = 0;
  reject_row.semantic_id = 7;
  reject_row.final_score = 0.60F;
  reject_row.selected = true;
  reject_row.accepted = false;
  reject_row.stage = "assign_candidate";
  reject_row.reject_reason = "assignment_margin_reject";
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows{reject_row};
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{7, Identity(7, 4)}};

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used, active_semantic_ids,
      prev_raw_to_semantic, score_debug_rows, identities));

  EXPECT_TRUE(result.assigned_track_to_sid.empty());
  const auto *row = FindRow(result.debug_rows, "phase5_birth_candidate", "ambiguous_recovery_pending");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->semantic_id, -1);
  EXPECT_FALSE(row->accepted);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, SideRecoverySuppressesLegacyApplyForPhase4) {
  const std::vector<Track> tracks{PersonTrack(101, cv::Rect2f(0, 0, 50, 100))};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.10F, 0.20F, 0.30F, 0.40F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids{11, 12};
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{11, Identity(11, 3)}, {12, Identity(12, 2)}};

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used, active_semantic_ids,
      prev_raw_to_semantic, score_debug_rows, identities));

  EXPECT_TRUE(result.assigned_track_to_sid.empty());
  EXPECT_TRUE(result.debug_rows.empty());
  ASSERT_EQ(result.erase_pending_raw_track_ids.size(), 1U);
  EXPECT_EQ(result.erase_pending_raw_track_ids[0], 101);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, DuplicateSplitIsHiddenBeforeBirthAllocation) {
  const std::vector<Track> tracks{
      PersonTrack(10, cv::Rect2f(0, 0, 100, 200)),
      PersonTrack(11, cv::Rect2f(10, 20, 60, 120)),
  };
  const std::vector<int> person_track_indices{0, 1};
  const std::vector<std::vector<float>> person_features{{0.90F}, {0.90F}};
  const std::unordered_map<int, int> assigned_track_to_sid{{0, 1}};
  const std::unordered_map<int, bool> sid_used{{1, true}};
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic{{10, 1}};
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used, active_semantic_ids,
      prev_raw_to_semantic, score_debug_rows, identities));

  const auto *row = FindRow(result.debug_rows, "phase5_birth_candidate", "duplicate_split_hidden");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->raw_track_id, 11);
  EXPECT_FALSE(row->accepted);
  EXPECT_EQ(result.assigned_track_to_sid.count(1), 0U);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, SkinnyAndWideMorphologyAreHidden) {
  const std::vector<Track> tracks{
      PersonTrack(21, cv::Rect2f(0, 0, 20, 120)),
      PersonTrack(22, cv::Rect2f(200, 0, 180, 100)),
  };
  const std::vector<int> person_track_indices{0, 1};
  const std::vector<std::vector<float>> person_features{{0.90F}, {0.90F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used, active_semantic_ids,
      prev_raw_to_semantic, score_debug_rows, identities));

  EXPECT_NE(FindRow(result.debug_rows, "phase5_birth_candidate", "skinny_partial_hidden"), nullptr);
  EXPECT_NE(FindRow(result.debug_rows, "phase5_birth_candidate", "wide_fragment_hidden"), nullptr);
  EXPECT_TRUE(result.assigned_track_to_sid.empty());
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, SmallNewPersonRemainsPendingWithPhase5DebugRow) {
  const std::vector<Track> tracks{PersonTrack(31, cv::Rect2f(0, 0, 40, 160))};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.90F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;
  BirthManager::Input captured_birth_input;
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used,
                         active_semantic_ids, prev_raw_to_semantic, score_debug_rows, identities);
  input.evaluate_birth = [&](const BirthManager::Input &birth_input) {
    captured_birth_input = birth_input;
    return Phase5PendingBirthResult(birth_input);
  };

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(input);

  EXPECT_TRUE(captured_birth_input.small_person_requires_stability);
  EXPECT_TRUE(result.assigned_track_to_sid.empty());
  const auto *row = FindRow(result.debug_rows, "phase5_birth_candidate", "phase5_birth_manager_pending");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->selected);
  EXPECT_FALSE(row->accepted);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, Phase5PendingUsesPhase5BirthCandidateRowWithoutAssignment) {
  const std::vector<Track> tracks{PersonTrack(41, cv::Rect2f(0, 0, 80, 180))};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.90F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used,
                         active_semantic_ids, prev_raw_to_semantic, score_debug_rows, identities);
  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(input);

  EXPECT_TRUE(result.assigned_track_to_sid.empty());
  const auto *row = FindRow(result.debug_rows, "phase5_birth_candidate", "phase5_birth_manager_pending");
  ASSERT_NE(row, nullptr);
  EXPECT_TRUE(row->selected);
  EXPECT_FALSE(row->accepted);
}

TEST(UnresolvedTrackFinalResolutionCoordinatorTest, AcceptedBirthAssignmentOutputsPlannedTrackAssignment) {
  const std::vector<Track> tracks{PersonTrack(51, cv::Rect2f(0, 0, 80, 180))};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.90F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::unordered_map<int, bool> sid_used;
  const std::vector<int> active_semantic_ids;
  const std::unordered_map<int, int> prev_raw_to_semantic;
  const std::vector<UnresolvedTrackFinalResolutionCoordinator::DebugRow> score_debug_rows;
  const std::unordered_map<int, IdentityRuntimeRecord> identities;
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid, sid_used,
                         active_semantic_ids, prev_raw_to_semantic, score_debug_rows, identities);
  input.evaluate_birth = [](const BirthManager::Input &birth_input) {
    BirthManager::Result result = AcceptedBirthResult(birth_input, 123);
    result.debug_row.stage = "phase5_new_semantic";
    return result;
  };

  const auto result = UnresolvedTrackFinalResolutionCoordinator::Resolve(input);

  ASSERT_EQ(result.assigned_track_to_sid.at(0), 123);
  const auto *row = FindRow(result.debug_rows, "phase5_new_semantic");
  ASSERT_NE(row, nullptr);
  EXPECT_EQ(row->semantic_id, 123);
  EXPECT_TRUE(row->selected);
  EXPECT_TRUE(row->accepted);
}
