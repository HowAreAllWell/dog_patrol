#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "active_assignment_input_collector.hpp"
#include "active_assignment_solver.hpp"

namespace {

using vision_demo_host::ActiveAssignmentInputCollector;
using vision_demo_host::ActiveAssignmentSolver;
using vision_demo_host::AssignmentCandidateBuilder;
using vision_demo_host::ClassId;
using vision_demo_host::LegacyIdentityRecord;
using vision_demo_host::Track;

Track PersonTrack(const int raw_track_id, const bool occlusion_suspect = false) {
  Track track;
  track.id = raw_track_id;
  track.class_id = ClassId::kPerson;
  track.confidence = 0.90F;
  track.bbox = cv::Rect2f(10.0F * static_cast<float>(raw_track_id), 20.0F, 40.0F, 80.0F);
  track.occlusion_suspect = occlusion_suspect;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

LegacyIdentityRecord Identity(const int semantic_id, const int missing_frames = 0) {
  LegacyIdentityRecord identity;
  identity.semantic_id = semantic_id;
  identity.class_id = ClassId::kPerson;
  identity.missing_frames = missing_frames;
  return identity;
}

ActiveAssignmentInputCollector::Input BaseInput(
    const std::vector<Track> &tracks,
    const std::vector<int> &person_track_indices,
    const std::vector<std::vector<float>> &person_features,
    const std::unordered_map<int, int> &assigned_track_to_sid,
    const std::vector<int> &active_semantic_ids,
    const std::unordered_map<int, bool> &used_semantic_ids,
    const std::unordered_map<int, LegacyIdentityRecord> &identities) {
  ActiveAssignmentInputCollector::Input input;
  input.tracks = &tracks;
  input.person_track_indices = &person_track_indices;
  input.person_features = &person_features;
  input.assigned_track_to_sid = &assigned_track_to_sid;
  input.active_semantic_ids = &active_semantic_ids;
  input.semantic_id_used = [&](const int semantic_id) {
    const auto it = used_semantic_ids.find(semantic_id);
    return it != used_semantic_ids.end() && it->second;
  };
  input.find_identity = [&](const int semantic_id) -> const LegacyIdentityRecord * {
    const auto it = identities.find(semantic_id);
    return it == identities.end() ? nullptr : &it->second;
  };
  input.score_evidence = [](const Track &, const LegacyIdentityRecord &, const std::vector<float> &feature) {
    ActiveAssignmentInputCollector::ScoreEvidence evidence;
    evidence.app_cost = feature.empty() ? 0.90F : feature[0];
    evidence.geo_cost = feature.size() < 2U ? 0.80F : feature[1];
    evidence.time_cost = 0.10F;
    evidence.final_score = 0.25F;
    return evidence;
  };
  return input;
}

}  // namespace

TEST(ActiveAssignmentInputCollectorTest, FiltersAlreadyAssignedTracksAndKeepsFeatureRowsAligned) {
  const std::vector<Track> tracks{PersonTrack(101), PersonTrack(102), PersonTrack(103)};
  const std::vector<int> person_track_indices{0, 1, 2};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}, {0.12F, 0.22F}, {0.13F, 0.23F}};
  const std::unordered_map<int, int> assigned_track_to_sid{{1, 7}};
  const std::vector<int> active_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, LegacyIdentityRecord> identities{{10, Identity(10)}};

  const auto result = ActiveAssignmentInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, active_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.unassigned_track_indices.size(), 2U);
  EXPECT_EQ(result.unassigned_track_indices[0], 0);
  EXPECT_EQ(result.unassigned_track_indices[1], 2);
  ASSERT_EQ(result.builder_tracks.size(), 2U);
  EXPECT_EQ(result.builder_tracks[0].raw_track_id, 101);
  EXPECT_EQ(result.builder_tracks[1].raw_track_id, 103);
  ASSERT_EQ(result.selected_features.size(), 2U);
  EXPECT_FLOAT_EQ(result.selected_features[0][0], 0.11F);
  EXPECT_FLOAT_EQ(result.selected_features[1][0], 0.13F);
  ASSERT_EQ(result.builder_scores.size(), 2U);
  EXPECT_FLOAT_EQ(result.builder_scores[1].app_cost, 0.13F);
  EXPECT_FLOAT_EQ(result.builder_scores[1].geo_cost, 0.23F);
}

TEST(ActiveAssignmentInputCollectorTest, FiltersUsedSemanticIds) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> active_semantic_ids{10, 11};
  const std::unordered_map<int, bool> used_semantic_ids{{10, true}};
  const std::unordered_map<int, LegacyIdentityRecord> identities{{10, Identity(10)}, {11, Identity(11, 4)}};

  const auto result = ActiveAssignmentInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, active_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.free_semantic_ids.size(), 1U);
  EXPECT_EQ(result.free_semantic_ids[0], 11);
  ASSERT_EQ(result.builder_candidates.size(), 1U);
  EXPECT_EQ(result.builder_candidates[0].semantic_id, 11);
  EXPECT_EQ(result.builder_candidates[0].missing_frames, 4);
  ASSERT_EQ(result.builder_scores.size(), 1U);
  EXPECT_EQ(result.builder_scores[0].candidate_col, 0);
}

TEST(ActiveAssignmentInputCollectorTest, ExcludesOcclusionSuspectTracks) {
  const std::vector<Track> tracks{PersonTrack(101, true), PersonTrack(102)};
  const std::vector<int> person_track_indices{0, 1};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}, {0.12F, 0.22F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> active_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, LegacyIdentityRecord> identities{{10, Identity(10)}};

  const auto result = ActiveAssignmentInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, active_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.unassigned_track_indices.size(), 1U);
  EXPECT_EQ(result.unassigned_track_indices[0], 1);
  ASSERT_EQ(result.builder_tracks.size(), 1U);
  EXPECT_EQ(result.builder_tracks[0].raw_track_id, 102);
}

TEST(ActiveAssignmentInputCollectorTest, PropagatesMissingGateRejectEvidenceToBuilder) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.15F, 0.25F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> active_semantic_ids{10, 11};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, LegacyIdentityRecord> identities{{10, Identity(10)}, {11, Identity(11)}};
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid,
                         active_semantic_ids, used_semantic_ids, identities);
  input.score_evidence = [](const Track &, const LegacyIdentityRecord &identity, const std::vector<float> &) {
    ActiveAssignmentInputCollector::ScoreEvidence evidence;
    evidence.final_score = identity.semantic_id == 10 ? 0.20F : 0.30F;
    evidence.passes_missing_identity_gate = identity.semantic_id != 10;
    evidence.passes_missing_appearance_gate = identity.semantic_id != 11;
    return evidence;
  };

  const auto collected = ActiveAssignmentInputCollector::Collect(input);
  const auto built = AssignmentCandidateBuilder::BuildActiveAssignments(
      collected.builder_tracks, collected.builder_candidates, collected.builder_scores);

  ASSERT_EQ(built.debug_rows.size(), 2U);
  EXPECT_EQ(built.debug_rows[0].semantic_id, 10);
  EXPECT_EQ(built.debug_rows[0].reject_reason, "missing_identity_gate_reject");
  EXPECT_GE(built.cost_matrix[0][0], ActiveAssignmentSolver::kBigCost * 0.5F);
  EXPECT_EQ(built.debug_rows[1].semantic_id, 11);
  EXPECT_EQ(built.debug_rows[1].reject_reason, "missing_appearance_gate_reject");
  EXPECT_GE(built.cost_matrix[0][1], ActiveAssignmentSolver::kBigCost * 0.5F);
}

TEST(ActiveAssignmentInputCollectorTest, PropagatesAcceptedCandidateScoresAndPairwisePendingRows) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.14F, 0.24F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> active_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, LegacyIdentityRecord> identities{{10, Identity(10)}};
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid,
                         active_semantic_ids, used_semantic_ids, identities);
  input.score_evidence = [](const Track &, const LegacyIdentityRecord &, const std::vector<float> &) {
    ActiveAssignmentInputCollector::ScoreEvidence evidence;
    evidence.app_cost = 0.14F;
    evidence.geo_cost = 0.24F;
    evidence.time_cost = 0.04F;
    evidence.final_score = 0.18F;
    return evidence;
  };

  const auto collected = ActiveAssignmentInputCollector::Collect(input);
  auto rows = AssignmentCandidateBuilder::BuildActiveAssignments(
      collected.builder_tracks, collected.builder_candidates, collected.builder_scores).debug_rows;
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].stage, "assign_candidate");
  EXPECT_FLOAT_EQ(rows[0].app_cost, 0.14F);
  EXPECT_FLOAT_EQ(rows[0].geo_cost, 0.24F);
  EXPECT_FLOAT_EQ(rows[0].time_cost, 0.04F);
  EXPECT_FLOAT_EQ(rows[0].final_score, 0.18F);

  const std::vector<ActiveAssignmentSolver::Assignment> assignments{
      ActiveAssignmentSolver::Assignment{0, 0, 0, 10, 0.82F, 0.18F, 0.11F, true, "", true},
  };
  AssignmentCandidateBuilder::ApplyActiveSolverResults(assignments, true, &rows);

  EXPECT_TRUE(rows[0].selected);
  EXPECT_FLOAT_EQ(rows[0].margin, 0.11F);
  EXPECT_FALSE(rows[0].accepted);
  EXPECT_EQ(rows[0].reject_reason, "phase4_pairwise_assignment_pending");
}
