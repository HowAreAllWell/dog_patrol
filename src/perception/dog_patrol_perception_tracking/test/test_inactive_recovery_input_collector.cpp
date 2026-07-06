#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "assignment_candidate_builder.hpp"
#include "inactive_recovery_input_collector.hpp"

namespace {

using vision_demo_host::AssignmentCandidateBuilder;
using vision_demo_host::ClassId;
using vision_demo_host::InactiveRecoveryInputCollector;
using vision_demo_host::InactiveRecoverySolver;
using vision_demo_host::IdentityRuntimeRecord;
using vision_demo_host::Track;

Track PersonTrack(const int raw_track_id, const bool occlusion_suspect = false) {
  Track track;
  track.id = raw_track_id;
  track.class_id = ClassId::kPerson;
  track.confidence = 0.90F;
  track.bbox = cv::Rect2f(10.0F * static_cast<float>(raw_track_id), 20.0F, 40.0F, 80.0F);
  track.occlusion_suspect = occlusion_suspect;
  return track;
}

IdentityRuntimeRecord Identity(const int semantic_id,
                              const int missing_frames = 200,
                              const int occlusion_protect_remaining = 0) {
  IdentityRuntimeRecord identity;
  identity.semantic_id = semantic_id;
  identity.class_id = ClassId::kPerson;
  identity.missing_frames = missing_frames;
  identity.occlusion_protect_remaining = occlusion_protect_remaining;
  return identity;
}

InactiveRecoveryInputCollector::Input BaseInput(
    const std::vector<Track> &tracks,
    const std::vector<int> &person_track_indices,
    const std::vector<std::vector<float>> &person_features,
    const std::unordered_map<int, int> &assigned_track_to_sid,
    const std::vector<int> &inactive_semantic_ids,
    const std::unordered_map<int, bool> &used_semantic_ids,
    const std::unordered_map<int, IdentityRuntimeRecord> &identities) {
  InactiveRecoveryInputCollector::Input input;
  input.tracks = &tracks;
  input.person_track_indices = &person_track_indices;
  input.person_features = &person_features;
  input.assigned_track_to_sid = &assigned_track_to_sid;
  input.inactive_semantic_ids = &inactive_semantic_ids;
  input.semantic_id_used = [&](const int semantic_id) {
    const auto it = used_semantic_ids.find(semantic_id);
    return it != used_semantic_ids.end() && it->second;
  };
  input.find_identity = [&](const int semantic_id) -> const IdentityRuntimeRecord * {
    const auto it = identities.find(semantic_id);
    return it == identities.end() ? nullptr : &it->second;
  };
  input.can_recover_identity = [](const IdentityRuntimeRecord &identity) {
    return identity.occlusion_protect_remaining <= 0;
  };
  input.score_evidence = [](const Track &, const IdentityRuntimeRecord &identity, const std::vector<float> &feature) {
    InactiveRecoveryInputCollector::ScoreEvidence evidence;
    evidence.app_cost = feature.empty() ? 0.90F : feature[0];
    evidence.geo_cost = feature.size() < 2U ? 0.80F : feature[1];
    evidence.similarity = 1.0F - evidence.app_cost;
    evidence.recover_threshold = identity.missing_frames > 180 ? 0.75F : 0.85F;
    return evidence;
  };
  return input;
}

}  // namespace

TEST(InactiveRecoveryInputCollectorTest, FiltersAssignedAndOcclusionSuspectRecoveryTracks) {
  const std::vector<Track> tracks{PersonTrack(101), PersonTrack(102, true), PersonTrack(103)};
  const std::vector<int> person_track_indices{0, 1, 2};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}, {0.12F, 0.22F}, {0.13F, 0.23F}};
  const std::unordered_map<int, int> assigned_track_to_sid{{0, 7}};
  const std::vector<int> inactive_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{10, Identity(10)}};

  const auto result = InactiveRecoveryInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, inactive_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.recovery_track_indices.size(), 1U);
  EXPECT_EQ(result.recovery_track_indices[0], 2);
  ASSERT_EQ(result.solver_tracks.size(), 1U);
  EXPECT_EQ(result.solver_tracks[0].raw_track_id, 103);
  ASSERT_EQ(result.selected_features.size(), 1U);
  EXPECT_FLOAT_EQ(result.selected_features[0][0], 0.13F);
  ASSERT_EQ(result.solver_scores.size(), 1U);
  EXPECT_EQ(result.solver_scores[0].track_row, 0);
  EXPECT_FLOAT_EQ(result.solver_scores[0].app_cost, 0.13F);
}

TEST(InactiveRecoveryInputCollectorTest, FiltersUsedInactiveSemanticIds) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> inactive_semantic_ids{10, 11};
  const std::unordered_map<int, bool> used_semantic_ids{{10, true}};
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{10, Identity(10)}, {11, Identity(11)}};

  const auto result = InactiveRecoveryInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, inactive_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.free_semantic_ids.size(), 1U);
  EXPECT_EQ(result.free_semantic_ids[0], 11);
  ASSERT_EQ(result.solver_candidates.size(), 1U);
  EXPECT_EQ(result.solver_candidates[0].semantic_id, 11);
  ASSERT_EQ(result.solver_scores.size(), 1U);
  EXPECT_EQ(result.solver_scores[0].candidate_col, 0);
}

TEST(InactiveRecoveryInputCollectorTest, ExcludesNonRecoverableInactiveIdentitiesFromScores) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> inactive_semantic_ids{10, 11};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{
      {10, Identity(10, 200, 3)},
      {11, Identity(11, 200, 0)},
  };

  const auto result = InactiveRecoveryInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, inactive_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.solver_candidates.size(), 2U);
  ASSERT_EQ(result.solver_scores.size(), 1U);
  EXPECT_EQ(result.solver_scores[0].candidate_col, 1);
  EXPECT_EQ(result.solver_candidates[static_cast<std::size_t>(result.solver_scores[0].candidate_col)].semantic_id, 11);
}

TEST(InactiveRecoveryInputCollectorTest, AdaptsStrictAndRelaxedRecoverThresholds) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.11F, 0.21F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> inactive_semantic_ids{10, 11};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{
      {10, Identity(10, 120)},
      {11, Identity(11, 200)},
  };

  const auto result = InactiveRecoveryInputCollector::Collect(BaseInput(
      tracks, person_track_indices, person_features, assigned_track_to_sid, inactive_semantic_ids,
      used_semantic_ids, identities));

  ASSERT_EQ(result.solver_scores.size(), 2U);
  EXPECT_FLOAT_EQ(result.solver_scores[0].recover_threshold, 0.85F);
  EXPECT_FLOAT_EQ(result.solver_scores[1].recover_threshold, 0.75F);
}

TEST(InactiveRecoveryInputCollectorTest, PropagatesMissingIdentityGateEvidenceToSolverRows) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.05F, 0.10F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> inactive_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{10, Identity(10)}};
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid,
                         inactive_semantic_ids, used_semantic_ids, identities);
  input.score_evidence = [](const Track &, const IdentityRuntimeRecord &, const std::vector<float> &) {
    InactiveRecoveryInputCollector::ScoreEvidence evidence;
    evidence.app_cost = 0.05F;
    evidence.geo_cost = 0.10F;
    evidence.similarity = 0.95F;
    evidence.recover_threshold = 0.75F;
    evidence.passes_missing_identity_gate = false;
    return evidence;
  };

  const auto collected = InactiveRecoveryInputCollector::Collect(input);
  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.45F;
  const auto solved =
      InactiveRecoverySolver::SolveHungarian(collected.solver_tracks, collected.solver_candidates,
                                             collected.solver_scores, config);
  const auto rows = AssignmentCandidateBuilder::BuildInactiveRecoveryRows(solved.candidates);

  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].reject_reason, "missing_identity_gate_reject");
  EXPECT_FALSE(rows[0].accepted);
  EXPECT_TRUE(solved.assignments.empty());
}

TEST(InactiveRecoveryInputCollectorTest, PropagatesAcceptedCandidateScoresAndSelectedRowUpdates) {
  const std::vector<Track> tracks{PersonTrack(101)};
  const std::vector<int> person_track_indices{0};
  const std::vector<std::vector<float>> person_features{{0.10F, 0.22F}};
  const std::unordered_map<int, int> assigned_track_to_sid;
  const std::vector<int> inactive_semantic_ids{10};
  const std::unordered_map<int, bool> used_semantic_ids;
  const std::unordered_map<int, IdentityRuntimeRecord> identities{{10, Identity(10)}};
  auto input = BaseInput(tracks, person_track_indices, person_features, assigned_track_to_sid,
                         inactive_semantic_ids, used_semantic_ids, identities);
  input.score_evidence = [](const Track &, const IdentityRuntimeRecord &, const std::vector<float> &) {
    InactiveRecoveryInputCollector::ScoreEvidence evidence;
    evidence.app_cost = 0.10F;
    evidence.geo_cost = 0.22F;
    evidence.similarity = 0.90F;
    evidence.recover_threshold = 0.75F;
    return evidence;
  };

  const auto collected = InactiveRecoveryInputCollector::Collect(input);
  InactiveRecoverySolver::Config config;
  config.recovery_max_cost = 0.45F;
  const auto solved =
      InactiveRecoverySolver::SolveHungarian(collected.solver_tracks, collected.solver_candidates,
                                             collected.solver_scores, config);
  auto rows = AssignmentCandidateBuilder::BuildInactiveRecoveryRows(solved.candidates);

  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].stage, "inactive_recover_candidate");
  EXPECT_FLOAT_EQ(rows[0].app_cost, 0.10F);
  EXPECT_FLOAT_EQ(rows[0].geo_cost, 0.22F);
  EXPECT_FLOAT_EQ(rows[0].final_score, 0.90F);
  EXPECT_TRUE(rows[0].accepted);

  AssignmentCandidateBuilder::ApplyInactiveRecoveryAssignments(solved.assignments, &rows);

  ASSERT_EQ(solved.assignments.size(), 1U);
  EXPECT_TRUE(rows[0].selected);
  EXPECT_TRUE(rows[0].accepted);
  EXPECT_FLOAT_EQ(rows[0].margin, solved.assignments[0].margin);
}
