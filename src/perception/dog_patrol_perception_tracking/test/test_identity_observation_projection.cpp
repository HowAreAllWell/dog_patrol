#include <gtest/gtest.h>

#include "identity_observation_projection.hpp"

namespace {

using vision_demo_host::AssociationEvidence;
using vision_demo_host::ClassId;
using vision_demo_host::IdentityManager;
using vision_demo_host::IdentityObservationProjection;
using vision_demo_host::IdentityState;
using vision_demo_host::LegacyIdentitySnapshot;
using vision_demo_host::TrackletObservation;

LegacyIdentitySnapshot MakeSnapshot(const int semantic_id, const bool seen_this_frame, const int missing_frames) {
  LegacyIdentitySnapshot snapshot;
  snapshot.semantic_id = semantic_id;
  snapshot.class_id = ClassId::kPerson;
  snapshot.confidence = 0.6F + static_cast<float>(semantic_id) * 0.1F;
  snapshot.bbox = cv::Rect2f(10.0F * semantic_id, 5.0F, 30.0F, 40.0F);
  snapshot.reliable_bbox = cv::Rect2f(100.0F + 10.0F * semantic_id, 15.0F, 32.0F, 42.0F);
  snapshot.has_reliable_geometry = true;
  snapshot.seen_this_frame = seen_this_frame;
  snapshot.missing_frames = missing_frames;
  return snapshot;
}

TrackletObservation MakeObservation(const int raw_track_id) {
  TrackletObservation obs;
  obs.raw_track_id = raw_track_id;
  obs.class_id = ClassId::kCar;
  obs.confidence = 0.91F;
  obs.bbox = cv::Rect2f(200.0F, 20.0F, 50.0F, 60.0F);
  obs.occlusion_suspect = true;
  obs.low_score_update = true;
  obs.just_recovered = true;
  obs.association.stage = "assign_candidate";
  obs.association.fused_cost = 0.22F;
  obs.association.passed_final_cost_gate = true;
  return obs;
}

IdentityManager::ScoreDebugRow MakeDebugRow(const int raw_track_id, const int semantic_id,
                                            const bool accepted, const bool selected,
                                            const std::string &stage) {
  IdentityManager::ScoreDebugRow row;
  row.raw_track_id = raw_track_id;
  row.semantic_id = semantic_id;
  row.accepted = accepted;
  row.selected = selected;
  row.stage = stage;
  row.app_cost = 0.11F;
  row.geo_cost = 0.22F;
  row.time_cost = 0.33F;
  row.final_score = 0.44F;
  row.margin = 0.55F;
  row.continuity_used = true;
  row.feature_update_allowed = false;
  row.geometry_update_allowed = true;
  row.feature_update_reason = "global_merge_split_freeze";
  row.geometry_update_reason = "allowed_update";
  row.reject_reason = accepted ? "" : "candidate_kept_for_debug";
  return row;
}

TEST(IdentityObservationProjectionTest, MapsSnapshotStatesWithoutChangingSemantics) {
  IdentityObservationProjection::Input active_input;
  active_input.mode = IdentityManager::Mode::kNormal;
  active_input.max_missing_frames = 3;
  active_input.snapshots.push_back(MakeSnapshot(1, true, 0));
  auto active_result = IdentityObservationProjection::Build(active_input);
  ASSERT_EQ(active_result.identities.size(), 1U);
  EXPECT_EQ(active_result.identities.front().state, IdentityState::kActive);

  IdentityObservationProjection::Input merged_input;
  merged_input.mode = IdentityManager::Mode::kMerged;
  merged_input.max_missing_frames = 3;
  merged_input.snapshots.push_back(MakeSnapshot(2, false, 1));
  auto merged_result = IdentityObservationProjection::Build(merged_input);
  ASSERT_EQ(merged_result.identities.size(), 1U);
  EXPECT_EQ(merged_result.identities.front().state, IdentityState::kMerged);

  IdentityObservationProjection::Input split_input;
  split_input.mode = IdentityManager::Mode::kSplitRecovery;
  split_input.max_missing_frames = 3;
  split_input.snapshots.push_back(MakeSnapshot(3, false, 1));
  auto split_result = IdentityObservationProjection::Build(split_input);
  ASSERT_EQ(split_result.identities.size(), 1U);
  EXPECT_EQ(split_result.identities.front().state, IdentityState::kSplitRecovery);

  IdentityObservationProjection::Input occluded_input;
  occluded_input.mode = IdentityManager::Mode::kNormal;
  occluded_input.max_missing_frames = 3;
  occluded_input.snapshots.push_back(MakeSnapshot(4, false, 3));
  auto occluded_result = IdentityObservationProjection::Build(occluded_input);
  ASSERT_EQ(occluded_result.identities.size(), 1U);
  EXPECT_EQ(occluded_result.identities.front().state, IdentityState::kOccluded);

  IdentityObservationProjection::Input lost_input;
  lost_input.mode = IdentityManager::Mode::kNormal;
  lost_input.max_missing_frames = 3;
  lost_input.snapshots.push_back(MakeSnapshot(5, false, 4));
  auto lost_result = IdentityObservationProjection::Build(lost_input);
  ASSERT_EQ(lost_result.identities.size(), 1U);
  EXPECT_EQ(lost_result.identities.front().state, IdentityState::kLost);
}

TEST(IdentityObservationProjectionTest, UsesReliableGeometryFallbackWhenSupportingObservationMissing) {
  IdentityObservationProjection::Input input;
  input.mode = IdentityManager::Mode::kNormal;
  input.max_missing_frames = 10;
  auto snapshot = MakeSnapshot(7, false, 2);
  snapshot.supporting_raw_track_id = 42;
  input.snapshots.push_back(snapshot);

  const auto result = IdentityObservationProjection::Build(input);
  ASSERT_EQ(result.identities.size(), 1U);
  const auto &identity = result.identities.front();
  ASSERT_TRUE(identity.supporting_raw_track_id.has_value());
  EXPECT_EQ(*identity.supporting_raw_track_id, 42);
  EXPECT_FALSE(identity.supporting_tracklet.has_value());
  EXPECT_EQ(identity.class_id, snapshot.class_id);
  EXPECT_FLOAT_EQ(identity.confidence, snapshot.confidence);
  EXPECT_EQ(identity.bbox, snapshot.reliable_bbox);
  EXPECT_FALSE(identity.assignment.accepted);
  EXPECT_EQ(result.SemanticIdForRawTrack(42), 7);
}

TEST(IdentityObservationProjectionTest, VisibleSupportingObservationOverridesSnapshotAndCarriesBestAssignmentEvidence) {
  IdentityObservationProjection::Input input;
  input.mode = IdentityManager::Mode::kNormal;
  input.max_missing_frames = 10;
  input.primary_semantic_id = 9;
  input.feature_update_frozen = true;

  auto snapshot = MakeSnapshot(9, true, 0);
  snapshot.supporting_raw_track_id = 77;
  snapshot.class_id = ClassId::kPerson;
  snapshot.confidence = 0.3F;
  snapshot.bbox = cv::Rect2f(1.0F, 2.0F, 3.0F, 4.0F);
  input.snapshots.push_back(snapshot);

  auto observation = MakeObservation(77);
  input.observations_by_raw_track_id.emplace(77, observation);
  input.debug_rows.push_back(MakeDebugRow(77, 9, false, true, "raw_continuity"));
  input.debug_rows.push_back(MakeDebugRow(77, 9, true, false, "assign_candidate"));

  const auto result = IdentityObservationProjection::Build(input);
  ASSERT_EQ(result.identities.size(), 1U);
  const auto &identity = result.identities.front();
  EXPECT_TRUE(identity.primary);
  EXPECT_TRUE(result.feature_update_frozen);
  ASSERT_TRUE(identity.supporting_tracklet.has_value());
  EXPECT_EQ(identity.class_id, observation.class_id);
  EXPECT_FLOAT_EQ(identity.confidence, observation.confidence);
  EXPECT_EQ(identity.bbox, observation.bbox);
  EXPECT_TRUE(identity.occlusion_suspect);
  EXPECT_TRUE(identity.low_score_update);
  EXPECT_TRUE(identity.just_recovered);
  EXPECT_EQ(identity.association.stage, "assign_candidate");
  EXPECT_EQ(identity.assignment.stage, "assign_candidate");
  EXPECT_TRUE(identity.assignment.accepted);
  EXPECT_FALSE(identity.assignment.feature_update_allowed);
  EXPECT_TRUE(identity.assignment.geometry_update_allowed);
  EXPECT_EQ(identity.assignment.feature_update_reason, "global_merge_split_freeze");
  EXPECT_EQ(result.SemanticIdForRawTrack(77), 9);
}

}  // namespace
