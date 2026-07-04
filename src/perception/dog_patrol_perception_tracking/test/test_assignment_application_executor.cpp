#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "assignment_application_executor.hpp"
#include "assignment_application_plan.hpp"
#include "legacy_identity_store.hpp"
#include "raw_semantic_binding_store.hpp"
#include "vision_demo_host/modules/feature_update_policy.hpp"
#include "vision_demo_host/types.hpp"

namespace {

using vision_demo_host::AssignmentApplicationExecutor;
using vision_demo_host::AssignmentApplicationPlan;
using vision_demo_host::FeatureUpdatePolicy;
using vision_demo_host::LegacyIdentityStore;
using vision_demo_host::RawSemanticBindingStore;
using vision_demo_host::Track;

struct TestDebugRow {
  int frame_idx{-1};
  int track_idx{-1};
  int semantic_id{-1};
  int raw_track_id{-1};
  bool accepted{false};
  bool feature_update_allowed{false};
  bool geometry_update_allowed{false};
  std::string feature_update_reason;
  std::string geometry_update_reason;
};

Track MakeTrack(const int raw_track_id, const float x) {
  Track track;
  track.id = raw_track_id;
  track.class_id = vision_demo_host::ClassId::kPerson;
  track.confidence = 0.9F;
  track.bbox = cv::Rect2f(x, 0.0F, 50.0F, 100.0F);
  track.is_confirmed = true;
  track.association.stage = "stage1_confirmed_high";
  track.association.passed_final_cost_gate = true;
  return track;
}

AssignmentApplicationPlan::Application Application(const int track_idx,
                                                   const int raw_track_id,
                                                   const int semantic_id,
                                                   const char *stage,
                                                   const float cost,
                                                   const float margin) {
  AssignmentApplicationPlan::Application application;
  application.track_idx = track_idx;
  application.raw_track_id = raw_track_id;
  application.semantic_id = semantic_id;
  application.accepted_stage = stage;
  application.assignment_cost = cost;
  application.assignment_margin = margin;
  application.found_accepted_row = true;
  return application;
}

AssignmentApplicationPlan::RawMapping RawMapping(const int raw_track_id, const int semantic_id) {
  AssignmentApplicationPlan::RawMapping mapping;
  mapping.raw_track_id = raw_track_id;
  mapping.semantic_id = semantic_id;
  return mapping;
}

}  // namespace

TEST(AssignmentApplicationExecutorTest, AppliesAcceptedMutationsProtectsOcclusionAndReplacesRawBindings) {
  AssignmentApplicationPlan::Result plan;
  plan.applications.push_back(Application(0, 101, 7, "assign_candidate", 0.25F, 0.15F));
  plan.next_raw_to_semantic_entries.push_back(RawMapping(101, 7));

  std::vector<Track> tracks{MakeTrack(101, 0.0F)};
  std::unordered_map<int, std::size_t> feature_index{{0, 0}};
  std::vector<std::vector<float>> features{{1.0F, 0.0F, 0.0F}};
  std::vector<TestDebugRow> rows{
      TestDebugRow{9, 0, 7, 101, true, false, false, "", ""},
      TestDebugRow{9, 2, 8, 108, false, false, false, "", ""},
  };
  LegacyIdentityStore store;
  auto &existing = store.Upsert(3);
  existing.semantic_id = 3;
  existing.class_id = vision_demo_host::ClassId::kPerson;
  existing.missing_frames = 0;
  existing.seen_this_frame = false;
  existing.occlusion_protect_remaining = 0;
  RawSemanticBindingStore bindings;
  bindings.Bind(99, 3);

  int upsert_calls = 0;
  AssignmentApplicationExecutor<TestDebugRow>::Config config;
  config.frame_idx = 9;
  config.occlusion_protect_frames = 30;
  config.protect_unseen_active_people = true;

  AssignmentApplicationExecutor<TestDebugRow>::Execute(
      plan, {0}, tracks, feature_index, features, &rows, &store, &bindings, config,
      [](const Track &, int, int, const vision_demo_host::LegacyIdentityRecord *, float, float, bool) {
        FeatureUpdatePolicy::Decision decision;
        decision.feature_update_allowed = true;
        decision.geometry_update_allowed = true;
        decision.feature_update_reason = "allowed_update";
        decision.geometry_update_reason = "allowed_update";
        return decision;
      },
      [&](const Track &, int semantic_id, const std::vector<float> &feature,
          const FeatureUpdatePolicy::Decision &, float assignment_cost, float assignment_margin) {
        ++upsert_calls;
        EXPECT_EQ(semantic_id, 7);
        EXPECT_EQ(feature.size(), 3U);
        EXPECT_FLOAT_EQ(assignment_cost, 0.25F);
        EXPECT_FLOAT_EQ(assignment_margin, 0.15F);
      });

  EXPECT_EQ(upsert_calls, 1);
  EXPECT_TRUE(rows[0].feature_update_allowed);
  EXPECT_TRUE(rows[0].geometry_update_allowed);
  EXPECT_EQ(rows[0].feature_update_reason, "allowed_update");
  EXPECT_EQ(rows[0].geometry_update_reason, "allowed_update");
  EXPECT_EQ(rows[1].feature_update_reason, "update_blocked_by_rejected_assignment");
  EXPECT_EQ(rows[1].geometry_update_reason, "update_blocked_by_rejected_assignment");
  EXPECT_EQ(store.Find(3)->occlusion_protect_remaining, 30);
  EXPECT_EQ(bindings.SemanticIdForRawTrack(101), 7);
  EXPECT_EQ(bindings.SemanticIdForRawTrack(99), -1);
}

TEST(AssignmentApplicationExecutorTest, SideRecoveryForcesGeometryUpdate) {
  AssignmentApplicationPlan::Result plan;
  plan.applications.push_back(Application(0, 201, 11, "merged_side_recovery", 0.30F, 0.40F));

  std::vector<Track> tracks{MakeTrack(201, 0.0F)};
  std::unordered_map<int, std::size_t> feature_index{{0, 0}};
  std::vector<std::vector<float>> features{{0.0F, 1.0F, 0.0F}};
  std::vector<TestDebugRow> rows{TestDebugRow{4, 0, 11, 201, true, false, false, "", ""}};
  LegacyIdentityStore store;
  RawSemanticBindingStore bindings;

  bool forced_geometry = false;
  AssignmentApplicationExecutor<TestDebugRow>::Config config;
  config.frame_idx = 4;

  AssignmentApplicationExecutor<TestDebugRow>::Execute(
      plan, {0}, tracks, feature_index, features, &rows, &store, &bindings, config,
      [&](const Track &, int, int, const vision_demo_host::LegacyIdentityRecord *, float, float,
          const bool force_geometry_update) {
        forced_geometry = force_geometry_update;
        FeatureUpdatePolicy::Decision decision;
        decision.geometry_update_allowed = force_geometry_update;
        decision.geometry_update_reason = force_geometry_update ? "allowed_update" : "";
        decision.feature_update_reason = "unreliable_low_quality_observation";
        return decision;
      },
      [](const Track &, int, const std::vector<float> &, const FeatureUpdatePolicy::Decision &decision, float,
         float) {
        EXPECT_TRUE(decision.geometry_update_allowed);
        EXPECT_EQ(decision.geometry_update_reason, "allowed_update");
      });

  EXPECT_TRUE(forced_geometry);
  EXPECT_TRUE(rows[0].geometry_update_allowed);
  EXPECT_EQ(rows[0].geometry_update_reason, "allowed_update");
}
