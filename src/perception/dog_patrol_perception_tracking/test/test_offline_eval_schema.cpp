#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "vision_demo_host/tools/offline_eval_schema.hpp"

TEST(OfflineEvalSchemaTest, PerFrameCsvHeaderAppendsPrimaryRecoveryDebugFields) {
  const std::string expected =
      "frame_idx,det_count,track_count,track_state,primary_semantic_id,primary_raw_track_id_debug,"
      "bearing_base_rad,sid_mode,sid_freeze,visible_semantic_ids,primary_decision_reason,"
      "primary_reject_reason,primary_recovery_reason,primary_supporting_raw_track_id_debug";

  EXPECT_EQ(vision_demo_host::tools::PerFrameCsvHeader(), expected);
}

TEST(OfflineEvalSchemaTest, PerFrameCsvHelpDocumentsPendingRecoveryReasonTokens) {
  const std::string help = vision_demo_host::tools::PerFrameCsvHelp();
  const std::vector<std::string> required_terms{
      "per_frame.csv",
      "PENDING_RECOVERY",
      "primary_recovery_reason",
      "primary_supporting_raw_track_id_debug",
      "center_jump",
      "low_score",
      "assoc_gate",
      "merged",
      "split_recovery",
      "pending",
      "pending_recovery_hold_missing_identity_evidence",
      "appended",
      "UDP",
  };
  for (const auto &term : required_terms) {
    EXPECT_NE(help.find(term), std::string::npos) << term;
  }
}

TEST(OfflineEvalSchemaTest, TrackletHypothesesCsvHeaderIsStable) {
  const std::string expected =
      "frame_idx,hypothesis_idx,status,raw_track_id,class_id,score,x,y,w,h,reason,related_raw_track_id,"
      "assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,assoc_appearance_used,"
      "assoc_final_gate,assoc_reject_reason";

  EXPECT_EQ(vision_demo_host::tools::TrackletHypothesesCsvHeader(), expected);
}

TEST(OfflineEvalSchemaTest, TrackletHypothesesSchemaDocumentsReviewFields) {
  const std::vector<std::string> required_fields{
      "frame_idx",
      "hypothesis_idx",
      "raw_track_id",
      "class_id",
      "score",
      "x,y,w,h",
      "status",
      "reason",
      "related_raw_track_id",
      "assoc_stage",
      "assoc_cost",
      "assoc_iou",
      "assoc_motion_dist",
      "assoc_app_dist",
      "assoc_appearance_used",
      "assoc_final_gate",
      "assoc_reject_reason",
      "0",
      "join key",
      "phase3_shadow_state.csv",
  };

  const std::string help = vision_demo_host::tools::TrackletHypothesesCsvHelp();
  EXPECT_NE(help.find("tracklet_hypotheses.csv"), std::string::npos);
  EXPECT_NE(help.find("760"), std::string::npos);
  EXPECT_NE(help.find("795"), std::string::npos);
  EXPECT_NE(help.find("1030"), std::string::npos);
  for (const auto &field : required_fields) {
    EXPECT_NE(help.find(field), std::string::npos) << field;
  }
}

TEST(OfflineEvalSchemaTest, SidScoresCsvHeaderDocumentsFeatureUpdatePolicyReasons) {
  const std::string expected =
      "frame_idx,sid_mode,track_idx,raw_track_id,semantic_id,app_cost,geo_cost,time_cost,final_score,"
      "stage,selected,margin,accepted,reject_reason,continuity_used,feature_update_allowed,"
      "geometry_update_allowed,feature_update_reason,geometry_update_reason";

  EXPECT_EQ(vision_demo_host::tools::SidScoresCsvHeader(), expected);

  const std::string help = vision_demo_host::tools::SidScoresCsvHelp();
  const std::vector<std::string> required_terms{
      "sid_scores.csv",
      "feature_update_allowed",
      "geometry_update_allowed",
      "sid_freeze",
      "feature_update_reason",
      "geometry_update_reason",
      "allowed_update",
      "global_merge_split_freeze",
      "overlapping_track_freeze",
      "unreliable_low_quality_observation",
      "insufficient_stable_frames",
      "update_blocked_by_rejected_assignment",
      "Phase 4/5",
  };
  for (const auto &term : required_terms) {
    EXPECT_NE(help.find(term), std::string::npos) << term;
  }
}

TEST(OfflineEvalSchemaTest, Phase3ShadowStateCsvHeaderIsStable) {
  const std::string expected =
      "frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,carrier_raw_track_id,"
      "candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,candidate_y,candidate_w,"
      "candidate_h,reason,related_raw_track_id,hypothesis_status,candidate_stable_frames,group_age_frames,"
      "group_last_update_frame,decision_app_cost,decision_geo_cost,decision_time_cost,decision_final_score,"
      "decision_margin,decision_selected,decision_accepted,pairwise_selected_pairs,pairwise_alternate_pairs,"
      "pairwise_selected_final_cost,pairwise_alternate_final_cost,pairwise_selected_app_cost,"
      "pairwise_alternate_app_cost,pairwise_margin,pairwise_appearance_override";

  EXPECT_EQ(vision_demo_host::tools::Phase3ShadowStateCsvHeader(), expected);
}

TEST(OfflineEvalSchemaTest, Phase3ShadowStateHelpDocumentsShadowOnlyContract) {
  const std::vector<std::string> required_terms{
      "phase3_shadow_state.csv",
      "frame_idx",
      "event_type",
      "group_id",
      "semantic_ids",
      "carrier_semantic_id",
      "carrier_raw_track_id",
      "candidate_raw_track_id",
      "candidate_semantic_id",
      "candidate_score",
      "candidate_x",
      "candidate_y",
      "candidate_w",
      "candidate_h",
      "reason",
      "related_raw_track_id",
      "hypothesis_status",
      "candidate_stable_frames",
      "group_age_frames",
      "group_last_update_frame",
      "decision_app_cost",
      "decision_geo_cost",
      "decision_time_cost",
      "decision_final_score",
      "decision_margin",
      "decision_selected",
      "decision_accepted",
      "NewBirthCandidate",
      "new_birth_candidate_pending",
      "new_birth_candidate_hidden",
      "new_birth_candidate_allocated",
      "small_new_person_pending",
      "small_stable_new_person_promoted",
      "phase5_new_semantic",
      "phase5_birth_manager_allocated",
      "ambiguous_recovery_pending",
      "duplicate_split_hidden",
      "skinny_partial_hidden",
      "wide_fragment_hidden",
      "pairwise_assignment_matrix",
      "pairwise_selected_pairs",
      "pairwise_alternate_pairs",
      "pairwise_selected_final_cost",
      "pairwise_alternate_final_cost",
      "pairwise_selected_app_cost",
      "pairwise_alternate_app_cost",
      "pairwise_margin",
      "pairwise_appearance_override",
      "2x2 pairwise assignment",
      "single_blob_handoff_decision",
      "single_blob_continuity_kept",
      "single_blob_handoff_eligible",
      "single_blob_rejected_by_missing_age",
      "single_blob_rejected_by_appearance_or_geometry_margin",
      "single_blob_handoff_accepted",
      "split_candidate_enter",
      "split_candidate_update",
      "split_candidate_end",
      "phase4_merged_split_handoff",
      "merged_split_handoff",
      "phase4_merged_side_recovery",
      "side_reappearance_candidate",
      "merged_side_recovery",
      "phase4_merged_single_blob_handoff",
      "merged_single_blob_handoff",
      "phase4_pairwise_assignment",
      "tracklet_hypotheses.csv",
      "0-based",
      "Join candidate rows",
      "group_last_update_frame",
      "01:746-771",
      "01:793-795",
      "01:1015-1031",
      "02:790-850",
      "shadow-only",
      "semantic id",
      "primary",
      "overlay",
      "UDP",
  };

  const std::string help = vision_demo_host::tools::Phase3ShadowStateCsvHelp();
  for (const auto &term : required_terms) {
    EXPECT_NE(help.find(term), std::string::npos) << term;
  }
}

TEST(OfflineEvalSchemaTest, IdentityOfflineMetricsHelpDocumentsAdditiveOutput) {
  const std::string help = vision_demo_host::tools::IdentityOfflineMetricsHelp();
  const std::vector<std::string> required_terms{
      "identity_metrics.json",
      "identity_metrics.md",
      "additive",
      "per_frame.csv",
      "identities.csv",
      "sid_scores.csv",
      "phase3_shadow_state.csv",
      "tracklet_hypotheses.csv",
      "unavailable",
      "PENDING_RECOVERY",
      "primary decision",
      "identity states",
      "target lifecycle counts",
      "VisibleIdentity",
      "OccludedIdentity",
      "MergedGroup",
      "SplitCandidate",
      "LostIdentity",
      "assignment stages",
      "feature_update_reason",
      "geometry_update_reason",
      "event_type",
      "NewBirthCandidate",
      "Phase 4 handoff",
      "tracklet hypothesis",
  };
  for (const auto &term : required_terms) {
    EXPECT_NE(help.find(term), std::string::npos) << term;
  }
}
