#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "vision_demo_host/tools/offline_eval_schema.hpp"

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

TEST(OfflineEvalSchemaTest, Phase3ShadowStateCsvHeaderIsStable) {
  const std::string expected =
      "frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,carrier_raw_track_id,"
      "candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,candidate_y,candidate_w,"
      "candidate_h,reason,related_raw_track_id,hypothesis_status,candidate_stable_frames,group_age_frames,"
      "group_last_update_frame";

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
      "split_candidate_enter",
      "split_candidate_update",
      "split_candidate_end",
      "phase4_merged_split_handoff",
      "merged_split_handoff",
      "--sid-enable-phase4-merged-split-handoff",
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
      "LegacyIdentityMatcher",
  };

  const std::string help = vision_demo_host::tools::Phase3ShadowStateCsvHelp();
  for (const auto &term : required_terms) {
    EXPECT_NE(help.find(term), std::string::npos) << term;
  }
}
