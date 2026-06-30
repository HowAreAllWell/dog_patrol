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
