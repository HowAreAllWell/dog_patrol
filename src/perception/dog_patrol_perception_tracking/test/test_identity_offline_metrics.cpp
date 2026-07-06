#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "vision_demo_host/tools/identity_offline_metrics.hpp"
#include "vision_demo_host/tools/offline_eval_schema.hpp"

namespace {

std::filesystem::path MakeTempDir(const std::string &name) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void WriteText(const std::filesystem::path &path, const std::string &text) {
  std::ofstream ofs(path);
  ofs << text;
}

}  // namespace

TEST(IdentityOfflineMetricsTest, AggregatesIdentityAcceptanceDistributionsFromDebugCsvs) {
  const std::filesystem::path dir = MakeTempDir("vision_demo_identity_metrics_full");
  WriteText(dir / "per_frame.csv",
            vision_demo_host::tools::PerFrameCsvHeader() +
                "\n"
                "0,2,1,LOCKED,1,10,0.000000,NORMAL,0,1,locked_primary,,,\n"
                "1,2,1,PENDING_RECOVERY,1,-1,0.000000,MERGED,1,1,pending_recovery_from_identity_state,"
                "visible_primary_low_score_update,low_score,10\n"
                "2,2,1,PENDING_RECOVERY,1,-1,0.000000,MERGED,1,1,"
                "pending_recovery_hold_missing_identity_evidence,,pending,10\n");
  WriteText(dir / "identities.csv",
            "frame_idx,semantic_id,identity_state,visible,supporting_raw_track_id,class_id,score,x,y,w,h,"
            "missing_frames,primary,occlusion_suspect,low_score_update,just_recovered,assignment_stage,"
            "assignment_accepted,assignment_reject_reason\n"
            "0,1,VISIBLE,1,10,0,0.90,1,2,3,4,0,1,0,0,0,assign_candidate,1,\n"
            "1,1,MERGED,0,-1,0,0.10,1,2,3,4,1,1,1,0,0,inactive_recover_candidate,0,"
            "missing_appearance_gate_reject\n"
            "1,3,LOST,0,12,0,0.10,1,2,3,4,12,0,0,0,0,,0,\n");
  WriteText(dir / "sid_scores.csv",
            vision_demo_host::tools::SidScoresCsvHeader() +
                "\n"
                "1,NORMAL,0,10,1,0.1,0.2,0.0,0.3,assign_candidate,1,0.2,1,,1,1,1,allowed_update,"
                "allowed_update\n"
                "1,NORMAL,1,11,-1,0.5,0.6,0.0,0.7,phase5_birth_candidate,0,0.0,0,"
                "small_new_person_pending,0,0,0,update_blocked_by_rejected_assignment,"
                "insufficient_stable_frames\n");
  WriteText(dir / "phase3_shadow_state.csv",
            vision_demo_host::tools::Phase3ShadowStateCsvHeader() +
                "\n"
                "1,0,split_candidate_enter,4,1|2,1,10,11,-1,0.850000,1,2,3,4,duplicate_split_hidden,10,"
                "split_candidate,1,2,1,0.1,0.2,0.0,0.3,0.4,1,0,,,,,,,,\n"
                "1,1,new_birth_candidate_allocated,-1,,1,10,12,3,0.900000,1,2,3,4,"
                "phase5_birth_manager_allocated,-1,tracked,3,0,1,0.1,0.2,0.0,0.3,0.4,1,1,,,,,,,,\n"
                "1,2,phase4_pairwise_assignment,4,1|2,1,10,11,2,0.900000,1,2,3,4,"
                "pairwise_appearance_override,10,tracked,3,2,1,0.1,0.2,0.0,0.3,0.4,1,1,1-10,2-11,"
                "0.3,0.6,0.1,0.5,0.3,1\n");
  WriteText(dir / "tracklet_hypotheses.csv",
            vision_demo_host::tools::TrackletHypothesesCsvHeader() +
                "\n"
                "1,0,split_candidate,11,0,0.850000,1,2,3,4,duplicate_output_hidden,10,high_iou,0.2,0.8,"
                "0.1,0.2,1,0,duplicate_output_hidden\n");

  const auto metrics = vision_demo_host::tools::BuildIdentityOfflineMetrics(dir, "synthetic");

  EXPECT_TRUE(metrics.inputs.at("per_frame.csv").available);
  EXPECT_EQ(metrics.primary_state_counts.at("LOCKED"), 1);
  EXPECT_EQ(metrics.primary_state_counts.at("PENDING_RECOVERY"), 2);
  EXPECT_EQ(metrics.primary_decision_reason_counts.at("pending_recovery_from_identity_state"), 1);
  EXPECT_EQ(metrics.primary_decision_reason_counts.at("pending_recovery_hold_missing_identity_evidence"), 1);
  EXPECT_EQ(metrics.primary_reject_reason_counts.at("visible_primary_low_score_update"), 1);
  EXPECT_EQ(metrics.primary_recovery_reason_counts.at("low_score"), 1);
  EXPECT_EQ(metrics.primary_recovery_reason_counts.at("pending"), 1);
  EXPECT_EQ(metrics.identity_state_counts.at("VISIBLE"), 1);
  EXPECT_EQ(metrics.identity_state_counts.at("MERGED"), 1);
  EXPECT_EQ(metrics.target_lifecycle_counts.at("VisibleIdentity"), 1);
  EXPECT_EQ(metrics.target_lifecycle_counts.at("MergedGroup"), 1);
  EXPECT_EQ(metrics.target_lifecycle_counts.at("NewBirthCandidate"), 1);
  EXPECT_EQ(metrics.assignment_stage_counts.at("assign_candidate"), 2);
  EXPECT_EQ(metrics.assignment_reject_reason_counts.at("missing_appearance_gate_reject"), 1);
  EXPECT_EQ(metrics.feature_update_reason_counts.at("allowed_update"), 1);
  EXPECT_EQ(metrics.geometry_update_reason_counts.at("insufficient_stable_frames"), 1);
  EXPECT_EQ(metrics.phase3_event_type_counts.at("split_candidate_enter"), 1);
  EXPECT_EQ(metrics.birth_reason_counts.at("phase5_birth_manager_allocated"), 1);
  EXPECT_EQ(metrics.phase4_handoff_event_counts.at("phase4_pairwise_assignment"), 1);
  EXPECT_EQ(metrics.tracklet_hypothesis_status_counts.at("split_candidate"), 1);
  EXPECT_EQ(metrics.tracklet_hypothesis_reason_counts.at("duplicate_output_hidden"), 1);

  std::string error;
  EXPECT_TRUE(vision_demo_host::tools::WriteIdentityOfflineMetricsFiles(dir, metrics, &error)) << error;
  EXPECT_TRUE(std::filesystem::exists(dir / "identity_metrics.json"));
  EXPECT_TRUE(std::filesystem::exists(dir / "identity_metrics.md"));
}

TEST(IdentityOfflineMetricsTest, MarksMissingOptionalDebugInputsUnavailable) {
  const std::filesystem::path dir = MakeTempDir("vision_demo_identity_metrics_missing");
  WriteText(dir / "per_frame.csv",
            vision_demo_host::tools::PerFrameCsvHeader() +
                "\n"
                "0,0,0,OCCLUDED,-1,-1,0.000000,NORMAL,0,,no_primary_identity,,,\n");

  const auto metrics = vision_demo_host::tools::BuildIdentityOfflineMetrics(dir, "minimal");

  EXPECT_TRUE(metrics.inputs.at("per_frame.csv").available);
  EXPECT_FALSE(metrics.inputs.at("identities.csv").available);
  EXPECT_FALSE(metrics.inputs.at("sid_scores.csv").available);
  EXPECT_FALSE(metrics.inputs.at("phase3_shadow_state.csv").available);
  EXPECT_FALSE(metrics.inputs.at("tracklet_hypotheses.csv").available);
  EXPECT_EQ(metrics.primary_state_counts.at("OCCLUDED"), 1);
  EXPECT_TRUE(metrics.identity_state_counts.empty());
  EXPECT_TRUE(metrics.target_lifecycle_counts.empty());
  EXPECT_TRUE(metrics.phase4_handoff_event_counts.empty());
}
