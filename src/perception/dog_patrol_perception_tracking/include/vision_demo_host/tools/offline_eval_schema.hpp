#pragma once

#include <string>

namespace vision_demo_host::tools {

inline std::string TrackletHypothesesCsvHeader() {
  return "frame_idx,hypothesis_idx,status,raw_track_id,class_id,score,x,y,w,h,reason,related_raw_track_id,"
         "assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,assoc_appearance_used,"
         "assoc_final_gate,assoc_reject_reason";
}

inline std::string TrackletHypothesesCsvHelp() {
  return
      "Hypotheses acceptance CSV:\n"
      "  tracklet_hypotheses.csv is written when --save-tracks-csv=true.\n"
      "  Schema: frame_idx,hypothesis_idx,status,raw_track_id,class_id,score,x,y,w,h,reason,"
      "related_raw_track_id,assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,"
      "assoc_appearance_used,assoc_final_gate,assoc_reject_reason\n"
      "  frame_idx is the offline video frame index, starting at 0. It is the join key for Phase 3\n"
      "  phase3_shadow_state.csv hypothesis_input and SplitCandidate evidence rows.\n"
      "  Use frame_idx around 760, 795, and 1030 to review tracked, suppressed, and hidden candidates.\n"
      "  status/reason explains whether a candidate is final tracked output, a suppressed new-track duplicate,\n"
      "  or duplicate output hidden; related_raw_track_id links a suppressed or hidden candidate to the raw track\n"
      "  that caused the suppression relation.\n";
}

inline std::string Phase3ShadowStateCsvHeader() {
  return "frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,carrier_raw_track_id,"
         "candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,candidate_y,candidate_w,"
         "candidate_h,reason,related_raw_track_id,hypothesis_status,candidate_stable_frames,group_age_frames,"
         "group_last_update_frame,decision_app_cost,decision_geo_cost,decision_time_cost,decision_final_score,"
         "decision_margin,decision_selected,decision_accepted,pairwise_selected_pairs,pairwise_alternate_pairs,"
         "pairwise_selected_final_cost,pairwise_alternate_final_cost,pairwise_selected_app_cost,"
         "pairwise_alternate_app_cost,pairwise_margin,pairwise_appearance_override";
}

inline std::string Phase3ShadowStateCsvHelp() {
  return
      "Phase 3 shadow state CSV:\n"
      "  phase3_shadow_state.csv is written when --save-tracks-csv=true.\n"
      "  Schema: frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,"
      "carrier_raw_track_id,candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,"
      "candidate_y,candidate_w,candidate_h,reason,related_raw_track_id,hypothesis_status,"
      "candidate_stable_frames,group_age_frames,group_last_update_frame,"
      "decision_app_cost,decision_geo_cost,decision_time_cost,decision_final_score,decision_margin,"
      "decision_selected,decision_accepted,pairwise_selected_pairs,pairwise_alternate_pairs,"
      "pairwise_selected_final_cost,pairwise_alternate_final_cost,pairwise_selected_app_cost,"
      "pairwise_alternate_app_cost,pairwise_margin,pairwise_appearance_override\n"
      "  frame_idx is the same 0-based offline video frame index used by tracklet_hypotheses.csv.\n"
      "  Join candidate rows by frame_idx plus candidate_raw_track_id/raw_track_id, reason, and\n"
      "  related_raw_track_id. group_last_update_frame uses the same frame_idx convention.\n"
      "  This file is shadow-only. It records identity-layer debug rows derived from tracklet_hypotheses.csv\n"
      "  and does not change semantic id assignment, primary state, overlay, UDP, or LegacyIdentityMatcher output.\n"
      "  Phase 5 birth observability emits NewBirthCandidate shadow lifecycle rows derived from legacy\n"
      "  sid_scores.csv rows and currently remains shadow-only: event_type=new_birth_candidate_pending\n"
      "  for small new-person stability waits, event_type=new_birth_candidate_hidden for legacy hidden or\n"
      "  pending birth decisions, and event_type=new_birth_candidate_allocated for accepted new_semantic\n"
      "  allocation. Reasons include small_new_person_pending, small_stable_new_person_promoted,\n"
      "  new_semantic_allocated, ambiguous_recovery_pending, duplicate_split_hidden,\n"
      "  skinny_partial_hidden, and wide_fragment_hidden. candidate_semantic_id=-1 means no semantic id\n"
      "  was allocated; decision_* fields mirror the legacy birth/new_semantic score row when present.\n"
      "  With --sid-enable-phase5-birth-manager=true, accepted birth allocation is applied by the\n"
      "  Phase 5 path behind IdentityManager and recorded as stage=phase5_new_semantic plus\n"
      "  reason=phase5_birth_manager_allocated; false keeps legacy new_semantic allocation.\n"
      "  It emits hypothesis_input rows plus MergedGroup lifecycle rows: merged_group_enter,\n"
      "  merged_group_update, and merged_group_end. It also emits SplitCandidate lifecycle rows:\n"
      "  split_candidate_enter, split_candidate_update, and split_candidate_end. SplitCandidate rows preserve\n"
      "  evidence reason and related_raw_track_id so they can be linked back to tracklet_hypotheses.csv.\n"
      "  Single-blob carrier evaluation emits event_type=single_blob_handoff_decision rows with\n"
      "  reason values such as single_blob_continuity_kept, single_blob_handoff_eligible,\n"
      "  single_blob_rejected_by_missing_age, single_blob_rejected_by_appearance_or_geometry_margin,\n"
      "  and single_blob_handoff_accepted. Decision cost fields mirror the legacy merged_candidate\n"
      "  score row as shadow-only evidence and do not migrate merged single-blob handoff behavior.\n"
      "  The accepted Phase 4 handoff paths are enabled by default; pass an enable flag as false\n"
      "  to roll that specific rule back to its legacy path.\n"
      "  With --sid-enable-phase4-merged-split-handoff=true, the migrated Phase 4 split handoff\n"
      "  emits event_type=phase4_merged_split_handoff with reason=merged_split_handoff.\n"
      "  With --sid-enable-phase4-merged-side-recovery=true, the migrated Phase 4 side recovery\n"
      "  emits event_type=phase4_merged_side_recovery with reason=merged_side_recovery.\n"
      "  With --sid-enable-phase4-merged-single-blob-handoff=true, the migrated Phase 4\n"
      "  single-blob handoff emits event_type=phase4_merged_single_blob_handoff with\n"
      "  reason=merged_single_blob_handoff and reuses the single_blob_handoff_decision cost fields.\n"
      "  Side-reappearance observability emits event_type=side_reappearance_candidate with\n"
      "  reason=side_reappearance_candidate when a side raw track is recovered by legacy merged_side_recovery;\n"
      "  the migrated Phase 4 row is the flag-enabled replacement. Both rows link the candidate raw id\n"
      "  to the preceding group/carrier and missing semantic guess.\n"
      "  2x2 pairwise assignment observability emits event_type=pairwise_assignment_matrix when two\n"
      "  candidate tracks and two missing semantic identities have a valid selected-vs-alternate matrix.\n"
      "  pairwise_* fields record selected pairs, alternate pairs, final-cost sums, appearance-cost sums,\n"
      "  margin, and whether the appearance override would trigger.\n"
      "  With --sid-enable-phase4-pairwise-assignment=true, the migrated Phase 4 pairwise path\n"
      "  emits event_type=phase4_pairwise_assignment with reason=pairwise_appearance_override and reuses\n"
      "  the pairwise_* matrix fields. With the flag disabled these rows remain shadow-only evidence.\n"
      "  Acceptance review windows: 01:746-771 for group lifecycle, 01:793-795 for hidden split candidates,\n"
      "  01:1015-1031 for split recovery evidence, and 02:790-850 for the second dataset handoff case.\n";
}

}  // namespace vision_demo_host::tools
