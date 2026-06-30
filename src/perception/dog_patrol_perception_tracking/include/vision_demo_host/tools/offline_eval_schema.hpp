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
      "  Use frame_idx around 760, 795, and 1030 to review tracked, suppressed, and hidden candidates.\n"
      "  status/reason explains whether a candidate is final tracked output, a suppressed new-track duplicate,\n"
      "  or duplicate output hidden; related_raw_track_id links a suppressed or hidden candidate to the raw track\n"
      "  that caused the suppression relation.\n";
}

inline std::string Phase3ShadowStateCsvHeader() {
  return "frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,carrier_raw_track_id,"
         "candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,candidate_y,candidate_w,"
         "candidate_h,reason,related_raw_track_id,hypothesis_status,candidate_stable_frames,group_age_frames,"
         "group_last_update_frame";
}

inline std::string Phase3ShadowStateCsvHelp() {
  return
      "Phase 3 shadow state CSV:\n"
      "  phase3_shadow_state.csv is written when --save-tracks-csv=true.\n"
      "  Schema: frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,"
      "carrier_raw_track_id,candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,"
      "candidate_y,candidate_w,candidate_h,reason,related_raw_track_id,hypothesis_status,"
      "candidate_stable_frames,group_age_frames,group_last_update_frame\n"
      "  This file is shadow-only. It records identity-layer debug rows derived from tracklet_hypotheses.csv\n"
      "  and does not change semantic id assignment, primary state, overlay, UDP, or LegacyIdentityMatcher output.\n"
      "  It emits hypothesis_input rows plus MergedGroup lifecycle rows: merged_group_enter,\n"
      "  merged_group_update, and merged_group_end. It also emits SplitCandidate lifecycle rows:\n"
      "  split_candidate_enter, split_candidate_update, and split_candidate_end. SplitCandidate rows preserve\n"
      "  evidence reason and related_raw_track_id so they can be linked back to tracklet_hypotheses.csv.\n";
}

}  // namespace vision_demo_host::tools
