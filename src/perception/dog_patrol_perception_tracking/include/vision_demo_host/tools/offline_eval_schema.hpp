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

}  // namespace vision_demo_host::tools
