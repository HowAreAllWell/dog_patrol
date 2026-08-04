#pragma once

#include <algorithm>

#include "dog_patrol_perception_tracking/modules/association_utils.hpp"

#include "identity_runtime_record.hpp"
#include "identity_runtime_snapshot.hpp"

namespace dog_patrol_perception_tracking {

class IdentityRuntimeRecordLifecycle {
 public:
  static void BeginFrame(IdentityRuntimeRecord *record) {
    if (record == nullptr) {
      return;
    }
    record->seen_this_frame = false;
  }

  static void ApplyObservation(const Track &track, const int frame_index, const float assignment_cost,
                               const float assignment_margin, IdentityRuntimeRecord *record) {
    if (record == nullptr) {
      return;
    }
    record->class_id = track.class_id;
    record->last_bbox = track.bbox;
    record->last_center = association::BBoxCenter(track.bbox);
    record->last_assignment_cost = assignment_cost;
    record->last_assignment_margin = assignment_margin;
    record->last_seen_frame = frame_index;
    record->missing_frames = 0;
    record->seen_this_frame = true;
    record->supporting_raw_track_id = track.id;
    record->confidence = track.confidence;
  }

  static void ProtectIfUnseenActivePerson(const int occlusion_protect_frames, IdentityRuntimeRecord *record) {
    if (record == nullptr || record->class_id != ClassId::kPerson || record->seen_this_frame ||
        record->missing_frames != 0) {
      return;
    }
    record->occlusion_protect_remaining =
        std::max(record->occlusion_protect_remaining, std::max(1, occlusion_protect_frames));
  }

  static void AgeOneFrame(IdentityRuntimeRecord *record) {
    if (record == nullptr) {
      return;
    }
    if (record->occlusion_protect_remaining > 0) {
      record->occlusion_protect_remaining -= 1;
    }
    if (record->seen_this_frame) {
      return;
    }
    record->supporting_raw_track_id = -1;
    record->confidence = 0.0F;
    record->missing_frames += 1;
  }

  static void MarkCarrierMissingForHandoff(IdentityRuntimeRecord *record) {
    if (record == nullptr) {
      return;
    }
    record->seen_this_frame = false;
    record->supporting_raw_track_id = -1;
    record->confidence = 0.0F;
    if (record->missing_frames == 0) {
      record->missing_frames = 1;
    }
  }

  static IdentityRuntimeSnapshot BuildSnapshot(const int semantic_id, const IdentityRuntimeRecord &record) {
    IdentityRuntimeSnapshot snapshot;
    snapshot.semantic_id = semantic_id;
    snapshot.class_id = record.class_id;
    snapshot.bbox = record.last_bbox;
    snapshot.reliable_bbox = record.feature_geometry.has_reliable_geometry ? record.feature_geometry.reliable_bbox
                                                                           : record.last_bbox;
    snapshot.has_reliable_geometry = record.feature_geometry.has_reliable_geometry;
    snapshot.missing_frames = record.missing_frames;
    snapshot.occlusion_protect_remaining = record.occlusion_protect_remaining;
    snapshot.seen_this_frame = record.seen_this_frame;
    snapshot.supporting_raw_track_id = record.supporting_raw_track_id;
    snapshot.confidence = record.confidence;
    return snapshot;
  }
};

}  // namespace dog_patrol_perception_tracking
