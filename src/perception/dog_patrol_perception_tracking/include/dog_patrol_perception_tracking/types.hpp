#pragma once

#include <cstdint>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace dog_patrol_perception_tracking {

enum class ClassId : int {
  kPerson = 0,
  kCar = 1,
  kUnknown = -1,
};

struct Detection {
  ClassId class_id{ClassId::kUnknown};
  float confidence{0.0F};
  cv::Rect2f bbox;
};

struct AssociationEvidence {
  std::string stage;
  float fused_cost{1.0F};
  float iou{0.0F};
  float motion_dist{0.0F};
  float motion_term{1.0F};
  float app_dist{1.0F};
  bool appearance_used{false};
  bool low_score_detection{false};
  bool recovered_from_lost{false};
  bool passed_final_cost_gate{false};
  std::string reject_reason;
};

struct Track {
  int id{-1};
  ClassId class_id{ClassId::kUnknown};
  float confidence{0.0F};
  cv::Rect2f bbox;
  bool is_confirmed{false};
  int time_since_update{0};
  bool authoritative{false};
  bool occlusion_suspect{false};
  std::vector<float> appearance_feature;
  AssociationEvidence association;
  bool just_recovered{false};
  bool low_score_update{false};
};

enum class TrackletHypothesisStatus {
  kTracked,
  kTentative,
  kLostPrediction,
  kSuppressedDuplicateCandidate,
  kSplitCandidate,
  kLowQualityCandidate,
};

struct TrackletHypothesis {
  int raw_track_id{-1};
  ClassId class_id{ClassId::kUnknown};
  float confidence{0.0F};
  cv::Rect2f bbox;
  TrackletHypothesisStatus status{TrackletHypothesisStatus::kTracked};
  std::string candidate_reason;
  std::optional<int> related_raw_track_id;
  AssociationEvidence association;
};

struct TrackletObservation {
  int raw_track_id{-1};
  ClassId class_id{ClassId::kUnknown};
  float confidence{0.0F};
  cv::Rect2f bbox;
  bool is_confirmed{false};
  int time_since_update{0};
  bool authoritative{false};
  bool occlusion_suspect{false};
  std::vector<float> appearance_feature;
  AssociationEvidence association;
  bool just_recovered{false};
  bool low_score_update{false};

  static TrackletObservation FromTrack(const Track &track) {
    TrackletObservation obs;
    obs.raw_track_id = track.id;
    obs.class_id = track.class_id;
    obs.confidence = track.confidence;
    obs.bbox = track.bbox;
    obs.is_confirmed = track.is_confirmed;
    obs.time_since_update = track.time_since_update;
    obs.authoritative = track.authoritative;
    obs.occlusion_suspect = track.occlusion_suspect;
    obs.appearance_feature = track.appearance_feature;
    obs.association = track.association;
    obs.just_recovered = track.just_recovered;
    obs.low_score_update = track.low_score_update;
    return obs;
  }

  Track ToTrack() const {
    Track track;
    track.id = raw_track_id;
    track.class_id = class_id;
    track.confidence = confidence;
    track.bbox = bbox;
    track.is_confirmed = is_confirmed;
    track.time_since_update = time_since_update;
    track.authoritative = authoritative;
    track.occlusion_suspect = occlusion_suspect;
    track.appearance_feature = appearance_feature;
    track.association = association;
    track.just_recovered = just_recovered;
    track.low_score_update = low_score_update;
    return track;
  }
};

enum class PrimaryState {
  kIdle,
  kLocked,
  kOccluded,
  kLost,
  kPendingRecovery,
};

struct PrimaryTargetResult {
  PrimaryState state{PrimaryState::kIdle};
  std::optional<Track> primary_track;
  int primary_target_id{-1};  // Business-stable ID (not raw tracker ID).
  int raw_track_id{-1};       // Tracker-provided raw ID for debug/internal use.
  int missing_frames{0};
};

// Shared trust predicate for any public projection of the current primary.
// Frame-specific representability (image bounds and metadata) remains the
// responsibility of the projection boundary.
inline bool IsTrustedCurrentPrimary(const PrimaryTargetResult &primary) {
  if (primary.state != PrimaryState::kLocked || primary.primary_target_id <= 0 ||
      !primary.primary_track.has_value()) {
    return false;
  }
  const Track &track = primary.primary_track.value();
  return track.authoritative && track.id == primary.raw_track_id && track.is_confirmed &&
         track.class_id == ClassId::kPerson && !track.occlusion_suspect &&
         !track.low_score_update && !track.association.low_score_detection &&
         !track.just_recovered && !track.association.recovered_from_lost &&
         (track.association.stage.empty() || track.association.passed_final_cost_gate) &&
         std::isfinite(track.confidence) && track.confidence >= 0.0F &&
         track.confidence <= 1.0F;
}

enum class IdentityState {
  kActive,
  kOccluded,
  kInactive,
  kLost,
  kMerged,
  kSplitRecovery,
  kUnknown,
};

struct IdentityAssignmentEvidence {
  float app_cost{0.0F};
  float geo_cost{0.0F};
  float time_cost{0.0F};
  float final_score{0.0F};
  float margin{0.0F};
  bool selected{false};
  bool accepted{false};
  bool continuity_used{false};
  bool feature_update_allowed{false};
  bool geometry_update_allowed{false};
  std::string feature_update_reason;
  std::string geometry_update_reason;
  std::string stage;
  std::string reject_reason;
};

struct IdentityObservation {
  int semantic_id{-1};
  IdentityState state{IdentityState::kUnknown};
  std::optional<int> supporting_raw_track_id;
  std::optional<TrackletObservation> supporting_tracklet;
  ClassId class_id{ClassId::kUnknown};
  float confidence{0.0F};
  cv::Rect2f bbox;
  int missing_frames{0};
  bool visible{false};
  bool primary{false};
  bool occlusion_suspect{false};
  bool low_score_update{false};
  bool just_recovered{false};
  AssociationEvidence association;
  IdentityAssignmentEvidence assignment;
};

struct IdentityManagerResult {
  std::vector<IdentityObservation> identities;
  int primary_semantic_id{-1};
  bool feature_update_frozen{false};

  int SemanticIdForRawTrack(int raw_track_id) const {
    for (const auto &identity : identities) {
      if (identity.supporting_raw_track_id.has_value() && *identity.supporting_raw_track_id == raw_track_id) {
        return identity.semantic_id;
      }
    }
    return -1;
  }
};

inline std::string ClassIdToString(ClassId class_id) {
  switch (class_id) {
    case ClassId::kPerson:
      return "person";
    case ClassId::kCar:
      return "car";
    default:
      return "unknown";
  }
}

inline std::string PrimaryStateToString(const PrimaryState state) {
  switch (state) {
    case PrimaryState::kIdle:
      return "IDLE";
    case PrimaryState::kLocked:
      return "LOCKED";
    case PrimaryState::kOccluded:
      return "OCCLUDED";
    case PrimaryState::kPendingRecovery:
      return "PENDING_RECOVERY";
    case PrimaryState::kLost:
      return "LOST";
    default:
      return "UNKNOWN";
  }
}

inline std::string IdentityStateToString(const IdentityState state) {
  switch (state) {
    case IdentityState::kActive:
      return "ACTIVE";
    case IdentityState::kOccluded:
      return "OCCLUDED";
    case IdentityState::kInactive:
      return "INACTIVE";
    case IdentityState::kLost:
      return "LOST";
    case IdentityState::kMerged:
      return "MERGED";
    case IdentityState::kSplitRecovery:
      return "SPLIT_RECOVERY";
    default:
      return "UNKNOWN";
  }
}

}  // namespace dog_patrol_perception_tracking
