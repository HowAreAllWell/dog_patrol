#include "dog_patrol_perception_tracking/modules/primary_recovery_debug.hpp"

#include <algorithm>
#include <sstream>

namespace dog_patrol_perception_tracking {
namespace {

const IdentityObservation *FindPrimaryIdentity(const PrimaryTargetResult &primary,
                                               const IdentityManagerResult &identity_result) {
  if (primary.primary_target_id <= 0) {
    return nullptr;
  }
  for (const auto &identity : identity_result.identities) {
    if (identity.semantic_id == primary.primary_target_id) {
      return &identity;
    }
  }
  return nullptr;
}

std::string TokenFromRejectReason(const std::string_view reason) {
  if (reason == "visible_primary_center_jump") {
    return "center_jump";
  }
  if (reason == "visible_primary_low_score_update") {
    return "low_score";
  }
  if (reason == "visible_primary_assoc_gate_failed") {
    return "assoc_gate";
  }
  return "";
}

std::string TokenFromIdentityState(const IdentityObservation *identity) {
  if (identity == nullptr) {
    return "";
  }
  if (identity->state == IdentityState::kMerged) {
    return "merged";
  }
  if (identity->state == IdentityState::kSplitRecovery) {
    return "split_recovery";
  }
  return "";
}

}  // namespace

int PrimarySupportingRawTrackIdDebug(const PrimaryTargetResult &primary,
                                     const IdentityManagerResult &identity_result) {
  if (primary.raw_track_id > 0) {
    return primary.raw_track_id;
  }
  const auto *identity = FindPrimaryIdentity(primary, identity_result);
  if (identity != nullptr && identity->supporting_raw_track_id.has_value()) {
    return *identity->supporting_raw_track_id;
  }
  return -1;
}

std::string PrimaryRecoveryReasonToken(const PrimaryTargetResult &primary,
                                       const IdentityManagerResult &identity_result,
                                       const std::string_view decision_reason,
                                       const std::string_view reject_reason) {
  if (primary.state != PrimaryState::kPendingRecovery) {
    return "";
  }

  if (decision_reason == "pending_recovery_visible_primary_sanity_rejected") {
    const auto token = TokenFromRejectReason(reject_reason);
    if (!token.empty()) {
      return token;
    }
  }

  if (decision_reason == "pending_recovery_from_identity_state") {
    const auto token = TokenFromIdentityState(FindPrimaryIdentity(primary, identity_result));
    if (!token.empty()) {
      return token;
    }
  }

  return "pending";
}

std::string BuildPrimaryOverlayLine(const PrimaryTargetResult &primary,
                                    const IdentityManagerResult &identity_result,
                                    const std::string_view decision_reason,
                                    const std::string_view reject_reason) {
  std::ostringstream line;
  line << PrimaryStateToString(primary.state)
       << " id=" << primary.primary_target_id
       << " raw=" << PrimarySupportingRawTrackIdDebug(primary, identity_result);

  const auto reason_token = PrimaryRecoveryReasonToken(primary, identity_result, decision_reason, reject_reason);
  if (!reason_token.empty()) {
    line << " reason=" << reason_token;
  }
  if (identity_result.feature_update_frozen) {
    line << " freeze";
  }
  return line.str();
}

cv::Point CompactOverlayTrackLabelPoint(const cv::Size &frame_size, const cv::Rect2f &bbox) {
  const int tx = std::min(frame_size.width - 2, std::max(0, static_cast<int>(bbox.x) + 4));
  const int ty = std::min(frame_size.height - 2, std::max(14, static_cast<int>(bbox.y) + 16));
  return cv::Point(tx, ty);
}

}  // namespace dog_patrol_perception_tracking
