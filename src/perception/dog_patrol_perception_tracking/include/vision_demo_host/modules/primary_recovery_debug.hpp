#pragma once

#include <string>
#include <string_view>

#include <opencv2/core.hpp>

#include "vision_demo_host/types.hpp"

namespace vision_demo_host {

int PrimarySupportingRawTrackIdDebug(const PrimaryTargetResult &primary,
                                     const IdentityManagerResult &identity_result);

std::string PrimaryRecoveryReasonToken(const PrimaryTargetResult &primary,
                                       const IdentityManagerResult &identity_result,
                                       std::string_view decision_reason,
                                       std::string_view reject_reason);

std::string BuildPrimaryOverlayLine(const PrimaryTargetResult &primary,
                                    const IdentityManagerResult &identity_result,
                                    std::string_view decision_reason,
                                    std::string_view reject_reason);

cv::Point CompactOverlayTrackLabelPoint(const cv::Size &frame_size, const cv::Rect2f &bbox);

}  // namespace vision_demo_host
