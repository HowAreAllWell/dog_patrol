#pragma once

#include <string>
#include <string_view>

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

}  // namespace vision_demo_host
