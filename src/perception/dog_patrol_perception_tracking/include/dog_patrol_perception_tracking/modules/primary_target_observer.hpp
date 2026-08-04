#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"
#include "dog_patrol_perception_tracking/source_frame_metadata.hpp"

namespace dog_patrol_perception_tracking {

// Current-frame, ROS-independent view of the trusted primary target. The
// image is a deep-owned crop and remains valid after the source frame is
// released or reused by the camera pipeline.
struct PrimaryTargetObservation {
  int target_id{-1};
  SourceFrameMetadata source;
  cv::Rect bbox;
  float confidence{0.0F};
  cv::Mat target_image;
};

class PrimaryTargetObserver {
 public:
  struct Output {
    PrimaryTargetResult primary;
    std::optional<PrimaryTargetObservation> observation;
    std::string primary_decision_reason;
    std::string primary_reject_reason;
  };

  PrimaryTargetObserver();
  explicit PrimaryTargetObserver(PrimaryTargetManager::Config config);

  // Advances primary selection using current identity evidence and returns an
  // observation only when that same frame contains a representable, trusted
  // primary. No mission state or state_seq is required.
  Output Update(const std::vector<IdentityObservation> &identities,
                const SourceFrameMetadata &source,
                const cv::Mat &source_image);

  PrimaryTargetResult CurrentPrimary() const;

 private:
  static std::optional<PrimaryTargetObservation> BuildObservation(
      const PrimaryTargetResult &primary,
      const SourceFrameMetadata &source,
      const cv::Mat &source_image);

  PrimaryTargetManager primary_manager_;
};

}  // namespace dog_patrol_perception_tracking
