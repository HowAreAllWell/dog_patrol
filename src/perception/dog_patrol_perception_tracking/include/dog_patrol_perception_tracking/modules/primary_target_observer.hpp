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

// In-process consumer boundary for standalone integrations. Receiving
// std::nullopt explicitly clears the current-frame value, preventing a sink
// from accidentally replaying a historical target.
class PrimaryTargetObservationSink {
 public:
  virtual ~PrimaryTargetObservationSink() = default;
  virtual void Consume(std::optional<PrimaryTargetObservation> observation) = 0;
};

class LatestPrimaryTargetObservation final : public PrimaryTargetObservationSink {
 public:
  void Consume(std::optional<PrimaryTargetObservation> observation) override;
  std::optional<PrimaryTargetObservation> Current() const;

 private:
  mutable std::mutex mutex_;
  std::optional<PrimaryTargetObservation> current_;
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
  PrimaryTargetObserver(PrimaryTargetManager::Config config,
                        std::shared_ptr<PrimaryTargetObservationSink> sink);

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
  std::shared_ptr<PrimaryTargetObservationSink> sink_;
};

}  // namespace dog_patrol_perception_tracking
#include <memory>
#include <mutex>
