#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dog_patrol_perception_tracking/modules/mission_coordinator.hpp"
#include "dog_patrol_perception_tracking/source_frame_metadata.hpp"

namespace dog_patrol_perception_tracking {

// Records immutable source frames without making the perception loop wait for
// JPEG encoding or disk I/O. All public methods may be called concurrently.
class EventPhotoRecorder {
 public:
  static constexpr std::uint8_t WAIT_STARTED = 0;
  static constexpr std::uint8_t PASSED = 1;
  static constexpr std::uint8_t NOT_PASSED = 2;

  struct Config {
    std::string root{"~/.ros/dog_patrol/photos"};
    std::size_t frame_history_capacity{16};
    std::size_t write_queue_capacity{16};
    std::uint64_t max_frame_age_ns{1000000000ULL};
    // Voice JSON results are retained even when image capture is disabled.
    bool photos_enabled{true};
  };

  struct VoiceConfidence {
    std::uint8_t vote_count{0};
    std::uint8_t confidence_percent{0};
    std::vector<std::uint8_t> matching_channels;
  };

  struct Metrics {
    std::uint64_t enqueued{0};
    std::uint64_t written{0};
    std::uint64_t queue_dropped{0};
    std::uint64_t write_errors{0};
    std::uint64_t missing_frame{0};
    std::uint64_t invalid_events{0};
    std::uint64_t pending_dropped{0};
  };

  using ErrorSink = std::function<void(const std::string &)>;

  explicit EventPhotoRecorder(Config config, ErrorSink error_sink = {});
  ~EventPhotoRecorder();
  EventPhotoRecorder(const EventPhotoRecorder &) = delete;
  EventPhotoRecorder &operator=(const EventPhotoRecorder &) = delete;

  // published_events must contain only events actually published after ReID
  // filtering. trusted_target_id is zero when this frame has no visible target.
  void ObserveFrame(const std::optional<MissionSnapshot> &mission,
                    int trusted_target_id,
                    const std::vector<MissionEventAction> &published_events,
                    const SourceFrameMetadata &metadata, const cv::Mat &bgr,
                    std::uint64_t mission_stamp_ns = 0);

  // Event timestamps and source-frame timestamps must use the same clock.
  void RecordVoice(std::uint32_t state_seq, int target_id,
                   std::uint32_t round_number, std::uint8_t event,
                   std::uint64_t event_stamp_ns,
                   std::optional<VoiceConfidence> confidence = std::nullopt);

  // Waits for queued writes. Unmatched voice events are reported and discarded.
  void Flush();
  Metrics GetMetrics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_patrol_perception_tracking
