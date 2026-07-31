#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <dog_patrol_interfaces/msg/mission_event.hpp>
#include <dog_patrol_interfaces/msg/mission_state.hpp>
#include <dog_patrol_interfaces/msg/target_bounding_box.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vision_demo_host/modules/mission_coordinator.hpp"
#include "vision_demo_host/modules/perception_readiness.hpp"

namespace vision_demo_host {

// Metadata is carried alongside each processed camera frame. The patrol
// interface has no camera-frame-number field, so the number remains runtime
// diagnostic metadata while timestamp, optical frame, and source dimensions
// are represented by TargetBoundingBox's Header and fields.
struct SourceFrameMetadata {
  std::uint64_t source_timestamp_ns{0};
  std::uint32_t camera_frame_number{0};
  bool camera_frame_number_available{false};
  int image_width{0};
  int image_height{0};
  std::string optical_frame_id;
};

// The sole ROS transport boundary for mission-aware perception output. The
// subscriber callback only validates and stores a snapshot under a mutex.
// Call PublishReadiness/ProcessFrame from the serialized live processing path;
// they own coordinator and readiness state and never run detector/tracker/
// identity work from a ROS subscription callback.
class MissionRosAdapter {
 public:
  struct Config {
    std::string mission_state_topic{"/mission/state"};
    std::string mission_event_topic{"/mission/event"};
    std::string target_bbox_topic{"/perception/selected_target_bbox"};
    // The live node assigns this subscriber a callback group separate from
    // camera/inference so state reception remains responsive while a frame is
    // being processed. A null group uses the node default (unit-test default).
    rclcpp::CallbackGroup::SharedPtr mission_state_callback_group;
    // Authorization is required for aggregate perception readiness but is
    // not owned by this repository. It stays not-ready until an integration
    // explicitly accepts the temporary placeholder or replaces it.
    bool authorization_placeholder_ready{false};
    std::string authorization_placeholder_detail;
    MissionCoordinator::Config coordinator;
  };

  MissionRosAdapter(rclcpp::Node &node, Config config);

  MissionRosAdapter(const MissionRosAdapter &) = delete;
  MissionRosAdapter &operator=(const MissionRosAdapter &) = delete;

  static rclcpp::QoS MissionStateQos();
  static rclcpp::QoS MissionEventQos();
  static rclcpp::QoS TargetBoundingBoxQos();

  // Rejects unknown enum values and invalid state/block/target combinations.
  static std::optional<MissionSnapshot> MissionFromMessage(
      const dog_patrol_interfaces::msg::MissionState &message);

  // Maps a current-frame action to original-image, clamped half-open integer
  // coordinates. It never turns an empty/off-image box into a fabricated box.
  static std::optional<dog_patrol_interfaces::msg::TargetBoundingBox> TargetBoxFromAction(
      const FreshTargetBoxAction &action, const SourceFrameMetadata &metadata);

  // Accepts only a current-or-newer valid snapshot. Equal state_seq is the
  // same authoritative snapshot; stale uint32 sequence values are discarded.
  bool StoreMissionState(const dog_patrol_interfaces::msg::MissionState &message);
  std::optional<MissionSnapshot> CurrentMission() const;
  std::optional<MissionSnapshot> PreviousMission() const;

  DetectionTrackingReadinessContributor &detection_tracking_readiness();
  void AddRequiredReadinessContributor(std::unique_ptr<PerceptionReadinessContributor> contributor);
  bool ReplaceRequiredReadinessContributor(
      std::string capability, std::unique_ptr<PerceptionReadinessContributor> contributor);

  // Call after detector/tracker initialization and periodically while the
  // live loop is running so one aggregate READY action is published for each
  // eligible STARTUP state sequence.
  void PublishReadiness();

  // Processes actions from the current source frame only. Missing mission
  // state, invalid metadata, stale state, and disallowed phases publish
  // nothing; the coordinator remains the owner of loss/reacquisition policy.
  void ProcessFrame(const MissionCoordinator::FrameInput &input,
                    const SourceFrameMetadata &metadata);

  // Publishes at most one TARGET_CONFIRMED for an unblocked PATROL state
  // sequence after the live primary selector has locked a stable semantic ID.
  bool PublishTargetConfirmed(const MissionSnapshot &mission,
                              const PrimaryTargetResult &primary,
                              const SourceFrameMetadata &metadata);

 private:
  static std::optional<MissionPhase> MissionPhaseFromMessage(std::uint8_t state);
  static std::optional<MissionBlockCause> MissionBlockCauseFromMessage(std::uint8_t cause);
  static builtin_interfaces::msg::Time TimeMessage(std::uint64_t nanoseconds);
  static bool IsTrustedPrimary(const PrimaryTargetResult &primary, int semantic_id);
  static dog_patrol_interfaces::msg::MissionEvent EventMessage(
      PerceptionMissionEvent event, int target_id, std::uint32_t observed_state_seq,
      std::uint64_t source_timestamp_ns);

  // Caller must hold mission_mutex_.
  bool MatchesLatestMissionUnderLock(const MissionSnapshot &mission) const;

  rclcpp::Node &node_;
  Config config_;
  rclcpp::Subscription<dog_patrol_interfaces::msg::MissionState>::SharedPtr mission_state_subscription_;
  rclcpp::Publisher<dog_patrol_interfaces::msg::MissionEvent>::SharedPtr mission_event_publisher_;
  rclcpp::Publisher<dog_patrol_interfaces::msg::TargetBoundingBox>::SharedPtr target_bbox_publisher_;

  mutable std::mutex mission_mutex_;
  MissionStateSequenceCursor state_sequence_;
  std::optional<MissionSnapshot> latest_mission_;
  std::optional<MissionSnapshot> previous_mission_;
  std::optional<std::uint32_t> confirmed_patrol_state_seq_;

  MissionCoordinator coordinator_;
  PerceptionReadinessAggregator readiness_aggregator_;
  DetectionTrackingReadinessContributor *detection_tracking_readiness_{nullptr};
};

}  // namespace vision_demo_host
