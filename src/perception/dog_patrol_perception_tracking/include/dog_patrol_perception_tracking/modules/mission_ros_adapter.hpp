#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <dog_patrol_interfaces/msg/mission_event.hpp>
#include <dog_patrol_interfaces/msg/mission_state.hpp>
#include <dog_patrol_interfaces/msg/target_bounding_box.hpp>
#include <dog_patrol_perception_interfaces/msg/capability_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_tracking/modules/mission_frame_transaction.hpp"
#include "dog_patrol_perception_tracking/modules/perception_readiness.hpp"

namespace dog_patrol_perception_tracking {

// The sole ROS transport boundary for mission-aware perception output. The
// subscriber callback only validates and stores a snapshot under a mutex.
// Call PublishCapabilityStatus/ProcessFrame from the serialized live processing path;
// they own coordinator and readiness state and never run detector/tracker/
// identity work from a ROS subscription callback.
class MissionRosAdapter {
 public:
  struct Config {
    std::string mission_state_topic{"/mission/state"};
    std::string mission_event_topic{"/mission/event"};
    std::string target_bbox_topic{"/perception/selected_target_bbox"};
    std::string capability_status_topic{"/perception/capability_status"};
    // The live node assigns this subscriber a callback group separate from
    // camera/inference so state reception remains responsive while a frame is
    // being processed. A null group uses the node default (unit-test default).
    rclcpp::CallbackGroup::SharedPtr mission_state_callback_group;
    PrimaryTargetManager::Config primary;
    MissionCoordinator::Config coordinator;
  };

  MissionRosAdapter(rclcpp::Node &node, Config config);

  MissionRosAdapter(const MissionRosAdapter &) = delete;
  MissionRosAdapter &operator=(const MissionRosAdapter &) = delete;

  static rclcpp::QoS MissionStateQos();
  static rclcpp::QoS MissionEventQos();
  static rclcpp::QoS TargetBoundingBoxQos();
  static rclcpp::QoS CapabilityStatusQos();

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
  PrimaryTargetResult CurrentPrimary() const;

  DetectionTrackingReadiness &detection_tracking_readiness();

  // Call after detector/tracker initialization and periodically while the
  // live loop is running. This publishes only tracking's own current status,
  // associated with the current STARTUP; the orchestrator owns aggregation.
  void PublishCapabilityStatus();

  // Processes actions from the current source frame only. Mission state and
  // primary-selection ordering stay inside the frame transaction; callers pass
  // only current-frame perception evidence and metadata.
  MissionFrameTransaction::Output ProcessFrame(
      const std::vector<IdentityObservation> &identities,
      MissionCoordinator::TimePoint source_time,
      const SourceFrameMetadata &metadata);

 private:
  static std::optional<MissionPhase> MissionPhaseFromMessage(std::uint8_t state);
  static std::optional<MissionBlockCause> MissionBlockCauseFromMessage(std::uint8_t cause);
  static builtin_interfaces::msg::Time TimeMessage(std::uint64_t nanoseconds);
  static dog_patrol_interfaces::msg::MissionEvent EventMessage(
      PerceptionMissionEvent event, int target_id, std::uint32_t observed_state_seq,
      std::uint64_t source_timestamp_ns);

  rclcpp::Node &node_;
  Config config_;
  rclcpp::Subscription<dog_patrol_interfaces::msg::MissionState>::SharedPtr mission_state_subscription_;
  rclcpp::Publisher<dog_patrol_interfaces::msg::MissionEvent>::SharedPtr mission_event_publisher_;
  rclcpp::Publisher<dog_patrol_interfaces::msg::TargetBoundingBox>::SharedPtr target_bbox_publisher_;
  rclcpp::Publisher<dog_patrol_perception_interfaces::msg::CapabilityStatus>::SharedPtr
      capability_status_publisher_;

  mutable std::mutex mission_mutex_;
  MissionStateSequenceCursor state_sequence_;
  std::optional<MissionSnapshot> latest_mission_;
  std::optional<MissionSnapshot> previous_mission_;

  MissionFrameTransaction frame_transaction_;
  DetectionTrackingReadiness detection_tracking_readiness_;
};

}  // namespace dog_patrol_perception_tracking
