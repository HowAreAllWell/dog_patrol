#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
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
    std::chrono::steady_clock::duration state_timeout{std::chrono::milliseconds{750}};
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

  // Rejects unknown enum values and invalid state/target combinations.
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

  void ReportDetectionTrackingRuntimeStatus(
      DetectionTrackingReadiness::RuntimeStatus status);

  // Call after detector/tracker initialization and periodically while the
  // live loop is running. This publishes only tracking's own current status,
  // associated with the current STARTUP; the orchestrator owns aggregation.
  void PublishCapabilityStatus();

  // Processes actions from the current source frame only. Mission state and
  // primary-selection ordering stay inside the frame transaction; callers pass
  // only current-frame perception evidence and metadata.
  // confirm_gate（可选，默认空 = 未启用，行为与不接 ReID 完全一致）：
  // kTargetConfirmed 发布前的短时 ReID 去重闸门，返回 false 抑制该次发布
  //（含问询在途的 hold）。语义与线程约束见 reid_confirm_gate.hpp。
  MissionFrameTransaction::Output ProcessFrame(
      const std::vector<IdentityObservation> &identities,
      MissionCoordinator::TimePoint source_time,
      const SourceFrameMetadata &metadata,
      const std::function<bool(int target_id)> *confirm_gate = nullptr);

  // ReID 去重排除注入（评审 R04）：进入选人资格边界（PrimaryTargetManager
  // 排除表）。线程安全：内部持 mission_mutex_，可在帧处理路径调用。
  void ExcludeForReidDedup(int semantic_id);

  // 跨天清理选人层 ReID 排除项（评审 N03）：与 gate 的 SyncDay 配对调用；
  // 线程安全：内部持 mission_mutex_。
  void ClearReidDedupExclusions();

 private:
  static std::optional<MissionPhase> MissionPhaseFromMessage(std::uint8_t state);
  static builtin_interfaces::msg::Time TimeMessage(std::uint64_t nanoseconds);
  static dog_patrol_interfaces::msg::MissionEvent EventMessage(
      PerceptionMissionEvent event, int target_id, std::uint32_t observed_state_seq,
      std::uint64_t source_timestamp_ns);
  bool MissionStateAvailableLocked() const;

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
  std::uint64_t latest_mission_stamp_ns_{0};
  std::optional<MissionSnapshot> previous_mission_;
  std::optional<std::chrono::steady_clock::time_point> last_state_received_;
  bool mission_state_conflicted_{false};

  MissionFrameTransaction frame_transaction_;
  DetectionTrackingReadiness detection_tracking_readiness_;
};

}  // namespace dog_patrol_perception_tracking
