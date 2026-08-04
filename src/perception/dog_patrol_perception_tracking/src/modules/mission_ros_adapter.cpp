#include "dog_patrol_perception_tracking/modules/mission_ros_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dog_patrol_perception_tracking {
namespace {

using MissionEventMessage = dog_patrol_interfaces::msg::MissionEvent;
using MissionStateMessage = dog_patrol_interfaces::msg::MissionState;

bool IsTargetPhase(const MissionPhase phase) {
  return phase == MissionPhase::kConfirmTarget || phase == MissionPhase::kApproachTarget ||
         phase == MissionPhase::kVerifyIdentity || phase == MissionPhase::kTrackIntruder;
}

bool IsFinite(const float value) { return std::isfinite(value); }

}  // namespace

MissionRosAdapter::MissionRosAdapter(rclcpp::Node &node, Config config)
    : node_(node),
      config_(std::move(config)),
      frame_transaction_(config_.primary, config_.coordinator) {
  if (config_.mission_state_topic.empty() || config_.mission_event_topic.empty() ||
      config_.target_bbox_topic.empty() || config_.capability_status_topic.empty()) {
    throw std::invalid_argument("mission ROS topic names must not be empty");
  }

  mission_event_publisher_ = node_.create_publisher<MissionEventMessage>(
      config_.mission_event_topic, MissionEventQos());
  target_bbox_publisher_ = node_.create_publisher<dog_patrol_interfaces::msg::TargetBoundingBox>(
      config_.target_bbox_topic, TargetBoundingBoxQos());
  capability_status_publisher_ =
      node_.create_publisher<dog_patrol_perception_interfaces::msg::CapabilityStatus>(
          config_.capability_status_topic, CapabilityStatusQos());
  rclcpp::SubscriptionOptions mission_state_subscription_options;
  mission_state_subscription_options.callback_group = config_.mission_state_callback_group;
  mission_state_subscription_ = node_.create_subscription<MissionStateMessage>(
      config_.mission_state_topic, MissionStateQos(),
      [this](const MissionStateMessage::SharedPtr message) { StoreMissionState(*message); },
      mission_state_subscription_options);
}

rclcpp::QoS MissionRosAdapter::MissionStateQos() {
  return rclcpp::QoS(rclcpp::KeepLast{1}).reliable().transient_local();
}

rclcpp::QoS MissionRosAdapter::MissionEventQos() {
  return rclcpp::QoS(rclcpp::KeepLast{10}).reliable().durability_volatile();
}

rclcpp::QoS MissionRosAdapter::TargetBoundingBoxQos() {
  return rclcpp::QoS(rclcpp::KeepLast{5}).best_effort().durability_volatile();
}

rclcpp::QoS MissionRosAdapter::CapabilityStatusQos() {
  return rclcpp::QoS(rclcpp::KeepLast{1}).reliable().transient_local();
}

std::optional<MissionPhase> MissionRosAdapter::MissionPhaseFromMessage(const std::uint8_t state) {
  switch (state) {
    case MissionStateMessage::STARTUP:
      return MissionPhase::kStartup;
    case MissionStateMessage::PATROL:
      return MissionPhase::kPatrol;
    case MissionStateMessage::CONFIRM_TARGET:
      return MissionPhase::kConfirmTarget;
    case MissionStateMessage::APPROACH_TARGET:
      return MissionPhase::kApproachTarget;
    case MissionStateMessage::VERIFY_IDENTITY:
      return MissionPhase::kVerifyIdentity;
    case MissionStateMessage::TRACK_INTRUDER:
      return MissionPhase::kTrackIntruder;
    default:
      return std::nullopt;
  }
}

std::optional<MissionBlockCause> MissionRosAdapter::MissionBlockCauseFromMessage(
    const std::uint8_t cause) {
  switch (cause) {
    case MissionStateMessage::BLOCK_NONE:
      return MissionBlockCause::kNone;
    case MissionStateMessage::BLOCK_TARGET_LOST:
      return MissionBlockCause::kTargetLost;
    case MissionStateMessage::BLOCK_EXECUTION_ERROR:
      return MissionBlockCause::kExecutionError;
    default:
      return std::nullopt;
  }
}

std::optional<MissionSnapshot> MissionRosAdapter::MissionFromMessage(
    const MissionStateMessage &message) {
  const auto phase = MissionPhaseFromMessage(message.state);
  const auto block_cause = MissionBlockCauseFromMessage(message.block_cause);
  if (!phase.has_value() || !block_cause.has_value() ||
      message.target_id > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  const int target_id = static_cast<int>(message.target_id);
  if ((phase.value() == MissionPhase::kStartup || phase.value() == MissionPhase::kPatrol) &&
      target_id != 0) {
    return std::nullopt;
  }
  if (IsTargetPhase(phase.value()) && target_id <= 0) {
    return std::nullopt;
  }
  if (message.blocked != (block_cause.value() != MissionBlockCause::kNone)) {
    return std::nullopt;
  }
  if (message.blocked && (!IsTargetPhase(phase.value()) || target_id <= 0)) {
    return std::nullopt;
  }

  return MissionSnapshot{message.state_seq, phase.value(), target_id, message.blocked,
                         block_cause.value()};
}

builtin_interfaces::msg::Time MissionRosAdapter::TimeMessage(const std::uint64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;
  const std::uint64_t seconds = nanoseconds / kNanosecondsPerSecond;
  stamp.sec = seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())
                  ? std::numeric_limits<std::int32_t>::max()
                  : static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return stamp;
}

std::optional<dog_patrol_interfaces::msg::TargetBoundingBox> MissionRosAdapter::TargetBoxFromAction(
    const FreshTargetBoxAction &action, const SourceFrameMetadata &metadata) {
  if (action.target_id <= 0 || metadata.source_timestamp_ns == 0U || metadata.image_width <= 0 ||
      metadata.image_height <= 0 || metadata.optical_frame_id.empty() || action.bbox.width <= 0.0F ||
      action.bbox.height <= 0.0F || !IsFinite(action.bbox.x) || !IsFinite(action.bbox.y) ||
      !IsFinite(action.bbox.width) || !IsFinite(action.bbox.height) || !IsFinite(action.confidence)) {
    return std::nullopt;
  }

  const float x_end = action.bbox.x + action.bbox.width;
  const float y_end = action.bbox.y + action.bbox.height;
  if (!IsFinite(x_end) || !IsFinite(y_end)) {
    return std::nullopt;
  }
  const auto clamp_x = [width = metadata.image_width](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(width));
  };
  const auto clamp_y = [height = metadata.image_height](const double value) {
    return std::clamp(value, 0.0, static_cast<double>(height));
  };
  const double x_min = clamp_x(std::floor(static_cast<double>(action.bbox.x)));
  const double y_min = clamp_y(std::floor(static_cast<double>(action.bbox.y)));
  const double x_max = clamp_x(std::ceil(static_cast<double>(x_end)));
  const double y_max = clamp_y(std::ceil(static_cast<double>(y_end)));
  if (x_max <= x_min || y_max <= y_min) {
    return std::nullopt;
  }

  dog_patrol_interfaces::msg::TargetBoundingBox message;
  message.header.stamp = TimeMessage(metadata.source_timestamp_ns);
  message.header.frame_id = metadata.optical_frame_id;
  message.target_id = static_cast<std::uint32_t>(action.target_id);
  message.image_width = static_cast<std::uint32_t>(metadata.image_width);
  message.image_height = static_cast<std::uint32_t>(metadata.image_height);
  message.x_min = static_cast<std::uint32_t>(x_min);
  message.y_min = static_cast<std::uint32_t>(y_min);
  message.x_max = static_cast<std::uint32_t>(x_max);
  message.y_max = static_cast<std::uint32_t>(y_max);
  message.confidence = action.confidence;
  return message;
}

bool MissionRosAdapter::StoreMissionState(const MissionStateMessage &message) {
  const auto mission = MissionFromMessage(message);
  if (!mission.has_value()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mission_mutex_);
  if (latest_mission_.has_value() && latest_mission_->state_seq == mission->state_seq &&
      (latest_mission_->phase != mission->phase || latest_mission_->target_id != mission->target_id ||
       latest_mission_->blocked != mission->blocked ||
       latest_mission_->block_cause != mission->block_cause)) {
    return false;
  }
  if (!state_sequence_.AcceptsCurrentOrNewer(mission->state_seq)) {
    return false;
  }
  if (!latest_mission_.has_value() || latest_mission_->state_seq != mission->state_seq) {
    previous_mission_ = latest_mission_;
  }
  latest_mission_ = mission;
  return true;
}

std::optional<MissionSnapshot> MissionRosAdapter::CurrentMission() const {
  std::lock_guard<std::mutex> lock(mission_mutex_);
  return latest_mission_;
}

std::optional<MissionSnapshot> MissionRosAdapter::PreviousMission() const {
  std::lock_guard<std::mutex> lock(mission_mutex_);
  return previous_mission_;
}

PrimaryTargetResult MissionRosAdapter::CurrentPrimary() const {
  std::lock_guard<std::mutex> lock(mission_mutex_);
  return frame_transaction_.CurrentPrimary();
}

void MissionRosAdapter::ReportDetectionTrackingRuntimeStatus(
    DetectionTrackingReadiness::RuntimeStatus status) {
  std::lock_guard<std::mutex> lock(mission_mutex_);
  detection_tracking_readiness_.ReportRuntimeStatus(std::move(status));
}

dog_patrol_interfaces::msg::MissionEvent MissionRosAdapter::EventMessage(
    const PerceptionMissionEvent event, const int target_id,
    const std::uint32_t observed_state_seq, const std::uint64_t source_timestamp_ns) {
  dog_patrol_interfaces::msg::MissionEvent message;
  message.header.stamp = TimeMessage(source_timestamp_ns);
  message.observed_state_seq = observed_state_seq;
  message.target_id = target_id > 0 ? static_cast<std::uint32_t>(target_id) : 0U;
  message.source = MissionEventMessage::SOURCE_PERCEPTION;
  switch (event) {
    case PerceptionMissionEvent::kTargetConfirmed:
      message.event = MissionEventMessage::TARGET_CONFIRMED;
      message.detail = "largest eligible current-frame semantic target selected";
      break;
    case PerceptionMissionEvent::kTargetLost:
      message.event = MissionEventMessage::TARGET_LOST;
      message.detail = "current semantic target has no fresh trusted bbox";
      break;
    case PerceptionMissionEvent::kTargetReacquired:
      message.event = MissionEventMessage::TARGET_REACQUIRED;
      message.detail = "same semantic target has a fresh trusted bbox";
      break;
  }
  return message;
}

void MissionRosAdapter::PublishCapabilityStatus() {
  std::lock_guard<std::mutex> lock(mission_mutex_);
  if (!latest_mission_.has_value() || latest_mission_->phase != MissionPhase::kStartup) {
    return;
  }
  const auto contribution = detection_tracking_readiness_.Contribution();
  dog_patrol_perception_interfaces::msg::CapabilityStatus message;
  message.header.stamp = node_.get_clock()->now();
  message.capability = contribution.capability;
  message.observed_startup_state_seq = latest_mission_->state_seq;
  message.diagnostic = contribution.detail;
  switch (contribution.readiness) {
    case PerceptionReadiness::kNotReady:
      message.status = dog_patrol_perception_interfaces::msg::CapabilityStatus::NOT_READY;
      break;
    case PerceptionReadiness::kReady:
      message.status = dog_patrol_perception_interfaces::msg::CapabilityStatus::READY;
      break;
    case PerceptionReadiness::kFailure:
      message.status = dog_patrol_perception_interfaces::msg::CapabilityStatus::ERROR;
      break;
  }
  capability_status_publisher_->publish(message);
}

MissionFrameTransaction::Output MissionRosAdapter::ProcessFrame(
    const std::vector<IdentityObservation> &identities,
    const MissionCoordinator::TimePoint source_time,
    const SourceFrameMetadata &metadata) {
  // Keep frame-transaction stateful one-shot decisions and publication in one
  // short mission-state critical section. Detector/tracker/identity work has
  // already completed in the live pipeline; this only serializes transaction
  // latches with StoreMissionState so an action cannot be dropped after its
  // latch was committed.
  std::lock_guard<std::mutex> lock(mission_mutex_);
  MissionFrameTransaction::Output output = frame_transaction_.Update(
      {latest_mission_, previous_mission_, identities, source_time, metadata});
  for (const auto &event : output.events) {
    mission_event_publisher_->publish(
        EventMessage(event.event, event.target_id, event.observed_state_seq,
                     metadata.source_timestamp_ns));
  }
  if (!output.target_box.has_value()) {
    return output;
  }
  const auto message = TargetBoxFromAction(output.target_box.value(), metadata);
  if (message.has_value()) {
    target_bbox_publisher_->publish(message.value());
  }
  return output;
}

}  // namespace dog_patrol_perception_tracking
