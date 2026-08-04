#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_tracking/modules/mission_ros_adapter.hpp"

namespace {

using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityObservation;
using dog_patrol_perception_tracking::IdentityState;
using dog_patrol_perception_tracking::MissionBlockCause;
using dog_patrol_perception_tracking::MissionCoordinator;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionRosAdapter;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::SourceFrameMetadata;

constexpr int kSmokeSemanticId = 42;
constexpr int kSmokeRawTrackId = 7;
constexpr std::uint64_t kSourceEpochNs = 1710000000000000000ULL;

IdentityObservation TrustedObservation() {
  IdentityObservation observation;
  observation.semantic_id = kSmokeSemanticId;
  observation.state = IdentityState::kActive;
  observation.visible = true;
  observation.class_id = ClassId::kPerson;
  observation.supporting_raw_track_id = kSmokeRawTrackId;
  observation.bbox = cv::Rect2f{100.25F, 200.5F, 300.0F, 400.0F};
  observation.confidence = 0.92F;
  return observation;
}

class MissionRosAdapterSmoke final : public rclcpp::Node {
 public:
  MissionRosAdapterSmoke() : Node("mission_ros_adapter_smoke") {
    declare_parameter<std::string>("mission.state_topic", "/issue84/smoke/mission/state");
    declare_parameter<std::string>("mission.event_topic", "/issue84/smoke/mission/event");
    declare_parameter<std::string>("mission.selected_target_bbox_topic",
                                   "/issue84/smoke/perception/selected_target_bbox");
    declare_parameter<std::string>("perception.camera_optical_frame_id",
                                   "issue84_smoke_optical_frame");
    declare_parameter<double>("smoke.reacquire_retention_sec", 30.0);

    MissionRosAdapter::Config config;
    config.mission_state_topic = get_parameter("mission.state_topic").as_string();
    config.mission_event_topic = get_parameter("mission.event_topic").as_string();
    config.target_bbox_topic = get_parameter("mission.selected_target_bbox_topic").as_string();
    config.coordinator.lost_event_timeout = std::chrono::milliseconds{500};
    config.coordinator.reacquire_retention = std::chrono::duration_cast<MissionCoordinator::Duration>(
        std::chrono::duration<double>(get_parameter("smoke.reacquire_retention_sec").as_double()));
    optical_frame_id_ = get_parameter("perception.camera_optical_frame_id").as_string();
    adapter_ = std::make_unique<MissionRosAdapter>(*this, std::move(config));
    adapter_->ReportDetectionTrackingRuntimeStatus({true, true, {}});
    timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this] { Tick(); });
    RCLCPP_INFO(
        get_logger(),
        "issue #84 headless smoke ready; publish STARTUP(100), PATROL(101), "
        "CONFIRM_TARGET(102,target=42), then blocked TARGET_LOST CONFIRM_TARGET(103,target=42)");
  }

 private:
  SourceFrameMetadata Metadata() const {
    SourceFrameMetadata metadata;
    metadata.source_timestamp_ns = kSourceEpochNs + source_offset_ns_;
    metadata.image_width = 1280;
    metadata.image_height = 1024;
    metadata.optical_frame_id = optical_frame_id_;
    return metadata;
  }

  void Tick() {
    source_offset_ns_ += 100000000ULL;
    source_time_ += std::chrono::milliseconds{100};
    adapter_->PublishCapabilityStatus();

    const std::optional<MissionSnapshot> mission = adapter_->CurrentMission();
    if (!mission.has_value()) {
      return;
    }
    if (!last_state_seq_.has_value() || last_state_seq_.value() != mission->state_seq) {
      last_state_seq_ = mission->state_seq;
      ticks_in_state_ = 0U;
    } else {
      ++ticks_in_state_;
    }

    const SourceFrameMetadata metadata = Metadata();
    if (mission->phase == MissionPhase::kPatrol && !mission->blocked) {
      adapter_->ProcessFrame({TrustedObservation()}, source_time_, metadata);
      return;
    }

    const bool trusted_target_frame =
        mission->target_id == kSmokeSemanticId &&
        ((mission->blocked && mission->block_cause == MissionBlockCause::kTargetLost) ||
         (!mission->blocked && ticks_in_state_ == 0U));
    const std::vector<IdentityObservation> identities =
        trusted_target_frame ? std::vector<IdentityObservation>{TrustedObservation()}
                              : std::vector<IdentityObservation>{};
    adapter_->ProcessFrame(identities, source_time_, metadata);
  }

  std::unique_ptr<MissionRosAdapter> adapter_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string optical_frame_id_;
  std::optional<std::uint32_t> last_state_seq_;
  std::uint32_t ticks_in_state_{0U};
  std::uint64_t source_offset_ns_{0U};
  MissionCoordinator::TimePoint source_time_{};
};

}  // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionRosAdapterSmoke>());
  rclcpp::shutdown();
  return 0;
}
