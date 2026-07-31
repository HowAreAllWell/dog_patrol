#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dog_patrol_interfaces/msg/mission_event.hpp>
#include <dog_patrol_interfaces/msg/target_bounding_box.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vision_demo_host/modules/mission_ros_adapter.hpp"
#include "vision_demo_host/modules/primary_target_manager.hpp"

namespace {

using namespace std::chrono_literals;
using vision_demo_host::ClassId;
using vision_demo_host::IdentityObservation;
using vision_demo_host::IdentityState;
using vision_demo_host::MissionBlockCause;
using vision_demo_host::MissionCoordinator;
using vision_demo_host::MissionPhase;
using vision_demo_host::MissionRosAdapter;
using vision_demo_host::MissionSnapshot;
using vision_demo_host::MutableReadinessContributor;
using vision_demo_host::PerceptionReadiness;
using vision_demo_host::PrimaryState;
using vision_demo_host::PrimaryTargetManager;
using vision_demo_host::PrimaryTargetResult;
using vision_demo_host::SourceFrameMetadata;

using MissionEventMessage = dog_patrol_interfaces::msg::MissionEvent;
using TargetBoundingBoxMessage = dog_patrol_interfaces::msg::TargetBoundingBox;

constexpr int kFirstSemanticId = 42;
constexpr int kNextSemanticId = 99;
constexpr int kFirstRawId = 7;
constexpr int kRecoveredRawId = 8;
constexpr int kNextRawId = 9;
constexpr std::uint64_t kSourceEpochNs = 1710000000000000000ULL;

IdentityObservation Person(const int semantic_id, const int raw_id, const cv::Rect2f &bbox) {
  IdentityObservation identity;
  identity.semantic_id = semantic_id;
  identity.state = IdentityState::kActive;
  identity.supporting_raw_track_id = raw_id;
  identity.class_id = ClassId::kPerson;
  identity.confidence = 0.92F;
  identity.bbox = bbox;
  identity.visible = true;
  identity.association.stage = "issue87_integrated_current_frame";
  identity.association.passed_final_cost_gate = true;
  return identity;
}

PrimaryTargetManager::Config PrimaryConfig() {
  PrimaryTargetManager::Config config;
  config.min_person_area_px = 100.0F;
  return config;
}

class MissionPipelineIntegrationDriver final : public rclcpp::Node {
 public:
  MissionPipelineIntegrationDriver()
      : Node("issue87_mission_pipeline_integration"), primary_manager_(PrimaryConfig()) {
    MissionRosAdapter::Config config;
    config.mission_state_topic = "/issue87/integration/mission/state";
    config.mission_event_topic = "/issue87/integration/mission/event";
    config.target_bbox_topic = "/issue87/integration/perception/selected_target_bbox";
    adapter_ = std::make_unique<MissionRosAdapter>(*this, config);
    adapter_->detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});
    if (!adapter_->ReplaceRequiredReadinessContributor(
            "authorization", std::make_unique<MutableReadinessContributor>(
                                 "authorization", PerceptionReadiness::kReady,
                                 "issue #87 integration authorization provider"))) {
      throw std::runtime_error("failed to replace authorization readiness contributor");
    }

    external_event_publisher_ =
        create_publisher<MissionEventMessage>(config.mission_event_topic, MissionRosAdapter::MissionEventQos());
    event_subscription_ = create_subscription<MissionEventMessage>(
        config.mission_event_topic, MissionRosAdapter::MissionEventQos(),
        [this](const MissionEventMessage::SharedPtr message) { events_.push_back(*message); });
    bbox_subscription_ = create_subscription<TargetBoundingBoxMessage>(
        config.target_bbox_topic, MissionRosAdapter::TargetBoundingBoxQos(),
        [this](const TargetBoundingBoxMessage::SharedPtr message) { boxes_.push_back(*message); });
    stage_entered_at_ = std::chrono::steady_clock::now();
  }

  void Step() {
    if (finished_) {
      return;
    }
    if (std::chrono::steady_clock::now() - stage_entered_at_ > 10s) {
      Fail("timed out in stage " + StageName(stage_));
      return;
    }

    const auto mission = adapter_->CurrentMission();
    if (!mission.has_value()) {
      return;
    }

    switch (stage_) {
      case Stage::kWaitStartup:
        StartMission(mission.value());
        break;
      case Stage::kWaitFirstPatrol:
        SelectFirstTarget(mission.value());
        break;
      case Stage::kWaitFirstConfirm:
        PublishFirstConfirmBox(mission.value());
        break;
      case Stage::kWaitConfirmBox:
        AdvanceToApproach(mission.value());
        break;
      case Stage::kWaitApproach:
        PublishApproachBox(mission.value());
        break;
      case Stage::kWaitApproachBox:
        PublishPreThresholdAbsence(mission.value());
        break;
      case Stage::kWaitPreThreshold:
        PublishLossThreshold(mission.value());
        break;
      case Stage::kWaitLostBlock:
        ReacquireWithNewRawTrack(mission.value());
        break;
      case Stage::kWaitUnblocked:
        PublishResumedBox(mission.value());
        break;
      case Stage::kWaitResumedBox:
        AdvanceToVerification(mission.value());
        break;
      case Stage::kWaitVerification:
        PublishVerificationBox(mission.value());
        break;
      case Stage::kWaitVerificationBox:
        CompleteAuthorization(mission.value());
        break;
      case Stage::kWaitSecondPatrol:
        SelectNextEligibleTarget(mission.value());
        break;
      case Stage::kWaitSecondConfirm:
        FinishIfIntegratedRoutePassed(mission.value());
        break;
    }
  }

  bool finished() const { return finished_; }
  bool success() const { return success_; }
  const std::string &failure() const { return failure_; }

  void FailFromMain(std::string message) { Fail(std::move(message)); }

 private:
  enum class Stage {
    kWaitStartup,
    kWaitFirstPatrol,
    kWaitFirstConfirm,
    kWaitConfirmBox,
    kWaitApproach,
    kWaitApproachBox,
    kWaitPreThreshold,
    kWaitLostBlock,
    kWaitUnblocked,
    kWaitResumedBox,
    kWaitVerification,
    kWaitVerificationBox,
    kWaitSecondPatrol,
    kWaitSecondConfirm,
  };

  static std::string StageName(const Stage stage) {
    switch (stage) {
      case Stage::kWaitStartup:
        return "wait_startup";
      case Stage::kWaitFirstPatrol:
        return "wait_first_patrol";
      case Stage::kWaitFirstConfirm:
        return "wait_first_confirm";
      case Stage::kWaitConfirmBox:
        return "wait_confirm_box";
      case Stage::kWaitApproach:
        return "wait_approach";
      case Stage::kWaitApproachBox:
        return "wait_approach_box";
      case Stage::kWaitPreThreshold:
        return "wait_pre_threshold";
      case Stage::kWaitLostBlock:
        return "wait_lost_block";
      case Stage::kWaitUnblocked:
        return "wait_unblocked";
      case Stage::kWaitResumedBox:
        return "wait_resumed_box";
      case Stage::kWaitVerification:
        return "wait_verification";
      case Stage::kWaitVerificationBox:
        return "wait_verification_box";
      case Stage::kWaitSecondPatrol:
        return "wait_second_patrol";
      case Stage::kWaitSecondConfirm:
        return "wait_second_confirm";
    }
    return "unknown";
  }

  void Advance(const Stage next) {
    stage_ = next;
    stage_entered_at_ = std::chrono::steady_clock::now();
  }

  void Fail(std::string message) {
    failure_ = std::move(message);
    success_ = false;
    finished_ = true;
  }

  bool Require(const bool condition, const std::string &message) {
    if (!condition) {
      Fail(message);
      return false;
    }
    return true;
  }

  SourceFrameMetadata Metadata(const std::uint64_t offset_ns) const {
    SourceFrameMetadata metadata;
    metadata.source_timestamp_ns = kSourceEpochNs + offset_ns;
    metadata.camera_frame_number = static_cast<std::uint32_t>(offset_ns / 100000000ULL + 1U);
    metadata.camera_frame_number_available = true;
    metadata.image_width = 1280;
    metadata.image_height = 1024;
    metadata.optical_frame_id = "issue87_hik_camera_optical_frame";
    return metadata;
  }

  std::vector<IdentityObservation> BothPeople(const int first_raw_id) const {
    return {
        Person(kFirstSemanticId, first_raw_id, cv::Rect2f{100.25F, 200.5F, 300.0F, 400.0F}),
        Person(kNextSemanticId, kNextRawId, cv::Rect2f{700.0F, 250.0F, 200.0F, 300.0F}),
    };
  }

  PrimaryTargetResult UpdatePrimary(const MissionSnapshot &mission,
                                    const std::vector<IdentityObservation> &identities,
                                    const MissionCoordinator::TimePoint source_time) {
    return primary_manager_.UpdateForMission(identities, mission, adapter_->PreviousMission(), source_time);
  }

  void ProcessFrame(const MissionSnapshot &mission,
                    const std::vector<IdentityObservation> &identities,
                    const MissionCoordinator::TimePoint source_time,
                    const std::uint64_t source_offset_ns) {
    const auto primary = UpdatePrimary(mission, identities, source_time);
    adapter_->ProcessFrame({mission, identities, primary, source_time}, Metadata(source_offset_ns));
  }

  void PublishExternalEvent(const MissionSnapshot &mission, const std::uint8_t source,
                            const std::uint8_t event, const int target_id) {
    MissionEventMessage message;
    message.header.stamp = get_clock()->now();
    message.observed_state_seq = mission.state_seq;
    message.target_id = target_id > 0 ? static_cast<std::uint32_t>(target_id) : 0U;
    message.source = source;
    message.event = event;
    message.detail = "issue #87 integrated lifecycle driver";
    external_event_publisher_->publish(message);
  }

  std::size_t EventCount(const std::uint8_t event, const int target_id,
                         const std::optional<std::uint8_t> source = std::nullopt) const {
    std::size_t count = 0U;
    for (const auto &message : events_) {
      if (message.event == event && message.target_id == static_cast<std::uint32_t>(target_id) &&
          (!source.has_value() || message.source == source.value())) {
        ++count;
      }
    }
    return count;
  }

  void StartMission(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kStartup) {
      return;
    }
    if (!startup_observed_at_.has_value()) {
      startup_observed_at_ = std::chrono::steady_clock::now();
      return;
    }
    if (std::chrono::steady_clock::now() - startup_observed_at_.value() < 500ms ||
        external_event_publisher_->get_subscription_count() < 1U) {
      return;
    }
    adapter_->PublishReadiness();
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_NAVIGATION, MissionEventMessage::READY, 0);
    Advance(Stage::kWaitFirstPatrol);
  }

  void SelectFirstTarget(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kPatrol) {
      return;
    }
    const std::size_t ready_count = EventCount(
        MissionEventMessage::READY, 0, MissionEventMessage::SOURCE_PERCEPTION);
    if (ready_count == 0U) {
      return;
    }
    if (!Require(ready_count == 1U,
                 "aggregate perception READY was not emitted exactly once")) {
      return;
    }

    const auto identities = BothPeople(kFirstRawId);
    const auto primary = UpdatePrimary(mission, identities, source_origin_);
    if (!Require(primary.state == PrimaryState::kLocked &&
                     primary.primary_target_id == kFirstSemanticId,
                 "first patrol frame did not select the largest eligible semantic target")) {
      return;
    }
    if (!Require(adapter_->PublishTargetConfirmed(mission, primary, Metadata(0U)),
                 "first-frame TARGET_CONFIRMED was not published")) {
      return;
    }
    adapter_->ProcessFrame({mission, identities, primary, source_origin_}, Metadata(0U));
    Advance(Stage::kWaitFirstConfirm);
  }

  void PublishFirstConfirmBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kConfirmTarget || mission.target_id != kFirstSemanticId) {
      return;
    }
    const std::size_t confirmation_count = EventCount(
        MissionEventMessage::TARGET_CONFIRMED, kFirstSemanticId,
        MissionEventMessage::SOURCE_PERCEPTION);
    if (confirmation_count == 0U) {
      return;
    }
    if (!Require(confirmation_count == 1U,
                 "first target confirmation did not cross the ROS seam exactly once") ||
        !Require(boxes_.empty(), "PATROL published a target bounding box")) {
      return;
    }
    ProcessFrame(mission, BothPeople(kFirstRawId), source_origin_ + 100ms, 100000000ULL);
    Advance(Stage::kWaitConfirmBox);
  }

  void AdvanceToApproach(const MissionSnapshot &mission) {
    if (boxes_.empty()) {
      return;
    }
    if (!Require(boxes_.size() == 1U, "CONFIRM_TARGET published an unexpected bbox count") ||
        !Require(boxes_.front().target_id == static_cast<std::uint32_t>(kFirstSemanticId) &&
                     boxes_.front().header.frame_id == "issue87_hik_camera_optical_frame" &&
                     boxes_.front().header.stamp.sec == 1710000000 &&
                     boxes_.front().header.stamp.nanosec == 100000000U &&
                     boxes_.front().x_min == 100U && boxes_.front().y_min == 200U &&
                     boxes_.front().x_max == 401U && boxes_.front().y_max == 601U,
                 "fresh bbox lost semantic ID, source timestamp/frame, or half-open coordinates")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_NAVIGATION,
                         MissionEventMessage::TARGET_POSITION_READY, kFirstSemanticId);
    Advance(Stage::kWaitApproach);
  }

  void PublishApproachBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kApproachTarget || mission.target_id != kFirstSemanticId ||
        mission.blocked) {
      return;
    }
    ProcessFrame(mission, BothPeople(kFirstRawId), source_origin_ + 200ms, 200000000ULL);
    Advance(Stage::kWaitApproachBox);
  }

  void PublishPreThresholdAbsence(const MissionSnapshot &mission) {
    if (boxes_.size() < 2U) {
      return;
    }
    if (!Require(boxes_.size() == 2U, "APPROACH_TARGET did not publish one fresh bbox")) {
      return;
    }
    const std::vector<IdentityObservation> only_distractor{
        Person(kNextSemanticId, kNextRawId, cv::Rect2f{700.0F, 250.0F, 200.0F, 300.0F})};
    ProcessFrame(mission, only_distractor, source_origin_ + 699ms, 699000000ULL);
    Advance(Stage::kWaitPreThreshold);
  }

  void PublishLossThreshold(const MissionSnapshot &mission) {
    if (!Require(EventCount(MissionEventMessage::TARGET_LOST, kFirstSemanticId) == 0U,
                 "TARGET_LOST was emitted before the default 0.5 second threshold") ||
        !Require(boxes_.size() == 2U, "missing target reused a cached or fabricated bbox")) {
      return;
    }
    const std::vector<IdentityObservation> only_distractor{
        Person(kNextSemanticId, kNextRawId, cv::Rect2f{700.0F, 250.0F, 200.0F, 300.0F})};
    ProcessFrame(mission, only_distractor, source_origin_ + 700ms, 700000000ULL);
    Advance(Stage::kWaitLostBlock);
  }

  void ReacquireWithNewRawTrack(const MissionSnapshot &mission) {
    if (!mission.blocked || mission.block_cause != MissionBlockCause::kTargetLost ||
        mission.target_id != kFirstSemanticId) {
      return;
    }
    const std::size_t loss_count = EventCount(
        MissionEventMessage::TARGET_LOST, kFirstSemanticId,
        MissionEventMessage::SOURCE_PERCEPTION);
    if (loss_count == 0U) {
      return;
    }
    if (!Require(loss_count == 1U,
                 "TARGET_LOST did not block the real mission supervisor exactly once") ||
        !Require(boxes_.size() == 2U, "blocked loss state published a target bbox")) {
      return;
    }
    ProcessFrame(mission, BothPeople(kRecoveredRawId), source_origin_ + 800ms, 800000000ULL);
    Advance(Stage::kWaitUnblocked);
  }

  void PublishResumedBox(const MissionSnapshot &mission) {
    if (mission.blocked || mission.block_cause != MissionBlockCause::kNone ||
        mission.phase != MissionPhase::kApproachTarget || mission.target_id != kFirstSemanticId ||
        EventCount(MissionEventMessage::TARGET_REACQUIRED, kFirstSemanticId,
                   MissionEventMessage::SOURCE_PERCEPTION) < 1U) {
      return;
    }
    if (!Require(EventCount(MissionEventMessage::TARGET_REACQUIRED, kFirstSemanticId,
                            MissionEventMessage::SOURCE_PERCEPTION) == 1U,
                 "same semantic target did not unblock through one TARGET_REACQUIRED") ||
        !Require(boxes_.size() == 2U, "blocked reacquisition frame published before supervisor unblock")) {
      return;
    }
    ProcessFrame(mission, BothPeople(kRecoveredRawId), source_origin_ + 900ms, 900000000ULL);
    Advance(Stage::kWaitResumedBox);
  }

  void AdvanceToVerification(const MissionSnapshot &mission) {
    if (boxes_.size() < 3U) {
      return;
    }
    if (!Require(boxes_.size() == 3U &&
                     boxes_.back().target_id == static_cast<std::uint32_t>(kFirstSemanticId),
                 "fresh bbox did not resume for the same semantic target after unblock")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_NAVIGATION,
                         MissionEventMessage::ARRIVED_AND_STOPPED, kFirstSemanticId);
    Advance(Stage::kWaitVerification);
  }

  void PublishVerificationBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kVerifyIdentity || mission.target_id != kFirstSemanticId ||
        mission.blocked) {
      return;
    }
    ProcessFrame(mission, BothPeople(kRecoveredRawId), source_origin_ + 1000ms, 1000000000ULL);
    Advance(Stage::kWaitVerificationBox);
  }

  void CompleteAuthorization(const MissionSnapshot &mission) {
    if (boxes_.size() < 4U) {
      return;
    }
    if (!Require(boxes_.size() == 4U &&
                     boxes_.back().target_id == static_cast<std::uint32_t>(kFirstSemanticId),
                 "VERIFY_IDENTITY did not retain the current fresh target bbox")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_PERCEPTION,
                         MissionEventMessage::AUTHORIZED, kFirstSemanticId);
    Advance(Stage::kWaitSecondPatrol);
  }

  void SelectNextEligibleTarget(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kPatrol || mission.target_id != 0) {
      return;
    }
    const auto identities = BothPeople(kRecoveredRawId);
    const auto primary = UpdatePrimary(mission, identities, source_origin_ + 1100ms);
    if (!Require(!primary_manager_.IsMissionEligible(identities.front()),
                 "handled semantic target remained mission-eligible after returning to patrol") ||
        !Require(identities.front().visible && identities.front().semantic_id == kFirstSemanticId,
                 "handled target was removed from perception observations") ||
        !Require(primary.state == PrimaryState::kLocked &&
                     primary.primary_target_id == kNextSemanticId &&
                     primary.raw_track_id == kNextRawId,
                 "first new patrol frame did not select the next-largest eligible target")) {
      return;
    }
    if (!Require(adapter_->PublishTargetConfirmed(mission, primary, Metadata(1100000000ULL)),
                 "next eligible target was not confirmed")) {
      return;
    }
    adapter_->ProcessFrame({mission, identities, primary, source_origin_ + 1100ms},
                           Metadata(1100000000ULL));
    Advance(Stage::kWaitSecondConfirm);
  }

  void FinishIfIntegratedRoutePassed(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kConfirmTarget || mission.target_id != kNextSemanticId) {
      return;
    }
    const std::size_t confirmation_count = EventCount(
        MissionEventMessage::TARGET_CONFIRMED, kNextSemanticId,
        MissionEventMessage::SOURCE_PERCEPTION);
    if (confirmation_count == 0U) {
      return;
    }
    if (!Require(confirmation_count == 1U,
                 "next eligible semantic target did not reach the mission supervisor") ||
        !Require(boxes_.size() == 4U, "second PATROL frame published an unexpected bbox")) {
      return;
    }
    success_ = true;
    finished_ = true;
  }

  PrimaryTargetManager primary_manager_;
  std::unique_ptr<MissionRosAdapter> adapter_;
  rclcpp::Publisher<MissionEventMessage>::SharedPtr external_event_publisher_;
  rclcpp::Subscription<MissionEventMessage>::SharedPtr event_subscription_;
  rclcpp::Subscription<TargetBoundingBoxMessage>::SharedPtr bbox_subscription_;
  std::vector<MissionEventMessage> events_;
  std::vector<TargetBoundingBoxMessage> boxes_;
  MissionCoordinator::TimePoint source_origin_{};
  Stage stage_{Stage::kWaitStartup};
  std::chrono::steady_clock::time_point stage_entered_at_{};
  std::optional<std::chrono::steady_clock::time_point> startup_observed_at_;
  bool finished_{false};
  bool success_{false};
  std::string failure_;
};

}  // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int exit_code = 1;
  try {
    auto driver = std::make_shared<MissionPipelineIntegrationDriver>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(driver);
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (rclcpp::ok() && !driver->finished() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      driver->Step();
      std::this_thread::sleep_for(5ms);
    }
    if (!driver->finished()) {
      driver->FailFromMain("overall integration deadline exceeded");
    }
    executor.spin_some();
    executor.remove_node(driver);
    if (driver->success()) {
      std::cout << "issue #87 integrated mission lifecycle passed" << std::endl;
      exit_code = 0;
    } else {
      std::cerr << "issue #87 integrated mission lifecycle failed: " << driver->failure()
                << std::endl;
    }
  } catch (const std::exception &exception) {
    std::cerr << "issue #87 integration exception: " << exception.what() << std::endl;
  }
  rclcpp::shutdown();
  return exit_code;
}
