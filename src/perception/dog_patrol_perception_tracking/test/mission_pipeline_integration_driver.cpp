#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <dog_patrol_interfaces/msg/mission_event.hpp>
#include <dog_patrol_interfaces/msg/mission_state.hpp>
#include <dog_patrol_interfaces/msg/target_bounding_box.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_perception_tracking/modules/det_filter.hpp"
#include "dog_patrol_perception_tracking/modules/identity_manager.hpp"
#include "dog_patrol_perception_tracking/modules/mission_ros_adapter.hpp"
#include "dog_patrol_perception_tracking/modules/mot_tracker.hpp"
#include "dog_patrol_perception_tracking/modules/preprocess_infer.hpp"
#include "dog_patrol_perception_tracking/modules/primary_target_manager.hpp"

namespace {

using namespace std::chrono_literals;
using dog_patrol_perception_tracking::ClassId;
using dog_patrol_perception_tracking::IdentityObservation;
using dog_patrol_perception_tracking::IdentityState;
using dog_patrol_perception_tracking::MissionBlockCause;
using dog_patrol_perception_tracking::MissionCoordinator;
using dog_patrol_perception_tracking::MissionFrameTransaction;
using dog_patrol_perception_tracking::MissionPhase;
using dog_patrol_perception_tracking::MissionRosAdapter;
using dog_patrol_perception_tracking::MissionSnapshot;
using dog_patrol_perception_tracking::MutableReadinessContributor;
using dog_patrol_perception_tracking::PerceptionMissionEvent;
using dog_patrol_perception_tracking::PerceptionReadiness;
using dog_patrol_perception_tracking::PrimaryState;
using dog_patrol_perception_tracking::PrimaryTargetManager;
using dog_patrol_perception_tracking::SourceFrameMetadata;

using MissionEventMessage = dog_patrol_interfaces::msg::MissionEvent;
using MissionStateMessage = dog_patrol_interfaces::msg::MissionState;
using TargetBoundingBoxMessage = dog_patrol_interfaces::msg::TargetBoundingBox;

constexpr int kFirstSemanticId = 42;
constexpr int kNextSemanticId = 99;
constexpr int kFirstRawId = 7;
constexpr int kRecoveredRawId = 8;
constexpr int kNextRawId = 9;
constexpr std::uint64_t kSourceEpochNs = 1710000000000000000ULL;

enum class EvidenceSlot : std::size_t {
  kSelection,
  kConfirmation,
  kApproach,
  kMissingBeforeTimeout,
  kMissingAtTimeout,
  kReacquired,
  kResumed,
  kVerification,
  kSecondPatrol,
  kCount,
};

struct EvidenceFrame {
  std::size_t source_frame_index{0U};
  std::vector<IdentityObservation> identities;
  SourceFrameMetadata metadata;
};

struct MissionEvidence {
  std::array<EvidenceFrame, static_cast<std::size_t>(EvidenceSlot::kCount)> frames;
  int first_semantic_id{-1};
  int next_semantic_id{-1};
  int first_raw_id{-1};
  int recovered_raw_id{-1};
  int next_raw_id{-1};
  bool visual_pipeline{false};
  std::size_t decoded_frames{0U};
  std::size_t detection_positive_frames{0U};
  std::size_t track_positive_frames{0U};
  std::string source_description;
};

struct VisualReplayConfig {
  std::string video_path;
  std::string detector_engine_path;
  std::string tracker_config_path;
};

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

constexpr std::size_t SlotIndex(const EvidenceSlot slot) {
  return static_cast<std::size_t>(slot);
}

#ifdef DOG_PATROL_PERCEPTION_TRACKING_ENABLE_ORIN_RUNTIME
SourceFrameMetadata ReplayMetadata(const std::size_t frame_index, const double fps,
                                   const int width, const int height,
                                   const std::string &frame_id) {
  SourceFrameMetadata metadata;
  const double safe_fps = fps > 0.0 ? fps : 30.0;
  metadata.source_timestamp_ns =
      kSourceEpochNs + static_cast<std::uint64_t>(std::llround(
                           static_cast<double>(frame_index) * 1000000000.0 / safe_fps));
  metadata.camera_frame_number_available = false;
  metadata.image_width = width;
  metadata.image_height = height;
  metadata.optical_frame_id = frame_id;
  return metadata;
}
#endif

const IdentityObservation *VisiblePersonWithSemanticId(
    const std::vector<IdentityObservation> &identities, const int semantic_id) {
  for (const auto &identity : identities) {
    if (identity.semantic_id == semantic_id && identity.class_id == ClassId::kPerson &&
        identity.visible && identity.state == IdentityState::kActive) {
      return &identity;
    }
  }
  return nullptr;
}

#ifdef DOG_PATROL_PERCEPTION_TRACKING_ENABLE_ORIN_RUNTIME
std::vector<const IdentityObservation *> VisiblePeople(
    const std::vector<IdentityObservation> &identities) {
  std::vector<const IdentityObservation *> people;
  for (const auto &identity : identities) {
    if (identity.semantic_id > 0 && identity.class_id == ClassId::kPerson && identity.visible &&
        identity.state == IdentityState::kActive && identity.supporting_raw_track_id > 0) {
      people.push_back(&identity);
    }
  }
  std::sort(people.begin(), people.end(), [](const auto *lhs, const auto *rhs) {
    return lhs->bbox.area() > rhs->bbox.area();
  });
  return people;
}
#endif

MissionEvidence BuildSyntheticEvidence() {
  MissionEvidence evidence;
  evidence.first_semantic_id = kFirstSemanticId;
  evidence.next_semantic_id = kNextSemanticId;
  evidence.first_raw_id = kFirstRawId;
  evidence.recovered_raw_id = kRecoveredRawId;
  evidence.next_raw_id = kNextRawId;
  evidence.source_description = "deterministic synthetic identity fixture";

  const auto both_people = [](const int first_raw_id) {
    return std::vector<IdentityObservation>{
        Person(kFirstSemanticId, first_raw_id,
               cv::Rect2f{100.25F, 200.5F, 300.0F, 400.0F}),
        Person(kNextSemanticId, kNextRawId,
               cv::Rect2f{700.0F, 250.0F, 200.0F, 300.0F}),
    };
  };
  const std::vector<IdentityObservation> only_distractor{
      Person(kNextSemanticId, kNextRawId,
             cv::Rect2f{700.0F, 250.0F, 200.0F, 300.0F})};
  const std::array<std::uint64_t, SlotIndex(EvidenceSlot::kCount)> offsets{
      0U,         100000000ULL, 200000000ULL, 699000000ULL, 700000000ULL,
      800000000ULL, 900000000ULL, 1000000000ULL, 1100000000ULL};

  for (std::size_t index = 0U; index < evidence.frames.size(); ++index) {
    auto &frame = evidence.frames[index];
    frame.source_frame_index = index;
    frame.metadata.source_timestamp_ns = kSourceEpochNs + offsets[index];
    frame.metadata.camera_frame_number = static_cast<std::uint32_t>(index + 1U);
    frame.metadata.camera_frame_number_available = true;
    frame.metadata.image_width = 1280;
    frame.metadata.image_height = 1024;
    frame.metadata.optical_frame_id = "issue87_hik_camera_optical_frame";
  }
  evidence.frames[SlotIndex(EvidenceSlot::kSelection)].identities = both_people(kFirstRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kConfirmation)].identities = both_people(kFirstRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kApproach)].identities = both_people(kFirstRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kMissingBeforeTimeout)].identities = only_distractor;
  evidence.frames[SlotIndex(EvidenceSlot::kMissingAtTimeout)].identities = only_distractor;
  evidence.frames[SlotIndex(EvidenceSlot::kReacquired)].identities = both_people(kRecoveredRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kResumed)].identities = both_people(kRecoveredRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kVerification)].identities = both_people(kRecoveredRawId);
  evidence.frames[SlotIndex(EvidenceSlot::kSecondPatrol)].identities = both_people(kRecoveredRawId);
  return evidence;
}

#ifdef DOG_PATROL_PERCEPTION_TRACKING_ENABLE_ORIN_RUNTIME
std::optional<MissionEvidence> FindVisualMissionEvidence(
    const std::vector<EvidenceFrame> &frames, std::string *error) {
  for (std::size_t selection = 0U; selection + 1U < frames.size(); ++selection) {
    const auto selection_people = VisiblePeople(frames[selection].identities);
    if (selection_people.size() < 2U) {
      continue;
    }
    const int target_id = selection_people[0]->semantic_id;
    const int initial_raw_id = selection_people[0]->supporting_raw_track_id.value();
    if (VisiblePersonWithSemanticId(frames[selection + 1U].identities, target_id) == nullptr) {
      continue;
    }

    for (std::size_t missing = selection + 3U; missing + 4U < frames.size(); ++missing) {
      const auto *before_missing =
          VisiblePersonWithSemanticId(frames[missing - 2U].identities, target_id);
      if (before_missing == nullptr ||
          VisiblePersonWithSemanticId(frames[missing].identities, target_id) != nullptr) {
        continue;
      }
      if (before_missing->supporting_raw_track_id != initial_raw_id) {
        break;
      }

      std::size_t reacquired = missing + 1U;
      while (reacquired < frames.size() &&
             VisiblePersonWithSemanticId(frames[reacquired].identities, target_id) == nullptr) {
        ++reacquired;
      }
      if (reacquired < missing + 2U || reacquired + 3U >= frames.size()) {
        continue;
      }
      const auto *recovered =
          VisiblePersonWithSemanticId(frames[reacquired].identities, target_id);
      if (recovered == nullptr || recovered->supporting_raw_track_id == initial_raw_id ||
          VisiblePersonWithSemanticId(frames[reacquired + 1U].identities, target_id) == nullptr ||
          VisiblePersonWithSemanticId(frames[reacquired + 2U].identities, target_id) == nullptr) {
        continue;
      }

      const auto second_patrol_people = VisiblePeople(frames[reacquired + 3U].identities);
      const IdentityObservation *next = nullptr;
      for (const auto *person : second_patrol_people) {
        if (person->semantic_id != target_id) {
          next = person;
          break;
        }
      }
      if (next == nullptr) {
        continue;
      }

      MissionEvidence evidence;
      evidence.first_semantic_id = target_id;
      evidence.next_semantic_id = next->semantic_id;
      evidence.first_raw_id = initial_raw_id;
      evidence.recovered_raw_id = recovered->supporting_raw_track_id.value();
      evidence.next_raw_id = next->supporting_raw_track_id.value();
      evidence.visual_pipeline = true;
      evidence.frames[SlotIndex(EvidenceSlot::kSelection)] = frames[selection];
      evidence.frames[SlotIndex(EvidenceSlot::kConfirmation)] = frames[selection + 1U];
      evidence.frames[SlotIndex(EvidenceSlot::kApproach)] = frames[missing - 2U];
      evidence.frames[SlotIndex(EvidenceSlot::kMissingBeforeTimeout)] = frames[missing];
      evidence.frames[SlotIndex(EvidenceSlot::kMissingAtTimeout)] = frames[reacquired - 1U];
      evidence.frames[SlotIndex(EvidenceSlot::kReacquired)] = frames[reacquired];
      evidence.frames[SlotIndex(EvidenceSlot::kResumed)] = frames[reacquired + 1U];
      evidence.frames[SlotIndex(EvidenceSlot::kVerification)] = frames[reacquired + 2U];
      evidence.frames[SlotIndex(EvidenceSlot::kSecondPatrol)] = frames[reacquired + 3U];
      return evidence;
    }
  }
  if (error != nullptr) {
    *error = "no two-person visual sequence contained largest-target loss, same-semantic "
             "raw-track recovery, and a next eligible target";
  }
  return std::nullopt;
}
#endif

std::optional<MissionEvidence> BuildVisualEvidence(const VisualReplayConfig &config,
                                                   std::string *error) {
#ifndef DOG_PATROL_PERCEPTION_TRACKING_ENABLE_ORIN_RUNTIME
  (void)config;
  if (error != nullptr) {
    *error = "visual replay requires a build with TRACKING_ENABLE_ORIN_RUNTIME=ON";
  }
  return std::nullopt;
#else
  dog_patrol_perception_tracking::PreprocessInfer::Config infer_config;
  infer_config.detector_runtime_path = config.detector_engine_path;
  infer_config.raw_conf_threshold = 0.10F;
  dog_patrol_perception_tracking::PreprocessInfer infer(infer_config);
  if (!infer.Initialize(error)) {
    return std::nullopt;
  }

  dog_patrol_perception_tracking::DetFilter det_filter({0.10F, 0.10F});
  dog_patrol_perception_tracking::MotTracker::Config tracker_config;
  tracker_config.tracker_yaml_path = config.tracker_config_path;
  dog_patrol_perception_tracking::MotTracker tracker(tracker_config);
  if (!tracker.Initialize(error)) {
    return std::nullopt;
  }

  dog_patrol_perception_tracking::IdentityManager identity_manager;
  if (!identity_manager.Initialize(error)) {
    return std::nullopt;
  }
  PrimaryTargetManager::Config visual_primary_config;
  visual_primary_config.min_person_area_px = 100.0F;
  PrimaryTargetManager visual_primary(visual_primary_config);

  cv::VideoCapture capture(config.video_path);
  if (!capture.isOpened()) {
    if (error != nullptr) {
      *error = "failed to open visual evidence video: " + config.video_path;
    }
    return std::nullopt;
  }
  const double fps = capture.get(cv::CAP_PROP_FPS);
  std::vector<EvidenceFrame> frames;
  std::size_t detection_positive_frames = 0U;
  std::size_t track_positive_frames = 0U;
  cv::Mat image;
  while (capture.read(image)) {
    if (image.empty()) {
      break;
    }
    const auto detections = infer.Infer(image);
    const auto tracks = tracker.Update(det_filter.Filter(detections), image);
    const auto previous_primary = visual_primary.GetState();
    const auto identities = identity_manager.Update(
        dog_patrol_perception_tracking::TrackletObservationsFromTracks(tracks),
        tracker.LastTrackletHypotheses(), previous_primary, &image);
    (void)visual_primary.Update(identities.identities);

    EvidenceFrame frame;
    frame.source_frame_index = frames.size();
    frame.identities = identities.identities;
    frame.metadata = ReplayMetadata(frame.source_frame_index, fps, image.cols, image.rows,
                                    "issue87_visual_replay_optical_frame");
    frames.push_back(std::move(frame));
    detection_positive_frames += detections.empty() ? 0U : 1U;
    track_positive_frames += tracks.empty() ? 0U : 1U;
  }

  auto evidence = FindVisualMissionEvidence(frames, error);
  if (!evidence.has_value()) {
    return std::nullopt;
  }
  evidence->decoded_frames = frames.size();
  evidence->detection_positive_frames = detection_positive_frames;
  evidence->track_positive_frames = track_positive_frames;
  evidence->source_description = config.video_path;
  return evidence;
#endif
}

PrimaryTargetManager::Config PrimaryConfig() {
  PrimaryTargetManager::Config config;
  config.min_person_area_px = 100.0F;
  return config;
}

bool ParseVisualReplayConfig(const int argc, char *argv[],
                             std::optional<VisualReplayConfig> *config,
                             std::string *error) {
  if (argc == 1) {
    config->reset();
    return true;
  }
  VisualReplayConfig parsed;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      *error = "missing value for " + argument;
      return false;
    }
    const std::string value = argv[++index];
    if (argument == "--visual-video") {
      parsed.video_path = value;
    } else if (argument == "--detector-engine") {
      parsed.detector_engine_path = value;
    } else if (argument == "--tracker-config") {
      parsed.tracker_config_path = value;
    } else {
      *error = "unknown argument: " + argument;
      return false;
    }
  }
  if (parsed.video_path.empty() || parsed.detector_engine_path.empty() ||
      parsed.tracker_config_path.empty()) {
    *error = "visual replay requires --visual-video, --detector-engine, and --tracker-config";
    return false;
  }
  *config = std::move(parsed);
  return true;
}

class MissionPipelineIntegrationDriver final : public rclcpp::Node {
 public:
  explicit MissionPipelineIntegrationDriver(MissionEvidence evidence)
      : Node("mission_contract_integration"),
        evidence_(std::move(evidence)) {
    MissionRosAdapter::Config config;
    config.mission_state_topic = "/dog_patrol/integration/mission/state";
    config.mission_event_topic = "/dog_patrol/integration/mission/event";
    config.target_bbox_topic = "/dog_patrol/integration/perception/selected_target_bbox";
    config.primary = PrimaryConfig();
    adapter_ = std::make_unique<MissionRosAdapter>(*this, config);
    adapter_->detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});
    if (!adapter_->ReplaceRequiredReadinessContributor(
            "authorization", std::make_unique<MutableReadinessContributor>(
                                 "authorization", PerceptionReadiness::kReady,
                                 "mission contract integration authorization provider"))) {
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
    std::cout << "mission contract evidence source=" << evidence_.source_description
              << " target=" << evidence_.first_semantic_id << " raw=" << evidence_.first_raw_id
              << "->" << evidence_.recovered_raw_id
              << " next_target=" << evidence_.next_semantic_id << std::endl;
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

  const EvidenceFrame &Frame(const EvidenceSlot slot) const {
    return evidence_.frames[SlotIndex(slot)];
  }

  const IdentityObservation *TargetObservation(const EvidenceSlot slot) const {
    return VisiblePersonWithSemanticId(Frame(slot).identities, evidence_.first_semantic_id);
  }

  bool BoxMatchesEvidence(const TargetBoundingBoxMessage &box, const EvidenceSlot slot) const {
    const auto *identity = TargetObservation(slot);
    if (identity == nullptr) {
      return false;
    }
    const auto &metadata = Frame(slot).metadata;
    const auto clamp_x = [&metadata](const double value) {
      return std::clamp(value, 0.0, static_cast<double>(metadata.image_width));
    };
    const auto clamp_y = [&metadata](const double value) {
      return std::clamp(value, 0.0, static_cast<double>(metadata.image_height));
    };
    const auto x_min = static_cast<std::uint32_t>(clamp_x(std::floor(identity->bbox.x)));
    const auto y_min = static_cast<std::uint32_t>(clamp_y(std::floor(identity->bbox.y)));
    const auto x_max = static_cast<std::uint32_t>(
        clamp_x(std::ceil(static_cast<double>(identity->bbox.x + identity->bbox.width))));
    const auto y_max = static_cast<std::uint32_t>(
        clamp_y(std::ceil(static_cast<double>(identity->bbox.y + identity->bbox.height))));
    const auto expected_seconds =
        static_cast<std::int32_t>(metadata.source_timestamp_ns / 1000000000ULL);
    const auto expected_nanoseconds =
        static_cast<std::uint32_t>(metadata.source_timestamp_ns % 1000000000ULL);
    return box.target_id == static_cast<std::uint32_t>(evidence_.first_semantic_id) &&
           box.header.frame_id == metadata.optical_frame_id &&
           box.header.stamp.sec == expected_seconds &&
           box.header.stamp.nanosec == expected_nanoseconds &&
           box.image_width == static_cast<std::uint32_t>(metadata.image_width) &&
           box.image_height == static_cast<std::uint32_t>(metadata.image_height) &&
           box.x_min == x_min && box.y_min == y_min && box.x_max == x_max && box.y_max == y_max;
  }

  MissionFrameTransaction::Output ProcessFrame(
      const MissionCoordinator::TimePoint source_time,
      const EvidenceSlot slot) {
    const auto &frame = Frame(slot);
    return adapter_->ProcessFrame(frame.identities, source_time, frame.metadata);
  }

  void PublishExternalEvent(const MissionSnapshot &mission, const std::uint8_t source,
                            const std::uint8_t event, const int target_id) {
    MissionEventMessage message;
    message.header.stamp = get_clock()->now();
    message.observed_state_seq = mission.state_seq;
    message.target_id = target_id > 0 ? static_cast<std::uint32_t>(target_id) : 0U;
    message.source = source;
    message.event = event;
    message.detail = "mission contract integration external event owner";
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

  static bool OutputHasEvent(const MissionFrameTransaction::Output &output,
                             const PerceptionMissionEvent event,
                             const int target_id) {
    return std::any_of(output.events.begin(), output.events.end(),
                       [event, target_id](const auto &action) {
                         return action.event == event && action.target_id == target_id;
                       });
  }

  void StartMission(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kStartup) {
      return;
    }
    if (!startup_observed_at_.has_value()) {
      startup_observed_at_ = std::chrono::steady_clock::now();
      return;
    }
    const auto observer_grace = evidence_.visual_pipeline ? 3s : 500ms;
    if (std::chrono::steady_clock::now() - startup_observed_at_.value() < observer_grace ||
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

    const auto &frame = Frame(EvidenceSlot::kSelection);
    const auto output = adapter_->ProcessFrame(frame.identities, source_origin_, frame.metadata);
    const auto &primary = output.primary;
    if (!Require(primary.state == PrimaryState::kLocked &&
                     primary.primary_target_id == evidence_.first_semantic_id &&
                     primary.raw_track_id == evidence_.first_raw_id,
                 "first patrol frame did not select the largest eligible semantic target")) {
      return;
    }
    if (!Require(OutputHasEvent(output, PerceptionMissionEvent::kTargetConfirmed,
                                evidence_.first_semantic_id),
                 "first-frame TARGET_CONFIRMED was not produced")) {
      return;
    }
    Advance(Stage::kWaitFirstConfirm);
  }

  void PublishFirstConfirmBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kConfirmTarget ||
        mission.target_id != evidence_.first_semantic_id) {
      return;
    }
    const std::size_t confirmation_count = EventCount(
        MissionEventMessage::TARGET_CONFIRMED, evidence_.first_semantic_id,
        MissionEventMessage::SOURCE_PERCEPTION);
    if (confirmation_count == 0U) {
      return;
    }
    if (!Require(confirmation_count == 1U,
                 "first target confirmation did not cross the ROS seam exactly once") ||
        !Require(boxes_.empty(), "PATROL published a target bounding box")) {
      return;
    }

    // Exercise the adapter's public state seam before accepting another frame. These
    // inputs model late DDS delivery, a same-sequence split-brain publisher, and a
    // same-sequence target mismatch. None may replace the real supervisor snapshot.
    MissionStateMessage stale;
    stale.state_seq = mission.state_seq - 1U;
    stale.state = MissionStateMessage::PATROL;
    stale.target_id = 0U;
    stale.blocked = false;
    stale.block_cause = MissionStateMessage::BLOCK_NONE;

    MissionStateMessage conflicting;
    conflicting.state_seq = mission.state_seq;
    conflicting.state = MissionStateMessage::APPROACH_TARGET;
    conflicting.target_id = static_cast<std::uint32_t>(evidence_.first_semantic_id);
    conflicting.blocked = false;
    conflicting.block_cause = MissionStateMessage::BLOCK_NONE;

    MissionStateMessage wrong_target = conflicting;
    wrong_target.state = MissionStateMessage::CONFIRM_TARGET;
    wrong_target.target_id = static_cast<std::uint32_t>(evidence_.next_semantic_id);

    if (!Require(!adapter_->StoreMissionState(stale),
                 "stale state_seq replaced the authoritative supervisor state") ||
        !Require(!adapter_->StoreMissionState(conflicting),
                 "conflicting same-sequence state replaced the authoritative supervisor state") ||
        !Require(!adapter_->StoreMissionState(wrong_target),
                 "wrong target_id replaced the authoritative supervisor state")) {
      return;
    }
    const auto retained = adapter_->CurrentMission();
    if (!Require(retained.has_value() && retained->state_seq == mission.state_seq &&
                     retained->phase == mission.phase && retained->target_id == mission.target_id,
                 "a rejected state input changed the authoritative mission snapshot")) {
      return;
    }

    const auto &frame = Frame(EvidenceSlot::kConfirmation);
    std::vector<IdentityObservation> wrong_target_only;
    for (const auto &identity : frame.identities) {
      if (identity.semantic_id == evidence_.next_semantic_id) {
        wrong_target_only.push_back(identity);
      }
    }
    const auto rejected_output = adapter_->ProcessFrame(
        wrong_target_only, source_origin_ + 50ms, frame.metadata);
    if (!Require(rejected_output.events.empty() && !rejected_output.target_box.has_value(),
                 "wrong target evidence produced a public event or cached bbox")) {
      return;
    }

    (void)ProcessFrame(source_origin_ + 100ms, EvidenceSlot::kConfirmation);
    Advance(Stage::kWaitConfirmBox);
  }

  void AdvanceToApproach(const MissionSnapshot &mission) {
    if (boxes_.empty()) {
      return;
    }
    if (!Require(boxes_.size() == 1U, "CONFIRM_TARGET published an unexpected bbox count") ||
        !Require(BoxMatchesEvidence(boxes_.front(), EvidenceSlot::kConfirmation),
                 "fresh bbox lost semantic ID, source timestamp/frame, or half-open coordinates")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_NAVIGATION,
                         MissionEventMessage::TARGET_POSITION_READY,
                         evidence_.first_semantic_id);
    Advance(Stage::kWaitApproach);
  }

  void PublishApproachBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kApproachTarget ||
        mission.target_id != evidence_.first_semantic_id ||
        mission.blocked) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 200ms, EvidenceSlot::kApproach);
    Advance(Stage::kWaitApproachBox);
  }

  void PublishPreThresholdAbsence(const MissionSnapshot &mission) {
    (void)mission;
    if (boxes_.size() < 2U) {
      return;
    }
    if (!Require(boxes_.size() == 2U, "APPROACH_TARGET did not publish one fresh bbox")) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 699ms, EvidenceSlot::kMissingBeforeTimeout);
    Advance(Stage::kWaitPreThreshold);
  }

  void PublishLossThreshold(const MissionSnapshot &mission) {
    (void)mission;
    if (!Require(EventCount(MissionEventMessage::TARGET_LOST,
                            evidence_.first_semantic_id) == 0U,
                 "TARGET_LOST was emitted before the default 0.5 second threshold") ||
        !Require(boxes_.size() == 2U, "missing target reused a cached or fabricated bbox")) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 700ms, EvidenceSlot::kMissingAtTimeout);
    Advance(Stage::kWaitLostBlock);
  }

  void ReacquireWithNewRawTrack(const MissionSnapshot &mission) {
    if (!mission.blocked || mission.block_cause != MissionBlockCause::kTargetLost ||
        mission.target_id != evidence_.first_semantic_id) {
      return;
    }
    const std::size_t loss_count = EventCount(
        MissionEventMessage::TARGET_LOST, evidence_.first_semantic_id,
        MissionEventMessage::SOURCE_PERCEPTION);
    if (loss_count == 0U) {
      return;
    }
    if (!Require(loss_count == 1U,
                 "TARGET_LOST did not block the real mission supervisor exactly once") ||
        !Require(boxes_.size() == 2U, "blocked loss state published a target bbox")) {
      return;
    }
    const auto *recovered = TargetObservation(EvidenceSlot::kReacquired);
    if (!Require(recovered != nullptr &&
                     recovered->supporting_raw_track_id == evidence_.recovered_raw_id &&
                     evidence_.recovered_raw_id != evidence_.first_raw_id,
                 "visual target did not recover under the same semantic ID with a new raw track")) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 800ms, EvidenceSlot::kReacquired);
    Advance(Stage::kWaitUnblocked);
  }

  void PublishResumedBox(const MissionSnapshot &mission) {
    if (mission.blocked || mission.block_cause != MissionBlockCause::kNone ||
        mission.phase != MissionPhase::kApproachTarget ||
        mission.target_id != evidence_.first_semantic_id ||
        EventCount(MissionEventMessage::TARGET_REACQUIRED, evidence_.first_semantic_id,
                   MissionEventMessage::SOURCE_PERCEPTION) < 1U) {
      return;
    }
    if (!Require(EventCount(MissionEventMessage::TARGET_REACQUIRED,
                            evidence_.first_semantic_id,
                            MissionEventMessage::SOURCE_PERCEPTION) == 1U,
                 "same semantic target did not unblock through one TARGET_REACQUIRED") ||
        !Require(boxes_.size() == 2U, "blocked reacquisition frame published before supervisor unblock")) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 900ms, EvidenceSlot::kResumed);
    Advance(Stage::kWaitResumedBox);
  }

  void AdvanceToVerification(const MissionSnapshot &mission) {
    if (boxes_.size() < 3U) {
      return;
    }
    if (!Require(boxes_.size() == 3U &&
                     boxes_.back().target_id ==
                         static_cast<std::uint32_t>(evidence_.first_semantic_id),
                 "fresh bbox did not resume for the same semantic target after unblock")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_NAVIGATION,
                         MissionEventMessage::ARRIVED_AND_STOPPED,
                         evidence_.first_semantic_id);
    Advance(Stage::kWaitVerification);
  }

  void PublishVerificationBox(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kVerifyIdentity ||
        mission.target_id != evidence_.first_semantic_id ||
        mission.blocked) {
      return;
    }
    (void)ProcessFrame(source_origin_ + 1000ms, EvidenceSlot::kVerification);
    Advance(Stage::kWaitVerificationBox);
  }

  void CompleteAuthorization(const MissionSnapshot &mission) {
    if (boxes_.size() < 4U) {
      return;
    }
    if (!Require(boxes_.size() == 4U &&
                     boxes_.back().target_id ==
                         static_cast<std::uint32_t>(evidence_.first_semantic_id),
                 "VERIFY_IDENTITY did not retain the current fresh target bbox") ||
        !Require(EventCount(MissionEventMessage::AUTHORIZED,
                            evidence_.first_semantic_id) == 0U &&
                     EventCount(MissionEventMessage::UNAUTHORIZED,
                                evidence_.first_semantic_id) == 0U,
                 "tracking published an authorization result")) {
      return;
    }
    PublishExternalEvent(mission, MissionEventMessage::SOURCE_PERCEPTION,
                         MissionEventMessage::AUTHORIZED, evidence_.first_semantic_id);
    Advance(Stage::kWaitSecondPatrol);
  }

  void SelectNextEligibleTarget(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kPatrol || mission.target_id != 0) {
      return;
    }
    const auto &frame = Frame(EvidenceSlot::kSecondPatrol);
    const auto *handled = TargetObservation(EvidenceSlot::kSecondPatrol);
    const auto output = adapter_->ProcessFrame(frame.identities, source_origin_ + 1100ms,
                                               frame.metadata);
    const auto &primary = output.primary;
    if (!Require(handled != nullptr && handled->visible &&
                     handled->semantic_id == evidence_.first_semantic_id,
                 "handled target was removed from perception observations") ||
        !Require(primary.state == PrimaryState::kLocked &&
                     primary.primary_target_id == evidence_.next_semantic_id &&
                     primary.raw_track_id == evidence_.next_raw_id,
                 "first new patrol frame did not select the next-largest eligible target")) {
      return;
    }
    if (!Require(OutputHasEvent(output, PerceptionMissionEvent::kTargetConfirmed,
                                evidence_.next_semantic_id),
                 "next eligible target was not confirmed")) {
      return;
    }
    Advance(Stage::kWaitSecondConfirm);
  }

  void FinishIfIntegratedRoutePassed(const MissionSnapshot &mission) {
    if (mission.phase != MissionPhase::kConfirmTarget ||
        mission.target_id != evidence_.next_semantic_id) {
      return;
    }
    const std::size_t confirmation_count = EventCount(
        MissionEventMessage::TARGET_CONFIRMED, evidence_.next_semantic_id,
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

  MissionEvidence evidence_;
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
  std::optional<VisualReplayConfig> visual_config;
  std::string error;
  if (!ParseVisualReplayConfig(argc, argv, &visual_config, &error)) {
    std::cerr << "mission contract integration argument error: " << error << std::endl;
    return 2;
  }

  MissionEvidence evidence = BuildSyntheticEvidence();
  if (visual_config.has_value()) {
    auto visual_evidence = BuildVisualEvidence(visual_config.value(), &error);
    if (!visual_evidence.has_value()) {
      std::cerr << "mission contract visual evidence failed: " << error << std::endl;
      return 1;
    }
    evidence = std::move(visual_evidence.value());
    std::cout << "mission contract visual pipeline decoded=" << evidence.decoded_frames
              << " detection_positive=" << evidence.detection_positive_frames
              << " track_positive=" << evidence.track_positive_frames
              << " selection_frame="
              << evidence.frames[SlotIndex(EvidenceSlot::kSelection)].source_frame_index
              << " missing_frames="
              << evidence.frames[SlotIndex(EvidenceSlot::kMissingBeforeTimeout)].source_frame_index
              << ".."
              << evidence.frames[SlotIndex(EvidenceSlot::kMissingAtTimeout)].source_frame_index
              << " reacquired_frame="
              << evidence.frames[SlotIndex(EvidenceSlot::kReacquired)].source_frame_index
              << std::endl;
  }

  int ros_argc = 1;
  char *ros_argv[] = {argv[0], nullptr};
  rclcpp::init(ros_argc, ros_argv);
  int exit_code = 1;
  try {
    auto driver = std::make_shared<MissionPipelineIntegrationDriver>(std::move(evidence));
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
      std::cout << "integrated mission contract lifecycle passed" << std::endl;
      exit_code = 0;
    } else {
      std::cerr << "integrated mission contract lifecycle failed: " << driver->failure()
                << std::endl;
    }
  } catch (const std::exception &exception) {
    std::cerr << "mission contract integration exception: " << exception.what() << std::endl;
  }
  rclcpp::shutdown();
  return exit_code;
}
