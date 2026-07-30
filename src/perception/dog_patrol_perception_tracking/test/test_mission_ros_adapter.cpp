#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <dog_patrol_interfaces/msg/mission_event.hpp>
#include <dog_patrol_interfaces/msg/mission_state.hpp>
#include <dog_patrol_interfaces/msg/target_bounding_box.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>

#include "vision_demo_host/modules/mission_ros_adapter.hpp"

namespace {

using vision_demo_host::ClassId;
using vision_demo_host::FreshTargetBoxAction;
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
using vision_demo_host::PrimaryTargetResult;
using vision_demo_host::SourceFrameMetadata;
using vision_demo_host::Track;

using MissionEventMessage = dog_patrol_interfaces::msg::MissionEvent;
using MissionStateMessage = dog_patrol_interfaces::msg::MissionState;
using TargetBoundingBoxMessage = dog_patrol_interfaces::msg::TargetBoundingBox;

MissionStateMessage State(const std::uint32_t sequence, const std::uint8_t phase,
                          const std::uint32_t target_id = 0U, const bool blocked = false,
                          const std::uint8_t cause = MissionStateMessage::BLOCK_NONE) {
  MissionStateMessage message;
  message.state_seq = sequence;
  message.state = phase;
  message.target_id = target_id;
  message.blocked = blocked;
  message.block_cause = cause;
  return message;
}

IdentityObservation TrustedPerson(const int semantic_id, const int raw_track_id,
                                  const cv::Rect2f bbox = cv::Rect2f{1.2F, 2.3F, 30.0F, 40.0F}) {
  IdentityObservation identity;
  identity.semantic_id = semantic_id;
  identity.state = IdentityState::kActive;
  identity.visible = true;
  identity.class_id = ClassId::kPerson;
  identity.supporting_raw_track_id = raw_track_id;
  identity.bbox = bbox;
  identity.confidence = 0.92F;
  return identity;
}

PrimaryTargetResult LockedPrimary(const int semantic_id, const int raw_track_id,
                                  const cv::Rect2f bbox = cv::Rect2f{1.2F, 2.3F, 30.0F, 40.0F}) {
  Track track;
  track.id = raw_track_id;
  track.class_id = ClassId::kPerson;
  track.is_confirmed = true;
  track.authoritative = true;
  track.bbox = bbox;
  track.confidence = 0.92F;

  PrimaryTargetResult primary;
  primary.state = PrimaryState::kLocked;
  primary.primary_target_id = semantic_id;
  primary.raw_track_id = raw_track_id;
  primary.primary_track = track;
  return primary;
}

SourceFrameMetadata Metadata(const std::uint64_t source_timestamp_ns) {
  SourceFrameMetadata metadata;
  metadata.source_timestamp_ns = source_timestamp_ns;
  metadata.camera_frame_number = 77U;
  metadata.camera_frame_number_available = true;
  metadata.image_width = 640;
  metadata.image_height = 480;
  metadata.optical_frame_id = "hik_camera_optical_frame";
  return metadata;
}

bool SpinUntil(rclcpp::Executor &executor, const std::function<bool()> &condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return false;
}

bool WaitUntil(const std::function<bool()> &condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  return false;
}

class MissionRosAdapterTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(MissionRosAdapterTest, MapsOnlyCompatibleMissionStateMessages) {
  dog_patrol_interfaces::msg::MissionState message;
  message.state_seq = 17U;
  message.state = dog_patrol_interfaces::msg::MissionState::CONFIRM_TARGET;
  message.target_id = 42U;
  message.blocked = false;
  message.block_cause = dog_patrol_interfaces::msg::MissionState::BLOCK_NONE;

  const auto snapshot = vision_demo_host::MissionRosAdapter::MissionFromMessage(message);

  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->state_seq, 17U);
  EXPECT_EQ(snapshot->target_id, 42);
  EXPECT_EQ(snapshot->phase, vision_demo_host::MissionPhase::kConfirmTarget);
  EXPECT_EQ(snapshot->block_cause, vision_demo_host::MissionBlockCause::kNone);
}

TEST_F(MissionRosAdapterTest, RejectsIncompatibleMissionStates) {
  EXPECT_FALSE(MissionRosAdapter::MissionFromMessage(
      State(1U, MissionStateMessage::PATROL, 42U))
                   .has_value());
  EXPECT_FALSE(MissionRosAdapter::MissionFromMessage(
      State(1U, MissionStateMessage::CONFIRM_TARGET, 0U))
                   .has_value());
  EXPECT_FALSE(MissionRosAdapter::MissionFromMessage(
      State(1U, MissionStateMessage::CONFIRM_TARGET, 42U, false,
            MissionStateMessage::BLOCK_TARGET_LOST))
                   .has_value());
  EXPECT_FALSE(MissionRosAdapter::MissionFromMessage(State(1U, 255U)).has_value());
}

TEST_F(MissionRosAdapterTest, DeclaresTheSharedContractQosProfiles) {
  const auto state = MissionRosAdapter::MissionStateQos().get_rmw_qos_profile();
  EXPECT_EQ(state.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(state.depth, 1U);
  EXPECT_EQ(state.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(state.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  const auto event = MissionRosAdapter::MissionEventQos().get_rmw_qos_profile();
  EXPECT_EQ(event.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(event.depth, 10U);
  EXPECT_EQ(event.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(event.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

  const auto bbox = MissionRosAdapter::TargetBoundingBoxQos().get_rmw_qos_profile();
  EXPECT_EQ(bbox.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(bbox.depth, 5U);
  EXPECT_EQ(bbox.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(bbox.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST_F(MissionRosAdapterTest, PreservesSemanticSourceTimeAndOriginalHalfOpenCoordinates) {
  FreshTargetBoxAction action;
  action.target_id = 42;
  action.observed_state_seq = 9U;
  action.bbox = cv::Rect2f{-1.2F, 2.3F, 642.0F, 480.0F};
  action.confidence = 0.91F;
  const auto metadata = Metadata(1710000000123456789ULL);

  const auto message = MissionRosAdapter::TargetBoxFromAction(action, metadata);

  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->target_id, 42U);
  EXPECT_EQ(message->header.stamp.sec, 1710000000);
  EXPECT_EQ(message->header.stamp.nanosec, 123456789U);
  EXPECT_EQ(message->header.frame_id, "hik_camera_optical_frame");
  EXPECT_EQ(message->image_width, 640U);
  EXPECT_EQ(message->image_height, 480U);
  EXPECT_EQ(message->x_min, 0U);
  EXPECT_EQ(message->y_min, 2U);
  EXPECT_EQ(message->x_max, 640U);
  EXPECT_EQ(message->y_max, 480U);
  EXPECT_FLOAT_EQ(message->confidence, 0.91F);

  action.bbox = cv::Rect2f{700.0F, 2.0F, 4.0F, 4.0F};
  EXPECT_FALSE(MissionRosAdapter::TargetBoxFromAction(action, metadata).has_value());
}

TEST_F(MissionRosAdapterTest, SerializesConcurrentMissionCallbacksWithoutRegressingStateSequence) {
  auto node = std::make_shared<rclcpp::Node>("mission_ros_adapter_concurrency");
  MissionRosAdapter::Config config;
  config.mission_state_topic = "/issue84/concurrency/mission/state";
  config.mission_event_topic = "/issue84/concurrency/mission/event";
  config.target_bbox_topic = "/issue84/concurrency/perception/selected_target_bbox";
  MissionRosAdapter adapter(*node, config);

  std::vector<std::thread> callbacks;
  for (std::uint32_t sequence = 1U; sequence <= 32U; ++sequence) {
    callbacks.emplace_back([&adapter, sequence] {
      adapter.StoreMissionState(State(sequence, MissionStateMessage::PATROL));
    });
  }
  for (auto &callback : callbacks) {
    callback.join();
  }

  const auto mission = adapter.CurrentMission();
  ASSERT_TRUE(mission.has_value());
  EXPECT_EQ(mission->state_seq, 32U);
  EXPECT_EQ(mission->phase, MissionPhase::kPatrol);
}

TEST_F(MissionRosAdapterTest, ReceivesMissionStateWhilePipelineCallbackIsBusy) {
  auto adapter_node = std::make_shared<rclcpp::Node>("mission_ros_adapter_callback_group");
  MissionRosAdapter::Config config;
  config.mission_state_topic = "/issue84/callback_group/mission/state";
  config.mission_event_topic = "/issue84/callback_group/mission/event";
  config.target_bbox_topic = "/issue84/callback_group/perception/selected_target_bbox";
  config.mission_state_callback_group = adapter_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  MissionRosAdapter adapter(*adapter_node, config);

  std::atomic<bool> pipeline_callback_started{false};
  std::atomic<bool> pipeline_callback_finished{false};
  rclcpp::TimerBase::SharedPtr busy_timer;
  busy_timer = adapter_node->create_wall_timer(std::chrono::milliseconds{10}, [&] {
    busy_timer->cancel();
    pipeline_callback_started.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds{250});
    pipeline_callback_finished.store(true);
  });

  auto probe = std::make_shared<rclcpp::Node>("mission_ros_adapter_callback_group_probe");
  auto state_publisher = probe->create_publisher<MissionStateMessage>(
      config.mission_state_topic, MissionRosAdapter::MissionStateQos());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
  executor.add_node(adapter_node);
  executor.add_node(probe);
  std::thread spin_thread([&executor] { executor.spin(); });

  ASSERT_TRUE(WaitUntil([&pipeline_callback_started] { return pipeline_callback_started.load(); }));
  state_publisher->publish(State(100U, MissionStateMessage::STARTUP));
  ASSERT_TRUE(WaitUntil([&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 100U;
  }));
  EXPECT_FALSE(pipeline_callback_finished.load());

  executor.cancel();
  spin_thread.join();
  executor.remove_node(probe);
  executor.remove_node(adapter_node);
}

TEST_F(MissionRosAdapterTest, DropsFrameOutputWhenMissionStateHasAdvanced) {
  auto adapter_node = std::make_shared<rclcpp::Node>("mission_ros_adapter_current_output");
  MissionRosAdapter::Config config;
  config.mission_state_topic = "/issue84/current_output/mission/state";
  config.mission_event_topic = "/issue84/current_output/mission/event";
  config.target_bbox_topic = "/issue84/current_output/perception/selected_target_bbox";
  MissionRosAdapter adapter(*adapter_node, config);

  auto probe = std::make_shared<rclcpp::Node>("mission_ros_adapter_current_output_probe");
  std::vector<TargetBoundingBoxMessage> boxes;
  auto bbox_subscription = probe->create_subscription<TargetBoundingBoxMessage>(
      config.target_bbox_topic, MissionRosAdapter::TargetBoundingBoxQos(),
      [&boxes](const TargetBoundingBoxMessage::SharedPtr message) { boxes.push_back(*message); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(probe);

  ASSERT_TRUE(adapter.StoreMissionState(
      State(102U, MissionStateMessage::CONFIRM_TARGET, 42U)));
  ASSERT_TRUE(adapter.StoreMissionState(State(103U, MissionStateMessage::PATROL)));
  const auto trusted = TrustedPerson(42, 7);
  adapter.ProcessFrame({MissionSnapshot{}, {trusted}, LockedPrimary(42, 7),
                        MissionCoordinator::TimePoint{}},
                       Metadata(1710000000000000000ULL));
  executor.spin_some();
  EXPECT_TRUE(boxes.empty());

  executor.remove_node(probe);
  executor.remove_node(adapter_node);
  (void)bbox_subscription;
}

TEST_F(MissionRosAdapterTest, ExplicitAuthorizationConfigurationAllowsReady) {
  auto adapter_node = std::make_shared<rclcpp::Node>("mission_ros_adapter_explicit_authorization");
  MissionRosAdapter::Config config;
  config.mission_state_topic = "/issue84/explicit_authorization/mission/state";
  config.mission_event_topic = "/issue84/explicit_authorization/mission/event";
  config.target_bbox_topic = "/issue84/explicit_authorization/perception/selected_target_bbox";
  config.authorization_placeholder_ready = true;
  MissionRosAdapter adapter(*adapter_node, config);
  adapter.detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});

  auto probe = std::make_shared<rclcpp::Node>("mission_ros_adapter_explicit_authorization_probe");
  auto state_publisher = probe->create_publisher<MissionStateMessage>(
      config.mission_state_topic, MissionRosAdapter::MissionStateQos());
  std::vector<MissionEventMessage> events;
  auto event_subscription = probe->create_subscription<MissionEventMessage>(
      config.mission_event_topic, MissionRosAdapter::MissionEventQos(),
      [&events](const MissionEventMessage::SharedPtr message) { events.push_back(*message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(probe);
  state_publisher->publish(State(110U, MissionStateMessage::STARTUP));
  ASSERT_TRUE(SpinUntil(executor, [&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 110U;
  }));
  adapter.PublishReadiness();
  ASSERT_TRUE(SpinUntil(executor, [&events] { return events.size() == 1U; }));
  EXPECT_EQ(events.front().event, MissionEventMessage::READY);
  EXPECT_EQ(events.front().observed_state_seq, 110U);

  executor.remove_node(probe);
  executor.remove_node(adapter_node);
  (void)event_subscription;
}

TEST_F(MissionRosAdapterTest, HeadlessRosSmokeForReadyTargetAndLossReacquisition) {
  auto adapter_node = std::make_shared<rclcpp::Node>("mission_ros_adapter_smoke");
  MissionRosAdapter::Config config;
  config.mission_state_topic = "/issue84/smoke/mission/state";
  config.mission_event_topic = "/issue84/smoke/mission/event";
  config.target_bbox_topic = "/issue84/smoke/perception/selected_target_bbox";
  MissionRosAdapter adapter(*adapter_node, config);
  adapter.detection_tracking_readiness().ReportRuntimeStatus({true, true, {}});

  auto probe = std::make_shared<rclcpp::Node>("mission_ros_adapter_probe");
  auto state_publisher = probe->create_publisher<MissionStateMessage>(
      config.mission_state_topic, MissionRosAdapter::MissionStateQos());
  std::vector<MissionEventMessage> events;
  std::vector<TargetBoundingBoxMessage> boxes;
  auto event_subscription = probe->create_subscription<MissionEventMessage>(
      config.mission_event_topic, MissionRosAdapter::MissionEventQos(),
      [&events](const MissionEventMessage::SharedPtr message) { events.push_back(*message); });
  auto bbox_subscription = probe->create_subscription<TargetBoundingBoxMessage>(
      config.target_bbox_topic, MissionRosAdapter::TargetBoundingBoxQos(),
      [&boxes](const TargetBoundingBoxMessage::SharedPtr message) { boxes.push_back(*message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(probe);

  state_publisher->publish(State(100U, MissionStateMessage::STARTUP));
  ASSERT_TRUE(SpinUntil(executor, [&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 100U;
  }));
  adapter.PublishReadiness();
  executor.spin_some();
  EXPECT_TRUE(events.empty());
  ASSERT_TRUE(adapter.ReplaceRequiredReadinessContributor(
      "authorization", std::make_unique<MutableReadinessContributor>(
                           "authorization", PerceptionReadiness::kReady,
                           "authorization runtime ready")));
  adapter.PublishReadiness();
  ASSERT_TRUE(SpinUntil(executor, [&events] { return events.size() == 1U; }));
  EXPECT_EQ(events.front().event, MissionEventMessage::READY);
  EXPECT_EQ(events.front().observed_state_seq, 100U);
  EXPECT_EQ(events.front().source, MissionEventMessage::SOURCE_PERCEPTION);

  state_publisher->publish(State(101U, MissionStateMessage::PATROL));
  ASSERT_TRUE(SpinUntil(executor, [&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 101U;
  }));
  const auto trusted = TrustedPerson(42, 7);
  const auto primary = LockedPrimary(42, 7);
  const auto source_time = MissionCoordinator::TimePoint{};
  adapter.ProcessFrame({MissionSnapshot{}, {trusted}, primary, source_time}, Metadata(1710000000000000000ULL));
  EXPECT_TRUE(boxes.empty());
  ASSERT_TRUE(adapter.PublishTargetConfirmed(adapter.CurrentMission().value(), primary,
                                             Metadata(1710000000000000000ULL)));
  ASSERT_TRUE(SpinUntil(executor, [&events] { return events.size() == 2U; }));
  EXPECT_EQ(events.back().event, MissionEventMessage::TARGET_CONFIRMED);
  EXPECT_EQ(events.back().target_id, 42U);

  state_publisher->publish(State(102U, MissionStateMessage::CONFIRM_TARGET, 42U));
  ASSERT_TRUE(SpinUntil(executor, [&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 102U;
  }));
  adapter.ProcessFrame({MissionSnapshot{}, {trusted}, primary, source_time}, Metadata(1710000000100000000ULL));
  ASSERT_TRUE(SpinUntil(executor, [&boxes] { return boxes.size() == 1U; }));
  EXPECT_EQ(boxes.back().target_id, 42U);
  EXPECT_EQ(boxes.back().header.stamp.sec, 1710000000);
  EXPECT_EQ(boxes.back().header.stamp.nanosec, 100000000U);

  adapter.ProcessFrame({MissionSnapshot{}, {}, PrimaryTargetResult{},
                        source_time + std::chrono::milliseconds{500}},
                       Metadata(1710000000600000000ULL));
  ASSERT_TRUE(SpinUntil(executor, [&events] { return events.size() == 3U; }));
  EXPECT_EQ(events.back().event, MissionEventMessage::TARGET_LOST);
  EXPECT_EQ(events.back().target_id, 42U);
  EXPECT_EQ(events.back().observed_state_seq, 102U);

  state_publisher->publish(State(103U, MissionStateMessage::CONFIRM_TARGET, 42U, true,
                                 MissionStateMessage::BLOCK_TARGET_LOST));
  ASSERT_TRUE(SpinUntil(executor, [&adapter] {
    const auto mission = adapter.CurrentMission();
    return mission.has_value() && mission->state_seq == 103U;
  }));
  adapter.ProcessFrame({MissionSnapshot{}, {trusted}, primary,
                        source_time + std::chrono::milliseconds{600}},
                       Metadata(1710000000700000000ULL));
  ASSERT_TRUE(SpinUntil(executor, [&events] { return events.size() == 4U; }));
  EXPECT_EQ(events.back().event, MissionEventMessage::TARGET_REACQUIRED);
  EXPECT_EQ(events.back().target_id, 42U);
  EXPECT_EQ(events.back().observed_state_seq, 103U);
  EXPECT_EQ(boxes.size(), 1U);
  ASSERT_TRUE(adapter.PreviousMission().has_value());
  EXPECT_EQ(adapter.PreviousMission()->state_seq, 102U);

  EXPECT_FALSE(adapter.StoreMissionState(State(99U, MissionStateMessage::PATROL)));
  EXPECT_FALSE(adapter.StoreMissionState(State(103U, MissionStateMessage::CONFIRM_TARGET, 42U)));
  EXPECT_EQ(adapter.CurrentMission()->state_seq, 103U);

  executor.remove_node(probe);
  executor.remove_node(adapter_node);
  (void)event_subscription;
  (void)bbox_subscription;
}

}  // namespace
