#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_patrol_interfaces/msg/mission_state.hpp"
#include "dog_patrol_perception_tracking/modules/mission_tracking_runtime.hpp"

#include <dog_patrol_reid_provider/msg/re_id_handled.hpp>

// 生产 MissionTrackingRuntime 的 handled 转发回归（评审 N06）。
// 用真实 MissionRosAdapter（StoreMissionState 直喂合法快照）+ 真实
// ReidConfirmGate 驱动生产帧处理路径，订阅 /perception/reid/handled 计数：
// 转发必须按"新的权威处置结果"去重——同 ID 次日新完成要再次转发；同值
// heartbeat/RECOVER→PATROL 携带/跨天保留的旧快照不重复转发。
// 空人员观测不触发问询，因此无需 mock dedup service。
namespace {

using namespace dog_patrol_perception_tracking;
using MissionStateMsg = dog_patrol_interfaces::msg::MissionState;
using namespace std::chrono_literals;

class ReidHandledForwardingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo *info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    const std::string suffix = std::string("_") + info->name();
    handled_topic_ = "/reid_forward_test/handled" + suffix;

    sub_node_ = rclcpp::Node::make_shared("reid_forward_sub" + suffix);
    handled_sub_ =
        sub_node_->create_subscription<dog_patrol_reid_provider::msg::ReIDHandled>(
            handled_topic_, 10,
            [this](const dog_patrol_reid_provider::msg::ReIDHandled::SharedPtr msg) {
              const std::lock_guard<std::mutex> lk(counters_mtx_);
              if (msg->clear_all) {
                ++clear_count_;
              } else {
                ++handled_count_;
                last_track_ = msg->track_id;
              }
            });
    sub_spinner_ = std::thread([this]() {
      sub_executor_.add_node(sub_node_);
      sub_executor_.spin();
    });

    adapter_node_ = rclcpp::Node::make_shared("reid_forward_adapter" + suffix);
    MissionRosAdapter::Config adapter_config;
    adapter_config.mission_state_topic = "/reid_forward_test/state" + suffix;
    adapter_config.mission_event_topic = "/reid_forward_test/event" + suffix;
    adapter_config.target_bbox_topic = "/reid_forward_test/bbox" + suffix;
    adapter_config.capability_status_topic = "/reid_forward_test/status" + suffix;
    adapter_config.state_timeout = 30s;  // 测试内快照即时消费，不做心跳续期
    auto adapter = std::make_unique<MissionRosAdapter>(*adapter_node_, adapter_config);
    adapter_ = adapter.get();  // 所有权交给 runtime，测试侧留裸指针喂快照

    image_adapter_ = std::make_shared<TargetImageRosAdapter>(
        TargetImageRosAdapter::Config{},
        [](dog_patrol_perception_interfaces::msg::TrackedTargetImage::UniquePtr) {});

    ReidConfirmGate::Config gate_config;
    gate_config.service_name = "/reid_forward_test/dedup" + suffix;  // 不会被调用
    gate_config.handled_topic = handled_topic_;
    gate_ = std::make_unique<ReidConfirmGate>(*adapter_node_, gate_config);

    runtime_ = std::make_unique<MissionTrackingRuntime>(
        std::move(adapter), image_adapter_, PrimaryTargetCropConfig{}, gate_.get());

    // 等 handled 通道的发布/订阅匹配完成，避免首条消息丢失
    std::this_thread::sleep_for(400ms);
  }

  void TearDown() override {
    runtime_.reset();
    gate_.reset();
    image_adapter_.reset();
    adapter_node_.reset();
    sub_executor_.cancel();
    if (sub_spinner_.joinable()) {
      sub_spinner_.join();
    }
    sub_node_.reset();
  }

  static bool Store(MissionRosAdapter &adapter, const std::uint32_t seq,
                    const std::uint8_t state, const std::uint32_t target_id,
                    const std::uint32_t handled_target_id) {
    MissionStateMsg msg;
    msg.state_seq = seq;
    msg.state = state;
    msg.target_id = target_id;
    msg.handled_target_id = handled_target_id;
    return adapter.StoreMissionState(msg);
  }

  void RunFrame() {
    static const cv::Mat kBlank(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    SourceFrameMetadata metadata;
    metadata.image_width = 640;
    metadata.image_height = 480;
    runtime_->ProcessFrame({}, metadata, kBlank);
  }

  int HandledCount() {
    const std::lock_guard<std::mutex> lk(counters_mtx_);
    return handled_count_;
  }
  int ClearCount() {
    const std::lock_guard<std::mutex> lk(counters_mtx_);
    return clear_count_;
  }
  std::string LastTrack() {
    const std::lock_guard<std::mutex> lk(counters_mtx_);
    return last_track_;
  }

  // 有界等待转发到达（N06 的正向断言不允许无限等）
  bool WaitForHandled(const int expected, const int timeout_ms = 2000) {
    for (int i = 0; i < timeout_ms / 5 && HandledCount() < expected; ++i) {
      std::this_thread::sleep_for(5ms);
    }
    return HandledCount() >= expected;
  }

  // 负向断言：等一段时间确认没有新转发
  void ExpectNoNewHandled(const int expected) {
    std::this_thread::sleep_for(250ms);
    EXPECT_EQ(HandledCount(), expected);
  }

  std::string handled_topic_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Subscription<dog_patrol_reid_provider::msg::ReIDHandled>::SharedPtr handled_sub_;
  rclcpp::executors::SingleThreadedExecutor sub_executor_;
  std::thread sub_spinner_;
  rclcpp::Node::SharedPtr adapter_node_;
  MissionRosAdapter *adapter_{nullptr};  // 所有权在 runtime_ 内
  std::shared_ptr<TargetImageRosAdapter> image_adapter_;
  std::unique_ptr<ReidConfirmGate> gate_;
  std::unique_ptr<MissionTrackingRuntime> runtime_;
  std::mutex counters_mtx_;
  int handled_count_{0};
  int clear_count_{0};
  std::string last_track_;
};

// N06 主回归：同一 semantic ID 次日重新处置完成后必须再次转发。
TEST_F(ReidHandledForwardingTest, SameIdNewDayCompletionIsForwardedAgain) {
  ASSERT_TRUE(Store(*adapter_, 100U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  ASSERT_TRUE(WaitForHandled(1));
  EXPECT_EQ(LastTrack(), "semantic-7");

  // 跨天：免核验资格与日期无关（由截止时刻治理），同一 ID 的新处置结论
  // 经转换检测照常再次转发
  ASSERT_TRUE(Store(*adapter_, 101U, MissionStateMsg::VERIFY_IDENTITY, 7U, 0U));
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 102U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  EXPECT_TRUE(WaitForHandled(2));  // 旧实现停留在 1
  EXPECT_EQ(LastTrack(), "semantic-7");
}

// 同序号 heartbeat 重发与 RECOVER→PATROL/READY 携带的同一结论不重复转发。
TEST_F(ReidHandledForwardingTest, HeartbeatAndCarriedConclusionNotForwardedAgain) {
  const MissionStateMsg completion = [] {
    MissionStateMsg msg;
    msg.state_seq = 100U;
    msg.state = MissionStateMsg::RECOVER_PATROL;
    msg.target_id = 7U;
    msg.handled_target_id = 7U;
    return msg;
  }();
  ASSERT_TRUE(adapter_->StoreMissionState(completion));
  RunFrame();
  ASSERT_TRUE(WaitForHandled(1));

  ASSERT_TRUE(adapter_->StoreMissionState(completion));  // 同序号 heartbeat
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 101U, MissionStateMsg::PATROL, 0U, 7U));  // 携带
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 102U, MissionStateMsg::READY, 0U, 7U));
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 103U, MissionStateMsg::PATROL, 0U, 7U));
  RunFrame();
  ExpectNoNewHandled(1);
}

// 跨天仍保留的旧 handled 快照不得重新授予当日已处理资格。
TEST_F(ReidHandledForwardingTest, CrossDayRetainedSnapshotDoesNotRegrantToday) {
  ASSERT_TRUE(Store(*adapter_, 100U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  ASSERT_TRUE(WaitForHandled(1));

  ASSERT_TRUE(Store(*adapter_, 101U, MissionStateMsg::PATROL, 0U, 7U));
  RunFrame();
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 102U, MissionStateMsg::READY, 0U, 7U));
  RunFrame();
  ExpectNoNewHandled(1);
}

// 不同 ID 的新处置结果正常转发。
TEST_F(ReidHandledForwardingTest, DifferentIdCompletionIsForwarded) {
  ASSERT_TRUE(Store(*adapter_, 100U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  ASSERT_TRUE(WaitForHandled(1));

  ASSERT_TRUE(Store(*adapter_, 101U, MissionStateMsg::VERIFY_IDENTITY, 8U, 0U));
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 102U, MissionStateMsg::RECOVER_PATROL, 8U, 8U));
  RunFrame();
  EXPECT_TRUE(WaitForHandled(2));
  EXPECT_EQ(LastTrack(), "semantic-8");
}

// STARTUP reset 清除转发上下文与 provider handled；同 ID 再次处置正常转发。
TEST_F(ReidHandledForwardingTest, StartupResetThenSameIdCompletionForwardsAgain) {
  ASSERT_TRUE(Store(*adapter_, 100U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  ASSERT_TRUE(WaitForHandled(1));

  ASSERT_TRUE(Store(*adapter_, 101U, MissionStateMsg::STARTUP, 0U, 0U));
  RunFrame();
  for (int i = 0; i < 200 && ClearCount() < 1; ++i) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(ClearCount(), 1);

  ASSERT_TRUE(Store(*adapter_, 102U, MissionStateMsg::VERIFY_IDENTITY, 7U, 0U));
  RunFrame();
  ASSERT_TRUE(Store(*adapter_, 103U, MissionStateMsg::RECOVER_PATROL, 7U, 7U));
  RunFrame();
  EXPECT_TRUE(WaitForHandled(2));
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
