#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "dog_patrol_perception_tracking/modules/reid_confirm_gate.hpp"

#include <dog_patrol_reid_provider/msg/re_id_handled.hpp>
#include <dog_patrol_reid_provider/srv/re_id_dedup.hpp>

// 真实 ReidConfirmGate + 受控 mock service 的行为回归（评审 N02/N03/T01）：
//   - N02：reset/超时/被新问询替代后到达的旧响应必须丢弃，不得重建
//     锁存条目或排除队列；
//   - N03：跨天清理不依赖 Allow 被触发（全员被排除时没有 confirm 事件）；
//   - T01：新问询（Q2）的成功断言必须在 Q2 自身的超时预算内做出——删除
//     Q2 响应时该断言必须失败，不能被第二次 fail-open 满足；并用
//     Reentrant + 多线程 mock 构造 Q2 先返回、Q1 后返回的乱序时序。
namespace {

using dog_patrol_perception_tracking::ReidConfirmGate;
using DedupSrv = dog_patrol_reid_provider::srv::ReIDDedup;
using namespace std::chrono_literals;

// 受控 mock：service 回调收到请求后挂起，测试端按到达顺序释放并指定应答
// 内容。Reentrant 回调组 + MultiThreadedExecutor 允许 Q2 在 Q1 仍挂起时
// 被处理与应答（真正的乱序返回）。
class MockDedupService {
 public:
  struct Waiter {
    std::mutex mtx;
    std::condition_variable cv;
    bool released{false};
    int status{0};
    bool is_seen{false};
    bool handled{false};
    std::string track_id;
  };

  explicit MockDedupService(const std::string &service_name) {
    // 节点名只允许字母数字下划线（不能带话题名的前导 '/'）
    std::string node_name = service_name;
    node_name.erase(0, node_name.find_first_not_of('/'));
    node_ = rclcpp::Node::make_shared(node_name + "_mock");
    reentrant_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    service_ = node_->create_service<DedupSrv>(
        service_name,
        [this](const std::shared_ptr<DedupSrv::Request> request,
               std::shared_ptr<DedupSrv::Response> response) {
          const std::shared_ptr<Waiter> waiter = std::make_shared<Waiter>();
          {
            std::lock_guard<std::mutex> lk(mtx_);
            waiters_.push_back(waiter);
            arrived_.notify_all();
          }
          std::unique_lock<std::mutex> lk(waiter->mtx);
          waiter->cv.wait(lk, [&] { return waiter->released; });
          response->status = waiter->status;
          response->is_seen = waiter->is_seen;
          response->handled = waiter->handled;
          response->track_id = waiter->track_id;
          response->error_message.clear();
          (void)request;
        },
        rclcpp::ServicesQoS().get_rmw_qos_profile(), reentrant_group_);
    spinner_ = std::thread([this]() {
      executor_.add_node(node_);
      executor_.spin();
    });
  }

  ~MockDedupService() {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (const auto &waiter : waiters_) {
        std::lock_guard<std::mutex> wl(waiter->mtx);
        waiter->released = true;
        waiter->cv.notify_all();
      }
    }
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  // 有界等待第 index 个请求（按到达顺序，从 0 计）抵达服务端；超时返回
  // nullptr（请求丢失时测试应立即失败，而不是等外层 CTest 杀进程）。
  std::shared_ptr<Waiter> AwaitRequest(const std::size_t index,
                                       const int timeout_ms = 3000) {
    std::unique_lock<std::mutex> lk(mtx_);
    const bool arrived = arrived_.wait_for(
        lk, std::chrono::milliseconds(timeout_ms), [&] { return waiters_.size() > index; });
    return arrived ? waiters_[index] : nullptr;
  }

  bool HasRequest(const std::size_t index) {
    std::lock_guard<std::mutex> lk(mtx_);
    return waiters_.size() > index;
  }

  // 释放第 index 个请求并指定应答（未到达则记失败并返回）。
  void Respond(const std::size_t index, const bool is_seen, const bool handled) {
    const auto waiter = AwaitRequest(index);
    if (waiter == nullptr) {
      ADD_FAILURE() << "mock request " << index << " 未在期限内到达";
      return;
    }
    std::lock_guard<std::mutex> lk(waiter->mtx);
    waiter->is_seen = is_seen;
    waiter->handled = handled;
    waiter->released = true;
    waiter->cv.notify_all();
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::Service<DedupSrv>::SharedPtr service_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::mutex mtx_;
  std::condition_variable arrived_;
  std::vector<std::shared_ptr<Waiter>> waiters_;
};

class ReidConfirmGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo *info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    service_name_ = std::string("/reid_gate_test_") + info->name();
    day_ = "2026-10-10";
    mock_ = std::make_unique<MockDedupService>(service_name_);
    harness_ = rclcpp::Node::make_shared("reid_gate_test_harness");
  }

  void TearDown() override {
    gate_.reset();
    mock_.reset();
    harness_.reset();
  }

  // timeout/retry 取小值让超时与重问在测试时间尺度内可观察；需要断言
  // "结论不被迟到响应覆盖"的用例把 retry 拉长，使 Allow 从锁存应答而非
  // 重新问询。
  void MakeGate(const int timeout_ms, const double retry_sec = 0.05) {
    ReidConfirmGate::Config cfg;
    cfg.enable = true;
    cfg.service_name = service_name_;
    cfg.handled_topic = service_name_ + "_handled";
    cfg.timeout_ms = timeout_ms;
    cfg.retry_interval_sec = retry_sec;
    cfg.day_provider = [this]() { return day_; };
    gate_ = std::make_unique<ReidConfirmGate>(*harness_, cfg);
    crop_ = cv::Mat(128, 64, CV_8UC3, cv::Scalar(30, 40, 50));
    // 给 DDS 一点发现时间（服务未就绪时 Allow 会 fail-open 短暂锁存）
    std::this_thread::sleep_for(300ms);
  }

  // 发出首问并以"请求确实到达 mock"为准（Allow 返回值在服务未就绪时
  // 可能是 fail-open 的 true，不能作为发出依据）。
  bool UntilQueried(const int target_id) {
    for (int i = 0; i < 40; ++i) {
      gate_->Allow(target_id, crop_);
      if (mock_->HasRequest(0)) {
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return mock_->HasRequest(0);
  }

  // Q2 的成功断言窗口：必须短于 timeout，使删掉 Q2 响应时本函数拿不到
  // true（只能靠 fail-open 的用例会在此失败）——T01 的可变异验证点。
  bool AllowedWithinBudget(const int target_id, const int budget_ms) {
    for (int waited = 0; waited <= budget_ms; waited += 5) {
      if (gate_->Allow(target_id, crop_)) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  std::vector<int> Drain() {
    std::vector<int> taken;
    gate_->DrainExclusions([&taken](const int id) { taken.push_back(id); });
    return taken;
  }

  // 有界轮询等待排除事件到达（正向观察，不用固定 sleep）
  std::vector<int> AwaitExclusions(const int expected_count) {
    for (int i = 0; i < 400; ++i) {
      std::vector<int> taken = Drain();
      if (static_cast<int>(taken.size()) >= expected_count) {
        return taken;
      }
      std::this_thread::sleep_for(5ms);
    }
    return {};
  }

  std::string service_name_;
  std::string day_;
  cv::Mat crop_;
  rclcpp::Node::SharedPtr harness_;
  std::unique_ptr<MockDedupService> mock_;
  std::unique_ptr<ReidConfirmGate> gate_;
};

// N02：reset 清缓存后，在途旧响应不得重建条目/排除；reset 后的新问询正常。
TEST_F(ReidConfirmGateTest, ResetInvalidatesInFlightResponse) {
  MakeGate(3000);
  ASSERT_TRUE(UntilQueried(7));
  ASSERT_TRUE(mock_->AwaitRequest(0) != nullptr);

  gate_->NotifyHandledReset();
  mock_->Respond(0, true, true);  // 迟到的 seen+handled 响应
  std::this_thread::sleep_for(150ms);  // 给丢弃路径留处理窗口
  EXPECT_TRUE(Drain().empty());   // 旧响应被丢弃，未重建排除

  ASSERT_FALSE(gate_->Allow(7, crop_));  // reset 后重新问询（新代次）
  ASSERT_TRUE(mock_->AwaitRequest(1) != nullptr);
  mock_->Respond(1, true, true);
  const auto drained = AwaitExclusions(1);
  ASSERT_EQ(drained.size(), 1u);
  EXPECT_EQ(drained[0], 7);
}

// T01/N02：超时 fail-open 与新问询成功分开断言——Q2 的放行结论必须在
// Q2 自身超时预算内取得；Q1 的迟到 suppress 不能覆盖 Q2 的结论。
TEST_F(ReidConfirmGateTest, TimeoutFailsOpenAndNewResponseAcceptedWithinBudget) {
  MakeGate(120);
  ASSERT_TRUE(UntilQueried(7));
  ASSERT_TRUE(mock_->AwaitRequest(0) != nullptr);

  std::this_thread::sleep_for(180ms);      // 越过 Q1 在途超时
  EXPECT_TRUE(gate_->Allow(7, crop_));     // 断言一：超时 fail-open 放行
  std::this_thread::sleep_for(70ms);       // 越过放行重问窗口
  ASSERT_FALSE(gate_->Allow(7, crop_));    // 发起 Q2（pending）
  ASSERT_TRUE(mock_->AwaitRequest(1) != nullptr);

  mock_->Respond(0, true, true);           // Q1 迟到 suppress：按请求号丢弃
  std::this_thread::sleep_for(40ms);       // 只给丢弃路径留短窗，别挤占 Q2 预算
  EXPECT_TRUE(Drain().empty());

  mock_->Respond(1, false, false);         // Q2：新人
  // 断言二：Q2 成功在预算内成立——删除上一行时此处必须失败（T01）
  EXPECT_TRUE(AllowedWithinBudget(7, 60));
  EXPECT_TRUE(Drain().empty());
}

// T01 乱序：Q2 的 allow 先返回（Q1 仍挂起），随后 Q1 的 suppress 迟到，
// 不得覆盖 Q2 的放行结论或重建排除。
TEST_F(ReidConfirmGateTest, OutOfOrderQ2AllowThenLateQ1SuppressIsDropped) {
  MakeGate(120, 0.5);
  ASSERT_TRUE(UntilQueried(7));
  ASSERT_TRUE(mock_->AwaitRequest(0) != nullptr);

  std::this_thread::sleep_for(180ms);      // Q1 超时
  EXPECT_TRUE(gate_->Allow(7, crop_));     // fail-open
  std::this_thread::sleep_for(550ms);      // 越过 0.5s 放行窗口
  ASSERT_FALSE(gate_->Allow(7, crop_));    // 发起 Q2
  ASSERT_TRUE(mock_->AwaitRequest(1) != nullptr);

  mock_->Respond(1, false, false);         // Q2 allow 先返回（Q1 仍挂起）
  EXPECT_TRUE(AllowedWithinBudget(7, 60));  // Q2 结论在预算内生效

  mock_->Respond(0, true, true);           // Q1 suppress 迟到
  std::this_thread::sleep_for(150ms);
  EXPECT_TRUE(gate_->Allow(7, crop_));     // 仍放行（retry 窗口内取锁存）
  EXPECT_TRUE(Drain().empty());
}

// T01 乱序：Q2 的 suppress 先返回（正向观察排除事件），随后 Q1 的 allow
// 迟到，不得解除 Q2 的抑制。
TEST_F(ReidConfirmGateTest, OutOfOrderQ2SuppressThenLateQ1AllowCannotUnlock) {
  MakeGate(120);
  ASSERT_TRUE(UntilQueried(7));
  ASSERT_TRUE(mock_->AwaitRequest(0) != nullptr);

  std::this_thread::sleep_for(180ms);      // Q1 超时
  EXPECT_TRUE(gate_->Allow(7, crop_));     // fail-open
  std::this_thread::sleep_for(70ms);
  ASSERT_FALSE(gate_->Allow(7, crop_));    // 发起 Q2
  ASSERT_TRUE(mock_->AwaitRequest(1) != nullptr);

  mock_->Respond(1, true, true);           // Q2 suppress 先返回
  const auto drained = AwaitExclusions(1);
  ASSERT_EQ(drained.size(), 1u);
  EXPECT_EQ(drained[0], 7);

  mock_->Respond(0, false, false);         // Q1 allow 迟到
  std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(gate_->Allow(7, crop_));    // 抑制保持
  EXPECT_TRUE(Drain().empty());            // 无新增排除
}

// N03：跨天清理独立于 Allow 触发；锁存抑制与排除队列一并失效。
TEST_F(ReidConfirmGateTest, DayRolloverClearsLatchedSuppressWithoutAllow) {
  MakeGate(3000);
  ASSERT_TRUE(UntilQueried(7));
  mock_->Respond(0, true, true);  // seen+handled → 锁存 kSuppress + 排除
  ASSERT_EQ(AwaitExclusions(1).size(), 1u);
  ASSERT_FALSE(gate_->Allow(7, crop_));  // kSuppress：持续抑制

  day_ = "2026-10-11";
  EXPECT_TRUE(gate_->SyncDay());   // 跨天：不依赖任何 confirm 事件
  EXPECT_FALSE(gate_->SyncDay());  // 同日重复调用
  EXPECT_TRUE(Drain().empty());    // 待注入的旧排除也一并清掉

  ASSERT_FALSE(gate_->Allow(7, crop_));  // 锁存已清：重新问询
  ASSERT_TRUE(mock_->AwaitRequest(1) != nullptr);
  mock_->Respond(1, true, false);        // 新一天口径：见过但未处置
  // timeout=3000，此 true 只能来自 Q2 的真实响应（不会被 fail-open 满足）
  bool allowed = false;
  for (int i = 0; i < 30 && !allowed; ++i) {
    allowed = gate_->Allow(7, crop_);
    if (!allowed) {
      std::this_thread::sleep_for(10ms);
    }
  }
  EXPECT_TRUE(allowed);
  EXPECT_TRUE(Drain().empty());
}

// N03：跨天同时作废在途问询——跨天后返回的旧响应直接丢弃。
TEST_F(ReidConfirmGateTest, DayRolloverInvalidatesInFlightQuery) {
  MakeGate(3000);
  ASSERT_TRUE(UntilQueried(7));
  ASSERT_TRUE(mock_->AwaitRequest(0) != nullptr);

  day_ = "2026-10-11";
  ASSERT_TRUE(gate_->SyncDay());
  mock_->Respond(0, true, true);
  std::this_thread::sleep_for(150ms);
  EXPECT_TRUE(Drain().empty());
}

// 未启用的 gate 恒放行（reid.enable=false 与 merge 行为一致）。
TEST(ReidConfirmGateDisabledTest, DisabledGateAlwaysAllows) {
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("reid_gate_disabled_test");
  ReidConfirmGate::Config cfg;
  cfg.enable = false;
  ReidConfirmGate gate(*node, cfg);
  EXPECT_FALSE(gate.enabled());
  EXPECT_TRUE(gate.Allow(7, cv::Mat()));
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
