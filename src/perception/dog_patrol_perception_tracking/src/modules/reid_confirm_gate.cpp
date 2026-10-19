#include "dog_patrol_perception_tracking/modules/reid_confirm_gate.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <dog_patrol_reid_provider/msg/re_id_handled.hpp>
#include <dog_patrol_reid_provider/srv/re_id_dedup.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <deque>
#include <exception>
#include <future>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

namespace dog_patrol_perception_tracking {

namespace {
// 本地日期 "yyyy-mm-dd"（与 dog_patrol_reid_provider 的库日清零同口径）
std::string local_today() {
  const std::time_t tt = std::time(nullptr);
  const std::tm* tm_ptr = std::localtime(&tt);
  if (tm_ptr == nullptr) {
    return "1970-01-01";
  }
  std::tm tm_buf = *tm_ptr;
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday);
  return std::string(buf);
}
}  // namespace

// pimpl：helper 节点/线程与 rosidl 类型细节不进头文件。
struct ReidConfirmGate::Impl {
  enum class Decision { kNone, kPending, kAllow, kSuppress };
  struct Entry {
    Decision state{Decision::kNone};
    std::chrono::steady_clock::time_point decided_at{};
    std::uint64_t pending_request_id{0};  // 在途问询的请求号（迟到响应校验）
    cv::Mat crop_cache;  // 最近一次问询的裁剪（NotifyHandled 的外观兜底）
  };

  using DedupSrv = dog_patrol_reid_provider::srv::ReIDDedup;

  Config config;
  rclcpp::Logger logger{rclcpp::get_logger("reid_confirm_gate")};
  rclcpp::Node::SharedPtr helper;
  rclcpp::Client<DedupSrv>::SharedPtr client;
  rclcpp::Publisher<dog_patrol_reid_provider::msg::ReIDHandled>::SharedPtr
      handled_pub;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;

  std::mutex mtx;  // entries/exclusions/day/generation：帧线程与 helper 回调线程共享
  std::unordered_map<int, Entry> entries;
  std::deque<int> exclusions;  // seen+handled，待注入选人资格（R04）
  std::string day;             // 锁存结论所属日期（本地口径，跨天失效）
  // 取消代次（N02）：reset/跨天时递增，在途回调凭发起时捕获的代次比对，
  // 不一致的旧响应整体丢弃，不得重建条目或排除
  std::uint64_t generation{1};
  std::uint64_t next_request_id{1};  // 全局递增：识别"已被新问询替代"的旧响应

  // helper 执行器线程回调：问询响应到达，只更新对应条目的锁存结论。
  // 发布决策仍由下一帧的 Allow() 在调用方当前状态上下文里做（R05 重验）。
  void OnResponse(int target_id, std::uint64_t generation, std::uint64_t request_id,
                  std::shared_future<DedupSrv::Response::SharedPtr> fut) {
    bool suppress = false;
    bool seen_only = false;
    try {
      const auto response = fut.get();
      if (response->status == 0) {
        if (response->is_seen && response->handled) {
          suppress = true;  // R06-A：见过且已处置才抑制
        } else if (response->is_seen) {
          seen_only = true;  // 见过但未处置：照常走处置流程
        }
      }  // status!=0 按 ERROR=无结论 fail-open
    } catch (const std::exception&) {
      // future 异常（服务中断等）：按无结论处理
    }
    std::lock_guard<std::mutex> lk(mtx);
    // N02：迟到/失效响应直接丢弃，不得用 operator[] 重建条目——reset/跨天
    // 后的旧响应、超时 fail-open 后的迟到结论、被新问询替代的旧请求都在此
    // 拦下，避免旧结论污染新业务上下文。
    if (generation != this->generation) {
      return;
    }
    const auto it = entries.find(target_id);
    if (it == entries.end() || it->second.state != Decision::kPending ||
        it->second.pending_request_id != request_id) {
      return;
    }
    Entry& e = it->second;
    e.state = suppress ? Decision::kSuppress : Decision::kAllow;
    e.decided_at = std::chrono::steady_clock::now();
    if (suppress) {
      exclusions.push_back(target_id);
      RCLCPP_INFO(logger,
                  "[reid_gate] semantic_id=%d 今天已见过且已处置，抑制 confirm_target",
                  target_id);
    } else {
      RCLCPP_INFO(logger,
                  "[reid_gate] semantic_id=%d %s，放行 confirm_target", target_id,
                  seen_only ? "见过但未处置" : "新人");
    }
  }
};

ReidConfirmGate::ReidConfirmGate(rclcpp::Node& node, Config config) {
  if (!config.enable) {
    return;
  }
  impl_ = std::make_unique<Impl>();
  impl_->config = config;
  impl_->logger = node.get_logger();
  impl_->day = config.day_provider ? config.day_provider() : local_today();
  impl_->helper = std::make_shared<rclcpp::Node>("perception_reid_confirm_gate");
  impl_->client =
      impl_->helper->create_client<Impl::DedupSrv>(config.service_name);
  impl_->handled_pub =
      impl_->helper->create_publisher<dog_patrol_reid_provider::msg::ReIDHandled>(
          config.handled_topic, 10);
  Impl* impl = impl_.get();
  impl_->spinner = std::thread([impl]() {
    impl->executor.add_node(impl->helper);
    impl->executor.spin();
  });
  RCLCPP_INFO(node.get_logger(),
              "reid_confirm_gate 已启用（异步问询 service=%s, handled 下发=%s, "
              "timeout=%dms, retry=%.1fs）",
              config.service_name.c_str(), config.handled_topic.c_str(),
              config.timeout_ms, config.retry_interval_sec);
}

ReidConfirmGate::~ReidConfirmGate() {
  if (!impl_) {
    return;
  }
  impl_->executor.cancel();
  if (impl_->spinner.joinable()) {
    impl_->spinner.join();
  }
}

bool ReidConfirmGate::enabled() const { return impl_ != nullptr; }

bool ReidConfirmGate::Allow(int target_id, const cv::Mat& crop) {
  if (!impl_) {
    return true;
  }
  Impl& impl = *impl_;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(impl.mtx);
  // 跨天失效不在Allow里做（N03）：全员被排除时没有 confirm 事件、Allow
  // 不会被调用；日期推进由每帧固定入口 SyncDay() 独立处理。
  if (impl.entries.size() >= impl.config.max_tracked) {
    impl.entries.clear();  // 容量自愈：最坏情况多问一轮
  }
  Impl::Entry& e = impl.entries[target_id];
  switch (e.state) {
    case Impl::Decision::kSuppress:
      return false;  // 已见且已处置：对该 semantic_id 持续抑制
    case Impl::Decision::kAllow:
      if (now - e.decided_at <
          std::chrono::duration<double>(impl.config.retry_interval_sec)) {
        return true;  // 窗口内已有放行结论，不重复问询
      }
      break;  // 窗口过期：重新问询
    case Impl::Decision::kPending:
      if (now - e.decided_at <
          std::chrono::duration<int, std::milli>(impl.config.timeout_ms)) {
        return false;  // 问询在途：hold 本帧事件，等下一帧带结论重试
      }
      e.state = Impl::Decision::kAllow;  // 超时 fail-open
      e.decided_at = now;
      RCLCPP_WARN(impl.logger,
                  "[reid_gate] semantic_id=%d 问询超时(%dms)，按新人放行",
                  target_id, impl.config.timeout_ms);
      return true;
    case Impl::Decision::kNone:
      break;
  }
  // 发起新问询（异步）：本帧 hold。crop.clone() 保证连续内存且生命周期
  // 独立于当前帧（响应晚于当前帧到达）。
  if (!impl.client->service_is_ready() || crop.empty() ||
      crop.type() != CV_8UC3) {
    e.state = Impl::Decision::kAllow;  // fail-open：无 ReID 也不阻塞
    e.decided_at = now;
    return true;
  }
  e.crop_cache = crop.clone();
  e.state = Impl::Decision::kPending;
  e.decided_at = now;
  e.pending_request_id = impl.next_request_id++;
  const std::uint64_t generation = impl.generation;
  const std::uint64_t request_id = e.pending_request_id;
  auto request = std::make_shared<Impl::DedupSrv::Request>();
  request->track_id = "semantic-" + std::to_string(target_id);
  request->header.stamp = impl.helper->get_clock()->now();
  request->crop.height = e.crop_cache.rows;
  request->crop.width = e.crop_cache.cols;
  request->crop.encoding = "bgr8";
  request->crop.step =
      static_cast<std::uint32_t>(e.crop_cache.cols * 3);
  request->crop.data.assign(
      e.crop_cache.data,
      e.crop_cache.data + e.crop_cache.total() * e.crop_cache.elemSize());
  Impl* impl_ptr = &impl;
  impl.client->async_send_request(
      request,
      [impl_ptr, target_id, generation, request_id](
          std::shared_future<Impl::DedupSrv::Response::SharedPtr> fut) {
        impl_ptr->OnResponse(target_id, generation, request_id, fut);
      });
  return false;
}

void ReidConfirmGate::DrainExclusions(
    const std::function<void(int target_id)>& sink) {
  if (!impl_ || !sink) {
    return;
  }
  std::deque<int> taken;
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    taken.swap(impl_->exclusions);
  }
  for (const int target_id : taken) {
    sink(target_id);
  }
}

bool ReidConfirmGate::SyncDay() {
  if (!impl_) {
    return false;
  }
  const std::string today =
      impl_->config.day_provider ? impl_->config.day_provider() : local_today();
  std::lock_guard<std::mutex> lk(impl_->mtx);
  if (today == impl_->day) {
    return false;
  }
  // N03：跨天独立失效——provider 当日库清零后，锁存结论、待注入的排除
  // 队列与在途问询一并作废；调用方据此同步清理选人层的 ReID 排除项。
  impl_->day = today;
  impl_->entries.clear();
  impl_->exclusions.clear();
  ++impl_->generation;
  RCLCPP_INFO(impl_->logger,
              "[reid_gate] 跨天 %s：清空当日问询锁存/排除队列并作废在途响应",
              today.c_str());
  return true;
}

void ReidConfirmGate::NotifyHandled(int target_id) {
  if (!impl_ || target_id <= 0) {
    return;
  }
  cv::Mat crop;
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    const auto it = impl_->entries.find(target_id);
    if (it != impl_->entries.end()) {
      crop = it->second.crop_cache;  // 浅拷贝引用计数，互斥锁外只读
      // Verification changed this person's business disposition. The next
      // patrol confirmation must ask ReID again, even while an earlier allow
      // cache is still fresh. Clearing pending state also invalidates any
      // response issued before this result; a later query gets a new ID.
      it->second.state = Impl::Decision::kNone;
      it->second.pending_request_id = 0;
    }
  }
  dog_patrol_reid_provider::msg::ReIDHandled msg;
  msg.header.stamp = impl_->helper->get_clock()->now();
  msg.track_id = "semantic-" + std::to_string(target_id);
  if (!crop.empty()) {
    msg.crop.height = crop.rows;
    msg.crop.width = crop.cols;
    msg.crop.encoding = "bgr8";
    msg.crop.step = static_cast<std::uint32_t>(crop.cols * 3);
    msg.crop.data.assign(crop.data,
                         crop.data + crop.total() * crop.elemSize());
  }
  impl_->handled_pub->publish(msg);
  RCLCPP_INFO(impl_->logger,
              "[reid_gate] handled 结论下发：semantic_id=%d（%s兜底 crop）",
              target_id, crop.empty() ? "无" : "带");
}

void ReidConfirmGate::NotifyHandledReset() {
  if (!impl_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    impl_->entries.clear();
    impl_->exclusions.clear();
    ++impl_->generation;  // N02：reset 作废在途问询，旧响应到达即丢弃
  }
  dog_patrol_reid_provider::msg::ReIDHandled msg;
  msg.header.stamp = impl_->helper->get_clock()->now();
  msg.clear_all = true;
  impl_->handled_pub->publish(msg);
  RCLCPP_INFO(impl_->logger,
              "[reid_gate] 任务 reset：清空本地锁存并下发 handled 清除");
}

}  // namespace dog_patrol_perception_tracking
