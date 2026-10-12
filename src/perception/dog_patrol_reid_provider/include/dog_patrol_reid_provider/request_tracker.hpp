// -*- coding: utf-8 -*-
// 云端请求关联表：request_id -> 等待中的请求上下文。
// 三条访问路径（多线程执行器下并发）：
//   1) service 线程：add 后轮询 poll() 直到拿到结果或超过 await_deadline；
//   2) 云端响应回调：on_response() 回填结果（供 1 消费），并对超时未走的
//      请求做迟到绑定（条目保留到 expire_at 才被 sweep 清掉）；
//   3) 定时器：sweep() 清理完全过期条目。
// 纯标准库 + 内置互斥，不依赖 ROS，可独立自测。
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace dog_patrol_reid_provider {

// 云端响应的脱壳结果（从 ReIDResponse msg 转换而来，避免本头依赖 ROS）
struct CloudOutcome {
  bool succeeded = false;       // status == "succeeded"
  bool is_match = false;
  float similarity = -1.0F;
  std::string global_person_id;
  uint8_t match_type = 255;
  std::string error_code;
};

struct PendingCloud {
  std::string request_id;
  std::string local_reid_id;    // 该请求关联的端侧身份（迟到绑定的落点）
  std::string track_id;
  double await_deadline = 0.0;  // service 等待截止（steady 秒）；<=0 表示不等待（fire-and-forget）
  double expire_at = 0.0;       // 条目最终保留截止（迟到响应窗口）
  bool completed = false;       // 已收到响应
  bool timed_out_reported = false;  // service 已按超时应答（此后响应只做迟到绑定）
};

class RequestTracker {
 public:
  // 登记一条请求。await_deadline <= 0 表示 fire-and-forget（不阻塞等待）
  void add(const std::string& request_id, const std::string& local_reid_id,
           const std::string& track_id, double await_deadline, double expire_at) {
    std::lock_guard<std::mutex> lk(mtx_);
    PendingCloud p;
    p.request_id = request_id;
    p.local_reid_id = local_reid_id;
    p.track_id = track_id;
    p.await_deadline = await_deadline;
    p.expire_at = expire_at;
    pending_[request_id] = std::move(p);
  }

  // 响应到达：条目存在则回填结果并返回其上下文（含 local_reid_id 供绑定）；
  // 条目不存在（重复响应/已清理）返回 false
  bool on_response(const std::string& request_id, const CloudOutcome& oc,
                   PendingCloud& out) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = pending_.find(request_id);
    if (it == pending_.end()) {
      return false;
    }
    it->second.completed = true;
    outcome_[request_id] = oc;
    out = it->second;
    return true;
  }

  // service 线程取结果：已完成则取出并删除条目，返回 true
  bool poll(const std::string& request_id, CloudOutcome& oc) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = pending_.find(request_id);
    if (it == pending_.end() || !it->second.completed) {
      return false;
    }
    auto oit = outcome_.find(request_id);
    oc = (oit != outcome_.end()) ? oit->second : CloudOutcome{};
    outcome_.erase(request_id);
    pending_.erase(it);
    return true;
  }

  // 等待超时（service 应答前调用一次做标记，条目保留至 expire_at 供迟到绑定）
  bool mark_timeout(const std::string& request_id) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = pending_.find(request_id);
    if (it == pending_.end() || it->second.timed_out_reported) {
      return false;
    }
    it->second.timed_out_reported = true;
    return true;
  }

  // 清理完全过期条目，返回被清理的 request_id（供日志）
  std::vector<std::string> sweep(double now) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> removed;
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (now > it->second.expire_at) {
        removed.push_back(it->first);
        outcome_.erase(it->first);
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    return removed;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_.size();
  }

 private:
  mutable std::mutex mtx_;
  std::map<std::string, PendingCloud> pending_;
  std::map<std::string, CloudOutcome> outcome_;  // request_id -> 已到达待取走的结果
};

}  // namespace dog_patrol_reid_provider
