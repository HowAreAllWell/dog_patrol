#pragma once

#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dog_patrol_perception_tracking {

// confirm_target 发出前的短时 ReID 去重闸门（异步问询版）。
//
// 职责（对应评审 R04/R05/R06-A 的修复方案）：
//   - Allow() 非阻塞：未有结论时异步发起 /perception/reid/dedup 问询并
//     返回 false（hold 本帧 confirm 事件，靠事务层的确认重发机制再试）；
//     绝不在调用方持有的业务锁内同步等待 service（R05）；
//   - 结论 = is_seen && handled 才抑制（R06-A：仅"见过"但未处置的人不豁免，
//     处置资格以任务侧 supervisor 的 handled 结论为准，ReID 只按外观把该
//     结论延续到换 track/重启后的同人）；
//   - seen+handled 的结论同时进入排除队列，由帧处理路径 DrainExclusions
//     取走并注入选人资格边界（R04：被去重者不再占用 primary，同屏其他
//     新人可被选择）；
//   - NotifyHandled / NotifyHandledReset：把 supervisor 的 handled 结论
//     转发 ReID 节点落库（附带最近问询 crop 作为绑定过期时的外观兜底）；
//     任务 reset 时请求清除全部 handled 标记；
//   - fail-open：服务不在线 / 问询超时 / ERROR → 放行，ReID 故障不阻塞
//     原有检测链路。
//
// 线程模型：Allow/DrainExclusions/NotifyHandled 在帧处理串行路径调用；
// 问询响应回调在 gate 自带的 helper 执行器线程到达，内部互斥锁保护共享
// 状态。发布决策永远只发生在调用方的当前帧上下文里，迟到的问询结论只
// 更新锁存，不会凭旧状态补发事件（R05 的结果重验要求由此结构性满足）。
class ReidConfirmGate {
 public:
  struct Config {
    bool enable{true};
    std::string service_name{"/perception/reid/dedup"};
    std::string handled_topic{"/perception/reid/handled"};
    int timeout_ms{3000};            // 问询在途超时：fail-open 放行
    double retry_interval_sec{2.0};  // 放行结论的有效窗口，窗口过后可重问
    std::size_t max_tracked{64};     // 结论表容量，超限整体清空（自愈）
    // 日期源（"yyyy-mm-dd"，本地日期口径，与 provider 端侧库一致）。跨天时
    // 锁存结论整体失效——provider 库每日清零后，昨天的结论必须一并作废。
    // 默认取系统本地日期；测试可注入假日期。
    std::function<std::string()> day_provider;
  };

  ReidConfirmGate(rclcpp::Node &node, Config config);
  ~ReidConfirmGate();

  ReidConfirmGate(const ReidConfirmGate &) = delete;
  ReidConfirmGate &operator=(const ReidConfirmGate &) = delete;

  bool enabled() const;

  // true = 允许发出 confirm_target；false = 抑制（已见且已处置）或问询在途
  //（hold：本帧不发布，等下一帧带结论重试）。非阻塞。crop 为该目标当前帧
  // 的 BGR 裁剪（空图/服务不在线按 fail-open 放行）。
  bool Allow(int target_id, const cv::Mat &crop);

  // 取走自上次调用以来新产生的"seen+handled"排除结论（R04）。每条应送入
  // 选人资格边界（MissionRosAdapter::ExcludeForReidDedup）。
  void DrainExclusions(const std::function<void(int target_id)> &sink);

  // 每帧固定调用一次（在 DrainExclusions 之前）：独立处理跨天（N03），不依赖
  // Allow 被触发。发生跨天时清空当日问询锁存与排除队列、作废在途问询，并
  // 返回 true——调用方应同步清理选人层的 ReID 排除项
  // （MissionRosAdapter::ClearReidDedupExclusions）。
  bool SyncDay();

  // supervisor handled 结论转发：请求 ReID 节点把该 track_id 的身份标记
  // 已处置。自动附带该目标最近一次问询的 crop 作为外观兜底。
  // 同时作废该目标之前的缓存/在途问询，下次确认重新向 ReID 获取资格。
  void NotifyHandled(int target_id);

  // 任务 reset/新会话：请求 ReID 节点清除全部 handled 标记，并清空本地
  // 锁存结论（与选人层的 ReID 排除清理对齐）。
  void NotifyHandledReset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_patrol_perception_tracking
