# 人体检测节点 ↔ ReID 去重节点 接口对接文档 v1.0

2026-09-30 定稿。检测节点按本文接入即可，云端相关均为 ReID 节点内部行为，
检测侧无感知（当前阶段云端功能关闭，所有问询**立即**同步返回）。
接口定义文件：`dog_patrol_reid_provider/srv/ReIDDedup.srv`（本文与其逐字一致）。

## 1. 概要

| 项 | 值 |
|---|---|
| 服务名 | `/perception/reid/dedup` |
| 类型 | ROS 2 service（同步一对一），`dog_patrol_reid_provider/srv/ReIDDedup` |
| 提供方 | ReID 去重节点 `/perception/reid`（C++） |
| 调用方 | 人体检测节点（你们） |
| 检测侧额外依赖 | 无。不需要订阅任何 ReID 话题，只调这一个 service |

## 2. 服务定义（srv 全文）

```
# ---------- 请求 ----------
std_msgs/Header header      # stamp=裁剪图采集时刻; frame_id=相机坐标系(仅记录用)
string track_id             # 本地人员跟踪 ID，追踪侧分配，同一目标保持不变
sensor_msgs/Image crop      # 人体裁剪图 BGR8，整人框
float32 det_conf            # 该次检测置信度，缺省 0
string camera_id            # 相机 ID，单相机可留空
string task_id              # 巡检任务 ID，可空（透传扩展位，当前无消费者）
string event_id             # 感知事件 ID，可空（透传扩展位，当前无消费者）
---
# ---------- 响应 ----------
uint8 status                # 0=调用成功 1=失败(参数缺失/图不可用/提特征失败)
string track_id             # 回显请求的 track_id（对齐用）
bool is_seen                # 今天是否见过（外观事实：本狗端侧或其他狗云端命中）
bool handled                # 是否仍在"重复核验忽略时长"内（supervisor handled
                            #   结论经 ReID 外观延续且未到期；false=未处置/
                            #   仅见过/忽略时长已过，不能免核验）
float64 handled_until       # 免核验资格的固定截止时刻（系统墙钟 epoch 秒，绝对
                            #   值，生成响应时一次性算定；0=无资格）。检测侧以本地
                            #   墙钟与本时刻比较/折算，不得用"收到时刻+剩余秒数"
                            #   重建截止
string error_message        # status=1 时非空，其余为空
```

**功能边界（项目组 2026-09-30 确认 + handled 口径）**：ReID 去重只判断
"这个 track 的人今天是否见过"，**不做白/黑名单判定**。"是否处置完成"以
supervisor 下发的 handled 结论为准（经 `/perception/reid/handled` 落到
ReID 身份上，ReID 负责把该结论延续到同人换 track/重启后的新问询）——
**is_seen=true 且 handled=false 的人不豁免**，应照常走处置流程。核验通过
后的免核验资格受"重复核验忽略时长"约束（平台配置项，provider 参数
`handled_ignore_seconds`，默认 86400=24h，0=关闭）：资格从核验通过时刻起
完整持续，跨天不清、跨重启按墙钟精确倒计时；到期后 `handled` 变为 false，
同人重新出现应重新完整核验。

## 3. 请求字段怎么填

| 字段 | 必填 | 填法 |
|---|---|---|
| `header.stamp` | 建议 | 裁剪图真实采集时刻（frame_id 可不填）。仅用于记录，不影响判定 |
| `header.frame_id` | 否 | 相机坐标系名 |
| `track_id` | **是** | 追踪侧 ID，同一目标从出现到消失保持不变（如 `track-person-17`）。ReID 靠它做同一 track 的身份关联 |
| `crop` | **是** | 整人裁剪图，**encoding 必须为 `bgr8`**，尺寸不限（ReID 内部 resize 到 256×128）。其他 encoding 返回 status=1 |
| `det_conf` | 否 | 该帧 YOLO 置信度。ReID 侧配置了 `min_det_conf`（默认 0=不启用）时才生效 |
| `camera_id` / `task_id` / `event_id` | 否 | 当前阶段可全部留空，为将来云端联动预留的透传位 |

## 4. 响应字段怎么用

| 字段 | 含义 | 建议用法 |
|---|---|---|
| `status` | 0 成功 / 1 失败 | 1 时按"无结论"处理（可重试一次），**不要当作新人**；error_message 有原因 |
| `track_id` | 回显 | 多目标并发问询时确认对应关系 |
| `is_seen` | 这个人**今天**是否出现过（端侧特征库命中） | false → 首次出现，正常走你们的事件上报/后续流程；true → 今天已见过，是否抑制重复上报由你们/总控的策略决定，ReID 只给事实 |
| `handled` | 这个人是否**仍在重复核验忽略时长内**（supervisor 结论，按外观延续） | **免核验判定必须 is_seen && handled 同时成立**；仅 is_seen=true（见过但未处置/处置失败/被取消）不应跳过处置流程 |
| `handled_until` | 免核验资格的固定截止（epoch 秒，0=需重新核验） | 有效期来源为 provider 参数 `handled_ignore_seconds`（平台"重复核验忽略时长"，默认 24h，资格从核验通过时刻起完整持续、跨天不清）；以本地墙钟折算成本地截止，编码/传输延迟不顺延 |
| `error_message` | 失败原因 | 日志/调试用 |

## 5. 调用约定

1. **时机**：新 track 首次稳定出现（如连续 N 帧检出）必须调用一次；之后可选周期刷新（如每 2~3 s）。**重复调用安全**：同人再调用走特征合并，不会产生重复身份。
2. **耗时预期**：正常一次问询 ≈ 单帧推理耗时（Orin TensorRT 约 2~6 ms；OpenCV DNN CPU 后端约 0.7 s）+ 检索，同步返回。模型随节点启动**预加载**（TRT 反序列化约 1s，一次性），问询本身不含加载；建议调用方 service 超时设 3~5 s（余量覆盖极端情况）。
3. **超时/不可达**：检测侧按"无结论"处理并照常走自己的流程（与 status=1 同策略），不要因 ReID 不可用阻塞检测链路。
4. ReID 节点**每日凌晨自动清空**端侧库，跨天后同一人 is_seen 会重新变 false——这是设计行为（当天去重）。
5. ReID 节点把当日库持久化在 `data_dir/library.bin`（周期防抖落盘 + 退出兜底）：**重启/开关机后自动恢复当日库**，同日同人的 is_seen 结论不受影响；跨天文件自动作废（见第 4 条）。异常断电最多丢失最后 `persist.save_interval_sec`（默认 10 s）内的更新。

## 6. 调用代码示例

### Python（rclpy，推荐——负责人示例节点即 Python）

```python
import cv2
from sensor_msgs.msg import Image
from dog_patrol_reid_provider.srv import ReIDDedup


class PersonDetectionNode(...):
    def __init__(self):
        ...
        self.reid_cli = self.create_client(ReIDDedup, '/perception/reid/dedup')

    def _make_req(self, track_id: str, crop_bgr) -> ReIDDedup.Request:
        req = ReIDDedup.Request()
        req.track_id = track_id
        req.header.stamp = self.get_clock().now().to_msg()
        req.det_conf = 0.9                      # 可选，无则默认 0
        img = Image()
        img.height, img.width = crop_bgr.shape[:2]
        img.encoding = 'bgr8'                   # 必须是 bgr8
        img.step = crop_bgr.shape[1] * 3
        img.data = crop_bgr.tobytes()
        req.crop = img
        return req

    # 方式 A（推荐）：节点已在 spin，业务回调里异步问询、回调里拿结论
    def on_track_confirmed(self, track_id, crop_bgr):
        if not self.reid_cli.service_is_ready():
            return self.handle_no_reid()        # 服务不在线：按无结论处理，别阻塞
        fut = self.reid_cli.call_async(self._make_req(track_id, crop_bgr))
        fut.add_done_callback(lambda f: self._on_reid_answer(track_id, f))

    def _on_reid_answer(self, track_id, fut):
        if fut.exception() is not None:
            self.get_logger().warn('ReID 无应答，按无结论处理')
            return
        r = fut.result()
        if r.status != 0:                       # 1=ERROR：按无结论，不当作新人
            self.get_logger().warn(f'ReID 调用失败: {r.error_message}')
            return
        if r.is_seen:
            ...                                 # 今天已见过：按你们的策略抑制/降级重复上报
        else:
            ...                                 # 今日新人：正常走事件上报

    # 方式 B：可阻塞的上下文（独立线程/一次性脚本）里同步等待
    def ask_reid_blocking(self, track_id, crop_bgr, timeout_sec=5.0):
        fut = self.reid_cli.call_async(self._make_req(track_id, crop_bgr))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if not fut.done():
            return None                         # 超时=无结论
        r = fut.result()
        return None if r.status != 0 else r.is_seen
```

### C++（rclcpp）

```cpp
#include <future>
#include "dog_patrol_reid_provider/srv/reid_dedup.hpp"
#include <opencv2/core.hpp>

using dog_patrol_reid_provider::srv::ReIDDedup;
// 节点成员：
//   rclcpp::Client<ReIDDedup>::SharedPtr reid_cli_;
// 构造函数：
//   reid_cli_ = create_client<ReIDDedup>("/perception/reid/dedup");

// 在可阻塞的线程中调用；在回调里请改用 async_send_request + 回调
//（同 Python 方式 A）。返回 false = 无结论（超时/服务不在线/ERROR），
// 调用方照常走自己的流程，不要当作新人。
bool ask_reid(const std::string& track_id, const cv::Mat& crop_in,
              bool& is_seen) {
  if (!reid_cli_->service_is_ready()) return false;
  cv::Mat crop = crop_in.isContinuous() ? crop_in : crop_in.clone();
  auto req = std::make_shared<ReIDDedup::Request>();
  req->track_id = track_id;
  req->header.stamp = this->now();
  req->crop.height = crop.rows;
  req->crop.width = crop.cols;
  req->crop.encoding = "bgr8";                 // 必须是 bgr8
  req->crop.step = crop.cols * 3;
  req->crop.data.assign(crop.data,
                        crop.data + crop.total() * crop.elemSize());
  auto fut = reid_cli_->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return false;                              // 超时=无结论（5s 已含模型冷启动余量）
  }
  const auto resp = fut.get();
  if (resp->status != 0) return false;         // 原因在 resp->error_message
  is_seen = resp->is_seen;
  return true;
}
```

## 7. 编译对接

把 `dog_patrol_reid_provider` 放进与检测节点同一 colcon 工作空间的 `src/` 下：

- CMakeLists.txt：`find_package(dog_patrol_reid_provider REQUIRED)`，链接时用
  `rosidl_get_typesupport_target(...)` 同款写法（包 README 有完整示例）；
- package.xml：`<depend>dog_patrol_reid_provider</depend>`；
- Python 检测节点则直接 `from dog_patrol_reid_provider.srv import ReIDDedup`。

将来若负责人建立共享接口包，本 srv 迁移是机械改动，字段不变。

## 8. 参考实现与联调

- **现成参考客户端（Python）**：`shorttermreid/scripts/sim_detection_client.py`——
  读图填 msg、调服务、打印响应，检测节点可直接抄这段逻辑；
- **C++ 完整参考实现**：`shorttermreid/dog_patrol-merge-reid/`（旧感知仓库的
  集成副本）——confirm_target 发布前经 `ReidConfirmGate` 问询本 service、
  按结论抑制/放行，含参数、锁存、fail-open 全套处理；改动清单见该目录
  `REID_GATE_NOTES.md`，C++ 调用逻辑可对照 `reid_confirm_gate.cpp` 抄；
- 命令行手动测试（需一张 BGR8 图，data 传原始字节）：

```bash
ros2 service call /perception/reid/dedup dog_patrol_reid_provider/srv/ReIDDedup \
  "{track_id: 'track-1', crop: {height: 256, width: 128, encoding: 'bgr8', step: 384, data: [.]}}"
```

- 联调顺序：先 `cloud.enable:=false`（纯端侧，当前默认阶段）→ 检测节点接 service → 观测 `/perception/reid/status`（1 Hz JSON：库大小/模型状态）。
