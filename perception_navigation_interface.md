# 机器狗巡逻系统感知与导航交互协议

## 1. 文档目的

本文档定义机器狗巡逻任务中以下模块的顶层交互协议：

- 全局任务状态机；
- 感知与身份识别模块；
- 导航与运动控制模块；
- 上位机或人工处置端。

系统通过一个独立节点管理全局任务状态：

```text
mission_supervisor
```

感知和导航模块根据全局状态执行自己的内部流程，并通过统一事件接口向
`mission_supervisor` 报告已经发生的业务结果。

视觉仓库只提供感知拥有的语义 `target_id`、目标事件和新鲜 bbox；身份授权结论
由独立的授权模块产生，目标位置、停车和路线等导航决策由独立的导航模块产生。
`mission_supervisor` 只验证和编排这些结果，不实现授权或导航决策。

首版协议重点解决：

- 巡逻过程中发现可疑目标；
- 感知向导航传递目标 bounding box；
- 导航利用雷达和标定结果计算目标位置与距离；
- 机器狗接近目标并在约 3 米处停车；
- 独立授权模块返回身份认证结论；
- 认证成功后恢复巡逻；
- 认证失败后持续跟踪目标。

本文档不规定检测模型、人脸识别、雷达投影、路径规划和底盘控制算法的内部实现。

## 2. 设计原则

### 2.1 状态机只管理全局业务状态

`mission_supervisor` 不处理图像、点云、bbox、路径和速度。

它只负责：

- 发布当前权威全局状态；
- 接收模块事件；
- 检查事件是否合法；
- 执行状态转换；
- 保存当前目标 ID；
- 保存阻塞原因；
- 标记任务是否因故障阻塞。

### 2.2 只有状态机可以修改全局状态

感知模块不能直接要求系统进入某个状态，只能报告事件：

```text
TARGET_CONFIRMED
TARGET_REACQUIRED
AUTHORIZED
UNAUTHORIZED
```

导航模块也不能直接设置全局状态，只能报告事件：

```text
TARGET_POSITION_READY
ARRIVED_AND_STOPPED
```

状态机收到事件后，根据当前状态和事件来源决定是否接受。

### 2.3 状态、事件和连续数据分开

| 类型 | 含义 | 示例 |
|---|---|---|
| 状态 | 系统当前正在执行什么任务 | `PATROL`、`APPROACH_TARGET` |
| 事件 | 某件业务结果已经发生 | `TARGET_CONFIRMED`、`AUTHORIZED` |
| 连续数据 | 算法持续更新的数据 | bbox、目标距离 |

交互方向：

```text
状态机 -> 感知、导航：全局状态
感知、导航、上位机 -> 状态机：统一事件
感知 -> 导航：目标 bbox
导航 -> 状态机、调试端：目标距离和执行状态
```

### 2.4 首版只保留必要关联字段

首版同时只处理一个目标，因此不使用 UUID 会话 ID。

协议只保留：

- `state_seq`：区分不同的全局状态版本；
- `target_id`：区分不同目标；
- `header.stamp`：判断连续数据是否过期。

### 2.5 不设置自动 fallback 业务状态

目标丢失或模块技术故障时：

- 导航立即停止继续使用旧目标运动；
- 模块发布 `TARGET_LOST` 或 `EXECUTION_ERROR`；
- 状态机保持当前业务状态；
- 状态机设置 `blocked=true`；
- 状态机发布可区分的 `block_cause`；
- 等待人工处理或后续版本增加复位策略。

首版不自动换目标、不自动恢复巡逻，也不增加搜索或重试状态。

同一语义目标在 `TARGET_LOST` 后重新被感知到时，感知可以发送
`TARGET_REACQUIRED`。状态机只清除该 `TARGET_LOST` 阻塞，保留当前业务状态和
`target_id`；`EXECUTION_ERROR` 阻塞不能由此事件清除。

## 3. 系统组成

### 3.1 mission_supervisor

职责：

- 发布 `/mission/state`；
- 订阅 `/mission/event`；
- 保存当前 `state`、`state_seq` 和 `target_id`；
- 记录感知和导航 Ready 状态；
- 检查事件来源、状态版本和目标 ID；
- 根据状态转移表执行转换；
- 对重复事件保持幂等；
- 发布阻塞状态和诊断说明。

状态机不负责：

- 选择可疑人员；
- 读取 bbox；
- 计算目标空间位置；
- 判断机器人是否真正停车；
- 执行身份认证；
- 发布导航路径和底盘速度。

### 3.2 感知与识别模块

职责：

- 在巡逻期间持续检测可疑人员；
- 从多个候选中选择一个主目标；
- 为主目标分配稳定的 `target_id`；
- 报告 `TARGET_CONFIRMED`；
- 持续发布当前目标 bbox；
- 在接近过程中维持同一目标的视觉跟踪；
- 在认证状态向授权模块提供当前目标上下文；
- 经感知接入适配器上报独立授权模块产生的 `AUTHORIZED` 或 `UNAUTHORIZED`；
- 在入侵者跟踪状态继续发布同一目标 bbox；
- 目标丢失或算法故障时报告错误事件。

感知内部的首次人脸、语音提示、再次人脸和口令确认不进入全局状态机。

### 3.3 导航与控制模块

职责：

- 在巡逻状态执行 waypoint 巡逻；
- 在目标确认后暂停巡逻并保持停车；
- 根据 bbox、雷达、相机标定和 TF 建立目标地图位置；
- 目标位置稳定后报告 `TARGET_POSITION_READY`；
- 在接近状态向目标靠近；
- 持续发布目标距离和导航状态；
- 到达约 3 米并确认停车后报告 `ARRIVED_AND_STOPPED`；
- 在认证状态保持停车；
- 在入侵者状态持续跟踪目标；
- 目标数据失效或导航故障时停车并报告错误。

### 3.4 上位机或人工处置端

上位机只在入侵者处置完成后发布：

```text
HANDLING_COMPLETE
```

状态机验证当前确实处于 `TRACK_INTRUDER` 后，才允许返回巡逻。

## 4. 全局状态机

### 4.1 状态列表

```text
STARTUP
PATROL
CONFIRM_TARGET
APPROACH_TARGET
VERIFY_IDENTITY
TRACK_INTRUDER
```

状态关系：

```text
STARTUP
   |
   | 感知和导航均 READY
   v
PATROL
   |
   | 感知 TARGET_CONFIRMED
   v
CONFIRM_TARGET
   |
   | 导航 TARGET_POSITION_READY
   v
APPROACH_TARGET
   |
   | 导航 ARRIVED_AND_STOPPED
   v
VERIFY_IDENTITY
   |                  |
   | AUTHORIZED       | UNAUTHORIZED
   v                  v
PATROL          TRACK_INTRUDER
                       |
                       | HANDLING_COMPLETE
                       v
                     PATROL
```

### 4.2 状态转换表

| 当前状态 | 事件来源 | 事件 | 下一状态 |
|---|---|---|---|
| `STARTUP` | 感知和导航 | 两边均 `READY` | `PATROL` |
| `PATROL` | 感知 | `TARGET_CONFIRMED` | `CONFIRM_TARGET` |
| `CONFIRM_TARGET` | 导航 | `TARGET_POSITION_READY` | `APPROACH_TARGET` |
| `APPROACH_TARGET` | 导航 | `ARRIVED_AND_STOPPED` | `VERIFY_IDENTITY` |
| `VERIFY_IDENTITY` | 感知 | `AUTHORIZED` | `PATROL` |
| `VERIFY_IDENTITY` | 感知 | `UNAUTHORIZED` | `TRACK_INTRUDER` |
| `TRACK_INTRUDER` | 上位机 | `HANDLING_COMPLETE` | `PATROL` |
| 任一有活动目标的业务状态 | 感知 | `TARGET_REACQUIRED` | 保持原状态，仅解除 `TARGET_LOST` 阻塞 |

`TARGET_LOST` 和 `EXECUTION_ERROR` 不切换业务状态，只将当前状态标记为阻塞，并分别
设置 `BLOCK_TARGET_LOST` 和 `BLOCK_EXECUTION_ERROR`。

## 5. ROS 2 接口总览

| Topic | 消息类型 | 发布方 | 订阅方 | 用途 |
|---|---|---|---|---|
| `/mission/state` | `MissionState` | 状态机 | 感知、导航、上位机 | 当前权威全局状态 |
| `/mission/event` | `MissionEvent` | 感知、导航、上位机 | 状态机 | 离散业务事件 |
| `/perception/selected_target_bbox` | `TargetBoundingBox` | 感知 | 导航 | 当前目标 bbox |
| `/navigation/target_status` | `TargetNavigationStatus` | 导航 | 状态机、感知、上位机 | 距离和导航执行状态 |

所有自定义消息放在独立接口包：

```text
dog_patrol_interfaces
```

建议目录：

```text
dog_patrol_interfaces/
├── msg/
│   ├── MissionState.msg
│   ├── MissionEvent.msg
│   ├── TargetBoundingBox.msg
│   └── TargetNavigationStatus.msg
├── CMakeLists.txt
└── package.xml
```

## 6. 消息定义

### 6.1 MissionState.msg

```text
uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5

uint8 BLOCK_NONE=0
uint8 BLOCK_TARGET_LOST=1
uint8 BLOCK_EXECUTION_ERROR=2

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
bool blocked
uint8 block_cause
string detail
```

字段含义：

- `header.stamp`：状态发布时间；
- `state_seq`：权威状态版本；
- `state`：当前全局状态；
- `target_id`：当前目标，`0` 表示没有活动目标；
- `blocked`：当前任务是否因错误停止推进；
- `block_cause`：阻塞原因；未阻塞时为 `BLOCK_NONE`，目标丢失和技术故障分别为
  `BLOCK_TARGET_LOST`、`BLOCK_EXECUTION_ERROR`；
- `detail`：供日志和界面显示的说明，程序不能解析该字符串决定业务逻辑。

`state_seq` 在以下任一权威状态发生变化时加一：

- `state` 改变；
- `target_id` 改变；
- `blocked` 状态改变。
- `block_cause` 改变。

周期性重复发布同一个状态时，`state_seq` 不变。

推荐 QoS：

```text
reliability: RELIABLE
durability: TRANSIENT_LOCAL
history: KEEP_LAST
depth: 1
```

### 6.2 MissionEvent.msg

```text
uint8 SOURCE_PERCEPTION=0
uint8 SOURCE_NAVIGATION=1
uint8 SOURCE_OPERATOR=2

uint8 READY=0
uint8 TARGET_CONFIRMED=1
uint8 TARGET_POSITION_READY=2
uint8 ARRIVED_AND_STOPPED=3
uint8 AUTHORIZED=4
uint8 UNAUTHORIZED=5
uint8 TARGET_LOST=6
uint8 EXECUTION_ERROR=7
uint8 HANDLING_COMPLETE=8
uint8 TARGET_REACQUIRED=9

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 source
uint8 event
string detail
```

字段含义：

- `header.stamp`：事件产生时间；
- `observed_state_seq`：模块产生事件时看到的状态版本；
- `target_id`：事件对应目标；
- `source`：事件来源；
- `event`：事件类型；
- `detail`：诊断信息。

`TARGET_REACQUIRED` 必须由感知发布，并携带产生该事件时观察到的
`observed_state_seq` 和当前语义 `target_id`。它只在该目标已因 `TARGET_LOST` 被阻塞时
有效。

推荐 QoS：

```text
reliability: RELIABLE
durability: VOLATILE
history: KEEP_LAST
depth: 10
```

### 6.3 TargetBoundingBox.msg

```text
std_msgs/Header header
uint32 target_id

uint32 image_width
uint32 image_height

uint32 x_min
uint32 y_min
uint32 x_max
uint32 y_max

float32 confidence
```

字段要求：

- `header.stamp` 必须是对应图像的采集时间；
- `header.frame_id` 必须是对应相机光学坐标系；
- `target_id` 必须是当前主目标；
- bbox 原点为图像左上角；
- x 向右，y 向下；
- bbox 区间为 `[x_min, x_max)` 和 `[y_min, y_max)`；
- bbox 必须位于图像尺寸范围内。

下列未阻塞状态接受与当前权威 `target_id` 相同且未过期的新鲜 bbox：

| 全局状态 | 是否接受新鲜 bbox | 用途 |
|---|---|---|
| `STARTUP` | 否 | 尚无活动目标 |
| `PATROL` | 否 | 忽略上一次任务的旧 bbox |
| `CONFIRM_TARGET` | 是 | 建立目标位置，机器人保持停车 |
| `APPROACH_TARGET` | 是 | 更新目标位置并安全接近 |
| `VERIFY_IDENTITY` | 是 | 向授权模块提供当前目标 bbox；导航保持停车，不因 bbox 更新重新移动 |
| `TRACK_INTRUDER` | 是 | 更新目标位置并持续跟随 |

`VERIFY_IDENTITY` 中 bbox 只作为授权模块的当前目标输入；导航保持停车，不能因 bbox
更新重新移动。`blocked=true` 时导航不得继续使用 bbox 驱动运动；收到有效
`TARGET_REACQUIRED` 且状态机解除 `TARGET_LOST` 阻塞后，才按上表恢复处理。

推荐 QoS：

```text
reliability: BEST_EFFORT
durability: VOLATILE
history: KEEP_LAST
depth: 5
```

### 6.4 TargetNavigationStatus.msg

```text
uint8 WAITING_TARGET=0
uint8 APPROACHING=1
uint8 ARRIVED=2
uint8 HOLDING=3
uint8 TRACKING=4
uint8 BLOCKED=5

std_msgs/Header header
uint32 target_id
uint8 status
float32 distance_to_target
bool distance_valid
string detail
```

该消息用于：

- 显示目标距离；
- 显示导航当前阶段；
- 调试目标接近过程；
- 让识别模块按需了解机器狗与目标的距离。

该 Topic 不直接触发全局状态变化。真正触发状态变化的是 `/mission/event`。

推荐 QoS：

```text
reliability: RELIABLE
durability: VOLATILE
history: KEEP_LAST
depth: 5
```

## 7. ID 和状态版本

### 7.1 state_seq

`state_seq` 用于拒绝迟到事件。

示例：

```text
state_seq=10, state=PATROL
state_seq=11, state=CONFIRM_TARGET
state_seq=12, state=APPROACH_TARGET
```

事件必须携带模块产生事件时看到的 `state_seq`：

```text
event.observed_state_seq == current_state.state_seq
```

不相等时，状态机直接忽略该事件并输出限频日志。

### 7.2 target_id

`target_id` 由感知模块分配。

要求：

- `0` 表示无目标；
- 感知确认候选后分配非零 ID；
- 同一次目标处置期间保持不变；
- 不因 bbox 短时抖动变化；
- 进程运行期间尽量单调递增，不重复使用旧 ID；
- 返回 `PATROL` 后状态机将活动 `target_id` 清零。

### 7.3 TARGET_CONFIRMED 的特殊规则

在 `PATROL` 中，状态机的 `target_id` 为 `0`。

感知发现目标 `87` 时发送：

```text
source: SOURCE_PERCEPTION
event: TARGET_CONFIRMED
observed_state_seq: 当前 PATROL 的 state_seq
target_id: 87
```

状态机接受后进入 `CONFIRM_TARGET`，并将权威状态中的 `target_id` 设置为 `87`。

后续 bbox、导航事件和认证事件都必须继续使用目标 `87`。

在 `PATROL` 中，首个包含 eligible person 的帧必须立即选择其中面积最大的 eligible
person，分配非零语义 `target_id` 并发送 `TARGET_CONFIRMED`。此规则不等待多帧确认；
eligible 的判定由感知模块负责。

## 8. 各事件的产生条件

### 8.1 READY

感知 Ready 至少表示：

- 相机输入正常；
- 检测模型加载完成；
- 目标跟踪器可用；
- 身份认证接口可用。

导航 Ready 至少表示：

- 定位结果有效；
- `map -> base_footprint` TF 可用；
- 地图已加载；
- planner 和 controller 已激活；
- 巡逻 waypoint 已加载；
- `/NAV_CMD` 控制链路可用。

两个模块在进入新的 `STARTUP state_seq` 后分别发送一次 `READY`。

状态机只有在同一个 `STARTUP state_seq` 中同时收到感知和导航 Ready，才进入 `PATROL`。

### 8.2 TARGET_CONFIRMED

该事件只能由感知在 `PATROL` 中发送。

当首个包含 eligible person 的帧到达时，感知立即选择该帧面积最大的 eligible person，
分配稳定的语义 `target_id` 并发送事件；不等待多帧确认。感知仍须保证当前状态为
`PATROL`，并按自身规则决定 person 是否 eligible。

同一 `state_seq + target_id` 逻辑上只产生一次目标确认事件。

### 8.3 TARGET_POSITION_READY

该事件只能由导航在 `CONFIRM_TARGET` 中发送。

导航端建议按以下步骤建立目标位置：

1. 根据 bbox 图像时间戳匹配雷达数据；
2. 使用相机与雷达标定将雷达点投影到图像；
3. 选择 bbox 内的候选雷达点；
4. 通过聚类、中位数或深度连续性排除背景点；
5. 将目标位置转换到 `map` 坐标系；
6. 连续多帧验证位置稳定性。

初始建议判定条件：

```text
bbox_age <= 0.3 s
至少连续 3 帧目标位置有效
相邻目标位置跳变量 <= 0.5 m
target_id 与当前状态一致
```

事件一旦发送，导航继续维护目标位置，但不重复申请同一次状态转换。

### 8.4 ARRIVED_AND_STOPPED

该事件只能由导航在 `APPROACH_TARGET` 中发送。

必须同时满足：

```text
目标距离满足 3 米要求
目标距离仍然有效
bbox 和目标位置没有超时
机器人线速度低于停车阈值
机器人角速度低于停车阈值
停车条件持续一段确认时间
```

初始建议：

```text
distance_to_target <= 3.0 m
abs(linear_speed) < 0.05 m/s
abs(angular_speed) < 0.10 rad/s
稳定持续时间 >= 0.5 s
```

距离阈值必须结合机器狗前缘、雷达安装位置和安全距离实车标定。

### 8.5 AUTHORIZED

该事件只能由感知在 `VERIFY_IDENTITY` 中发送。

它表示独立授权模块的认证流程已经结束并确认人员获得授权；感知接入适配器只转发该
结果，不能自行作出授权决定。

状态机收到后：

- 进入 `PATROL`；
- 清空 `target_id`；
- 清除阻塞状态；
- 导航恢复巡逻；
- 感知停止当前目标的专用跟踪和认证。

### 8.6 UNAUTHORIZED

该事件只能由感知在 `VERIFY_IDENTITY` 中发送。

它表示独立授权模块的认证流程正常执行完毕，但人员未通过认证；感知接入适配器只转发
该结果，不能自行作出授权决定。

状态机收到后：

- 保留当前 `target_id`；
- 进入 `TRACK_INTRUDER`；
- 感知继续发布同一目标 bbox；
- 导航进入持续目标跟随模式。

摄像头故障、模型崩溃和目标丢失不能使用 `UNAUTHORIZED`，必须使用错误事件。

### 8.7 TARGET_LOST

感知或导航都可以发送该事件。

感知侧含义：

- 当前目标跟踪超时；
- 无法继续产生可信 bbox。

导航侧含义：

- bbox 持续过期；
- 目标位置持续失效；
- 无法确认当前跟踪目标的位置。

模块发送事件前，导航必须停止继续使用旧目标运动。

状态机保持当前业务状态并设置：

```text
blocked=true
block_cause=BLOCK_TARGET_LOST
detail="target lost: ..."
```

同一语义目标重新可见且可信 bbox 恢复时，感知以当前 `state_seq` 和相同 `target_id`
发送 `TARGET_REACQUIRED`。状态机只在当前阻塞原因为 `BLOCK_TARGET_LOST` 时接受它：
保留业务状态和活动目标，清除阻塞并递增 `state_seq`。错误目标、旧序号、重复或在目标
丢失前到达的事件都被拒绝。

### 8.8 EXECUTION_ERROR

用于报告技术故障，例如：

- 相机或雷达输入中断；
- TF 长时间不可用；
- 目标融合算法异常；
- planner 或 controller 不可用；
- 认证服务调用失败。

状态机不自动改变业务状态，只设置 `blocked=true` 和
`block_cause=BLOCK_EXECUTION_ERROR`。`TARGET_REACQUIRED` 不能清除这种阻塞。

### 8.9 HANDLING_COMPLETE

该事件只能由上位机或人工处置端发送。

要求：

- 当前状态为 `TRACK_INTRUDER`；
- `observed_state_seq` 匹配；
- `target_id` 匹配；
- 人工处置流程已经完成。

状态机接受后返回 `PATROL` 并清空当前目标。

## 9. 每个状态下的模块行为

### 9.1 STARTUP

感知：

- 初始化检测、跟踪和认证能力；
- 完成后发送感知 `READY`；
- 不报告可疑目标。

导航：

- 初始化定位、地图、规划和控制；
- 保持机器人停车；
- 完成后发送导航 `READY`。

状态机：

- 分别记录两个模块的 Ready；
- 两边都 Ready 后进入 `PATROL`。

### 9.2 PATROL

感知：

- 持续检测可疑人员；
- 在首个包含 eligible person 的帧中立即选择面积最大的目标；
- 分配语义 `target_id`；
- 发送 `TARGET_CONFIRMED`。

导航：

- 执行 waypoint 巡逻；
- 不处理旧目标 bbox；
- 发布正常巡逻状态。

### 9.3 CONFIRM_TARGET

感知：

- 开始持续发布当前目标 bbox；
- 保持相同 `target_id`。

导航：

- 取消或暂停巡逻目标；
- 保持机器人停车；
- 接收 bbox 并建立目标地图位置；
- 位置稳定后发送 `TARGET_POSITION_READY`。

在状态机进入 `APPROACH_TARGET` 前，导航不得向目标移动。

### 9.4 APPROACH_TARGET

感知：

- 持续跟踪同一目标；
- 持续发布 bbox。

导航：

- 根据更新后的目标位置靠近目标；
- 规划目标时保留约 3 米距离；
- 持续发布 `/navigation/target_status`；
- 到达并停车后发送 `ARRIVED_AND_STOPPED`；
- 发送事件后继续保持停车。

### 9.5 VERIFY_IDENTITY

感知：

- 看到新的 `state_seq` 后向授权模块启动一次请求；
- 持续向授权模块提供同一目标的新鲜 bbox；
- 提供所需的人脸、提示和口令等感知上下文；
- 只转发授权模块给出的 `AUTHORIZED` 或 `UNAUTHORIZED`。

导航：

- 取消目标接近动作；
- 持续输出停车命令；
- 不因 bbox 变化重新开始运动；
- 发布 `HOLDING` 状态。

### 9.6 TRACK_INTRUDER

感知：

- 停止身份认证流程；
- 继续跟踪同一目标；
- 持续发布 bbox。

导航：

- 根据 bbox 和雷达更新目标位置；
- 持续跟随同一目标；
- 保持安全跟随距离；
- 目标数据失效时立即停车并报告 `TARGET_LOST`。

状态机：

- 保留当前 `target_id`；
- 等待上位机发送 `HANDLING_COMPLETE`。

### 9.7 返回 PATROL

感知：

- 停止当前目标的专用跟踪；
- 清理认证上下文；
- 恢复常驻检测；
- 不再发布旧目标 bbox。

导航：

- 取消目标接近或跟随；
- 清除目标位置缓存；
- 清除距离到达锁存；
- 恢复暂停前的巡逻进度。

状态机：

- 将 `target_id` 设置为 `0`；
- 将 `blocked` 设置为 `false`；
- 发布新的 `PATROL state_seq`。

## 10. 完整业务流程

### 10.1 系统启动

状态机发布 `STARTUP`。感知和导航分别完成初始化并发送 `READY`。

状态机确认两边均就绪后进入 `PATROL`。

### 10.2 巡逻并检测

导航按照 waypoint 巡逻，感知同时检测可疑人员。

首个包含 eligible person 的帧到达时，感知立即选择其中面积最大的目标，分配
`target_id` 并发送 `TARGET_CONFIRMED`。

### 10.3 建立目标位置

状态机进入 `CONFIRM_TARGET`。

感知持续发布 bbox。导航暂停巡逻，通过 bbox 与雷达融合建立目标位置。

目标位置稳定后，导航发送 `TARGET_POSITION_READY`。

### 10.4 接近目标

状态机进入 `APPROACH_TARGET`。

导航持续更新目标位置并向目标靠近，同时发布距离。

距离满足约 3 米要求且机器人已经停车后，导航发送 `ARRIVED_AND_STOPPED`。

### 10.5 身份认证

状态机进入 `VERIFY_IDENTITY`。

导航保持停车，感知在内部执行完整认证。

授权模块确认成功后，感知接入适配器发送 `AUTHORIZED`，状态机返回 `PATROL`。

授权模块确认未通过后，感知接入适配器发送 `UNAUTHORIZED`，状态机进入
`TRACK_INTRUDER`。

### 10.6 入侵者跟踪

感知继续发布同一目标 bbox，导航持续跟随。

处置完成后，上位机发送 `HANDLING_COMPLETE`，状态机清理目标并返回 `PATROL`。

## 11. 时序、去重和过期处理

### 11.1 状态进入动作

模块只在 `state_seq` 变化时执行一次状态进入动作。

重复收到相同状态时：

- 不重复启动认证；
- 不重复清空目标；
- 不重复取消巡逻；
- 不重复发送状态转换事件。

### 11.2 事件合法性检查

状态机收到事件后依次检查：

1. `source` 是否允许发布该事件；
2. 当前状态是否允许该事件；
3. `observed_state_seq` 是否等于当前 `state_seq`；
4. `target_id` 是否匹配；
5. 相同事件是否已经处理。

`TARGET_CONFIRMED` 允许事件中的非零目标 ID 替换 `PATROL` 中的 `0`。

Ready 事件在 `STARTUP` 中使用 `target_id=0`。

### 11.3 事件幂等键

状态机使用以下组合识别重复事件：

```text
source + event + observed_state_seq + target_id
```

同一个组合最多引起一次状态转换。

### 11.4 bbox 过期

导航根据 bbox 的 `header.stamp` 计算年龄。

过期 bbox：

- 不用于更新目标位置；
- 不用于继续目标跟随；
- 不触发 `TARGET_POSITION_READY`；
- 不触发 `ARRIVED_AND_STOPPED`。

初始最大允许年龄：

```text
0.3 s
```

具体数值需要根据相机帧率、检测延迟和网络延迟实测。

### 11.5 跨 Topic 顺序

ROS 2 不保证 `/mission/state` 和 bbox 在不同 Topic 上严格按发布时间顺序到达。

导航处理 bbox 时必须同时满足：

- 当前状态允许使用 bbox；
- bbox 的 `target_id` 等于当前权威目标；
- bbox 没有过期。

否则直接丢弃，等待后续连续 bbox。

## 12. 状态机处理逻辑

状态机可以使用显式转换表：

```python
transitions = {
    (PATROL, SOURCE_PERCEPTION, TARGET_CONFIRMED): CONFIRM_TARGET,
    (CONFIRM_TARGET, SOURCE_NAVIGATION, TARGET_POSITION_READY): APPROACH_TARGET,
    (APPROACH_TARGET, SOURCE_NAVIGATION, ARRIVED_AND_STOPPED): VERIFY_IDENTITY,
    (VERIFY_IDENTITY, SOURCE_PERCEPTION, AUTHORIZED): PATROL,
    (VERIFY_IDENTITY, SOURCE_PERCEPTION, UNAUTHORIZED): TRACK_INTRUDER,
    (TRACK_INTRUDER, SOURCE_OPERATOR, HANDLING_COMPLETE): PATROL,
}
```

Ready 使用单独的两个布尔标志：

```text
perception_ready
navigation_ready
```

错误事件使用单独处理逻辑：

```text
TARGET_LOST 或 EXECUTION_ERROR
    -> 保持 state
    -> blocked=true
    -> state_seq 加一
    -> 发布新的 MissionState
```

状态转换必须在同一个回调临界区完成：

1. 验证事件；
2. 更新状态；
3. 更新目标 ID；
4. 增加 `state_seq`；
5. 记录事件已处理；
6. 发布新状态。

## 13. 联调验收条件

### 13.1 STARTUP

- 感知和导航能够分别发送 `READY`；
- 只有两边都 Ready 后才进入 `PATROL`；
- 重复 Ready 不重复转换；
- 启动期间机器人保持停车。

### 13.2 PATROL

- 导航正常巡逻；
- 感知同时检测；
- 首个包含 eligible person 的帧立即选择面积最大的主目标并触发状态变化；
- 感知只确认一个主目标，不设置多帧确认延迟；
- 状态机保存正确 `target_id`。

### 13.3 CONFIRM_TARGET

- 导航暂停巡逻并保持停车；
- 感知 bbox 的目标 ID 正确；
- 导航拒绝过期 bbox 和错误目标；
- 目标位置稳定后只报告一次 Ready。

### 13.4 APPROACH_TARGET

- 感知持续发布同一目标 bbox；
- 导航能够持续发布有效距离；
- 未达到 3 米条件时不进入认证；
- 机器人真正停车后才发送到达事件。

### 13.5 VERIFY_IDENTITY

- 导航始终保持停车；
- 感知只启动一次完整认证；
- 认证内部步骤不出现在全局状态中；
- 技术错误不会被当成未授权。

### 13.6 TRACK_INTRUDER

- 感知持续发布同一目标 bbox；
- 导航自动进入持续跟踪；
- 不需要额外“开始跟踪”命令；
- 目标丢失后导航立即停车；
- 只有上位机确认处置完成后才能返回巡逻。

### 13.7 返回巡逻

- 状态机清空 `target_id`；
- 感知停止旧目标 bbox；
- 导航清理目标缓存；
- 导航恢复巡逻；
- 旧状态版本的事件不会再次生效。

## 14. 联调参数

| 参数 | 负责方 | 初始建议 |
|---|---|---|
| 首帧目标选择规则 | 感知 | 首个含 eligible person 的帧中面积最大的目标立即确认 |
| 目标检测置信度 | 感知 | 根据模型测试 |
| bbox 发布频率 | 感知 | 不低于 10 Hz |
| bbox 最大允许年龄 | 导航 | 0.3 s |
| 目标位置稳定帧数 | 导航 | 3 帧 |
| 目标位置最大跳变 | 导航 | 0.5 m |
| 接近停止距离 | 导航 | 3.0 m |
| 停车线速度阈值 | 导航 | 0.05 m/s |
| 停车角速度阈值 | 导航 | 0.10 rad/s |
| 停车确认时间 | 导航 | 0.5 s |
| bbox/目标丢失超时 | 双方 | 0.5～1.0 s |
| 巡逻恢复策略 | 导航 | 恢复暂停前 waypoint |

这些参数必须在实车联调后确定，不能只根据仿真结果固定。

## 15. 最终接口结论

首版系统只需要四个跨模块消息接口：

```text
MissionState
MissionEvent
TargetBoundingBox
TargetNavigationStatus
```

核心流程：

```text
感知和导航 READY
    -> 状态机开始 PATROL

感知确认目标
    -> TARGET_CONFIRMED
    -> 状态机进入 CONFIRM_TARGET

感知持续发布 bbox
    -> 导航融合雷达得到目标位置
    -> TARGET_POSITION_READY
    -> 状态机进入 APPROACH_TARGET

导航接近并停车
    -> ARRIVED_AND_STOPPED
    -> 状态机进入 VERIFY_IDENTITY

认证成功
    -> AUTHORIZED
    -> 返回 PATROL

认证失败
    -> UNAUTHORIZED
    -> 进入 TRACK_INTRUDER

处置完成
    -> HANDLING_COMPLETE
    -> 返回 PATROL
```

在该设计中：

- 全局状态只有状态机可以修改；
- 感知、导航和上位机共用一个事件消息；
- 高频 bbox 不经过状态机；
- 连续距离不直接触发状态变化；
- `state_seq` 负责过滤旧事件；
- `target_id` 负责关联同一目标；
- 不使用 UUID 和重复状态字段；
- 不暴露身份认证内部步骤；
- 首版不实现自动 fallback。
