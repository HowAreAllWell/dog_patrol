# 巡逻系统总控、导航与感知实现说明

本文是当前工作区的完整实现说明，面向需要理解、联调和维护整个系统的开发者。它以总状态机为主线，说明总控如何发布任务状态，导航和感知如何响应，以及三方如何通过事件完成正常流程和异常恢复。

本文不是只描述感知模块的设计草案，也不是消息字段的孤立列表。阅读顺序固定为：

```text
总状态机决定当前任务阶段
  -> 导航按公共状态执行定位、路径和运动策略
  -> 感知按公共状态执行检测、跟踪和认证
  -> 导航/感知通过事件向总控报告结果
  -> 总控执行下一次状态转移
```

## 1. 系统职责和控制边界

系统由三个业务模块组成：

| 模块 | 唯一职责 | 不负责的事情 |
|---|---|---|
| `mission_supervisor` 总状态机 | 保存唯一公共任务状态，校验事件，执行状态转移和恢复收口 | 不读取图像、点云，不计算目标位置，不发布速度 |
| 导航协调器 | 处理 bbox、雷达、TF 和标定，生成目标地图位置、目标路径、停车和巡逻恢复策略 | 不自行修改公共任务状态，不实现身份认证 |
| 感知模块 | 图像检测、MOT、语义目标选择、bbox 发布、认证流程和目标丢失判断 | 不计算地图坐标，不生成导航路径，不决定恢复巡逻 |

底层导航链仍由 FAST-LIVO-DOG、定位、Nav2、waypoint、Pure Pursuit、RL/PRIEST 和 DWB 适配器组成。导航协调器是这些链路之上的任务适配层，不直接替换底盘控制器。

### 1.1 唯一控制关系

```text
mission_supervisor
  -- /mission/state --> navigation_mission_coordinator
  -- /mission/state --> perception tracking / face / authorization

perception
  -- /perception/selected_target_bbox --> navigation_mission_coordinator
  -- /mission/event ------------------> mission_supervisor

navigation_mission_coordinator
  -- /mission/event --> mission_supervisor

global_path_seq_publisher
  -- /waypoint_global_path --> navigation_path_mux

navigation_mission_coordinator
  -- /mission_global_path --> navigation_path_mux

navigation_path_mux
  -- /global_path --> Pure Pursuit / RL / DWB 链
```

只有总状态机可以改变公共 `MissionState`。感知和导航只能发布事件，不能直接把系统切换到另一个公共状态。导航内部可以有“接近中”“保持停止”等执行策略名称，但不能形成一套和总控并行的业务状态机。

`/global_path` 只允许由 `dog_patrol_navigation/navigation_path_mux` 发布。waypoint 发布器和导航协调器分别发布到
`/waypoint_global_path`、`/mission_global_path`，不能直接写下游共同消费的 `/global_path`。
mux 根据最新的 `MissionState` 选择路径来源：`PATROL`、`CONFIRM_TARGET` 使用 waypoint
路径；`APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER`、`RECOVER_PATROL` 使用
任务路径；`STARTUP` 暂不选择有效路径并输出空路径。

## 2. 公共任务状态机

### 2.1 状态含义

```text
STARTUP          等待感知和导航就绪
PATROL           执行普通 waypoint 巡检，同时感知持续检测
CONFIRM_TARGET   已发现候选目标，导航确认目标空间位置
APPROACH_TARGET  前往目标前约 3 m 的停留位置
VERIFY_IDENTITY  已到达并停车，感知执行内部认证
TRACK_INTRUDER   核验未通过后持续跟踪目标
RECOVER_PATROL   目标任务结束或异常后恢复被打断的巡检
```

### 2.2 主状态图

```text
STARTUP
   |
   | 感知 READY + 导航 READY
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
   |                         |
   | AUTHORIZED              | UNAUTHORIZED（核验超时语义）
   v                         v
RECOVER_PATROL           TRACK_INTRUDER
   ^                         |
   |                         | 目标仍有效：继续跟踪
   |                         |
   |                         | TARGET_LOST / EXECUTION_ERROR
   |                         v
   +------------------- RECOVER_PATROL
                             |
                             | PATROL_RECOVERY_COMPLETE
                             | 或恢复超时
                             v
                           PATROL
```

`TARGET_LOST` 和 `EXECUTION_ERROR` 在 `CONFIRM_TARGET`、`APPROACH_TARGET`、
`VERIFY_IDENTITY` 和 `TRACK_INTRUDER` 都按目标任务结束处理，统一进入
`RECOVER_PATROL`。`CONFIRM_TARGET` 的 `confirm_target_timeout` 也进入恢复；它表示
导航一直没有得到可靠空间位置，不等于感知已经确认目标丢失。

### 2.3 状态转换表

| 当前状态 | 触发条件 | 下一状态 | 处理结果 |
|---|---|---|---|
| `STARTUP` | 感知和导航都发布 `READY` | `PATROL` | 目标 ID 为 0，开始普通巡检 |
| `PATROL` | 感知发布 `TARGET_CONFIRMED` | `CONFIRM_TARGET` | 保存目标 ID，开始空间位置确认 |
| `CONFIRM_TARGET` | 导航发布 `TARGET_POSITION_READY` | `APPROACH_TARGET` | 导航暂停巡检并接管目标运动 |
| `CONFIRM_TARGET` | `confirm_target_timeout` 到期 | `RECOVER_PATROL` | 放弃本次目标任务，恢复巡检 |
| `APPROACH_TARGET` | 导航发布 `ARRIVED_AND_STOPPED` | `VERIFY_IDENTITY` | 清除移动目标路径，保持停止 |
| 目标任务状态 | 感知发布 `TARGET_LOST` | `RECOVER_PATROL` | 结束目标任务，导航恢复巡检 |
| 目标任务状态 | 导航或感知发布 `EXECUTION_ERROR` | `RECOVER_PATROL` | 执行统一异常恢复 |
| `VERIFY_IDENTITY` | 感知发布 `AUTHORIZED` | `RECOVER_PATROL` | 认证成功，恢复巡检 |
| `VERIFY_IDENTITY` | 感知发布 `UNAUTHORIZED` | `TRACK_INTRUDER` | 核验失败按超时处理，继续跟踪 |
| `RECOVER_PATROL` | 导航发布 `PATROL_RECOVERY_COMPLETE` | `PATROL` | 清空目标，开始新的巡检周期 |
| `RECOVER_PATROL` | 恢复 watchdog 超时 | `PATROL` | 兜底释放恢复等待，清空目标 |

### 2.4 总状态机的实现规则

`mission_supervisor` 负责：

- 周期发布 `/mission/state`；
- 保存 `state`、`target_id` 和 `state_seq`；
- 在 `STARTUP` 汇总感知和导航的 `READY`；
- 接收并验证 `/mission/event`；
- 对目标确认、位置确认、到达、认证、目标丢失和技术故障执行转移；
- 将目标任务结束统一收口到 `RECOVER_PATROL`；
- 恢复完成或恢复超时后切回新的 `PATROL`。

状态机不根据 `detail` 字符串决定业务逻辑，不读取 bbox，不判断雷达点云数量，也不直接向底盘发送速度。

### 2.5 就绪条件

总控不会因为单个模块启动就进入 `PATROL`。两个模块必须针对当前 `STARTUP state_seq`
分别发布合法的 `READY`：

- 感知至少确认图像输入、检测模型、MOT tracker、目标输出和认证能力可用；
- 导航至少确认定位、地图、`map -> base_footprint` TF、planner action 和原有控制链可用；
- `READY` 使用 `target_id=0`；
- 总控同时收到感知和导航的 `READY` 后，才发布 `PATROL`；
- 旧启动序号的 `READY` 不得让新的启动会话进入巡检。

感知中的 detection/tracking、face、voice 等 capability 可以先分别发布能力状态，再由感知
就绪协调器汇总为感知侧 `READY`。总控不解析这些内部能力的细节。

## 3. 状态序号、目标 ID 和事件校验

### 3.1 `state_seq`

`state_seq` 是公共状态版本。状态改变或活动目标改变时递增；同一个状态周期重复发布时不递增。

事件必须携带模块产生事件时观察到的序号：

```text
event.observed_state_seq == 当前 mission state_seq
```

序号不一致的事件是迟到事件，直接忽略。这样可以防止旧目标的 bbox、planner action 结果或认证结果污染新的巡检任务。

### 3.2 `target_id`

`target_id` 由感知侧分配，`0` 表示没有活动目标。

- 感知在 `PATROL` 发现可信目标时分配非零 ID；
- 同一次目标处置期间保持不变；
- raw tracker ID 改变时，semantic target ID 不应随之改变；
- bbox、目标相关事件和导航状态必须使用当前活动目标 ID；
- 返回 `PATROL` 后总控清空目标 ID；
- 新的 `PATROL state_seq` 允许感知重新评估画面中的目标。

### 3.3 事件来源和幂等

感知可以发布：`READY`、`TARGET_CONFIRMED`、`TARGET_LOST`、`AUTHORIZED`、`UNAUTHORIZED`、`EXECUTION_ERROR`。

导航可以发布：`READY`、`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED`、`EXECUTION_ERROR`、`PATROL_RECOVERY_COMPLETE`。

状态机处理事件时依次校验：来源、当前状态、`observed_state_seq`、目标 ID 和重复事件键。重复键为：

```text
source + event + observed_state_seq + target_id
```

当前公共协议不使用 `blocked`、`block_cause`、`BLOCKED`、`TARGET_REACQUIRED` 或 `HANDLING_COMPLETE` 作为业务闭环条件。

## 4. 导航协调器：按总状态执行策略

### 4.1 导航的输入和输出

| 方向 | Topic | 类型/内容 | 作用 |
|---|---|---|---|
| 输入 | `/mission/state` | `MissionState` | 唯一公共任务状态、序号和目标 ID |
| 输入 | `/perception/selected_target_bbox` | `TargetBoundingBox` | 感知提供的当前目标框 |
| 输入 | `/livox/lidar` | `sensor_msgs/PointCloud2` | 目标 ROI 的雷达点云 |
| 输入 | `/odom` | `nav_msgs/Odometry` | 运动速度和停止条件 |
| 输出 | `/mission/event` | `MissionEvent` | READY、位置就绪、到达、故障和恢复完成 |
| 输出 | `/navigation/target_point` | `geometry_msgs/PointStamped` | map 系目标位置，供显示和调试 |
| 输出 | `/navigation/target_goal` | `geometry_msgs/PoseStamped` | 本次目标 planner 目标 |
| 输出 | `/mission_global_path` | `nav_msgs/Path` | 协调器的目标处置和恢复私有路径，供 mux 选择 |
| 输出 | `/waypoint_sequence/pause` | `std_msgs/Empty` | 暂停原巡检 waypoint |
| 输出 | `/waypoint_sequence/resume` | `std_msgs/Empty` | 恢复原巡检 waypoint |
| 输出 | `/navigation/target_status` | `TargetNavigationStatus` | 导航内部观测状态和距离 |

waypoint 发布器输出 `/waypoint_global_path`，`navigation_path_mux` 将当前状态允许的
私有路径唯一转发为 `/global_path`。因此普通巡检和目标任务不会再通过多个发布者竞争同一
个控制 topic。

协调器不发布 `/NAV_CMD`。底盘命令仍由原有 move/DWB 适配链发布。

### 4.2 状态到导航策略的映射

| 公共状态 | 导航行为 | 是否继续巡检 | 是否融合 bbox | 是否生成目标路径 |
|---|---|---:|---:|---:|
| `STARTUP` | 等待定位、TF、planner 和控制链，保持停止 | 否 | 否 | 否 |
| `PATROL` | 执行原 waypoint/Nav2 巡检，清理目标缓存 | 是 | 否 | 否 |
| `CONFIRM_TARGET` | 后台通过 bbox+雷达确认目标位置 | 是 | 是 | 否 |
| `APPROACH_TARGET` | 暂停巡检，规划到目标附近并在约 3 m 处停车 | 否 | 是 | 是 |
| `VERIFY_IDENTITY` | 取消移动目标路径，持续停车 | 否 | 可继续 | 否 |
| `TRACK_INTRUDER` | 按最新目标位置持续更新跟踪路径 | 否 | 是 | 是 |
| `RECOVER_PATROL` | 取消目标任务，返回巡检断点并恢复 waypoint | 返回期间暂停 | 否 | 返回路径 |

### 4.2.1 路径来源切换

路径来源切换由 `navigation_path_mux` 完成，不由 waypoint 发布器或导航协调器互相覆盖：

| 公共状态 | mux 选择 | 说明 |
|---|---|---|
| `STARTUP` | 无 | 输出空路径，等待总控完成 READY |
| `PATROL`、`CONFIRM_TARGET` | `/waypoint_global_path` | 普通巡检继续使用当前 waypoint 单段路径 |
| `APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER`、`RECOVER_PATROL` | `/mission_global_path` | 目标接近、停车、跟踪和恢复路径由协调器控制 |

进入目标状态或恢复状态时，mux 先向 `/global_path` 发布空路径，再等待当前来源的新路径；
返回 `PATROL` 时，mux 不重放任务期间缓存的旧 waypoint 路径，只接受切换后新到达的
waypoint 私有路径。跨 topic 的到达先后不再决定哪一个发布者拥有路径。

### 4.3 目标位置确认

`CONFIRM_TARGET` 的含义是“导航正在确认目标空间位置”，不是感知的第二次视觉确认。导航按照以下链路处理：

```text
原图 bbox（1280x1024，camera_link）
  + /livox/lidar（livox_frame）
  + 相机-雷达标定
  + 同时间窗口的 TF
  -> 投影到 bbox ROI 的点
  -> 距离过滤和深度聚类
  -> 稳定性和跳变检查
  -> livox_frame -> base_link -> map
  -> 目标地图位置和目标距离
```

位置无效的典型原因包括：bbox 过期、bbox 和点云时间差过大、ROI 内点数不足、点云簇不稳定、目标超过有效雷达范围、TF 缺失或外参方向错误。位置没有稳定前：

- 不把像素坐标当成地图目标；
- 不把错误位置发送给 planner；
- 不伪造感知侧 `TARGET_LOST`；
- 在 `CONFIRM_TARGET` 中继续原巡检并后台重试；
- 由总控的确认超时负责最终收口。

稳定位置确认后，导航只发布一次 `TARGET_POSITION_READY`。

### 4.4 接近、停车和跟踪

`APPROACH_TARGET` 和 `TRACK_INTRUDER` 都把人视为可以移动的目标。导航按最新稳定位置更新 planner 目标，而不是永久使用第一次测量的终点。当前实现要区分两个距离：

- `planning_goal_distance=1.0 m`：用于生成能够接近目标的 planner 路径，目标点位于目标附近；
- `approach_distance/tracking_distance=3.0 m`：用于实时距离判断，接近到约 3 m 时清空目标路径并停车。

因此 `/navigation/target_goal` 不等于“允许机器狗实际走到的人身边位置”。实际停车由实时目标距离、速度和保持时间共同决定，不能只看 planner 目标点。

- 默认约每 `0.5 s` 重新规划；
- 目标位置变化超过约 `0.25 m` 时允许提前规划；
- 同时只保留一个有效 planner action；只有 action 已确认可用并发起请求时才记录 planner
  节流时间，action 未就绪时不伪造请求时间；
- 只有 action 结果仍匹配当前 `state_seq`、`target_id` 和请求代数时才替换路径；迟到结果
  只使当前请求失效，不清理或修改新的请求上下文；
- bbox 过期时停止使用旧目标路径，必要时发布空路径；
- 目标仍有可信 bbox 时持续跟踪，不等待操作员事件。

到达条件不能只看目标几何距离。当前判定原则是：目标距离进入约 `3.0 + 0.10 m` 容差，线速度不超过 `0.05 m/s`，角速度不超过 `0.10 rad/s`，并持续约 `0.5 s`。满足后发布 `ARRIVED_AND_STOPPED`，并保持停车等待认证。

### 4.5 巡检恢复

导航在首次进入 `APPROACH_TARGET` 时保存巡检中断时的 `map -> base_footprint` 位姿和 waypoint 进度。进入 `RECOVER_PATROL` 后：

1. 取消目标 planner/action；
2. 清空目标路径、目标点、目标距离和到达锁存；
3. 停止消费旧目标 bbox；
4. 优先规划返回保存的巡检中断位姿；
5. 将恢复路径缓存并按固定周期重发；进入中断位姿容差后清空运动路径并保持停止，
   直到稳定停车；
6. 向 waypoint 发送 resume；mux 切回 `PATROL` 后只接受状态切换之后到达的 waypoint 路径；
7. 发布一次 `PATROL_RECOVERY_COMPLETE`。

如果原任务没有 waypoint、恢复路径为空或恢复过程出现技术问题，不能让总控永久等待。恢复 watchdog 到期后总控直接进入 `PATROL`，清空目标，不创建第二层恢复状态。

### 4.6 导航坐标约定

| 坐标系 | 用途 |
|---|---|
| `map` | 全局路径、目标点、standoff goal |
| `base_footprint` | 平面导航和巡检恢复参考 |
| `base_link` | 机器狗主体中心，外参转换终点 |
| `livox_frame` | 雷达点云 frame |
| `camera_link` | bbox 关联的相机 frame |

当前 bbox 按 `1280x1024` 原图坐标解释，雷达有效处理范围默认约 `0.40 m` 到 `15.0 m`。动态 `map -> base_footprint` TF 必须由定位系统提供，不能用静态外参代替。

## 5. 感知内部状态机

### 5.1 感知处理链

```text
相机输入
  -> Detector
  -> MOT tracker
  -> Identity / semantic target
  -> PrimaryTargetManager
  -> MissionFrameTransaction
  -> MissionCoordinator
  -> MissionRosAdapter
  -> bbox / MissionEvent
```

感知内部有三种 ID：

- raw track ID：MOT 对相邻帧目标的短期关联 ID；
- semantic target ID：跨 raw track 变化保持同一业务目标的 ID；
- `MissionState.target_id`：总控当前认可的目标 ID。

raw track ID 可以变化，任务期间的 semantic target ID 和公共 `target_id` 不应因此变化。

### 5.2 感知内部生命周期

```text
IDLE
  -> LOCKED
  -> OCCLUDED / PENDING_RECOVERY
  -> LOCKED        （短时遮挡或跟踪恢复）
  -> LOST          （超过内部丢失阈值）
```

`OCCLUDED`、`PENDING_RECOVERY` 和 `LOST` 只属于感知内部，不直接写入 `/mission/state`。感知进入内部 `LOST` 后，可以在新的 `PATROL state_seq` 中重新选择目标，但不能在旧任务序号中恢复旧任务 bbox。

### 5.3 主目标选择

在 `PATROL` 中，感知从当前有效 identity 中选择业务主目标。选择需要满足检测类别、置信度、面积、轨迹权威性和目标连续性等内部条件。当前帧出现多个合格目标时，按现有主目标选择规则选出一个目标；选定后分配非零 semantic ID。

感知在当前 `PATROL state_seq` 下只对同一个目标发送一次 `TARGET_CONFIRMED`。总控接受后进入 `CONFIRM_TARGET`，感知才开始向导航持续发布该目标 bbox。

### 5.4 `MissionFrameTransaction`

每一帧按以下顺序处理：

1. 使用当前 identity 列表更新主目标管理器；
2. 读取最新 `/mission/state`；
3. 在 `PATROL` 发现可信主目标时发送 `TARGET_CONFIRMED`；
4. 在目标任务状态只输出当前 `target_id` 对应的 bbox；
5. 校验 bbox 边界、尺寸、时间戳和 frame；
6. 如果当前任务目标持续没有可信 bbox，交给任务协调器累计最终丢失时间。

感知不使用旧缓存框冒充当前证据，也不使用其他人的 bbox 替代当前任务目标。

### 5.5 感知输出门禁

| 公共状态 | 是否发布当前目标 bbox | 感知行为 |
|---|---:|---|
| `STARTUP` | 否 | 等待相机、模型、tracker 和授权能力就绪，发送 `READY` |
| `PATROL` | 否 | 检测并选主目标，发送 `TARGET_CONFIRMED` |
| `CONFIRM_TARGET` | 是 | 保持同一目标，给导航提供空间定位证据 |
| `APPROACH_TARGET` | 是 | 持续跟踪接近中的目标 |
| `VERIFY_IDENTITY` | 是 | 给认证模块提供当前目标上下文 |
| `TRACK_INTRUDER` | 是 | 持续跟踪入侵者，支持导航更新路径 |
| `RECOVER_PATROL` | 否 | 清理旧目标和认证上下文，等待新的 `PATROL` |

### 5.6 目标丢失

感知任务级目标丢失同时覆盖：

1. `APPROACH_TARGET` 前往目标过程中丢失；
2. `VERIFY_IDENTITY` 核验过程中丢失；
3. `TRACK_INTRUDER` 持续跟踪过程中丢失。

短时漏检由 tracker 内部处理。当前任务目标连续超过 `target.lost_event_timeout_sec=10.0 s` 没有可信 bbox 时，感知只发布一次 `TARGET_LOST`。事件发送后，同一个 `state_seq` 内不再发布该旧任务 bbox。

如果整个相机或推理线程停止，没有新的 frame transaction 被处理，感知逻辑无法凭空触发基于帧处理的丢失事件；这种情况应由输入/能力监控报告技术故障。

## 6. 认证内部流程

认证只在总状态为 `VERIFY_IDENTITY` 且当前目标有效时运行。内部阶段不进入公共状态机。

```text
INITIAL_FACE
  只做人脸识别，不播放提示
  通过 -> AUTHORIZED
  失败 -> DUAL_FIRST

DUAL_FIRST
  人脸和口令并行核验
  任一通过 -> AUTHORIZED
  两者均失败 -> DUAL_SECOND

DUAL_SECOND
  人脸和口令再次并行核验
  任一通过 -> AUTHORIZED
  两者均失败 -> UNAUTHORIZED
```

最终两轮均失败的业务语义是“核验超时”，但公共事件仍叫 `UNAUTHORIZED`，总控据此进入 `TRACK_INTRUDER`。此时：

- 感知停止认证等待，继续发布同一个目标的 bbox；
- 导航按照最新 bbox 和雷达位置持续跟踪；
- 不需要人工发送完成事件；
- 目标最终丢失后由 `TARGET_LOST` 进入恢复巡检。

认证成功发布 `AUTHORIZED`，总控进入 `RECOVER_PATROL`，导航恢复原巡检。认证技术故障发布 `EXECUTION_ERROR`，不能伪装成 `UNAUTHORIZED`。

## 7. 异常闭环和恢复规则

### 7.1 目标最终丢失

```text
最后一次可信 bbox
  -> 短时漏检：感知内部 tracker 等待恢复
  -> 10 秒内重新出现：继续当前任务
  -> 连续 10 秒没有可信 bbox：发布一次 TARGET_LOST
  -> 总控进入 RECOVER_PATROL
  -> 导航停车、清理目标路径、恢复巡检断点
  -> PATROL_RECOVERY_COMPLETE 或恢复超时
  -> 总控进入新的 PATROL，target_id=0
```

感知不能在 `RECOVER_PATROL` 中继续发布旧目标 bbox，也不能重新发送旧任务的 `TARGET_CONFIRMED`。只有总控重新发布新的 `PATROL state_seq` 后，感知才恢复常驻目标选择。

### 7.2 目标位置确认失败

目标框存在但导航没有足够点云、TF 或时间同步数据时，不代表感知目标丢失。导航在 `CONFIRM_TARGET` 中继续原巡检并重试，记录具体诊断；总控等待 `confirm_target_timeout=5.0 s`。

超时后：

```text
CONFIRM_TARGET
  -> RECOVER_PATROL
  -> 导航恢复断点或恢复 watchdog 到期
  -> PATROL，target_id=0，state_seq 更新
  -> 感知重新评估当前画面
```

这条路径避免总控永久卡在 `CONFIRM_TARGET`，也避免把“目标可见但暂时无法定位”误判为最终目标丢失。

### 7.3 导航或感知技术故障

技术故障包括相机/雷达输入中断、TF 长时间不可用、点云融合异常、planner 或 controller 不可用、认证服务故障等。

- 目标任务期间发生故障：发布 `EXECUTION_ERROR`，总控进入 `RECOVER_PATROL`；
- 导航先停车、取消目标路径并清理缓存；
- 感知清理当前认证或目标上下文；
- `RECOVER_PATROL` 中再次出现错误不创建新的业务状态；
- 恢复成功或恢复超时都最终回到 `PATROL`；
- 没有活动目标时的系统技术故障只作为诊断，不改变正在执行的巡检状态。

### 7.4 恢复后重新检测

回到 `PATROL` 后：

- 总控发布新的 `state_seq`，`target_id=0`；
- 导航清理目标位置、standoff goal、目标路径和到达锁存；
- 感知清理旧 semantic target 的任务上下文和认证上下文；
- 感知恢复常驻检测；
- 当前画面仍有人时，可以在新的巡检状态序号下重新选中并启动新的确认流程。

### 7.5 业务事件和报警分离

`TARGET_LOST` 和 `EXECUTION_ERROR` 是驱动总状态机的业务事件，不等同于 UI 报警。当前实现没有另行增加一套报警事件类型：

- 目标最终丢失时，感知发布一次 `TARGET_LOST`，`detail` 应说明发生阶段和 10 秒超时原因；
- 导航或感知技术故障时发布 `EXECUTION_ERROR`，`detail` 应携带具体技术原因；
- UI、日志或上层记录系统可以根据事件和 `detail` 生成报警；
- 报警显示不能代替状态机事件，也不能靠报警清除目标任务。

后续如果增加独立 `/mission/alarm`，它应作为观测/告警接口增加，不能改变当前任务状态转移规则。

## 8. 消息接口

### 8.1 `MissionState.msg`

```text
uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5
uint8 RECOVER_PATROL=6

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
string detail
```

字段说明：

- `header.stamp`：状态发布时间；
- `state_seq`：公共状态版本；
- `state`：公共任务状态；
- `target_id`：当前活动目标，0 表示无目标；
- `detail`：日志和 UI 说明，不能作为程序业务判断条件。

推荐 QoS：可靠、transient local、keep last、depth 1。

### 8.2 `MissionEvent.msg`

```text
uint8 SOURCE_PERCEPTION=0
uint8 SOURCE_NAVIGATION=1

uint8 READY=0
uint8 TARGET_CONFIRMED=1
uint8 TARGET_POSITION_READY=2
uint8 ARRIVED_AND_STOPPED=3
uint8 AUTHORIZED=4
uint8 UNAUTHORIZED=5
uint8 TARGET_LOST=6
uint8 EXECUTION_ERROR=7
uint8 PATROL_RECOVERY_COMPLETE=8

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 source
uint8 event
string detail
```

`READY` 使用 `target_id=0`。目标相关事件使用当前任务目标 ID。`TARGET_LOST` 由感知发布，`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED` 和 `PATROL_RECOVERY_COMPLETE` 由导航发布。

推荐 QoS：可靠、volatile、keep last、depth 10。

### 8.3 `TargetBoundingBox.msg`

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

约束：

- `header.stamp` 是对应图像采集时间，不是任意发送时间；
- `header.frame_id` 为 `camera_link`；
- 当前图像尺寸为 `1280x1024`；
- 原点在左上角，x 向右，y 向下；
- bbox 使用 `[x_min, x_max)`、`[y_min, y_max)`；
- 坐标必须在图像范围内；
- `target_id` 必须等于总控当前活动目标。

推荐 QoS：best effort、volatile、keep last、depth 5。

### 8.4 `TargetNavigationStatus.msg`

该消息只是导航观测接口，不参与总状态转移，也不承载旧的 blocked 业务字段。

```text
uint8 WAITING_TARGET=0
uint8 APPROACHING=1
uint8 ARRIVED=2
uint8 HOLDING=3
uint8 TRACKING=4

std_msgs/Header header
uint32 target_id
uint8 status
float32 distance_to_target
bool distance_valid
string detail
```

导航默认约 10 Hz 发布。目标暂时不可用时可以发布 `HOLDING` 并在 `detail` 中记录原因；该消息不能直接驱动公共状态切换。

## 9. 时间、参数和执行条件

### 9.1 总控参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `state_publish_rate` | `10.0 Hz` | `/mission/state` 周期发布频率 |
| `confirm_target_timeout` | `5.0 s` | `CONFIRM_TARGET` 最大等待时间 |
| `patrol_recovery_timeout` | `10.0 s` | 恢复巡检 watchdog 时间 |
| `initial_state_seq` | `1` | 启动初始状态序号 |
| `processed_event_limit` | `256` | 已处理事件去重缓存上限 |

### 9.2 感知参数

| 参数/规则 | 当前值 | 含义 |
|---|---:|---|
| `target.lost_threshold_frames` | `100` 帧 | tracker/identity 内部目标生命周期阈值 |
| `target.lost_event_timeout_sec` | `10.0 s` | 任务级最终 `TARGET_LOST` 阈值 |
| `target.handled_ignore_absence_sec` | `30.0 s` | 已处理目标再次被选中的抑制时间，不是公共丢失超时 |
| 当前相机输入 | `10 Hz` | 任务级丢失时间按 source timestamp 计算 |
| 当前 bbox 图像 | `1280x1024` | 导航投影使用原图坐标 |

`lost_threshold_frames` 和 `lost_event_timeout_sec` 是两个不同层级的条件，不能相互替代。短时漏检由 tracker 处理，任务最终丢失由任务协调器上报。

### 9.3 导航融合和运动参数

| 参数/规则 | 当前值 | 含义 |
|---|---:|---|
| bbox 最大允许年龄 | `0.80 s` | 过期后不再更新目标位置或跟踪路径 |
| bbox/点云同步容差 | `0.12 s` | 投影使用的最大时间差 |
| 雷达距离 | `0.40~15.0 m` | ROI 点的有效距离范围 |
| 目标位置稳定样本 | `3` | 发布位置就绪前的稳定样本数 |
| planner 目标距离 | `1.0 m` | 生成接近目标附近的路径，实际停车仍使用 3 m 距离 |
| 目标路径重规划周期 | `0.50 s` | 接近和持续跟踪的默认周期 |
| 目标移动提前重规划 | `0.25 m` | 目标位置变化超过该值时提前规划 |
| 期望停留距离 | `3.0 m` | 接近和跟踪的 standoff 距离 |
| 到达距离容差 | `0.10 m` | 到达停车判定容差 |
| 到达线速度 | `0.05 m/s` | 到达时最大线速度 |
| 到达角速度 | `0.10 rad/s` | 到达时最大角速度 |
| 停车保持时间 | `0.50 s` | 到达条件持续时间 |
| 目标路径过期时间 | `0.90 s` | 超过时间未更新时停止使用目标路径 |
| 连续技术失败时间 | `3.0 s` | TF、融合或 planner 连续失败后报告技术故障 |

实际路径刷新频率还受 planner 计算时间、TF 可用性、点云融合和下游控制链影响；提高协调器 tick rate 不等于 planner 一定能更快返回路径。

## 10. 完整时序示例

### 10.1 正常巡检、认证成功

```text
总控：STARTUP
  -> 感知 READY、导航 READY
总控：PATROL
  -> 感知检测到可信目标
感知：TARGET_CONFIRMED(target=T)
总控：CONFIRM_TARGET(target=T)
  -> 感知持续发布 bbox
  -> 导航融合 bbox + 雷达 + TF
导航：TARGET_POSITION_READY(target=T)
总控：APPROACH_TARGET(target=T)
  -> 导航暂停巡检，持续规划 3 m standoff 路径
导航：ARRIVED_AND_STOPPED(target=T)
总控：VERIFY_IDENTITY(target=T)
  -> 导航停车
  -> 感知执行 INITIAL_FACE / DUAL_FIRST / DUAL_SECOND
感知：AUTHORIZED(target=T)
总控：RECOVER_PATROL(target=T)
  -> 导航返回巡检断点并恢复 waypoint
导航：PATROL_RECOVERY_COMPLETE(target=T)
总控：PATROL(target=0，新 state_seq)
```

### 10.2 认证失败并持续跟踪

```text
VERIFY_IDENTITY
  -> 两轮认证均失败
感知：UNAUTHORIZED(detail=verification_timeout)
总控：TRACK_INTRUDER
  -> 感知持续发布同一 target_id 的 bbox
  -> 导航持续更新目标地图位置和 3 m 跟踪路径
  -> 目标仍在：保持 TRACK_INTRUDER
  -> 目标连续丢失 10 s：TARGET_LOST
总控：RECOVER_PATROL
  -> 导航恢复巡检
  -> PATROL(target=0，新 state_seq)
```

### 10.3 确认位置失败

```text
PATROL
  -> TARGET_CONFIRMED
  -> CONFIRM_TARGET
  -> bbox 仍存在，但雷达点、TF 或时间同步不满足
  -> 导航继续原巡检并后台重试
  -> 5 s 内成功：TARGET_POSITION_READY -> APPROACH_TARGET
  -> 5 s 超时：RECOVER_PATROL -> PATROL
```

### 10.4 三方正常交互时序

下面的时序图从三个模块的角度展示正常目标处置流程。总状态机是控制中心：它接收感知
和导航事件，再把新的公共状态发布给相关模块；感知向导航发送连续 bbox，导航向总控
发送空间位置和到达结果。

```text
感知                    总状态机                    导航
 |                         |                         |
 |-- READY --------------->|                         |
 |                         |<---------------- READY -|
 |                         |                         |
 |-- TARGET_CONFIRMED ---->|                         |
 |                         |                         |
 |<-- MissionState --------|-- CONFIRM_TARGET ------>|
 |                         |                         |
 |-- bbox ------------------------------------------>|
 |                         |                         |
 |                         |<-- TARGET_POSITION_READY|
 |                         |                         |
 |<-- MissionState --------|-- APPROACH_TARGET ----->|
 |                         |                         |
 |-- bbox ------------------------------------------>|
 |                         |                         |
 |                         |<-- ARRIVED_AND_STOPPED -|
 |                         |                         |
 |<-- MissionState --------|-- VERIFY_IDENTITY ----->|
 |                         |                         |
 |<-- 核验请求/结果 -------|                         |
 |-- AUTHORIZED ---------->|                         |
 |   或 UNAUTHORIZED       |                         |
```

图中各条消息的含义如下：

1. 感知和导航分别向总控报告 `READY`，总控收到双方就绪后发布 `PATROL`。
2. 感知在巡检中选出目标，发布 `TARGET_CONFIRMED`；总控保存 `target_id` 并发布
   `CONFIRM_TARGET`。
3. 感知从 `CONFIRM_TARGET` 开始持续向导航发送同一目标的新鲜 bbox，导航用 bbox、
   雷达、TF 和标定计算目标地图位置。
4. 导航得到稳定位置后发送 `TARGET_POSITION_READY`；总控发布 `APPROACH_TARGET`，
   导航才暂停巡检并开始目标路径规划。
5. 感知在接近过程中继续发送 bbox，导航根据最新目标位置更新路径；满足距离和停车
   条件后发送 `ARRIVED_AND_STOPPED`。
6. 总控发布 `VERIFY_IDENTITY`。导航保持停车，感知在内部执行人脸和口令认证。
7. 感知将认证结果转换为公共事件：`AUTHORIZED` 进入恢复巡检，`UNAUTHORIZED`
   进入 `TRACK_INTRUDER`。

`MissionState` 是总控发出的公共状态，不是感知或导航发出的命令。图中的状态箭头表示
状态消息从总控流向模块；bbox 是感知到导航的连续数据；位置、到达和恢复结果是导航到
总控的事件。

## 11. 故障排查和验收

### 11.1 启动后一直 `STARTUP`

检查：

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/event
ros2 topic info /livox/lidar -v
ros2 action info /compute_path_to_pose
ros2 run tf2_ros tf2_echo map base_footprint
```

重点确认感知和导航是否都发布了 `READY`，以及两个事件的 `observed_state_seq` 是否属于当前启动状态。

### 11.2 有目标框但没有目标点

检查：

```bash
ros2 topic echo /perception/selected_target_bbox
ros2 topic echo /livox/lidar --once
ros2 topic echo /navigation/target_point
```

依次核对 bbox 是否为 `camera_link`、尺寸是否为 `1280x1024`、点云是否为 `livox_frame`、时间差是否在 `0.12 s` 内、ROI 是否有足够点、标定外参方向是否正确。

### 11.3 有目标点但没有路径

确认当前公共状态已经是 `APPROACH_TARGET` 或 `TRACK_INTRUDER`，而不是仍处于 `CONFIRM_TARGET`。再检查：

```bash
ros2 topic echo /mission/state
ros2 topic echo /navigation/target_goal
ros2 topic echo /global_path
ros2 action info /compute_path_to_pose
```

目标点有效但 planner 计算时间较长时，路径刷新频率会低于目标重规划周期，这是计算链路限制，不是状态机没有运行。

### 11.4 返回巡检后没有局部路径

确认总控已经进入新的 `PATROL`，导航已经发送 waypoint resume，且原 waypoint/Pure Pursuit/RL 链重新收到全局路径。恢复超时后总控仍会进入 `PATROL`，但下游巡检链仍需要自行重新生成有效巡检路径。

### 11.5 最低验收场景

1. 两边 READY 后只进入一次 `PATROL`；
2. 巡检发现人后只产生一次 `TARGET_CONFIRMED`；
3. bbox、点云、TF 正常时产生一次 `TARGET_POSITION_READY`；
4. 导航到约 3 m 且实际停车后产生一次 `ARRIVED_AND_STOPPED`；
5. 认证成功后经过 `RECOVER_PATROL` 回到新的 `PATROL`；
6. 两轮认证失败后进入 `TRACK_INTRUDER`，不依赖人工完成事件；
7. 接近、认证、持续跟踪三个阶段丢失目标超过 10 s 后都能恢复巡检；
8. 目标框存在但空间定位失败时不会伪造 `TARGET_LOST`，确认超时不会永久卡住；
9. 旧 `state_seq` 的事件、bbox、认证结果和 planner action 不影响新任务；
10. 恢复过程再次出错或恢复路径为空时，最终仍能回到 `PATROL`。

## 12. 设计结论

整个系统的业务主线只有一条：

```text
总控发布公共状态
  -> 导航和感知按状态执行
  -> 模块发布数据和结果事件
  -> 总控校验并转移状态
  -> 异常统一进入 RECOVER_PATROL
  -> 恢复完成或超时回到 PATROL
```

感知内部状态只解决检测、跟踪、认证和目标丢失；导航策略只解决目标空间位置、路径、停车和巡检恢复；总状态机负责把这些结果串成一个不会永久卡住的任务闭环。
