# dog_patrol 当前导航集成说明

本文是当前 `/mnt/nvme/workspace/dog_patrol` 工作区的实现说明。它描述的是已经接入的
代码和参数，不是未来设计草案。原 FAST-LIVO-DOG 的项目说明、建图、定位、驱动和原始
导航说明已放在本目录的上层 `README.md` 以及各子包的 README 中；本文只说明它们在
dog_patrol 中如何被总控、感知和导航协调器组合起来。

> **当前实现基线（2026-09-07）**：导航协调器只消费 `/mission/state`，不拥有第二套
> 业务状态机。当前消息没有 `blocked`、`block_cause`、`TARGET_REACQUIRED` 或
> `HANDLING_COMPLETE`；目标丢失和导航技术故障由总控统一收口到 `RECOVER_PATROL`。

## 1. 系统边界

当前系统分成四层，所有层都必须同时存在，不能把导航协调器误认为完整的导航栈：

1. **硬件与 FAST-LIVO-DOG 层**
   - Livox Mid-360 发布 `/livox/lidar` 和 IMU 数据。
   - MVS 相机发布原始图像 `/left_camera/image_raw`，当前感知接口使用 `1280x1024`。
   - FAST-LIVO、里程计桥接、3D/2D 定位和 TF 发布由 `src/navigation/fast_livo_dog`
     中的原始导航实现负责。
2. **move/Nav2 控制层**
   - Nav2 planner server 提供 `/compute_path_to_pose`。
   - `global_path_seq_publisher` 负责 RViz 目标点、waypoint 和普通巡检路径，输出到
     `/waypoint_global_path`。
   - `pure_pursuit` 从唯一的 `/global_path` 生成 `/subgoal` 和 `/final_goal`。
   - PRIEST/RL 节点生成 `/local_path`。
   - DWB 适配器跟踪局部路径并发布机器狗最终使用的 `/NAV_CMD`。
3. **导航任务协调层**
   - `navigation_path_mux` 根据 `/mission/state` 在 waypoint 路径和任务路径中选择唯一
     输出 `/global_path`。
   - `navigation_mission_coordinator` 订阅任务状态、感知 bbox、雷达和 odom。
   - 它把 bbox 和雷达点云融合成地图坐标目标，调用 Nav2 planner 生成目标接近路径，
     用实时雷达距离在约 3 m 处停止，并报告到达和执行错误事件。
   - 它不直接发布速度，不替换 DWB、Pure Pursuit 或 RL 局部路径。
4. **全局任务管理层**
   - `mission_supervisor` 是业务状态唯一所有者，发布 `/mission/state`，接收
     `/mission/event`，并按事件推进任务状态。
   - 感知和导航都只能通过事件请求状态变化，不能自行修改全局状态。

因此，实际的导航控制链是：

```text
Livox/MVS
  -> FAST-LIVO / localization / TF / odom
  -> Nav2 planner + move waypoint
  -> /waypoint_global_path
  -> dog_patrol_navigation/navigation_path_mux -> /global_path
  -> pure_pursuit -> /subgoal
  -> PRIEST/RL -> /local_path
  -> DWB adapter -> /NAV_CMD
```

目标处置链是：

```text
perception bbox + /livox/lidar
  -> navigation_mission_coordinator
  -> /navigation/target_point
  -> Nav2 /compute_path_to_pose
  -> /mission_global_path
  -> dog_patrol_navigation/navigation_path_mux -> /global_path
  -> 原有局部控制链
```

### 1.1 导航协调器的内部结构

`navigation_mission_coordinator.py` 是 ROS 边界和任务编排层，不再把所有导航算法状态
集中放在一个类中。它仍然是唯一负责以下工作的节点：

- 订阅 `/mission/state`、bbox、雷达和 odom；
- 根据权威 `MissionState` 执行一次状态进入动作；
- 驱动各内部组件，并在组件返回结果后发布 topic、事件和诊断；
- 在异步 planner 结果返回时校验当前 `state_seq`、任务状态、目标 ID 和 planner 类型；
- 维护 `/mission_global_path` 的发布、FollowPath 取消以及 waypoint pause/resume 通知。

它不拥有第二套业务状态机，也不直接发布速度或最终 `/global_path`。

内部模块职责如下：

| 模块 | 负责内容 | 不负责内容 |
|---|---|---|
| `navigation_policy.py` | 将 `MissionState` 映射为状态进入动作和周期执行权限 | 保存运行状态、执行 ROS 操作 |
| `navigation_target_fusion.py` | bbox/雷达同步、点云准备、投影、TF、目标滤波、新鲜度和融合健康 | 修改任务状态、调用 planner、发布路径 |
| `navigation_motion_controller.py` | standoff 目标、接近/跟踪重规划节流、距离/速度/持续时间到达判断 | ROS action、topic 发布、任务状态转换 |
| `navigation_planner_client.py` | `ComputePathToPose` 异步请求、generation、旧请求取消和结果回调 | 判断业务状态是否接受结果 |
| `patrol_recovery_controller.py` | 中断位姿、恢复路径缓存重发、恢复到达保持和 resume 标记 | 发布 mission event、控制 waypoint 节点 |
| `target_estimator.py` | 点云 ROI、深度聚类和目标位置估计/滤波基础算法 | 订阅 ROS topic、推进总任务状态 |

组件之间的调用顺序固定为：

```text
/mission/state
  -> navigation_mission_coordinator
  -> navigation_policy
  -> fusion / motion / planner / recovery
  -> coordinator 校验结果
  -> /mission/event、/mission_global_path、诊断 topic
```

关键一致性约束：

1. 状态切换先更新当前状态和目标，再清理不属于新状态的 fusion、motion 和 planner
   缓存；旧 planner action 通过 generation 失效，迟到结果不能重新发布路径。
2. 目标路径只写入 `/mission_global_path`，waypoint 路径只写入
   `/waypoint_global_path`；`navigation_path_mux` 按 `/mission/state` 选择来源并唯一发布
   `/global_path`。
3. 接近和跟踪 planner 结果必须同时匹配当前任务状态、当前 `state_seq`、当前
   `target_id` 和仍然有效的目标位置；目标在 action 计算期间移动超过阈值时丢弃旧路径，
   等待下一次规划。
4. 恢复路径由协调器缓存并周期重发；恢复完成或超时切回 `PATROL` 时清理任务路径，
   超时使用 `resume_from_current` 从当前位置重新规划，避免沿旧中断路径回头。
5. 运动组件只返回 `stop`、`arrived` 或 planner goal 等纯决策，真正的 ROS 发布和任务
   事件仍集中在协调器，便于审查状态序号和发布顺序。

这次拆分保持任务状态、topic、消息字段、参数含义和底层 Pure Pursuit、RL/PRIEST、DWB
控制链的外部约定不变。对于旧实现中可能造成错误运动或永久等待的边界，重构同时明确了
更严格的安全处理：planner 失败按请求上下文归属，失败重试受节流限制，过期结果不能影响
新任务；恢复时 TF 不可用或已经到位保持时只发送停止，不继续重发运动路径。

## 2. 节点和启动所有权

### 2.1 总入口

当前总入口是：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/orchestration/robot_console/robot_console/app.py
```

`app.py` 负责 UI 和模块启停。导航、感知、建图、定位的开关仍由 UI 管理。原始
FAST-LIVO-DOG README 中的启动命令可作为底层模块参考，但不要同时启动两套
`navigation.launch.py` 或两套定位节点。

### 2.2 导航主 launch

`src/navigation/fast_livo_dog/navigation/move/launch/navigation.launch.py` 的职责是
把 move/Nav2 链和任务协调器放在同一个导航生命周期中。默认行为如下：

| 相对启动时间 | 组件 | 条件 |
|---|---|---|
| 立即 | launch 参数解析、可选 map server 配置 | `use_map_server` 默认 `false` |
| 8 s | Nav2 core | 始终由 `navigation.launch.py` 包含 |
| 11 s | 外部控制链 | `start_external_nav=true` |
| 12 s | 任务导航层（mux + 任务协调器） | `start_mission_coordinator=true` |

外部控制链由 `priest_external_nav.launch.py` 启动以下组件：

- waypoint/RViz 目标点节点：`/clicked_point -> /waypoint_global_path`；
- Pure Pursuit：`/global_path -> /subgoal, /final_goal`；
- PRIEST/RL：`/global_path + /subgoal + /scan -> /local_path`；
- DWB adapter：`/local_path -> /NAV_CMD`；
- `nav_cmd_domain_bridge`；
- `pointcloud_to_laserscan`：`/cloud_registered -> /scan`。

路径 mux 不属于 `move` 控制包。它由
`dog_patrol_navigation/navigation_mission_coordinator.launch.py` 与任务协调器一起启动，
读取 `/mission/state`，在 `/waypoint_global_path` 和 `/mission_global_path` 中选择当前
允许的来源，并作为唯一节点发布最终 `/global_path`。因此 `priest_external_nav.launch.py`
只负责启动 move 侧控制链，完整任务运行仍由 `navigation.launch.py` 组合两层 launch。
该 launch 中的 `mission_state_topic`、`mission_path_topic`、`waypoint_path_topic` 和
`selected_global_path_topic` 会同时传给 mux 和协调器的 Ready 检查，避免自定义 topic
后两者观察不同的路径链路。

协调器本身也可以单独启动，但生产运行应由 `navigation.launch.py` 统一启动，避免
重复创建同名节点、重复发布控制路径或重复启动控制链：

```bash
ros2 launch dog_patrol_navigation navigation_mission_coordinator.launch.py
```

单独启动时可显式指定导航根目录和参数：

```bash
export DOG_PATROL_NAV_ROOT=/mnt/nvme/workspace/dog_patrol/src/navigation/fast_livo_dog
ros2 launch dog_patrol_navigation navigation_mission_coordinator.launch.py \
  params_file:=/mnt/nvme/workspace/dog_patrol/src/navigation/fast_livo_dog/navigation/dog_patrol_navigation/config/m20_patrol_navigation.yaml \
  device_parameters_file:=${DOG_PATROL_NAV_ROOT}/config/device_parameters.yaml \
  use_sim_time:=false
```

### 2.3 全局状态管理器

`dog_patrol_manager/mission_supervisor` 默认配置如下：

- 状态：`/mission/state`；可靠、transient-local、keep-last 1；默认 10 Hz；
- 事件：`/mission/event`；可靠、volatile、keep-last 10；
- 服务：`/mission/reset`，类型 `std_srvs/srv/Trigger`；
- 初始状态：`STARTUP`，初始 `state_seq=1`。

导航协调器不创建 `mission_supervisor`，也不负责业务状态迁移。启动 app 后，UI 或
总控应确保只存在一个 `mission_supervisor`。

## 3. 接口契约

### 3.1 导航协调器的输入

| Topic | 类型 | 发布者 | 约束和作用 |
|---|---|---|---|
| `/mission/state` | `dog_patrol_interfaces/msg/MissionState` | mission supervisor | 当前全局状态、`state_seq` 和目标 ID |
| `/perception/selected_target_bbox` | `dog_patrol_interfaces/msg/TargetBoundingBox` | 感知 tracking | 当前目标框；必须携带目标 ID、图像尺寸和 `camera_link` frame |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Livox/FAST-LIVO 驱动 | 雷达点云；默认 frame 必须是 `livox_frame` |
| `/odom` | `nav_msgs/msg/Odometry` | FAST-LIVO/odom bridge | 当前速度和 odom 数据；用于停止确认 |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Nav2 planner server | 协调器为目标接近、跟踪和巡检恢复请求全局路径 |

### 3.2 导航协调器的输出

| Topic | 类型 | 使用者 | 发送时机 |
|---|---|---|---|
| `/mission/event` | `dog_patrol_interfaces/msg/MissionEvent` | mission supervisor | READY、目标位置就绪、到达、执行错误 |
| `/navigation/target_status` | `dog_patrol_interfaces/msg/TargetNavigationStatus` | UI/总控/感知 | 默认 10 Hz，反馈距离和导航执行子状态 |
| `/navigation/target_point` | `geometry_msgs/msg/PointStamped` | UI/调试/后续模块 | bbox+点云稳定融合后发布，frame 为 `map` |
| `/navigation/target_goal` | `geometry_msgs/msg/PoseStamped` | UI/调试 | 每次调用 planner 前发布本次 planner 目标（当前距目标约 1 m），frame 为 `map` |
| `/mission_global_path` | `nav_msgs/msg/Path` | `navigation_mission_coordinator` | 协调器目标接近、跟踪和恢复的私有路径，供 mux 选择 |
| `/waypoint_global_path` | `nav_msgs/msg/Path` | `global_path_seq_publisher` | waypoint 发布器生成的当前目标单段巡检路径，供 mux 选择 |
| `/global_path` | `nav_msgs/msg/Path` | Pure Pursuit、RL/PRIEST | 仅由 `dog_patrol_navigation/navigation_path_mux` 发布的状态选择路径 |
| `/waypoint_sequence/pause` | `std_msgs/msg/Empty` | waypoint 节点 | 进入接近、核验、跟踪或恢复时暂停巡逻 waypoint |
| `/waypoint_sequence/resume` | `std_msgs/msg/Empty` | waypoint 节点 | 恢复阶段回到中断位姿后，或回到 `PATROL` 时恢复巡逻 |
| `/waypoint_sequence/resume_from_current` | `std_msgs/msg/Empty` | waypoint 节点 | 恢复超时时丢弃旧缓存路径，从当前机器人位姿重新规划当前巡检 waypoint |

协调器只检查 `/NAV_CMD` 是否存在，不发布 `/NAV_CMD`。真正给机器狗底层的控制命令
仍然来自 `priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py`。`/NAV_CMD` 的消息格式
和底盘协议由当前 fast_livo_dog move 链保持，不在协调器中重新定义。

`/mission_global_path` 和 `/waypoint_global_path` 是内部输入 topic，不应直接连接到
Pure Pursuit、RL 或 DWB。下游只订阅 `dog_patrol_navigation` 中 mux 输出的 `/global_path`，
从而避免协调器和 waypoint 发布器同时向同一个控制 topic 写入路径。

从任务路径切回 `PATROL` 时，mux 先发布空路径，并等待状态切换后新到达的 waypoint
路径；它不会直接重放任务期间缓存的旧 waypoint 路径。这样不依赖 mission state、resume
和路径三个 topic 的跨 topic 到达顺序。

### 3.3 消息字段规则

`MissionState.msg` 的状态值：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `STARTUP` | 等待感知和导航 READY |
| 1 | `PATROL` | 普通巡逻 |
| 2 | `CONFIRM_TARGET` | 已发现候选目标，等待导航得到地图位置 |
| 3 | `APPROACH_TARGET` | 导航到目标前的 3 m 位置 |
| 4 | `VERIFY_IDENTITY` | 到达并停止，等待感知识别认证 |
| 5 | `TRACK_INTRUDER` | 核验未通过后持续跟踪并维持 3 m 距离 |
| 6 | `RECOVER_PATROL` | 清理目标任务，返回巡检断点并恢复 waypoint |

`MissionEvent.msg` 的合法发布者：

- 感知发布：`READY`、`TARGET_CONFIRMED`、`AUTHORIZED`、`UNAUTHORIZED`、
  `TARGET_LOST`、`EXECUTION_ERROR`；
- 导航发布：`READY`、`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED`、
  `EXECUTION_ERROR`；
每个事件必须填写当前 `observed_state_seq`。状态切换后 supervisor 会递增
`state_seq`，旧序号事件会被拒绝；重复事件也会被去重。目标相关事件必须填写与
当前状态一致的非零 `target_id`，READY 的 `target_id` 必须是 0。

## 4. 全局状态和导航执行策略

导航协调器没有独立的业务状态机。`navigation_policy()` 是无状态映射函数，只把
总控发布的 `MissionState` 映射成当前一次状态进入动作和周期执行权限；导航模式名
（例如 `acquiring target`、`approaching target`）只用于日志和观测，不能驱动业务转移。
对应关系如下：

| 全局状态 | 导航模式 | 是否暂停巡逻 | 是否融合 bbox | 是否生成目标路径 | 进入时动作 |
|---|---|---:|---:|---:|---|
| `STARTUP` | `INITIALIZING` | 是 | 否 | 否 | 清空路径，等待就绪 |
| `PATROL` | `PATROLLING` | 否 | 否 | 否 | 清目标、恢复 waypoint |
| `CONFIRM_TARGET` | `ACQUIRING_TARGET` | 否 | 是 | 否 | 保持巡逻，后台接收并稳定融合目标位置 |
| `APPROACH_TARGET` | `APPROACHING_TARGET` | 是 | 是 | 是 | 按最新目标位置规划 3 m standoff |
| `VERIFY_IDENTITY` | `HOLDING_FOR_VERIFICATION` | 是 | 是 | 否 | 保持停止，等待认证 |
| `TRACK_INTRUDER` | `TRACKING_TARGET` | 是 | 是 | 是 | 按最新目标位置持续跟踪 |
| `RECOVER_PATROL` | `RECOVERING_PATROL` | 是 | 否 | 返回中断位姿 | 清理目标任务，返回巡检断点并恢复 waypoint |

### 4.1 STARTUP 到 PATROL

协调器启动后内部模式是 `INITIALIZING`。它每 0.1 s 检查一次：

1. `/livox/lidar` 在 `ready_lidar_timeout=0.50 s` 内有新数据；
2. `map -> base_footprint` TF 可用；
3. `/compute_path_to_pose` action server 已就绪；
4. `strict_ready_checks=true` 时，还要求 `/global_path` 有消费者、pause/resume
   有订阅者、`/NAV_CMD` 有发布者。

所有条件满足后，协调器向 `/mission/event` 发布一次导航 `READY`。感知也发布感知
`READY` 后，supervisor 才把全局状态从 `STARTUP` 变为 `PATROL`。只有导航 READY
而没有感知 READY 时，系统会保持 STARTUP，这是正常行为。

### 4.2 PATROL

进入 `PATROL` 时：

- 清理活动目标、目标位置和到达标志；
- 向 `/waypoint_sequence/resume` 发空消息；
- 不消费 bbox，不调用目标跟踪 planner；
- 普通巡逻由 waypoint 节点按照自己的 `waypoint_replan_period` 规划；
- mux 在 `PATROL` 和 `CONFIRM_TARGET` 选择 `/waypoint_global_path`，因此普通巡检仍由
  waypoint/Nav2 链路维护；协调器不直接写 `/global_path`。

感知确认候选目标后发布 `TARGET_CONFIRMED`，supervisor 转到 `CONFIRM_TARGET`。

### 4.3 CONFIRM_TARGET

进入后继续原来的巡逻链，但不把 bbox 直接当作终点，也不因目标确认生成运动路径。
协调器在后台需要同时找到：

- 未过期的 bbox；
- 时间接近的雷达点云；
- bbox 投影区域内足够且连续的雷达点；
- 连续样本稳定。

稳定融合得到地图坐标后，协调器发布 `TARGET_POSITION_READY`。supervisor 收到该
事件后转入 `APPROACH_TARGET`。如果一直没有稳定位置，不会盲目调用 planner，也不会
把感知图像像素坐标直接当成地图坐标。

### 4.4 APPROACH_TARGET

协调器每次使用最新的目标地图位置计算一个距目标约 1.0 m 的 planner goal，并调用
Nav2 `ComputePathToPose`；是否停止仍由实时雷达平面距离的 3.0 m 条件独立决定。目标在
接近过程中可以移动，因此不是一次性固定终点：

- 定时重规划周期为 `motion.approach_replan_period=0.50 s`；
- 目标相对上一次规划移动至少 `motion.target_replan_distance=0.25 m` 时提前重规划；
- 同时只有一个 action request 在飞行，实际频率受 planner 计算时间限制；
- action 返回路径后才更新 `/mission_global_path`，失败不会发布半成品路径；mux 只在
  `APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER` 和 `RECOVER_PATROL` 转发该路径；
- planner 不可用、目标被拒绝、请求异常或返回空路径都会绑定当前
  `state_seq + target_id + plan_kind` 记录；过期请求的失败不会污染新任务。只有确认
  action 可用并实际发起的 planner 尝试才写入请求节流时间；action 未就绪时不伪造时间，
  等待下一次可用性检查。已发起的恢复请求按 `approach_replan_period` 节流后重试；目标
  重规划失败或结果过期时先清空旧目标路径并停止，不能继续沿旧目标路径运动；
- 目标过期或 TF 暂时不可用时立即清空路径并停止；连续的融合、TF 或 planner 技术
  故障达到 `technical_error_timeout` 后才报告 `EXECUTION_ERROR`。单纯 bbox 超时
  不会由导航自动改写全局任务状态，目标丢失事件应由感知端发布 `TARGET_LOST`。

目标实时平面距离不大于 `3.0 + 0.10 = 3.10 m` 时，协调器发布空路径并持续停止。只有距离
满足、线速度不大于 `0.05 m/s`、角速度不大于 `0.10 rad/s`，并连续保持 `0.50 s`
后，才发布 `ARRIVED_AND_STOPPED`。这会使 supervisor 转到 `VERIFY_IDENTITY`。

### 4.5 VERIFY_IDENTITY

该状态是导航侧的保持阶段：

- 继续允许 bbox 融合，用于保持目标状态和 UI 距离显示；
- 不生成移动目标路径；
- 继续发布空路径，防止控制器继续使用旧路径；
- 感知内部执行自己的两次认证流程；
- 感知发布 `AUTHORIZED` 后总控进入 `RECOVER_PATROL`，导航恢复巡检后再回到 `PATROL`；
- 发布 `UNAUTHORIZED` 后进入 `TRACK_INTRUDER`。

认证流程的内部细节不由导航协调器解释，导航只关心这两个事件。

### 4.6 TRACK_INTRUDER

该状态和接近阶段的区别是：目标持续运动时，协调器持续更新目标位置并按
`tracking_distance=3.0 m` 重新计算 standoff goal。它使用
`tracking_replan_period=0.50 s` 和 `target_replan_distance=0.25 m`。如果目标数据
暂时过期，导航清空目标路径并保持停止；如果融合、TF 或 planner 技术故障持续超过
阈值，导航发布 `EXECUTION_ERROR`，由总控进入 `RECOVER_PATROL`。导航不发布
`TARGET_LOST`，也不等待重新获取事件或人工完成事件。

### 4.7 RECOVER_PATROL

`RECOVER_PATROL` 是总控的统一任务收口状态，不是导航自己的业务状态。进入该状态后：

- 协调器取消目标 planner 和 FollowPath 请求，清空目标路径及目标缓存，并持续发布空路径；
- 暂停 waypoint 巡逻；
- 使用进入 `APPROACH_TARGET` 时保存的 `map -> base_footprint` 位姿作为恢复目标；
- 恢复 planner 返回的路径后，协调器缓存并按 `stop_republish_period` 重发
  `/mission_global_path`，直到进入中断位姿容差；进入容差后清空运动路径并保持停止，
  避免到位保持期间旧路径继续驱动；
- 当前位姿或 TF 暂时不可用时只清空路径并保持停止；已经进入恢复位置容差且正在等待
  稳定停车时同样只保持停止，不继续重发恢复运动路径；
- 从恢复运动路径切换到停止时先立即发布一次空路径，后续空路径按
  `stop_republish_period` 周期重发；
- 到达位置容差 `0.25 m` 且速度稳定后发布 `/waypoint_sequence/resume` 和
  `PATROL_RECOVERY_COMPLETE`；不等待 waypoint 发布新的非空路径；
- 如果没有可用的中断位姿，协调器不伪造恢复目标，而是清理任务并直接发布
  `PATROL_RECOVERY_COMPLETE`；如果 `patrol_recovery_timeout=10.0 s` 到期，supervisor
  也会直接收口到 `PATROL`，不会形成二次错误状态。此时协调器不发送普通 `resume`，而是
  发送 `/waypoint_sequence/resume_from_current`，waypoint 节点清除暂停前缓存路径，从当前
  机器人位姿重新规划当前巡检 waypoint，避免机器人沿旧路径回到中断点。

`/mission/reset` 会让 supervisor 回到 `STARTUP`、清空目标并递增 `state_seq`。它不
重启节点。重新 READY 后才会再次进入巡逻。

## 5. bbox 与雷达融合流程

### 5.1 输入坐标和分辨率

当前约定是：

```text
/left_camera/image_raw       1280 x 1024
/perception/selected_target_bbox 的 x_min/y_min/x_max/y_max 在这张原图上
bbox header.frame_id         camera_link
/livox/lidar                  frame_id=livox_frame
```

`/left_camera/image` 是感知内部或调试用的 `640x512` 图像时，不能把它的像素框直接
当作 `1280x1024` 使用。若感知端输出的是 640x512 框，就必须在感知端先按比例放大
到原图坐标，或在协调器参数中明确调整图像尺寸；当前默认按 1280x1024 校验。

### 5.2 过滤和投影步骤

每次融合按以下顺序执行：

1. 只在 `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER`
   允许目标融合；`PATROL` 的 bbox 不会污染目标缓存。
2. 丢弃超过 `max_bbox_age=0.80 s` 的框，拒绝明显来自未来的时间戳。该上限覆盖
   当前 10 Hz 感知链约 0.31 s 的采集、推理和发布延迟，并额外容纳一次短暂调度抖动；
   不代表允许相机和雷达错配。
3. 从点云缓存中寻找与 bbox 时间最近的点云，时间差必须不超过
   `sync_tolerance=0.12 s`。
4. 按 `min_range=0.40 m` 和 `max_range=15.0 m` 过滤雷达点，并按
   `point_stride=1` 采样。
5. 用 `T_camera_lidar` 将雷达点投影到相机；投影区域和 bbox 的关系受
   `projection_margin_pixels=32.0`、ROI 内缩和宽高比容差约束。
6. 对 bbox 内的投影点按深度聚类，连续深度间隔不超过 `cluster_gap=0.40 m`，
   每个候选簇至少 `min_cluster_points=3` 个点。
7. 选择有效目标簇，经过 `stable_samples=3`、`stability_window=5`、跳变和散布
   检查，再用 `smoothing_alpha=0.55` 平滑。
8. 将稳定的 `livox_frame` 点转换到 `base_link`，再通过带时间戳的 TF 转换到
   `map`，发布 `/navigation/target_point`。

### 5.3 外参方向

当前 `device_parameters.yaml` 是标定唯一来源：

- `T_camera_lidar` 是 FAST-LIVO 沿用的历史参数名，实际运行语义是 lidar -> camera；
  `LIVMapper` 将它直接传入 `setLidarToCameraExtrinsic()`，协调器也直接用于投影，
  不得再次取逆；
- 设备文件中的 `T_lidar_base` 在原 `odom_bridge` 中实际作为 `T_body_base` 参与
  `T_camera_init_body * T_body_base`，也与系统发布的 `base_link -> livox_frame` 数值
  一致；协调器接收的是 `livox_frame` 点，因此读取后取逆得到 lidar -> base_link；
- `map -> base_link` 或 `map -> base_footprint` 的动态关系由 FAST-LIVO/定位 TF
  提供，不能用静态外参替代；
- `base_link` 是机器狗主体中心，`base_footprint` 是导航平面参考坐标，二者不能
  在参数中随意互换。

如果目标点整体偏移、上下楼层错误或点云看起来固定在世界原点，优先检查 frame_id、
TF 时间和外参方向，不要先调聚类阈值。

## 6. 目标路径和停止逻辑

### 6.1 standoff 目标计算

设机器人地图位置为 `(rx, ry)`，目标为 `(tx, ty)`，目标距离为 `d`。当 `d` 大于
允许停止距离时，发送的规划目标为：

```text
scale = (d - standoff_distance) / d
goal = robot + scale * (target - robot)
```

所以 planner 目标在目标和机器人之间，而不是目标中心。当前配置的
`planning_goal_distance=1.0 m` 只决定 planner 请求点距离目标约 1 m；实际是否到达
目标由最新雷达测得的 `approach_distance=3.0 m` 和 `arrival_distance_tolerance=0.10 m`
判定，并不是把规划目标放到人身上。目标朝向使用机器人指向目标的 yaw。3D 点云的 z
只用于目标融合和显示；当前目标 standoff planner 目标的 z 固定为 0，因为下游是 Nav2
平面路径和机器狗平面控制链。

### 6.2 planner 请求和路径发布

每次请求：

- action：`/compute_path_to_pose`；
- planner_id：`GridBased`；
- goal frame：`map`；
- `use_start=false`，由 planner 使用自己的当前起点；
- goal stamp：当前 ROS 时间；
- action 返回成功且任务状态、目标 ID、请求代数仍匹配时，才发布
  `/mission_global_path`；`navigation_path_mux` 再将它转发到 `/global_path`。

这意味着协调器产生的是 `/mission_global_path` 目标处置路径，普通巡逻由 waypoint 节点
产生 `/waypoint_global_path`。两者通过 `navigation_path_mux` 汇合，任何时刻只有被当前
公共状态允许的来源能够写入下游 `/global_path`。

### 6.3 到达条件

当前到达不是单纯距离判断，必须同时满足：

```text
distance <= 3.0 m + 0.10 m tolerance
linear_speed <= 0.05 m/s
angular_speed <= 0.10 rad/s
odom 新鲜度 <= 0.50 s
以上条件连续保持 >= 0.50 s
```

任一条件失效都会清除到达保持计时器。到达后必须重复发布停止路径，防止下游保留上
一次非空路径。

## 7. 当前参数总表

唯一可调配置文件：

```text
src/navigation/fast_livo_dog/navigation/dog_patrol_navigation/config/m20_patrol_navigation.yaml
```

`navigation.launch.py` 会把 `device_parameters_file` 单独传给协调器。设备标定不应
复制到下面的 YAML 中。

### 7.1 topics

| 参数 | 当前值 | 作用 |
|---|---|---|
| `topics.mission_state` | `/mission/state` | 订阅全局任务状态 |
| `topics.mission_event` | `/mission/event` | 发布导航事件 |
| `topics.target_bbox` | `/perception/selected_target_bbox` | 订阅感知选中目标框 |
| `topics.target_status` | `/navigation/target_status` | 发布导航目标状态 |
| `topics.lidar` | `/livox/lidar` | 订阅原始点云 |
| `topics.odom` | `/odom` | 订阅速度和 odom 时间 |
| `topics.global_path` | `/mission_global_path` | 协调器发布的目标处置/恢复私有路径 |
| `topics.selected_global_path` | `/global_path` | mux 选择后供 Pure Pursuit、RL 和 DWB 消费的路径 |
| `topics.pause_patrol` | `/waypoint_sequence/pause` | 暂停巡逻 waypoint |
| `topics.resume_patrol` | `/waypoint_sequence/resume` | 恢复巡逻 waypoint |
| `topics.resume_patrol_from_current` | `/waypoint_sequence/resume_from_current` | 恢复超时后从当前位姿重新规划巡逻 |
| `topics.nav_cmd` | `/NAV_CMD` | 只用于 READY 检查，不由协调器发布 |
| `topics.target_point` | `/navigation/target_point` | 发布地图系目标点 |
| `topics.target_goal` | `/navigation/target_goal` | 发布本次 planner standoff 目标 |

### 7.2 planner 和 frames

| 参数 | 当前值 | 作用 |
|---|---|---|
| `planner.action_name` | `/compute_path_to_pose` | Nav2 ComputePathToPose action |
| `planner.id` | `GridBased` | 发送给 planner server 的规划器 ID |
| `frames.global` | `map` | 全局路径、目标点和目标 goal frame |
| `frames.base` | `base_link` | 机器狗主体中心，目标点外参转换终点 |
| `frames.robot` | `base_footprint` | 平面导航 TF 和 readiness 检查 frame |
| `frames.lidar` | `livox_frame` | 点云 frame 校验和外参输入 frame |
| `frames.camera_optical` | `camera_link` | bbox frame 校验值 |

### 7.2.1 waypoint 巡检参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `waypoint_goal_tolerance` | `1.0 m` | 机器人进入当前巡检 waypoint 的距离容差后，切换到下一个 waypoint |
| `waypoint_replan_period` | `1.0 s` | 普通巡检 waypoint 路径的重新规划周期 |

这里的 `waypoint_goal_tolerance` 只用于普通巡检 waypoint 的切换，不是目标接近阶段的
`arrival_distance_tolerance=0.10 m`。waypoint 发布器每次只向当前 `current_index` 的
目标请求并发布一段 `/waypoint_global_path`，不会把后续 waypoint 的路径拼接到当前路径
中。导航协调器的目标路径和恢复路径发布到 `/mission_global_path`；两路都只通过 mux
进入最终 `/global_path`。

### 7.3 calibration

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `calibration.device_parameters_file` | 由 launch 传入 | 标定唯一来源，默认指向 fast_livo_dog/config/device_parameters.yaml |
| `calibration.image_width` | `1280` | bbox 原图宽度 |
| `calibration.image_height` | `1024` | bbox 原图高度 |
| `calibration.fx/fy/cx/cy` | `1291.9363/1291.9620/627.2076/515.7676` | 设备文件没有相机参数时的备用内参 |
| `calibration.distortion` | `[-0.09454294, 0.25490605, -0.00134425, -0.00195962]` | 设备文件没有畸变参数时的备用参数 |
| `calibration.lidar_to_camera` | 代码内 4x4 矩阵 | FAST-LIVO 实际按 lidar->camera 使用，协调器直接用于投影 |
| `calibration.lidar_to_base` | 代码内 4x4 矩阵 | 协调器内部使用的 lidar->base_link；设备文件矩阵读取后取逆 |

实际运行时优先读取设备文件中的相机内参和外参。只有 `device_parameters_file` 为空、
文件不存在或字段缺失时才使用 YAML 内的备用值。因此切换 stationary/mobile profile
后要确认 launch 传入的是同一份设备配置。

代码内备用外参的完整值如下。这里的矩阵按行展开，最后一行固定为
`[0, 0, 0, 1]`：

```text
calibration.lidar_to_camera:
  [ 0.011685, -0.999924,  0.003980, -0.029332,
    0.497425,  0.002360, -0.867504, -0.149256,
    0.867428,  0.012116,  0.497414,  0.014702,
    0.0,       0.0,       0.0,        1.0 ]

calibration.lidar_to_base:
  [ 0.881055, -0.020738,  0.472559,  0.327138,
    0.016898,  0.999780,  0.012370,  0.014138,
   -0.472713, -0.002913,  0.881212,  0.312380,
    0.0,       0.0,       0.0,        1.0 ]
```

`lidar_to_camera` 这个参数名是协调器内部消费方向；读取 fast_livo_dog 设备文件
时直接使用文件中的 `T_camera_lidar`。设备文件的 `T_lidar_base` 按原导航 TF 的实际
组合方向取逆一次，得到协调器内部的 `lidar_to_base`，再乘以 `livox_frame` 点。

### 7.4 fusion

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `max_bbox_age` | `0.80 s` | bbox 最大允许年龄；覆盖当前约 0.31 s 的感知发布延迟和短时调度抖动 |
| `sync_tolerance` | `0.12 s` | bbox 与雷达点云最大时间差 |
| `cloud_buffer_size` | `20` | 点云缓存数量 |
| `min_range/max_range` | `0.40/15.0 m` | 雷达点距离过滤范围 |
| `roi_horizontal_inset` | `0.15` | bbox 左右边缘内缩比例，减少边缘误投影 |
| `roi_top_inset` | `0.08` | bbox 上边缘内缩比例 |
| `roi_bottom_inset` | `0.15` | bbox 下边缘内缩比例 |
| `cluster_gap` | `0.40 m` | 深度聚类允许的相邻点间距 |
| `min_cluster_points` | `3` | 有效目标簇最少点数 |
| `point_stride` | `1` | 点云抽样步长，1 表示不抽样 |
| `projection_margin_pixels` | `32 px` | 投影判断的 bbox 像素边界余量 |
| `aspect_ratio_tolerance` | `0.02` | 图像宽高比例误差容忍度 |
| `stable_samples` | `3` | 至少需要的稳定样本数量 |
| `stability_window` | `5` | 稳定性判断窗口长度 |
| `max_sample_jump` | `0.60 m` | 相邻样本最大允许跳变 |
| `max_stability_spread` | `0.50 m` | 稳定窗口最大空间散布 |
| `smoothing_alpha` | `0.55` | 新样本平滑权重，越大越灵敏、越小越稳 |
| `target_timeout` | `0.90 s` | 目标位置超过该时间未更新即视为过期；过期后清空目标路径并停车 |
| `failure_timeout` | `3.0 s` | 连续技术失败超过该时间报告执行错误 |

这些参数控制“目标位置是否可信”，不等同于点云避障参数。不要通过放大 bbox ROI
来解决导航绕障问题；目标跟踪和局部障碍物避障是两条不同的数据链。

### 7.5 motion

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `approach_distance` | `3.0 m` | 接近目标时的期望停留距离 |
| `tracking_distance` | `3.0 m` | 跟踪目标时的期望停留距离 |
| `arrival_distance_tolerance` | `0.10 m` | 到达距离容差 |
| `arrival_linear_speed` | `0.05 m/s` | 到达确认的最大线速度 |
| `arrival_angular_speed` | `0.10 rad/s` | 到达确认的最大角速度 |
| `arrival_hold_time` | `0.50 s` | 速度和距离条件连续保持时间 |
| `approach_replan_period` | `0.50 s` | 接近状态的最短定时重规划周期 |
| `tracking_replan_period` | `0.50 s` | 跟踪状态的最短定时重规划周期 |
| `target_replan_distance` | `0.25 m` | 目标移动超过此距离时提前重规划 |
| `stop_republish_period` | `0.20 s` | 停止/阻塞时空路径重发周期 |

### 7.6 runtime

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `tick_rate` | `10.0 Hz` | 主状态和目标处理周期 |
| `status_rate` | `10.0 Hz` | `/navigation/target_status` 发布频率 |
| `strict_ready_checks` | `true` | 是否要求完整下游链路后才 READY |
| `ready_lidar_timeout` | `0.50 s` | READY 判断雷达新鲜度 |
| `ready_event_period` | `1.0 s` | STARTUP 等待期间 READY 检查/重发周期 |
| `technical_error_timeout` | `3.0 s` | 连续 TF/planner 技术异常报告错误的时间 |
| `tf_timeout` | `0.05 s` | TF 查询等待时间 |

`strict_ready_checks=true` 适合完整 app 运行；如果只单独调试协调器而下游没有订阅者，
它会一直报告等待 `/global_path` consumer、waypoint subscriber 或 `/NAV_CMD` publisher。
这不是 bbox 融合故障。调试时可以临时把它改为 `false`，测试完再恢复为 `true`。

## 8. 原有导航参数和本协调器参数的关系

以下两类参数不要混淆：

- `m20_patrol_navigation.yaml`：只控制任务协调器的状态、bbox/点云融合、目标规划和
  READY 检查；
- `config/nav_parameters.yaml` 以及 move 下的 Nav2 配置：控制 FAST-LIVO-DOG 原有
  定位、Nav2 costmap、DWB、Pure Pursuit、PRIEST/RL 和 `/NAV_CMD` 适配。

例如 `approach_replan_period=0.50` 只决定目标协调器多久请求一次新的目标全局路径，
不会提高 DWB 的控制频率；`arrival_linear_speed=0.05` 只决定何时向业务层报告到达，
不会改变底盘死区；`frames.robot=base_footprint` 也不会替换 move 中的
`odom_frame=camera_init_footprint`。move 链的 frame 配置必须继续遵守原
FAST-LIVO-DOG 的实现。

## 9. 故障处理和排查顺序

### 9.1 启动后一直 STARTUP

依次检查：

```bash
ros2 topic echo /mission/state
ros2 topic info /livox/lidar -v
ros2 topic info /global_path -v
ros2 topic info /NAV_CMD -v
ros2 action list | rg compute_path_to_pose
ros2 run tf2_ros tf2_echo map base_footprint
```

如果状态详情是 `waiting for perception and navigation ready`，先确认感知端是否发布
了 `MissionEvent.READY`。如果详情是 `waiting for /global_path consumer` 等，则是
导航控制链未启动或 topic 被 remap 成了别的名字。

### 9.2 有 bbox 但没有目标点

依次检查：

```bash
ros2 topic echo /perception/selected_target_bbox
ros2 topic echo /livox/lidar --once
ros2 topic echo /navigation/target_point
ros2 topic echo /navigation/target_status
```

重点核对：bbox 是否 `camera_link`、图像尺寸是否 `1280x1024`、点云是否
`livox_frame`、时间差是否小于 `0.12 s`、`device_parameters.yaml` 是否为当前相机
和雷达的 profile。目标点没有稳定发布前，协调器不会进入目标路径规划。

### 9.3 有目标点但没有全局路径

检查 planner action 和当前状态：

```bash
ros2 action info /compute_path_to_pose
ros2 topic echo /mission/state
ros2 topic echo /navigation/target_goal
ros2 topic echo /mission_global_path
ros2 topic echo /waypoint_global_path
ros2 topic echo /global_path
```

确认当前是 `APPROACH_TARGET` 或 `TRACK_INTRUDER`，目标没有超时，目标 goal 在 `map`
中且位于已知地图范围内。planner 计算时间过长时，实际路径刷新频率会低于 2 Hz，
这是 action 计算能力限制，不是 `tick_rate` 没有运行。

`/mission_global_path` 只有目标接近、跟踪或恢复时应有内容；`/waypoint_global_path` 只有
普通巡检时应被 mux 转发；最终 `/global_path` 的发布者应只有
`dog_patrol_navigation/navigation_path_mux`。

### 9.4 返回巡检后没有局部路径

确认总控已经进入新的 `PATROL`，`navigation_path_mux` 已切回 waypoint 来源，导航已经
发送 waypoint resume，且 `/waypoint_global_path` 和 `/global_path` 都重新收到当前巡检
路径。mux 不重放切换前缓存的 waypoint 路径，只接受 `PATROL` 状态切换后的路径消息。
恢复成功时先重发 `/mission_global_path` 直到返回中断点；恢复超时时清空任务路径，
通过 `resume_from_current` 从当前位置重新规划当前 waypoint。若只有 `/global_path` 而没有
`/local_path`，继续检查 RL 是否收到空路径后的清理日志、odom/scan 是否可用，以及是否存在
第二个 `/global_path` 发布者。

### 9.5 目标到了但业务状态不切换

确认 `/odom` 正常更新，并观察线速度、角速度是否同时低于到达阈值。距离只达到
3 m 但机器人仍在转向时，不会发布 `ARRIVED_AND_STOPPED`。另外要确认
`state_seq` 和 `target_id` 没有被旧消息污染。

## 10. 文档位置

当前 fast_livo_dog 原始文档已经迁入以下位置：

- `src/navigation/fast_livo_dog/README.md`：原项目完整 README，包含依赖、建图、定位、
  原始导航和第三方库说明；
- `src/navigation/fast_livo_dog/dbow3/README.md`：DBoW3 组件说明；
- `src/navigation/fast_livo_dog/livox_ros_driver2/README.md`：Livox 驱动说明；
- `src/navigation/fast_livo_dog/rpg_vikit/README.md`：rpg_vikit 组件说明；
- `src/navigation/fast_livo_dog/navigation/dog_patrol_navigation/docs/current_workspace_integration.md`：
  本文，说明 dog_patrol 当前协调器和 fast_livo_dog 导航链的集成逻辑。

子模块 README 保留原文；只有根 README 增加了当前 dog_patrol 路径迁移提示，避免原始
快速上手命令中的旧路径造成误操作。
