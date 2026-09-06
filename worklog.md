# worklog

## 2026-09-06 - waypoint 全局路径改为只发布当前目标单段路径

- 修改 `global_path_seq_publisher` 的 waypoint 规划逻辑：每次只向 Nav2
  `ComputePathToPose` 请求当前 `current_index` 对应目标点的路径。
- planner 返回后直接发布“机器人当前位置到当前 waypoint”的单段 `/global_path`，不再递归
  请求后续 waypoint，也不再把多个 waypoint 之间的路径拼接成一条控制路径。
- Pure Pursuit/RL 现在只能在当前目标段内寻找 subgoal，不会因为前瞻距离超过当前目标剩余
  距离而提前进入下一个 waypoint，尤其避免相邻目标方向相反时 subgoal 跳到反向路径。
- 当前 waypoint 到达后，原有 `current_index` 推进逻辑清空旧路径；下一次重规划再为新的
  当前 waypoint 请求单独路径。所有 waypoint 的可视化数据和 waypoint 状态发布保持不变。
- 保留 planner 请求版本校验、旧 action 取消、暂停/恢复缓存和
  `resume_from_current` 逻辑；本次只改变控制路径的生成范围。

## 2026-09-06 - 确认 waypoint 多段路径与 subgoal 前瞻冲突，移除独立 0.1 秒到达检查

- 纯 waypoint 导航测试已经复现：单个目标点使用 `goal_tolerance=0.5 m` 时没有问题；多个
  waypoint 组成直线往返、尤其相邻目标方向接近 180 度时，会出现当前目标尚未完成但路径
  和 subgoal 已经转向后续目标，随后局部路径回头、当前目标到达状态不更新等现象。
- 根因判断是：`global_path_seq_publisher` 将当前 waypoint 到后续 waypoint 的多个 Nav2
  路径段拼接后发布到控制链消费的 `/global_path`，而 Pure Pursuit/RL 根据整条路径和约
  `1.8 m` 前瞻距离寻找 subgoal。当前 waypoint 到机器人距离不足前瞻距离时，subgoal 会
  越过当前 waypoint 进入下一段，甚至进入反向路径；但 waypoint 管理器仍只按当前
  `current_index` 的 `0.5 m` 距离判断到达，因此两套路径进度不一致。
- `goal_tolerance=1.5 m` 不是正确修复，只是因为到达范围更大，通常能让 waypoint 索引在
  控制器明显转入下一段前提前推进，从而掩盖进度冲突。改成 `0.5 m` 后问题被暴露出来。
- 根本修复方向已经确定：控制器消费的 `/global_path` 只发布机器人到当前 waypoint 的
  单段路径；后续 waypoint 只作为 waypoint/标记数据保留，不能继续拼接进当前控制路径。
- 因为后续路径段不应提前暴露给 subgoal，独立的 `goal_check_period=0.1 s` 到达检查不再
  是必要逻辑。它只是额外增加一套 waypoint 进度定时器，不能解决多段路径导致的根因。
- 本次删除 `goal_check_period` 参数、独立到达检查定时器和对应 launch 传递；到达检查恢复
  到原有 `replan_period` 定时器内。单段 `/global_path` 的实现作为下一步独立修改和测试。

## 2026-09-06 - 待验证：普通 waypoint 导航的路径进度与 subgoal 可能不同步

- 当前需要在不启动感知目标任务的情况下，单独测试普通 RViz waypoint 导航，确认机器狗到达
  第一个目标后是否会偶发回头、局部路径是否会重新指向已经经过的目标，以及 RViz 中第一
  个 waypoint 是否正确变为已完成状态。
- 当前怀疑原因不是单独的 `waypoint_goal_tolerance=0.5`，而是两套进度逻辑可能不一致：
  `global_path_seq_publisher` 会从当前 waypoint 开始，将当前点到后续 waypoint 的多个
  Nav2 路径段拼接后发布到共享 `/global_path`；Pure Pursuit/RL 链会在整条路径上寻找当前
  位置附近的路径段并生成前瞻 `subgoal`；waypoint 发布器则只依据机器人到
  `current_index` 目标点的距离判断是否到达。
- 可能出现的现象是：控制链已经根据整条路径提前进入“当前点到下一个点”的路径段，但
  `current_index` 仍未推进；下一次重规划仍以旧 waypoint 为目标，导致 `/global_path`、
  `/local_path` 或 `subgoal` 短暂向后，机器人回头，且旧 waypoint 的完成颜色没有更新。
- 当前先不把该假设当作最终结论，也不继续修改代码。测试时应同时记录：
  `waypoint_sequence/status`、`global_path` 首尾坐标和长度、`subgoal`、`local_path`、
  机器人 TF 位姿以及 RViz waypoint 颜色变化。
- 已有的 0.1 s 到达检查和普通 `PATROL` 路径所有权修改只解决低频漏检与协调器抢占共享路径
  的问题，是否还需要将控制用 `/global_path` 限制为当前 waypoint 段，需根据本次纯导航实车
  测试结果决定。

## 2026-09-06 - 修复普通 waypoint 导航经过目标点后回头

- 修复普通 RViz waypoint 导航只在全局重规划定时器中检查到达的问题。原逻辑的检查周期通常
  为 1 秒，而路径会一次性覆盖当前点到后续点；机器狗可能在两次检查之间经过第一个点，
  但 `current_index` 仍停留在第一个点，下一次规划就会重新规划回刚才的点。
- `global_path_seq_publisher` 新增独立的 `goal_check_period`，默认 0.1 秒；到达检查不再
  依赖 `replan_period`，检测到当前点后立即推进索引、刷新 waypoint 颜色并为下一个点请求
  新路径。
- 普通 `PATROL` 状态下，导航协调器不再清空共享 `/global_path` 或取消普通 waypoint 控制器。
  `/global_path` 在巡检期间由 waypoint 节点独占维护，目标跟踪协调器只在真正进入目标处置
  状态后接管路径。
- launch 已同步传递 `waypoint_goal_check_period`，默认值为 0.1 秒；保留原有 waypoint
  `goal_tolerance=0.5` 配置。
- 增加 waypoint 到达推进和 `PATROL` 路径所有权回归测试，验证不会因低频检查或任务协调器
  的普通巡检状态切换而回到上一个目标点。

## 2026-09-05 - 恢复巡检超时后从当前位置重新规划

- 修复恢复巡检超时后可能沿暂停前缓存路径回到旧中断点的问题。根因是总控超时进入
  `PATROL` 后，原有 `/waypoint_sequence/resume` 会让 waypoint 节点立即重发暂停前的
  `_last_path`，这条路径的起点可能已经不适合机器狗当前位置。
- 导航协调器新增 `/waypoint_sequence/resume_from_current`。正常在中断点恢复完成时仍使用
  原有 `/waypoint_sequence/resume`；只有 `RECOVER_PATROL` 超时强制返回 `PATROL` 时，才
  发布新的当前位置恢复命令。
- waypoint 节点收到当前位置恢复命令后清除 `_last_path` 和 resume bridge，发布空路径取消
  下游旧路径，并保留当前巡检 waypoint 索引，从机器狗当前位姿立即重新请求路径。
- 总控状态机、目标 `target_id` 清理逻辑和正常中断点恢复逻辑未修改；超时后的行为是继续
  当前巡检任务，但不再返回原来的中断点。
- 更新导航 launch 传递和协调器配置，新增恢复话题参数；增加 waypoint 当前位置恢复单元
  测试。`move` 和 `dog_patrol_navigation` 编译通过，相关 pytest 通过（3 passed）。

## 2026-09-05 - 删除 BLOCKED 观测状态并统一协议枚举

- 保留 `TargetNavigationStatus.msg` 作为导航内部观测接口；导航协调器默认以 10 Hz 发布
  `/navigation/target_status`，用于查看等待目标、接近、到达、保持、跟踪和恢复等内部状态。
  删除 `BLOCKED=5`，目标数据暂不可用时使用 `HOLDING` 并在 `detail` 中说明原因；该消息
  不参与总状态机转移。
- 总控 `/mission/state` 的 `state_publish_rate` 默认保持 10 Hz。你看到的
  `DeclareLaunchArgument(... default_value="1.0")` 是旧的 1 Hz 配置，本工作区当前 launch 和
  节点默认值都已是 10.0。
- 感知任务级 `TARGET_LOST` 的 `target.lost_event_timeout_sec` 统一为 10.0 秒。当前 ROS 图像
  输入约 10 Hz，但 `camera.fps=30.0` 仍保留为 Hik MVS 直接采集模式的标称采集率；使用
  `ros_image` 时实际频率由 `/left_camera/image_raw` 决定。
- 删除旧事件 8、9 后，将 `PATROL_RECOVERY_COMPLETE` 调整为 8，使当前事件枚举连续为 0--8。
- `target.lost_threshold_frames` 从 180 调整为 100。按当前 10 Hz ROS 图像输入约为 10 秒；
  它仍是跟踪器内部帧级生命周期，任务级最终丢失事件仍以独立的
  `target.lost_event_timeout_sec=10.0` 为准。
- 同步更新任务级丢失测试用例为 10 秒，并将四秒遮挡测试调整为 40 帧，避免测试继续依赖旧的
  6 秒超时和 180 帧默认值。

## 2026-09-05 - 恢复导航观测状态并统一频率与目标丢失时限

- 恢复 `TargetNavigationStatus.msg` 作为导航内部观测接口，导航协调器以默认 10 Hz 发布
  `/navigation/target_status`，用于显示等待目标、接近、到达、保持、跟踪、恢复和暂时不可执行
  等状态，以及当前可用的目标距离。该接口不参与总状态机转移，不恢复 `MissionState` 的
  `blocked` 字段或旧 blocked/reacquire 协议。
- 总控 `mission_supervisor` 的 `state_publish_rate` 默认从 1 Hz 调整为 10 Hz，launch 默认值
  同步为 10 Hz；状态变化仍立即发布，周期发布用于给感知、导航和 UI 提供稳定的当前快照。
- 感知任务级目标丢失确认时限统一为 `target.lost_event_timeout_sec=10.0`，同步更新默认参数、
  运行资源参数和 `MissionCoordinator` 默认值。感知内部目标图像发布上限保持 10 Hz；不把任务级
  10 秒时限错误换算成固定丢失帧数。
- `MissionEvent` 的 `PATROL_RECOVERY_COMPLETE=10` 数值保持不变，用于兼容已有恢复事件线缆编号；
  中间的旧事件编号不会重新启用。

## 2026-09-05 - 清理目标重新获取和 blocked 兼容协议

- 删除跨模块消息中的旧兼容字段和事件：`MissionState` 不再包含 `blocked`、
  `block_cause`、`BLOCK_TARGET_LOST`、`BLOCK_EXECUTION_ERROR`；`MissionEvent` 不再包含
  `SOURCE_OPERATOR`、`HANDLING_COMPLETE`、`TARGET_REACQUIRED`。
- 删除未被运行代码订阅的 `TargetNavigationStatus.msg`，同步移除接口包生成清单、导航
  协调器发布器、`/navigation/target_status` 参数和相关测试/文档引用。目标距离和执行
  结果继续通过日志、`/navigation/target_point`、`/navigation/target_goal`、路径和
  `MissionEvent` 传递。
- 感知 `MissionCoordinator` 删除 `reacquire_retention_sec`、`LossCycle` 以及旧 blocked/
  reacquire 分支。目标连续丢失达到 `target.lost_event_timeout_sec`（默认 6 s）后只发布
  一次 `TARGET_LOST`；同一个 `state_seq` 内即使目标重新出现也不再发布旧任务 bbox，
  必须等待总控进入新的任务状态后才能重新建立目标任务。
- 总控、导航协调器、认证 provider、fake 联调工具和当前合同文档统一改为
  `TARGET_LOST/EXECUTION_ERROR -> RECOVER_PATROL -> PATROL` 的恢复路径；核验失败仍以
  `UNAUTHORIZED` 进入持续跟踪，不再等待人工完成事件。
- 验证：受影响的接口、总控、导航和 tracking 包 `colcon build` 通过；生成安装空间不再
  包含 `TargetNavigationStatus`。本机 ROS gtest 受沙箱 DDS socket/只读日志目录限制，
  pytest 受系统 anyio 插件与 pytest 版本不兼容影响，未作为代码失败处理。

## 2026-09-05 - 消除恢复巡检后的局部路径空窗

- 现场日志确认总控切回 `PATROL` 后，DWB 最终能够重新收到局部路径，但恢复交接期间会先
  出现空窗：waypoint 节点暂停时删除了最后一条有效巡检路径，恢复时又要等下一次周期定时器
  和 Navfn 异步规划完成，RL 在此期间只能持续发布空 `/local_path`。
- waypoint 节点现在暂停时仍对外发布空路径并停车，但在内部保留最后一条有效巡检路径；收到
  `/waypoint_sequence/resume` 后先更新时间戳并立即重发该缓存路径，再马上请求一次新的
  `ComputePathToPose`。缓存只负责短时接替，新的 Navfn 路径返回后会正常覆盖。
- 如果恢复后的第一次 Navfn 请求失败，节点保留缓存路径并按原周期继续重试，不再用该次失败
  的空结果覆盖缓存；没有有效缓存时仍按原安全行为发布空路径。
- 如果暂停前没有有效缓存路径，恢复不会使用无效数据，而是直接触发新规划。原有周期重试继续
  生效，因此 Navfn 某一轮返回空路径时，后续周期仍会重试。
- 将 waypoint 到达容差从 `1.5 m` 收紧为 `0.5 m`，避免距离最后一个巡检点仍较远时提前把
  序列判定完成并清空 `/global_path`、`/local_path`。
- 未修改 RL/PRIEST 推理、DWB 参数、目标跟踪路径、恢复位置判定或总状态机逻辑。
## 2026-09-04 - 固定外部导航链使用同一 Python 虚拟环境

- 现场表现为 `/local_path` 没有输出，历史日志同时出现 RL 节点 `torch` 导入失败和旧参数
  读取警告。检查发现主 `navigation.launch.py` 没有把控制台设置的 `DOG_PATROL_PYTHON`
  传递给 `priest_external_nav.launch.py`，外部 RL 进程可能回落到系统 `python3`。
- 主导航 launch 现在将 `python_executable` 和 `rl_python_executable` 显式传入外部导航
  launch；两者默认读取 `DOG_PATROL_PYTHON`，未设置时才使用 PATH 中的 `python3`。这样
  RL、pure pursuit、DWB adapter 和 NAV_CMD bridge 使用同一运行环境。
- 未修改 RL 推理逻辑、局部路径算法、TF 坐标系或控制参数。重新启动导航后，RL 节点必须先
  正常打印 ready；只有它正常运行且收到非空 `/global_path`、有效 `/scan` 后才会发布
  `/local_path`。

## 2026-09-04 - 目标融合增加 TF 时间滞后兜底

- 现场导航失败原因为 `map -> base_link` 的 TF 缓存落后于雷达点云时间戳，精确查询触发
  `Lookup would require extrapolation into the future`，连续 3 秒后被导航协调器上报为
  `NAVIGATION/EXECUTION_ERROR`，总控因此进入 `RECOVER_PATROL`。
- 目标融合现在仍优先按雷达时间戳精确查询 TF；仅当异常明确属于 future extrapolation
  时，改用 TF 缓存最新的 `map -> base_link` 变换继续计算目标地图位置。缺少 TF、旧时间
  外推或其他变换异常仍按原逻辑累计技术故障，不会被吞掉。
- 兜底日志每 2 秒限频，记录传感器时间领先 TF 缓存的秒数，便于继续排查 LIO/TF 发布
  延迟。未修改 `base_link`、`base_footprint`、`map` 坐标约定、雷达相机标定、3 m 停靠
  逻辑或恢复状态机。
- `adapter_path_timeout` 不属于当前 RL 节点声明的参数；当前启动链通过 `path_timeout`
  传给 DWB adapter。现场 WARN 来自旧的参数读取代码或旧运行环境，不能通过给 RL 节点
  增加一个无效兼容参数来解决。
- 验证：`python3 -m py_compile` 通过；`colcon test --packages-select
  dog_patrol_navigation` 为 23/23 通过。

## 2026-09-04 - 修复控制台子进程解释器未初始化

- 修复 `RobotMainWindow` 未定义 `python_executable`，导致任务管理器、相机和雷达等所有
  通过 `setup_process_env()` 启动的子进程在点击按钮时抛出 `AttributeError` 的问题。
- 主窗口现在优先读取 `DOG_PATROL_PYTHON`；未设置时使用启动 `app.py` 的
  `sys.executable`，保证子进程继承当前 `m20_nav` 虚拟环境。
- 删除 `TopicHzWorker` 中未使用且错误归属的同名字段；不改变传感器、任务管理器或导航
  的启动命令和业务逻辑。

## 2026-09-04 - 修复恢复巡检时重复 resume 导致局部路径延迟

- 现场表现为总控已经从 `RECOVER_PATROL` 切换到 `PATROL`，`/global_path` 已恢复，但
  `/local_path` 仍暂时为空。
- 原因是导航协调器在恢复点到达时发送一次 `/waypoint_sequence/resume`，总控切到 `PATROL`
  后又发送一次；waypoint 节点第二次 resume 会取消刚开始的异步 `ComputePathToPose` 请求，
  造成局部路径重新生成延迟。
- 导航协调器现在记录本次恢复是否已发送 resume。正常恢复完成后，`PATROL` 状态处理不再
  重复发送；恢复超时路径仍会在切回 `PATROL` 时发送一次 resume。
- 未修改 waypoint、RL 局部规划器、DWB 或路径参数，只收敛恢复交接时的消息顺序。

## 2026-09-04 - 恢复完成不依赖巡检路径

- `RECOVER_PATROL` 到达保存的巡检中断位置并稳定停车后，立即发布
  `PATROL_RECOVERY_COMPLETE`，不再等待 waypoint `active` 状态或新的非空 `/global_path`。
- 因此原巡检没有 waypoint 或当前没有巡检目标点时，也可以正常回到 `PATROL`；resume
  命令仍会发送给 waypoint 节点，但它不再是总状态机恢复完成的前置条件。
- 恢复到达判定只使用位置容差 `0.25 m` 和原有速度保持条件，不再检查朝向误差。

## 2026-09-04 - RECOVER_PATROL 返回巡检中断位姿

- 进入目标接近阶段时，导航协调器保存当前 `map -> base_footprint` 的位置和朝向，作为
  本次目标任务打断巡检时的恢复断点；waypoint 当前索引仍由原 waypoint 节点保留。
- `RECOVER_PATROL` 不再立即恢复 waypoint。导航先清空目标路径、保持 waypoint 暂停，使用
  现有 `ComputePathToPose` 规划器生成返回中断位姿的路径，并继续通过现有
  `/global_path` -> DWB/RL 控制链执行。
- 只有位置误差不超过 `0.25 m`，且线速度/角速度连续保持在原到达阈值内后，才清空返回
  路径并发布 `/waypoint_sequence/resume`，随后立即发布 `PATROL_RECOVERY_COMPLETE`。
- 增加参数 `motion.recovery_position_tolerance`，不再判断恢复时的朝向。如果进入接近阶段
  时暂时无法获取机器人 TF，恢复时
  记录警告并直接恢复 waypoint，避免把系统永久卡在恢复状态；恢复总超时仍由 supervisor
  直接回到 `PATROL`。

## 2026-09-04 - 导航协调器改为总控状态直接驱动

- 删除导航侧 `NavigationStateMachine` 和持久化导航模式，改为无状态
  `navigation_policy()`；总控 `/mission/state` 成为导航行为的唯一状态来源。
- 保持已调试的目标融合、雷达测距、3 m 停靠、0.5 秒动态目标重规划、Nav2 路径发布和
  DWB 控制链不变；只重构状态进入动作和运行条件。
- `CONFIRM_TARGET` 继续原 waypoint 巡检并后台定位目标；`APPROACH_TARGET` 才暂停巡检、
  清除巡检路径并切换到目标路径；`VERIFY_IDENTITY` 保持停车；`TRACK_INTRUDER` 持续跟踪。
- `RECOVER_PATROL` 清理目标上下文、恢复原 waypoint 并等待新巡检路径；新路径到达后上报
  `PATROL_RECOVERY_COMPLETE`，恢复超时仍由 supervisor 直接返回 `PATROL`。

## 2026-09-04 - 取消无目标执行错误对总任务的永久阻塞

- `EXECUTION_ERROR` 发生在活动目标任务中时仍进入 `RECOVER_PATROL`，恢复原 waypoint
  巡检；恢复阶段重复错误仍不创建新的错误分支。
- `target_id=0` 时的感知或导航技术错误改为诊断事件：总状态、`state_seq` 和巡检断点均
  保持不变，`blocked` 保持 `false`，不再要求调用 `/mission/reset` 才能继续。
- 导航缺少有效 TF、目标或路径时仍可在执行层清空无效路径，避免复用旧控制命令；这只是
  局部安全动作，不再把全局任务锁死。
- 同步更新总控状态机测试与感知导航接口文档；未提交、未 push。

## 2026-09-03 - 按现场日志放宽目标短时丢失与 bbox 新鲜度窗口

- 现场依据：感知链稳定以 10 Hz 处理和预览，但总控日志中同一目标多次在
  `TARGET_LOST` 后约 `0.095-0.20 s` 即 `TARGET_REACQUIRED`，说明 0.5 秒任务级丢失门限
  会把短暂的可信框空洞升级成全局阻塞；导航日志同时多次出现 bbox age
  `0.506-0.508 s`，仅略高于原 `max_bbox_age=0.50 s`。
- 感知：将生产 YAML、资产运行 YAML 和无参数 C++ 默认值中的
  `target.lost_event_timeout_sec` 从 `0.5 s` 调整为 `1.0 s`。目标需要连续 1 秒没有可信框
  才发布 `TARGET_LOST`，减少快速 lost/reacquired 导致的路径取消和重新捕获。
- 导航：将 `fusion.max_bbox_age` 从 `0.50 s` 调整为 `0.80 s`，将
  `fusion.target_timeout` 从 `0.60 s` 调整为 `0.90 s`，覆盖当前约 0.31 秒推理发布延迟和
  偶发调度抖动；超过目标保持时间后仍清空目标路径并停车。
- 精度边界：`fusion.sync_tolerance` 保持 `0.12 s`，没有允许旧 bbox 与错误时刻的雷达
  点云配对；`min_cluster_points=3`、ROI、聚类和 3 m 停止参数均未修改。

## 2026-09-03 - 将 MID360 点云发布频率恢复为 10 Hz

- 确认相机由独立硬件同步器通过 `Line0` 以约 10 Hz 触发；Livox 驱动的
  `publish_freq` 只控制 ROS 点云分包频率，不会改变相机硬件触发频率。
- 将 APP 使用的 `msg_MID360_launch.py` 从 `20.0 Hz` 恢复为 `10.0 Hz`，使单帧点云
  密度和相机输入周期回到迁移前配置，也避免 20 Hz 分包导致目标 bbox 内点数减少。
- 相机触发配置、其他 Livox 示例 launch、导航协调器和目标停车逻辑均未修改；传感器驱动
  需要重启后生效；未提交、未 push。

## 2026-09-03 - 修复进入 3 米范围后仍沿旧路径继续前进

- 现场日志确认测距和判定本身已经生效：`sensor_distance=3.21 m` 时尚未停车，降到
  `2.51 m` 时已经是 `stop=True`，但机器人随后仍前进到约 `1.1 m`。因此本次问题不是
  `approach_distance=3.0 m` 没有生效，而是停止命令在导航链路中的传播延迟。
- 原链路只由协调器发布空 `/global_path`，再等待 RL 节点清空 `/local_path`、adapter 取消
  FollowPath。已有的局部路径和异步 action 在这段时间仍可继续产生非零 `/NAV_CMD`。
- 修复后，协调器发布空 `/global_path` 的同时，通过 ROS 2 action 标准服务
  `/follow_path/_action/cancel_goal` 直接取消所有当前 FollowPath 目标。真正的零速度仍由
  `controller_server -> /cmd_vel -> adapter -> /NAV_CMD` 原链路产生，协调器不会成为第二个
  `/NAV_CMD` 发布者。
- 停车保持阶段按现有 `stop_republish_period=0.20 s` 重发空路径和取消请求，可处理某个异步
  FollowPath 目标恰好在第一次取消之后才被接受的竞争情况。正常巡逻、目标移动时的 0.5 s
  重规划、DWB 参数、RL 和 adapter 代码均未修改。
- 新增 `controller.action_name=/follow_path` 参数和 `action_msgs` 运行依赖；未提交、未 push。

## 2026-09-03 - 将 APP 使用的 MID360 点云发布频率调整为 20 Hz

- APP 的传感器按钮实际调用 `livox_ros_driver2 msg_MID360_launch.py`，该入口原
  `publish_freq=10.0`，现调整为 `20.0`，使 `/livox/lidar` 以约 20 Hz 聚合发布点云。
- 没有修改其他 HAP、mixed 或 RViz 示例 launch，避免影响 APP 不使用的驱动入口。
- 没有修改 `TopicHzWorker` 的 `window=10`；该数值是 `ros2 topic hz --window` 的统计
  样本窗口，不是雷达目标频率。
- `/scan` 的 `scan_time=0.1` 和 pointcloud_to_laserscan 参数本批不改；目标 bbox 融合直接
  消费 `/livox/lidar`，因此可直接获得 20 Hz 输入。
- 驱动需停止并重新启动后新频率才会生效；未提交、未 push。

## 2026-09-03 - 修复目标距离缩短与融合缓存崩溃

- 现场日志：同一目标雷达估计距离约 `7.44 m`，转换到地图后机器人到目标的平面距离却只有
  `3.92 m`；随后协调器因 `UnboundLocalError: estimate referenced before assignment`
  崩溃。
- 崩溃根因：同一个 bbox/点云 pair 已经完成估计后，协调器会复用缓存点继续做 TF 转换，
  但诊断日志仍读取只在首次估计分支内定义的局部变量 `estimate`。现在将 range、ROI 点数和
  cluster 点数一并缓存，复用路径不再访问未绑定变量。
- 距离根因：设备参数名 `T_lidar_base` 与实际组合方向不一致。原 `odom_bridge` 将其作为
  `T_body_base` 右乘到 body 位姿，矩阵的逆也与系统 `base_link -> livox_frame` 的 TF 数值
  对应；协调器此前直接乘 `livox_frame` 点，导致约 7 m 的目标被错误转换成约 3.7 m 水平
  距离和约 5.4 m 高度。
- 修复：不修改 FAST-LIVO 或设备 YAML；协调器读取设备矩阵后取逆，得到真正用于目标点的
  `livox_frame -> base_link`。代码内备用矩阵同步改为该逆矩阵。
- 目标行为：新增 `planning_goal_distance=1.0 m`，Nav2 规划终点放到人前约 1 m，避免终点
  正好落在人占据的 costmap 单元；停车仍由实时雷达目标在 `base_link` 下的水平距离控制，
  到 `3.0 + 0.1 m` 内立即清空路径。目标移动后仍按 0.5 s 周期或 0.25 m 位移更新方向。
- 诊断：`target measurement` 增加 `base_planar` 和 `base_z`；`target standoff` 同时打印
  `sensor_distance` 与 `map_distance`，可直接发现传感器外参或 TF 不一致。
- 边界：没有修改相机内参、雷达-相机外参、FAST-LIVO、Nav2/DWB 参数或总状态机；未提交、
  未 push。

## 2026-09-03 - 收紧目标 3 米停车并保持移动目标跟踪

- 现象：目标导航有时在超过 3 m 时就停止，且需要确认总状态切换到跟踪状态后，移动目标
  才会继续产生新的跟踪方向。
- 原因：原 `arrival_distance_tolerance=0.25 m` 的停止判据是
  `planar_distance <= 3.0 + 0.25 m`，因此允许机器人在 3.25 m 处停止；接近状态完成后，
  `APPROACH_TARGET` 会按流程停车等待认证，持续跟踪由总状态 `TRACK_INTRUDER` 负责。
- 参数：将 `arrival_distance_tolerance` 调为 `0.10 m`，目标期望距离和跟踪距离仍都是
  `3.0 m`，到达后仍需保持低速 `0.50 s` 才发布到达事件。
- 行为：协调器继续使用最新 bbox 融合出的地图目标位置计算 standoff goal。目标距离大于
  3.10 m 时，目标点移动或达到重规划周期就重新生成“目标前方 3 m”路径；进入
  `TRACK_INTRUDER` 后，机器人在 3 m 附近停止，目标移动超过更新阈值后会跟随目标方向。
- 坐标与内参：本批没有修改相机内参、雷达-相机外参、雷达-机体外参或坐标系；停止距离
  使用地图坐标下机器人与融合目标的水平距离，雷达相机欧氏距离仅用于观测诊断。
- 验证：同步更新导航协调器默认值和 YAML；更新 standoff 单测；未提交、未 push。

## 2026-09-03 - 修复目标融合稀疏点误报技术故障并增加距离诊断

- 现象：已经收到目标 bbox，但状态显示
  `NAVIGATION/EXECUTION_ERROR: not enough projected lidar points inside bbox ROI`；另有
  目标距离看起来超过 3 m 却提前停止的疑问。
- 距离边界：目标框融合使用 `fusion.min_range=0.40 m`、`fusion.max_range=15.0 m`，距离是
  Livox 点变换到相机坐标后的欧氏距离，并且点还必须在相机前方、投影到 bbox 内。独立的
  `pointcloud_to_laserscan` 使用 `range_max=10.0 m`，只影响 `/scan`、local/global costmap
  和 DWB，不限制 `/livox/lidar` 到目标框的融合。
- 融合鲁棒性：主 ROI 点数不足时自动使用较宽的 fallback ROI（水平 0.05、顶部 0.05、底部
  仍裁掉 0.20），保留对地面回波的抑制；多个深度簇同时存在时，要求候选簇至少达到最大
  有效簇的 50% 支持度，避免少量近处墙面/噪声点抢走人的距离。
- 状态机行为：`TargetObservationUnavailable` 被视为当前 bbox 的可恢复观测缺口，不再在
  连续 3 秒内直接升级为 `EXECUTION_ERROR`；仍会发布空路径等待下一帧 bbox/点云。TF、时间
  同步、无效 bbox 和 planner 故障仍按原技术故障路径处理。
- 参数一致性：协调器代码默认 `fusion.max_bbox_age` 与 YAML 统一为 `0.50 s`，避免单独
  启动节点时回退到 0.30 s 导致约 0.31 s 的感知处理延迟被误判为过期。
- 诊断日志：新增 `target measurement`，打印雷达估计距离、ROI 点数和深度簇点数；新增
  `target standoff`，打印机器人到目标的实际地图平面距离、目标期望距离和停车判据。前者
  是测距值，后者才是 3 m 接近/停止逻辑使用的距离，并且当前停车容差仍为 0.25 m。
- 验证：目标估计器和点云融合 mock 测试共 11 项通过；Python 语法检查通过；
  `dog_patrol_navigation` 已完成 symlink 编译；未提交、未 push。

## 2026-09-03 - 修复目标路径不发布并补充 RViz 目标显示

- 现象：目标流程已经进入 `APPROACH_TARGET`，但 RViz 没有 `/global_path`，也看不到
  目标人物位置和 3 m 临时导航目标。
- 日志结论：感知 bbox 到达协调器时的实际年龄稳定在 `0.309~0.315 s`，超过原来的
  `fusion.max_bbox_age=0.30 s`，导致后续框全部被拒绝；融合目标在 `0.60 s` 后失效，
  协调器按安全策略持续发布空路径。
- 参数：将 `max_bbox_age` 调为 `0.50 s`，只放宽图像采集到 bbox 发布的端到端年龄；
  相机 bbox 与 Livox 点云的配对误差仍严格使用 `sync_tolerance=0.12 s`，没有放宽
  传感器同步要求。
- 可视化：在实际 2D 定位导航使用的 `localization_2d.rviz` 中增加
  `/navigation/target_point`（目标人物地图位置）和 `/navigation/target_goal`（目标前
  3 m 的 planner goal）显示；原 `/global_path` 显示保持不变。
- 边界：没有修改 3 m 停止距离、到达判断、目标重规划周期、Nav2 planner 或状态机。

## 2026-09-03 - 修复 Livox 混合字段点云导致的目标融合故障

- 现象：检测到目标后任务进入 `CONFIRM_TARGET`，导航协调器上报
  `All fields need to have the same datatype`，总状态显示
  `NAVIGATION/EXECUTION_ERROR`，因此不会产生目标地图位置或目标路径。
- 根因：Humble 的 `sensor_msgs_py.read_points_numpy()` 会先检查整条
  `PointCloud2` 的全部字段，而 `/livox/lidar` 同时包含 `float32` 的 XYZ、`uint8` 的
  `tag/line` 和 `float64` 的 `timestamp`。即使调用时只指定 XYZ，仍会因其他字段类型
  不同而触发断言。
- 修复：改用 `read_points()` 只读取结构化 XYZ 字段，再以 NumPy 向量化方式组合成
  `N x 3 float32` 数组并显式过滤 NaN/Inf；不再使用不兼容的
  `read_points_numpy()`，也不引入逐点 Python 循环。
- 边界：没有修改 bbox、相机内外参、前景深度聚类、地图坐标变换、3 m 停止逻辑、
  planner 或任务状态机。
- 验证：使用与实时 Livox 相同的混合字段布局构造 PointCloud2，确认能够提取 XYZ 并
  删除非有限点；Python 语法检查通过，`dog_patrol_navigation` 已完成 symlink 编译。

## 2026-09-03 - 修正 ROS 图像输入的相机坐标系语义

- 目标：让 tracking 消费 fast_livo_dog 的 `/left_camera/image_raw` 时，检测框携带的来源
  坐标系与导航侧雷达投影标定严格一致，避免仍使用旧的 `hik_camera_optical_frame`。
- 完成：ROS Image 输入帧保存 `sensor_msgs/Image.header.frame_id`，生成目标 observation、
  `/perception/selected_target_bbox` 和目标 crop 时优先沿用该 frame；只有图像 header 为空时
  才回退到 `perception.camera_optical_frame_id`。
- 配置：tracking 默认回退 frame、仓库运行资产中的 tracking 配置均统一为
  `camera_link`；输入图像继续保持 `1280x1024`，检测网络内部 `640x640` 推理不改变 bbox
  的原图像素坐标语义。
- 边界：没有修改 fast_livo_dog 相机驱动或感知算法；相机驱动仍负责发布带正确 header 的
  `/left_camera/image_raw`，tracking 只消费该 topic 并传递来源元数据。

## 2026-09-03 - 增加完整感知任务统一 bringup

- 目标：按照 `startup_commands.md` 中已经验证过的启动参数，将原来需要多个终端分别运行的
  tracking、人脸、语音、readiness 和 authorization 收口为一个感知启动入口，同时不修改
  各感知包原有 launch。
- 新增：独立 `dog_patrol_perception_bringup` 包及 `perception_stack.launch.py`。该 launch
  启动 mission 模式 tracking、face launch、voice launch、`perception_readiness` 和
  `perception_authorization`，并统一连接 `/mission/state`、`/mission/event`、能力状态、
  目标 bbox、目标 crop 和授权 evidence/command topic。
- Tracking：组合启动固定使用 `camera.input_mode:=ros_image`，默认订阅
  `/left_camera/image_raw`，默认 frame 为 `camera_link`；支持 `preview` 和 `record` 开关，
  默认打开监视窗口、关闭录制。
- 资产：模型、白名单及运行 YAML 默认从仓库内
  `src/perception/dog_patrol_perception_assets_20260813` 解析；可通过
  `DOG_PATROL_ASSETS_ROOT` 或 launch 的 `assets_root` 覆盖，不再要求外部资产目录。
- UI：总控的“启动感知任务”按钮调用该统一 launch，并传入当前仓库资产目录和导航相机
  原图 topic；“停止感知任务”统一关闭 tracking、face、voice 和两个 orchestrator 节点。
- 边界：该 bringup 只编排已有感知节点，不启动相机/雷达驱动、导航栈或全局 mission
  supervisor；这些进程仍由 APP 对应按钮和常驻总控分别管理。

## 2026-09-03 - 明确 APP 的 ROS Domain 与 Fast DDS 生效范围

- 当前行为：`robot_console/app.py` 在创建 ROS 后台和所有 UI 子进程前设置
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；从当前导航设备配置
  `config/device_parameters.yaml` 读取 `ros_domain_id`，读取失败时使用 `42`，当前设备配置
  也是 `42`。
- Fast DDS：APP 在配置文件存在时使用导航目录下的 `config/fastdds_udp_only.xml`，由 APP
  拉起的传感器、定位、导航、任务管理和感知进程继承同一通信环境。该 XML 是当前 Orin
  部署规避 Fast DDS SHM 锁和 Python/C++ 互操作问题的运行配置，不属于任务状态机协议。
- 使用边界：只保证从 APP 启动的整套进程自动处于正确 Domain；手动执行 `ros2 run`、
  `ros2 launch` 或调试命令时不在 launch 内强制 Domain/Fast DDS，是否设置相关环境变量由
  启动该命令的终端负责。
- 本批决定：不再向各感知 launch 重复注入或强制修改 Domain/Fast DDS，也没有保留额外的
  reset 子进程环境扩展，避免把部署环境策略散落到业务模块。

## 2026-09-02 - 精简导航模块总览文档

- 更新 `src/navigation/README.md`，改为简洁的导航入口说明，保留当前 fast_livo_dog
  目录结构、app.py 总入口、导航数据链、关键坐标系和配置文件位置。
- 删除总览中容易与当前 app 管理方式混淆的冗长手动启动说明，明确
  `navigation.launch.py` 已包含 Nav2、move 外部控制链和导航任务协调器，不应重复启动
  `navigation_mission_coordinator`。
- 增加当前导航集成文档和原 FAST-LIVO-DOG README 的链接；本批只修改文档，没有改动
  导航、感知或控制代码，也没有提交或 push。

## 2026-09-02 - 增加导航协调器 mock 感知目标测试入口

- 目标：解决单独启动 tracking 时没有 `/perception/selected_target_bbox`，以及无法区分
  感知门禁、bbox-雷达融合、TF、planner 和总控状态机问题的测试困难。
- 新增：`tools/navigation_coordinator/mock_target_publisher.py`。该脚本只模拟感知端，
  按真实 `/mission/state` 的 `state_seq` 发布 `SOURCE_PERCEPTION/TARGET_CONFIRMED`，
  并在目标相关状态下以当前 ROS 时间持续发布 `TargetBoundingBox`；不伪造雷达、TF、
  Nav2 action、目标位置、路径或速度。
- 文档：在 `tools/fake_integration/README.md` 增加四级测试方法：真实协调器的 bbox/目标
  地图位置测试、真实目标路径和到达停止测试、已有 fake integration 的总控/感知状态交互
  测试，以及真实 tracking 的 mission 模式测试；同时说明 `run_fake_integration.py` 的
  fake navigation 不覆盖真实协调器。
- 运行约束：mock bbox 默认使用 `1280x1024`、`camera_link` 和实时戳；实际测试时 bbox
  像素区域必须覆盖相机中目标对应的雷达投影。真实 tracking 和 mock bbox 不能同时发布
  同一个 topic。
- 边界：本批没有修改导航协调器、感知算法或状态机逻辑，没有提交和 push。
- 验证：mock 脚本 Python 语法、相关文档尾随空白和 `git diff --check` 检查通过。

## 2026-09-02 - 增加无身份识别的导航跟踪闭环测试说明

- 目标：支持只运行真实人像检测/跟踪，不启动人脸和语音模型，验证目标确认、雷达测距、
  3 m 接近、持续跟踪以及处置结束后回到巡检状态。
- 新增：`tools/navigation_coordinator/mock_mission_events.py` 只模拟最终业务事件：在真实
  导航到达并进入 `VERIFY_IDENTITY` 后发布 `UNAUTHORIZED`，可在真实
  `TRACK_INTRUDER` 持续指定时间后发布操作员来源的 `HANDLING_COMPLETE`；不发布 bbox，
  不和真实 tracking 抢占 `/perception/selected_target_bbox`。
- 测试说明：补充只启用人像检测的完整命令链；通过 `fake_nodes.py --role capabilities` 只
  补齐 face/voice READY，不运行任何 face/voice 算法；明确 `run_fake_integration.py` 的
  fake navigation 不验证真实协调器距离和路径。
- 边界：本批没有修改生产状态机、tracking 或导航协调器逻辑，没有提交和 push。
- 验证：两个 mock 脚本 Python 语法、文档尾随空白和 `git diff --check` 检查通过。

## 2026-09-02 - 完善导航协调器集成文档并迁入 FAST-LIVO-DOG 原始说明

- 目标：把当前 `dog_patrol` 导航协调器的真实运行逻辑、状态行为、接口契约、坐标系、
  标定方向、目标融合、standoff 路径、停止条件、参数和故障排查完整记录，避免后续
  只看零散代码或旧仓 README 造成误启动、重复启动和坐标系误用。
- 文档：重写 `src/navigation/fast_livo_dog/navigation/dog_patrol_navigation/docs/current_workspace_integration.md`，
  增加系统边界、四层数据流、节点启动所有权、全局状态到导航内部模式的逐状态说明、
  READY/阻塞/reset 行为、bbox 与点云融合步骤、`T_camera_lidar`/`T_lidar_base` 方向、
  3 m 目标计算、planner action 语义、到达判定和所有当前 YAML 参数表。
- 启动说明：明确当前总入口为 `src/orchestration/robot_console/robot_console/app.py`，
  导航主入口为 `move/launch/navigation.launch.py`，并记录 Nav2、外部控制链和任务
  协调器的 8/11/12 秒启动时序及 `strict_ready_checks` 的实际影响。
- 原始文档迁移：将原 `/home/orin/workspace/fast_livo_dog/src/DeepRobotics_ws/README.md`
  复制到 `src/navigation/fast_livo_dog/README.md`，保留原 FAST-LIVO-DOG 依赖、建图、
  定位、导航和第三方库说明；在顶部增加当前 dog_patrol 路径提示。dbow3、livox_ros_driver2
  和 rpg_vikit 的原 README 已存在于迁入目录，并逐项校验哈希与原文一致。
- 边界：本批只增加和完善文档，没有修改导航协调器、Nav2、move、感知或控制代码，未提交、
  未 push，也没有删除原 fast_livo_dog 工作区文件。
- 验证：确认当前集成文档 576 行、迁入的根 README 878 行；三个子包 README 与原文件
  SHA-256 完全一致；当前新增文件状态和路径符合预期。

## 2026-09-02 - 对齐 fast_livo_dog 导航接口并修正目标外参方向

- 目标：确认 `dog_patrol_navigation` 使用的建图、定位和导航接口严格匹配 fast_livo_dog 的实际运行链，而不是只匹配配置文件中的名称。
- 接口核对：确认协调器使用 `/livox/lidar`、`/odom`、`/global_path`、`/compute_path_to_pose` 和 `/NAV_CMD`；对应 frame 为 `map`、`camera_init_footprint`、`base_footprint`、`base_link`、`livox_frame` 和相机原图 `camera_link`。这些与 fast_livo_dog 的 Livox、MVS、odom_bridge、Nav2 planner 和 move/DWB 链一致。
- 当时的外参判断：该批曾按参数命名将 `T_lidar_base` 视为
  `livox_frame -> base_link` 并直接使用。2026-09-03 的现场距离日志和原 `odom_bridge`
  矩阵组合证明这个判断不成立，后续“修复目标距离缩短与融合缓存崩溃”已改为取逆；以
  后续记录和当前代码为准。
- 路径职责：协调器只负责目标框与雷达点云融合、目标地图位置、目标 standoff 路径和目标导航事件；`global_path` 的跟踪仍由原 move/pure pursuit/RL/DWB/NAV_CMD 链负责，协调器不接管速度控制。
- 接口边界：`/perception/selected_target_bbox`、`/navigation/target_status`、`/navigation/target_point`、`/navigation/target_goal` 和 `/mission/*` 是 dog_patrol 任务集成层接口；它们不替换 fast_livo_dog 原有的传感器、定位或控制 topic。
- 文档：当时记录的 `T_lidar_base` 方向已由后续现场验证纠正，保留此项仅作为变更历史。
- 验证：确认原 fast_livo_dog 与迁入版本的设备参数一致；协调器源码无不存在的定位置信度接口；Python 语法和 `git diff --check` 通过；`dog_patrol_navigation` 重新编译成功。当前 Python 环境的 pytest 入口被系统 `anyio` 插件版本冲突阻断，未能运行 pytest 单测。

## 2026-09-02 - 移除导航协调器中不存在的定位置信度门禁

- 目标：`open3d_loc` 当前没有稳定发布 `/localization_3d_confidence`，导航协调器不应保留一个默认关闭且实际无效的定位置信度配置。
- 完成：从 `navigation_mission_coordinator` 删除 `Float32` 置信度订阅、缓存、参数声明、读取逻辑和 readiness 门禁；从 `m20_patrol_navigation.yaml` 删除 `require_localization_confidence`、`localization_confidence_topic` 和 `localization_min_confidence`。
- 当前行为：导航协调器只使用 `map -> base_footprint` TF、激光点云新鲜度、`ComputePathToPose` 服务和 `/global_path` 消费者检查导航 ready，不再等待不存在的定位置信度话题。
- 边界：DWB adapter 中同名的兼容参数仍保留，属于控制器自身的另一条配置链，本次没有误删。
- 验证：导航协调器源码和 YAML 中无定位置信度相关引用；Python 语法、`git diff --check` 和 `dog_patrol_navigation` 编译均通过。

## 2026-09-02 - 将感知运行资源迁入主仓感知模块

- 目标：不再依赖工作空间外的 `/home/orin/dog_patrol_perception_assets_20260813`，让感知模型和运行配置随 `dog_patrol` 的感知模块管理。
- 完成：将完整资源包迁移到 `src/perception/dog_patrol_perception_assets_20260813/`，保留 `face/`、`tracking/`、`voice/`、`runtime/`、白名单、TensorRT engine 和 Vosk 模型目录；删除外部旧资源目录。
- 配置：统一更新人脸 detector/recognition/whitelist、YOLO TensorRT engine 和 BotSort 配置路径；同步更新 `SHA256SUMS`，避免资源校验清单与配置内容不一致。
- 入口：`robot_console/app.py` 和 `main_window.py` 默认从仓库内 `src/perception` 计算资源路径；`dog_patrol_perception_bringup` 支持通过 `DOG_PATROL_ASSETS_ROOT` 或 `assets_root:=...` 覆盖默认位置；感知及导航文档同步更新。
- 关键结论：资源目录约 114 MB，模型和白名单均已在新位置找到；engine 文件继续遵循仓库现有 `.gitignore` 规则，不会因本次迁移自动进入 Git 提交。
- 验证：感知资源完整性检查通过；`SHA256SUMS` 校验通过；感知 launch `--show-args` 解析通过；`dog_patrol_perception_bringup` 和 `robot_console` 编译通过；源码中不再引用已删除的外部资源目录。
- 启动：进入 `/home/orin/workspace/dog_patrol` 后，source ROS 和工作空间即可直接运行 `python3 src/orchestration/robot_console/robot_console/app.py`，无需手动创建外部资源目录。

## 2026-09-02 - 迁入 fast_livo_dog 导航链和运行数据

- 目标：让 `dog_patrol` 内的导航模块可以独立于原 `/home/orin/workspace/fast_livo_dog` 运行，并统一使用主仓的导航坐标系、配置和数据目录。
- 完成：将 FAST-LIVO 前端、mapping、2D/3D localization、Livox/MVS 驱动、Nav2/move、DWB 控制和导航任务协调器迁入 `src/navigation/fast_livo_dog/`；同时迁入原导航 `data/` 目录，包括 2D/3D 地图、位姿、ScanContext/DBoW3/视觉定位数据及原始扫描图像。
- 路径：将 frontend、mapping、2D/3D localization、导航 launch、`priest_rl_publisher_nav_cmd.py` 和相关 C++ 默认路径改为 `DOG_PATROL_NAV_ROOT` 或当前仓库的 `src/navigation/fast_livo_dog`；地图、ArUco、ScanContext、视觉词袋和回环保存路径统一指向该目录下的 `data/`。
- 标定：导航协调器读取 `config/device_parameters.yaml` 中的相机内参、畸变、雷达到相机和雷达到 base 的标定；目标框投影和导航定位不再引用旧工作空间配置。
- 生命周期：`move/navigation.launch.py` 负责启动导航侧控制链和 `navigation_mission_coordinator`；不再单独重复启动协调器，避免重复节点和重复发布者。
- 保留项：Livox 回放工具的 `/home/livox/livox_test.lvx` 和仿真 checkpoint 的固定路径属于回放/仿真运行输入，没有误改成地图路径；旧导航工作空间路径在生产源码中已清理。
- 验证：迁入后的 dbow3、fast_gicp、scancontext、vikit、livox、fast_livo、localization、mapping、dog_patrol_navigation 和 move 均完成编译；迁入的 `data/` 与原 fast_livo_dog 数据逐项一致。

## 2026-09-02 - 接入系统总控 UI 和任务状态机复位

- 目标：把原 fast_livo_dog 的 `robot_console/app.py` 作为 dog_patrol 的系统入口，同时让导航和感知可以分别启停，避免每次关闭模块后遗留旧目标或旧任务状态。
- 完成：新增 `src/orchestration/robot_console/` 包，保留原 Qt 控制台的传感器、建图、定位、导航和感知按钮；UI 根据当前仓库计算 `DOG_PATROL_NAV_ROOT` 和 `DOG_PATROL_ASSETS_ROOT`，不再依赖原 fast_livo_dog 工作空间路径。
- 生命周期：app 启动时只启动一个常驻 `mission_supervisor`；2D 导航按钮管理导航/控制链和导航协调器；感知任务按钮管理 tracking、face、voice 和感知编排。导航与感知可以独立开关，但两侧都 READY 后全局状态机才会从 STARTUP 进入 PATROL。
- 状态复位：`MissionSupervisor` 新增 `/mission/reset`（`std_srvs/srv/Trigger`）；复位时清除当前目标、阻塞原因、感知/导航 ready 标志、已处理事件和任务会话序号，回到 STARTUP。UI 停止导航或感知时调用该服务，manager 进程本身只在 app 退出时关闭。
- 测试：增加状态机 reset 回归测试，确认复位会清除 active target、blocked 和 readiness，并递增 `state_seq`，避免旧事件重新推动新会话。
- 关键约束：不要同时手动启动两份 manager、导航 launch 或感知 launch；不要单独再启动 `navigation_mission_coordinator.launch.py`，否则会产生重复节点和重复发布。
- 验证：`robot_console`、`dog_patrol_manager` 和感知启动包编译通过；Python 语法及 launch 参数解析通过；状态机复位测试已加入现有测试集。

## 2026-09-02 - tracking 接入导航侧 ROS 图像输入

- 目标：感知 tracking 不再强制独占 Hikrobot MVS 相机，而是能够直接消费 fast_livo_dog 发布的导航相机图像，与导航使用同一相机数据源。
- 完成：tracking 节点新增 `camera.input_mode`（`mvs`/`ros_image`）和 `camera.image_topic` 参数；`ros_image` 模式订阅 `/left_camera/image_raw`，使用 `cv_bridge` 转换为 BGR8，后续检测、跟踪、目标 crop、预览和 mission 输出继续复用原处理链。
- 实时性：ROS 图像输入采用有界 latest-only 队列，回调只保留最新帧，推理跟不上时丢弃旧帧，避免队列积压造成延迟；图像 `Header.stamp` 作为 source timestamp 传给目标框、crop 和 mission frame transaction。
- 分辨率与坐标：tracking 输入默认保持 1280x1024；检测网络内部仍使用 640x640，不改变输入图像的 bbox 坐标语义。`/left_camera/image_raw` 的 frame 使用图像 header，默认配置统一为 `camera_link`；不要把 640x512 的内部图像 topic 当作目标框投影输入。
- 兼容性：默认 `camera.input_mode:=mvs` 不变，原 standalone MVS 模式和 `capture_ffv1` 工具不受影响；集成启动时由 perception bringup 显式覆盖为 `ros_image`，并绑定导航相机 topic。
- 依赖：tracking 的 CMake 和 package manifest 增加 `cv_bridge`、`sensor_msgs`；README 增加 ROS 图像模式启动示例和丢帧语义说明。
- 验证：tracking Orin runtime 编译通过；感知组合 launch 可解析；输入模式、图像 topic 和相机 frame 参数均已纳入配置。

## 2026-09-02 - 同步 fake integration 的外部图像联调入口

- 目标：在真实导航相机尚未完全接入总流程时，仍能用 fake navigation 验证感知状态机和授权流程，同时测试 tracking 的 ROS 图像输入。
- 完成：`tools/fake_integration/run_fake_integration.py` 新增 `--camera-input-mode` 和 `--image-topic`；选择 `ros_image` 时，将 `camera.input_mode` 与 `camera.image_topic` 传给真实 tracking 节点。默认仍为 `mvs`，不会改变原有联调行为。
- 文档：补充外部相机驱动、`/left_camera/image_raw`、tracking 参数、BotSort、人脸、语音模型和预览开关的完整命令示例，并明确不能同时打开 tracking 内部 MVS 输入。
- 验证：fake integration Python 入口和 tracking ROS image 配置的语法/参数检查通过；`git diff --check` 通过。本批只修改联调入口和文档，没有修改生产状态机逻辑。

## 2026-08-13 12:30 - 完成真实感知三流程现场验收

- 目标：接手并推进 R818、显示器和 Hik 相机已接入后的真实感知整流程现场验收。
- 完成：确认 `DISPLAY=:1`、R818 ADB、Hik MV-CU013-A0UC（序列号 `DB0060477`）、USB 音频和真实 face/voice 运行时可用；以 `/tmp` 临时部署 YAML 物化 tracking detector engine、相机序列号及 face engine/recognition/whitelist 路径；统一环境检查 PASS。现场运行真实 supervisor、tracking、face、voice、readiness、orchestrator，仅 fake navigation；`initial_face_pass`、`dual_pass`、`dual_reject` 全部通过。显示器目检确认只有一个 tracking 预览窗口，tracking 框稳定，人脸框叠加在同一窗口，偶尔短暂消失符合 face 新鲜度/质量门禁。
- 关键结论：当前人员不在原部署白名单；经用户授权，从真实 tracking 内存 crop 生成 8 个质量合格 embedding 到 `/tmp` 临时白名单，未保存原始图像、未覆盖部署白名单。`initial_face_pass` 仅产生 1 条初始 face evidence，约 0.92 秒直接授权，voice provider 日志为 0 字节；`dual_pass` 初始 face 拒绝、第一轮 face 拒绝、voice 通过，总耗时约 21.22 秒；`dual_reject` 初始 face、两轮 face/voice 均拒绝，最终进入 `TRACK_INTRUDER`，总耗时约 72.88 秒。三次 `functional.status=PASS` 且性能采集完整；tracking 约 28.5-28.9 FPS，face 稳态平均推理约 16.5 ms，GPU 峰值 99%，Tj 峰值 56.4°C。`ros2 run` 包装入口在 SIGINT 后报告段错误，但直接运行 tracking 二进制在 GDB 下正常退出，暂记为 ROS 包装退出残余风险，不影响功能验收。
- 涉及文件：`worklog.md`；现场报告位于 `data/diagnostics/fake_integration/20260813_115805/`、`data/diagnostics/fake_integration/20260813_115935/` 和 `data/diagnostics/fake_integration/20260813_122856/`；临时配置及白名单位于 `/tmp`，不进入 Git。
- 验证：统一 perception environment PASS；`initial_face_pass`、`dual_pass`、`dual_reject` 自动功能断言 PASS；真实 face 临时白名单 preflight READY；GDB 直接 tracking 正常退出；显示器人工目检通过。
- 后续：完成提交前 Standards/Spec 双轴复核，确认无阻塞问题后将本轮必要源码、测试和文档整理为一个本地提交，不推送；报告、临时 YAML和临时白名单均不得提交。

## 2026-08-13 11:19 - 收敛真实人脸接入与感知整流程验收入口

- 目标：检查已迁入的人脸算法是否破坏原代码，按已确认流程完成必要接入：初始仅人脸，失败后两轮人脸/语音并行；人脸 bbox 叠加到既有 tracking 预览；最终只 fake 导航并记录整条感知流程性能。
- 完成：将授权规则提炼为分阶段纯 Module，并新增 `AuthorizationCommand`、带 stage 的 `AuthorizationEvidence` 与 `FaceOverlay`；face/voice provider 改为 MissionState 门禁加命令驱动，支持替换/取消和迟到结果抑制；face crop 改为严格 latest-only，补齐实际推理 readiness、CUDA 资源关闭和指标；bbox 以原 crop 来源元数据回传，在同一 `VisualizerRecorder` canvas 绘制并抑制错目标、未来帧和过期结果；最终联调脚本改为 supervisor/tracking/face/voice/readiness/orchestrator 全真实、仅导航 fake，增加三种内部流程断言及阶段/系统资源报告；清理独立 face GUI 临时脚本和备份，更新稳定文档。
- 关键结论：原迁入版本的算法可在当前 Orin 真实 TensorRT 资产上运行，但原会话会同时启动 face/voice、队列可重排旧 crop、readiness 未做最小推理且预览另起路径，不能证明设计流程；上述问题已修正。用户已确认只保留算法运行所需内容：detector/recognition engine、TensorRT/CUDA 运行时和用于实际匹配的白名单保留；隐私门禁、许可证审计及额外白名单治理不纳入本次整理和验收。由于导航仍为 fake，感知内部小循环与主流程合并为同一最终验收。
- 涉及文件：`src/perception/dog_patrol_perception_interfaces/`、`src/perception/dog_patrol_perception_orchestrator/`、`src/perception/dog_patrol_perception_face/`、`src/perception/dog_patrol_perception_voice/`、`src/perception/dog_patrol_perception_tracking/`、`tools/fake_integration/`、`.github/workflows/ci.yml`、`.gitignore`、`README.md`、`src/perception/README.md`、`src/perception/requirements.md`。
- 验证：七包 portable clean-style build 通过；tracking Orin runtime build 通过；七包 `colcon test` 汇总 580 tests、0 errors/failures/skipped（其中 tracking 408）；真实 face ROS 集成测试 3/3 通过；真实 face preflight 返回 READY；installed face config/launch/executable smoke 通过；36 个 Python 文件 `ament_flake8` 无问题，`py_compile` 与 `git diff --check` 通过。
- 后续：当前 R818 未枚举且 ADB 无设备，当前 shell 也没有 `DISPLAY`，因此需用户最后参加三次现场验收：`initial_face_pass`、`dual_pass`、`dual_reject`，同时目检同一窗口 face bbox 并保留 `report.json` 性能数据。三项通过后更新本记录并将全部修改整理为一个本地提交；当前尚未提交或推送。

## 2026-08-11 11:40 - mission 模式端到端联调：face capability 修复至 READY，voice 阻塞暂停

- 目标：跑通正式 mission 模式链路（mission_supervisor → tracking → perception_face_provider → evidence），让 supervisor 离开 STARTUP 并进入 VERIFY_IDENTITY，验证 face 模型惰性加载与授权证据产出；全程不修改人脸目录以外的任何代码。
- 完成：① 系统 python（`ros2 run` 所用）缺 pycuda，通过把 venv `tools/face_verify/.venv` 里的 `pycuda` 与 `pycuda-2026.1.dist-info` 软链到 `~/.local/lib/python3.10/site-packages` 解决（`site.ENABLE_USER_SITE=True`，系统 python 自动加载，无需 PYTHONPATH；PYTHONPATH 覆盖式注入会破坏 ros2cli/rclpy 解析，不可用）；② 修复 `preflight.py` 在加载 TRT 引擎前未建 CUDA context 的 bug（`LogicError: explicit_context_dependent failed: invalid device context`），复用 `detector.ensure_cuda_context()`，在 `_check_detector/_check_recognition/_check_whitelist` 外 push/pop；③ face capability 从 ERROR 修复到 READY，`diagnostic: face preflight ready`。同时验证了 tracking engine 常驻 +434MiB、face 无 crop 不加载模型只 +68MiB。
- 关键结论：`install/` 是拷贝安装而非 symlink，改源包后必须手动 `cp` 同步到 `install/dog_patrol_perception_face/lib/python3.10/site-packages/` 才生效。orchestrator `readiness_node.py` 硬编码 REQUIRED_CAPABILITIES=("detection_tracking","face","voice")，voice 必须 READY 才发 PERCEPTION READY；voice readiness 当前 NOT_READY（缺 Vosk 模型目录、ALSA prompt_device 默认 `plughw:CARD=Device,DEV=0` 不匹配本机 HDMI `plughw:CARD=HDA,DEV=3`、以及 ADB/helper/tools 检查）。face 的 target_id 由 mission_supervisor 经事件自动对齐，无需人工猜。未修改 orchestrator/tracking 等任何只读代码。
- 涉及文件：`src/perception/dog_patrol_perception_face/dog_patrol_perception_face/preflight.py`（+ CUDA context push/pop，含 .bak 备份）。
- 验证：`/usr/bin/python3 -c "import pycuda.driver as c; c.init()"` OK（devices: 1）；`ros2 topic echo /perception/capability_status` 中 `capability: face / status: 1 / diagnostic: face preflight ready`。
- 后续：用户决定就此打住，voice 与完整链路待以后推进。遗留：proflight 修复仅同步到 install 拷贝，未重新 `colcon build`；后台仍可能有 supervisor/tracking/face_provider/face_readiness/voice_readiness/readiness_node 进程，停止方式 `pkill -f "install/dog_patrol_perception_face/lib"`、`pkill -f "install/dog_patrol_perception_tracking/lib"` 或按 PID kill。

## 2026-08-10 19:05 - 人脸真实模型 ROS 集成测试（crop→evidence 绑定）

- 目标：Layer 1：用真实 `perception_face_provider` 验证 TrackedTargetImage crop → AuthorizationEvidence 的绑定，跑在 Orin 真实引擎上。
- 完成：新增 `test/test_real_ros_integration.py` 三个真实模型集成测试（决策证据绑定到 state_seq+target_id、VERIFY_IDENTITY 进行中会话被打断发 CANCELLED、错 target/会话外 crop 被忽略），无 pycuda 环境自动 skip 保证 CI 绿色。顺带修复了集成测试暴露的 3 个真实部署 bug：① provider worker 线程没有 CUDA context（pycuda 上下文线程局部），新增 `ensure_cuda_context()` 并在 `_run` finally pop；② `TRTInference` 的 `_preproc_module/_preproc_kernel` 类级静态缓存跨 CUDA context 失效（第二个节点/worker 复用到已 pop context 的 kernel 句柄报 `cuFuncSetBlockShape failed: invalid resource handle`），改为实例属性；③ detector/recognition/verifier 从不释放 GPU 资源，补全 `close()` 并在 worker 线程 pop context 前释放，消除 pycuda 退出警告。新增 `windows_started` 指标与 node `get_metrics()` 供测试同步和运维观测。README 中 face 状态从“未迁入”更新为“真实 provider/模型已迁入并通过集成测试，readiness/orchestrator 端到端仍待完成”。
- 关键结论：真实 ROS 节点此前在 worker 线程做 GPU 推理必然失败，此 bug 只有在真实模型集成测试下才会暴露。已决策完成的会话离开 VERIFY_IDENTITY 不会发 CANCELLED（只有进行中的窗口被取消/替换时才发），这是 provider 的既定语义。
- 涉及文件：`src/perception/dog_patrol_perception_face/dog_patrol_perception_face/detector.py`、`recognition.py`、`verifier.py`、`provider.py`、`test/test_real_ros_integration.py`、`README.md`。
- 验证：Orin 上 `pytest test/test_real_ros_integration.py -p no:anyio` 3 passed（约 18s）；全包 pytest 50 passed；`colcon build` + `colcon test` 该包 50 tests、0 errors、0 failures、3 skipped（CI 无 pycuda 时集成测试按预期 skip）。
- 后续：readiness、orchestrator 端到端联调和整机人脸验收仍未完成；运行集成测试需 `source /opt/ros/humble/setup.bash && source install/setup.bash` 并把 face_rec venv 的 `lib/python3.10/site-packages` 与源包路径加入 `PYTHONPATH`。

## 2026-08-08 15:59 - 建立人脸算法主仓接入基线

- 目标：在不改动现有 tracking、mission 和预览行为的前提下，为外部负责人建立正式人脸代码目录和可执行的迁入协作门禁。
- 完成：新增可构建的 `dog_patrol_perception_face` ROS 2 package 骨架并纳入 CI、根目录和感知构建入口；库内 `COLLABORATION.md` 固定主目标 crop、mission 会话、异步失效、隐私资产、唯一预览和验收规则。人脸生产图像只消费 `TrackedTargetImage`；自带 GUI/相机/录制不得进入生产路径，人脸 overlay 后续必须通过可选 adapter 汇入 tracking 的 `VisualizerRecorder`。
- 关键结论：人脸接入基线提交为 `035f9d3796c435986b0ad33c01513c6422bd9209`（PR #47 squash merge）；后续算法迁入、预览整合及验收以该提交为参照，必须证明没有改变基线中的 tracking 主目标、实时推理、单相机、单窗口、preview/record 和 mission 行为。当前状态仍是 `scaffolded/not-integrated`，不包含算法、provider、readiness、模型或白名单，不构成生产人脸能力已接入。
- 涉及文件：`src/perception/dog_patrol_perception_face/`、`.github/workflows/ci.yml`、`README.md`、`src/perception/README.md`、`src/perception/requirements.md`、`worklog.md`。
- 验证：portable `colcon build` 7 个包通过；`colcon test-result --verbose` 为 507 tests、0 errors、0 failures、0 skipped；安装后的 package 可由 `ros2 pkg prefix` 解析且包含 `COLLABORATION.md`；YAML 解析和 `git diff --check` 通过。
- 后续：人脸负责人迁入前应先固定源仓提交、许可证和 allowlist；实现 PR 必须按协作规则补齐 crop-to-evidence、取消/旧结果拒绝、慢消费者不反压、隐私和唯一预览验收证据。

## 2026-08-08 15:10 - 修复开机即见人确认事件丢失

- 目标：诊断并解决 tracking 在 STARTUP 阶段已经看到目标、进入 PATROL 后流程停住的问题，并建立可重复的短回归场景。
- 完成：确认 `MissionFrameTransaction` 原先对每个 PATROL `state_seq` 只发送一次 `TARGET_CONFIRMED`；volatile mission event 在状态切换或订阅发现窗口丢失后不会重试。改为首次尝试后，在 authoritative state 未推进时按至少 100 ms 间隔重试同一 `state_seq`，保持公共消息合同不变；新增 transaction 单测和 ROS adapter 丢消费者 seam 测试。fake 联调脚本新增 `startup_visible`，只启动 fake readiness 和 navigation，真实 tracking 启动时直接验证目标已在画面内的状态推进。
- 关键结论：根因是一次性确认门禁吞掉了 volatile event，不是 primary manager 的 STARTUP/PATROL 判定错误。两次真机启动可见证据中，监督器均接受 `PERCEPTION/TARGET_CONFIRMED` 并推进 `PATROL -> CONFIRM_TARGET`；其中报告 `20260808_144038` 后续还推进到 `APPROACH_TARGET -> VERIFY_IDENTITY`。专用场景第二轮观察器因 volatile event 发现竞态漏收而误报 FAIL，但 supervisor 日志仍记录了接受和推进，不能视为 tracking 功能失败。性能继续只记录，不设门槛。
- 涉及文件：`src/perception/dog_patrol_perception_tracking/include/dog_patrol_perception_tracking/modules/mission_frame_transaction.hpp`、`src/perception/dog_patrol_perception_tracking/src/modules/mission_frame_transaction.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_mission_frame_transaction.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_mission_ros_adapter.cpp`、`src/perception/dog_patrol_perception_tracking/README.md`、`tools/fake_integration/run_fake_integration.py`、`tools/fake_integration/README.md`、运行报告目录 `data/diagnostics/fake_integration/20260808_143931/`、`data/diagnostics/fake_integration/20260808_144038/`、`data/diagnostics/fake_integration/20260808_150259/`、`data/diagnostics/fake_integration/20260808_150547/`、`worklog.md`。
- 验证：tracking 包 Orin runtime 构建通过；包 CTest `56/56` 通过；新增 transaction/ROS seam 测试通过；Python `py_compile`、`git diff --check` 和 debug 标记清理检查通过；监督器日志实机接受 `TARGET_CONFIRMED` 并离开 PATROL；退出后无残留 supervisor/tracking/fake 进程。
- 后续：fake 联调观察器对 volatile mission event 的接收仍有启动发现竞态，后续可把 `startup_visible` 的判定改为订阅 transient-local mission state 或增加监督器状态探针，避免仅因观察端漏收事件误报；不影响本次 tracking 修复和已有真机证据。

## 2026-08-08 12:25 - 开启预览的正常授权联调通过

- 目标：使用 fake 导航/人脸、真实 tracking/voice 重跑正常第一窗授权通过场景，并开启实时预览。
- 完成：通过 `--preview` 启动；在 STARTUP 阶段保持目标不入画，进入 PATROL 后目标入画，完整跑通 TARGET_CONFIRMED、TARGET_POSITION_READY、ARRIVED_AND_STOPPED、第一语音窗 PASSED、AUTHORIZED、返回 PATROL。
- 关键结论：有效轮功能 `PASS`，报告确认 `preview_enabled=true`。预览最终 29.92 FPS，1708 帧全部 enqueued/rendered/previewed，queue/render/write drop 和 render/write error 均为 0。此前一轮目标在 STARTUP 已入画时，tracking 锁定目标并产生一次 TARGET_CONFIRMED observation，但 supervisor 未推进且目标释放/重锁后未重发；该轮已中止，不计为测试结果，启动期目标事件竞态需单独诊断。
- 涉及文件：运行报告目录 `data/diagnostics/fake_integration/20260808_122416/`、`worklog.md`。
- 验证：`report.json` functional PASS、第一窗 voice PASSED、最终 PATROL；tracking 约 30 FPS、camera acquisition failures=0、lost packets=0；tegrastats 59 样本，RAM 3695–4235 MB、CPU 每核样本平均 27.042%、GPU GR3D 平均 43.525%、Tj 54.88–56.59°C；无残留进程。
- 后续：建议单独复现并诊断“STARTUP 阶段目标已在画面内”时 TARGET_CONFIRMED 的 state_seq/重试门禁，避免部署现场开机即见人时流程停在 PATROL。

## 2026-08-08 12:14 - 已授权目标短时离场再遇联调通过

- 目标：验证目标通过真实语音授权后，短时离开并再次进入画面时不会被重新当作可疑目标。
- 完成：新增 `authorized_reencounter` 场景，真实 tracking/voice 完成首次授权；脚本根据 tracking `det=0`/`det>0` 实际观测驱动离场和返回提示，返回后固定观察 10 秒。fake face readiness 改用 capability QoS 并在 STARTUP 每秒重试，避免启动竞态。fake 联调入口增加 `--preview`，按需打开 tracking 实时诊断叠图，并在无 `$DISPLAY` 时提前拒绝。
- 关键结论：生产参数 `target.handled_ignore_absence_sec=30` 按“连续不可见”计时，持续可见时不失效。本轮实际不可见约 23.24 秒，返回后始终保持 `PATROL(state_seq=6,target_id=0)`，未再次进入确认/语音流程；voice evidence 始终只有首次 `PASSED` 一条。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、`tools/fake_integration/run_fake_integration.py`、`tools/fake_integration/README.md`、运行报告目录 `data/diagnostics/fake_integration/20260808_121234/`、`worklog.md`。
- 验证：`report.json` functional PASS、absence/return 均观测到；返回后 tracking 连续约 10 秒 `det=1, state=IDLE, primary_id=-1`；71 个 tegrastats 样本，RAM 3657–4185 MB、CPU 每核样本 0–100%（平均 21.397%）、GPU GR3D 0–99%（平均 31.803%）、Tj 53.97–55.81°C；`py_compile`、`git diff --check` 通过，无残留进程。
- 后续：若要验证豁免过期边界，可分别测试连续离场略短于 30 秒与超过 30 秒；当前结果只覆盖 30 秒内再次出现。

## 2026-08-08 - tracking-only 目标丢失超时联调通过

- 目标：关闭真实 voice/face 算法，验证目标锁定后离场、超过 6 秒 retention 仍不出现的行为。
- 完成：新增 `tracking_loss_timeout` 场景；修正 test-only capability publisher 为 reliable + transient-local + depth 16，避免 face/voice READY 覆盖；真实目标在稳定 bbox 后离场，tracking 发布一次 `TARGET_LOST`，随后观察 8 秒未出现 `TARGET_REACQUIRED`。
- 关键结论：功能报告 `PASS`，`target_lost_seen=true`、`target_reacquired_seen=false`、evidence_count=0；mission 保持 TARGET_LOST block。真实 voice/face 算法未启动，只有 test-only capability READY 节点。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、`tools/fake_integration/run_fake_integration.py`、运行报告目录 `data/diagnostics/fake_integration/20260808_115737/`、`worklog.md`。
- 验证：mission log 只有 `TARGET_LOST(state_seq=5,target_id=1)`，无 `TARGET_REACQUIRED`；结构化资源 54 样本，RAM 3481–3732 MB、GPU GR3D 0–68%、Tj 54.4–55.5°C；无残留进程。
- 后续：可继续验证 retention 超时后目标重新进入时是否按预期分配新业务目标，或 blocked 状态下导航停车行为。

## 2026-08-08 - tracking-only 目标丢失/重获联调通过

- 目标：关闭真实 voice/face 算法，只保留真实 tracking，验证目标离场超过约 0.5 秒后再在 retention 窗内回到画面的流程。
- 完成：新增 `tracking_reacquire` 场景；不启动 voice readiness/provider 或人脸节点，仅用 test-only capability READY 补齐 mission 启动条件。真实目标锁定后离开画面，tracking 发布 `TARGET_LOST`；在 6 秒 retention 内返回，tracking 发布 `TARGET_REACQUIRED`。
- 关键结论：功能报告 `PASS`；mission log 接受 `TARGET_LOST`（seq=5）和 `TARGET_REACQUIRED`（seq=6），保持同一 `target_id=1`；报告无 voice evidence，确认按需语音/人脸算法未启动。性能仍只记录，不设门槛。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、`tools/fake_integration/run_fake_integration.py`、`tools/fake_integration/README.md`、运行报告目录 `data/diagnostics/fake_integration/20260808_115029/`、`worklog.md`。
- 验证：`report.json` tracking_reacquire PASS；tracking 日志出现 `LOCKED -> OCCLUDED -> LOCKED`，末段约 30 FPS；`tegrastats` 37 样本，RAM 3454–3718 MB、GPU GR3D 8–90%、Tj 53.8–54.8°C；无残留进程。
- 后续：可继续测试 retention 超时后的新目标分配或目标丢失期间的取消场景。

## 2026-08-08 - 补充联调资源结构化记录

- 目标：将 fake 联调中的整机资源原始采样和启动进程资源数据纳入结构化报告。
- 完成：`run_fake_integration.py` 解析 `tegrastats.log`，在 `report.json.performance.tegrastats_summary` 写入 RAM、每核 CPU、GPU `GR3D_FREQ`、Tj 温度的样本数/最小/最大/平均值；测试循环按秒记录启动进程 RSS 和累计 CPU ticks。原始日志仍保留。
- 关键结论：性能字段仍是 `OBSERVED`，不参与功能退出码和通过门槛；Jetson 统一内存只记录 RAM 使用量，不虚构独立 VRAM。
- 涉及文件：`tools/fake_integration/run_fake_integration.py`、`tools/fake_integration/README.md`、`worklog.md`。
- 验证：两个 Python 文件 `py_compile`、`git diff --check` 通过；使用既有 51 行 `tegrastats.log` 离线解析成功。
- 后续：下一次真实联调报告会同时包含结构化整机摘要和进程 RSS/CPU ticks 样本。

## 2026-08-08 - 第一窗拒绝、第二窗通过联调通过

- 目标：验证真实 voice 第一响应窗无应答、第二响应窗正确口令的任务级聚合流程。
- 完成：修正 fake navigation 在 supervisor 尚未订阅时丢失 volatile READY 的重试逻辑；重新执行真实 tracking/voice + fake nav/face。第一窗发布 `NOT_PASSED`，第二窗发布 `PASSED`，授权编排发布 `AUTHORIZED`，状态机回到 `PATROL`。
- 关键结论：功能报告 `PASS`；两条 evidence 均绑定 `state_seq=5,target_id=1,provider=voice`，顺序和预期完全一致。性能仍仅作观测，不设门槛。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、运行报告目录 `data/diagnostics/fake_integration/20260808_113631/`、`worklog.md`。
- 验证：`report.json` evidence 顺序为 `NOT_PASSED -> PASSED`；mission log 接受 `AUTHORIZED`；tracking 末段约 30 FPS、1453 frames、acquisition_failures=0、camera_lost_packets=0；无残留 dog_patrol/fake/R818/arecord 进程。
- 后续：可继续执行中途取消和目标丢失场景；fake navigation READY 已改为 STARTUP 期间重试，避免 volatile event 启动竞态。

## 2026-08-08 - 正常流程两轮语音无应答联调通过

- 目标：在真实 tracking/voice 链路上两轮语音窗口均不说话，验证无应答聚合和未授权流程。
- 完成：复用 fake navigation/fake face 固定流程；真实 voice 第一、第二窗口分别发布 `NOT_PASSED`，授权编排发布 `UNAUTHORIZED`，状态机从 `VERIFY_IDENTITY` 进入 `TRACK_INTRUDER`。
- 关键结论：功能报告 `PASS`，状态序列覆盖 STARTUP/PATROL/CONFIRM_TARGET/APPROACH_TARGET/VERIFY_IDENTITY/TRACK_INTRUDER；evidence 均绑定 `state_seq=5,target_id=1,provider=voice`，无应答行为符合预期。性能仍只记录，不设门槛。
- 涉及文件：运行报告目录 `data/diagnostics/fake_integration/20260808_112824/`、`worklog.md`。
- 验证：`report.json` functional PASS、evidence_count=2；mission log 接受 `UNAUTHORIZED`；tracking 末段约 30 FPS、1984 frames、acquisition_failures=0、camera_lost_packets=0；无残留 dog_patrol/fake/R818/arecord 进程。
- 后续：可继续执行中途取消、目标丢失和 voice 硬件恢复场景；错误场景仍不改变性能无门槛原则。

## 2026-08-08 - 正常流程本机 fake 联调通过

- 目标：在当前感知机器狗上用真实 tracking、真实 R818/Vosk voice、最小 fake navigation/fake face 跑通正常流程。
- 完成：修正 fake 节点对 ROS 参数的命令行解析和 rclpy logger 调用；完成一次真实 happy path。tracking 锁定真实目标 `target_id=1`，fake navigation 推进 `READY -> TARGET_POSITION_READY -> ARRIVED_AND_STOPPED`，mission 进入 `VERIFY_IDENTITY`，voice 第一响应窗识别 `blue star` 通过并发布 `PASSED`，状态机回到 `PATROL`。
- 关键结论：功能报告 `PASS`；唯一 evidence 为 `state_seq=5,target_id=1,provider=voice,result=PASSED`。性能仅作观测：tracking 末段约 30 FPS，未设置或触发性能门槛；`tegrastats` 采集完整。退出后无 dog_patrol/fake/R818/arecord 残留进程。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、`tools/fake_integration/run_fake_integration.py`、`worklog.md`；运行报告目录 `data/diagnostics/fake_integration/20260808_112536/`。
- 验证：`report.json` functional PASS；状态序列覆盖 STARTUP/PATROL/CONFIRM_TARGET/APPROACH_TARGET/VERIFY_IDENTITY/PATROL；tracking 日志末段 camera frames=674、acquisition_failures=0、camera_lost_packets=0；`tegrastats.log` 有完整采样。
- 后续：继续补充错误口令、取消和目标丢失场景；性能数据保留为优化依据，不改变功能退出判定。

## 2026-08-08 - 增加本机真实 tracking/voice fake 联调入口

- 目标：为感知团队机器狗提供固定流程的本机联调脚本，用最小 fake 导航和 fake 人脸替代尚未接入的真实模块。
- 完成：新增 `tools/fake_integration/fake_nodes.py` 和 `tools/fake_integration/run_fake_integration.py`；fake 导航只推进 READY、目标位置就绪和到达停车，fake 人脸只发布当前 STARTUP 的 `face=READY` 并观察真实 crop；脚本启动主仓真实 supervisor、tracking、orchestrator、voice provider，记录 mission/evidence，并可采集 `tegrastats` 原始资源日志。
- 关键结论：功能流程是唯一硬门槛，脚本退出码只由是否经过 VERIFY_IDENTITY 并收到非 CANCELLED evidence、最终进入预期状态决定；性能和资源只写入 `performance=OBSERVED`，不设阈值，资源采集缺失不改变功能判定。
- 涉及文件：`tools/fake_integration/fake_nodes.py`、`tools/fake_integration/run_fake_integration.py`、`tools/fake_integration/README.md`。
- 验证：两个 Python 文件 `py_compile` 通过；source ROS 与本仓 install 后两个入口的 `--help` 通过；尚未在真实设备执行完整联调。
- 后续：在当前感知机器狗上用真实 tracking、R818/Vosk voice 资产执行 happy path；根据首轮报告再补充取消、拒绝和目标丢失场景。

## 2026-08-07 18:53 - 复跑源仓 33 任务语音数据集

- 目标：使用已接入主仓当前 `main` 的语音算法复跑冻结源仓 33 个任务数据，检查准确率是否相对迁入基线回退。
- 完成：从 `main@49801af9f8032910e4de5e578043d56038ca2881` 在 `/var/tmp/dog_patrol_voice_replay.rLIe8F` clean install 依赖与 voice 包；核对冻结源仓 57 个 PCM 窗口、模型和聚合指纹后，经 installed `R818StreamingVoskSession` 回放全部 33 个任务；保存逐任务、逐窗结果报告。
- 关键结论：任务级 28/33 正确（84.85%），其中正例 18/20 通过、负例 10/13 正确拒绝，技术错误 0；窗口级正例 18/32 通过、负例 22/25 正确拒绝。结果与冻结源仓及 #36 基线完全相同，没有迁入后回退；原有误差仍是 3 个 `blue-sky-negative` 全部误放行，以及 `first-window-positive/run-03`、`second-window-positive/run-02` 两个正例拒绝，不能据此声称错误口令安全性已达标。
- 涉及文件：`worklog.md`；临时报告 `/var/tmp/dog_patrol_voice_replay.rLIe8F/current-main-33-run-replay.json`。
- 验证：PCM 57 files/218239040 bytes，聚合 SHA-256 `18248932edb35672ab3b223d23d45cac9fddbe6ec9e259cf7a80fc7335d36dc6`；模型聚合 SHA-256 `5c9c563ec18e9a7d176eacdb18788b1c7dde7decf40c92ee2c8374668ade9656`；报告 SHA-256 `74a1896a87c302cc087a77a83de79741b30a933998a138931547c3bcccb68c7b`；installed module 路径和预期错误样本名单机器断言通过。
- 后续：若要提升而非仅验证迁移等价性，应另开效果改进任务，先明确最终口令和错误口令误放行指标，再扩充独立负例集，避免只针对现有 3 条 `blue sky` 调参。

## 2026-08-07 18:35 - 完成 #37/#38 现场验收并归档来源

- 目标：恢复 clean install，重新通过 #37 自动门禁并完成 #38 真人现场矩阵和来源归档。
- 完成：在 `/var/tmp/dog_patrol_issue37.DznQL7` 非 symlink 构建六包；使用现场相机参数、Orin engine、冻结 Vosk model 和已安装 helper 通过统一环境检查；#37 完成 3/3 正常生命周期与 9/9 故障场景；#38 完成首窗通过、次窗通过和双窗无应答三项真人矩阵；更新稳定文档并创建、推送 annotated archive tag；PR #45 通过 CI 后 squash merge，#38 已关闭并移除 `ready-for-human`。
- 关键结论：#38 三项均 `passed=true`、清理完成、无迟到 evidence；执行提交为 `cf78d028aed921ea4a3e107fb9b3574cea5e98b6`，来源为 `b979a7fd33aac5c9ced9591bb507e483faf4aef5`，#38 报告 SHA-256 为 `f8ab6fcd7387a062a12abd1993abbd14ac979ee6347047dc9e08e0d25ffc8a5f`。设备最终由 `demo` PID 11856 持有 AC107，状态 `RUNNING`，无远端残留。结论不覆盖错误口令、最终口令、FRR/FAR 或安全准入。
- 涉及文件：`README.md`、`src/perception/requirements.md`、`src/perception/dog_patrol_perception_voice/README.md`、`docs/perception/voice/issue38_voice_field_acceptance.md`、`worklog.md`；本地 tag `archive/dog-patrol-deployment-b979a7f-issue38`。
- 验证：六包 clean build 通过；496 tests、0 errors/failures/skipped；统一环境检查 PASS；#37 CLI 返回 0；#38 CLI 返回 0；最终 ADB owner/status/残留检查通过。
- 后续：后续若验收最终口令、错误口令拒绝或 FRR/FAR，应另开票，不扩展本票结论。本地 `main` 因 PR squash 与四个原始提交分叉，未执行破坏性重置；远端权威 `origin/main` 已为合并提交 `49801af`。

## 2026-08-07 18:05 - 重启 #37/#38 验收尝试

- 目标：R818 换口后按源仓 ADB serial 启动 #37 hardware 与 #38 field。
- 完成：确认 R818 `2207:0001`、ADB serial `bc00144082110821990`、`/dev/ttyACM0` 和 AC107 `RUNNING/demo` 均恢复；实际启动 #37 和随后 #38 CLI。
- 关键结论：#37 在硬件接管前被统一环境门禁阻断，报告 `passed=false`；#38 正确读取该失败报告并在硬件前阻断，未触发现场语音矩阵。阻断项为相机 serial 未配置、detector runtime path 为空、当前测试汇总 1 failure、安装 acceptance 仍从源码导入/安装 helper 缺失。
- 涉及文件：`worklog.md`；报告写入 `/tmp/issue37_voice_acceptance.json`、`/tmp/issue38_voice_field_acceptance.json`。
- 验证：#37 CLI 返回 1，#38 CLI 返回 1；ADB 定向 `get-state` 通过，设备 owner/status 只读探测通过。
- 后续：先修复上述部署门禁并重新 clean-install，再重跑 #37；只有 #37 报告通过后才允许 #38 接管并等待用户现场说话。

## 2026-08-07 17:58 - 记录 Orin USB 设备拓扑

- 目标：记录当前 Orin USB 端口和已枚举设备，为 R818 接入排查提供基线。
- 完成：采集 `lsusb`、`lsusb -t`、USB sysfs VID/PID/序列号、串口节点和 ALSA 节点。
- 关键结论：当前 USB 设备为 Hikrobot `2bdf:0001`（Bus 2 Port 3.3，5 Gbps，序列 `DB0060477`）、ASIX `0b95:1790`（Bus 2 Port 3.4，5 Gbps，序列 `0041CEE5`）、Realtek USB 3 Hub `0bda:0420`、Realtek USB 2 Hub `0bda:5420`/下挂 `1a40:0101`、蓝牙 `13d3:3549`；未枚举 R818 `2207:0001`，也没有 `/dev/ttyACM*` 或 `/dev/ttyUSB*`。Orin 内建 APE/HDA 音频节点正常存在。
- 涉及文件：`worklog.md`。
- 验证：2026-08-07 17:57:32 运行 `lsusb`、`lsusb -t`、USB sysfs 枚举、串口/音频节点检查；结果已记录。
- 后续：R818 插入后应重新保存同一组快照，重点观察新 USB VID/PID、拓扑端口、ADB serial 和 CH9102 串口节点。

## 2026-08-07 17:45 - 加固 #38 现场报告门禁

- 目标：接手 #38 实现并复核现场验收入口，确保前置 #37 报告不可由不完整摘要绕过，且现场报告不泄露诊断细节。
- 完成：field 报告改为只保留任务结果、state/target 关联、清理摘要和部署资产指纹；#37 自动门禁新增 evidence/event、远端残留和 vendor owner 校验；补充脱敏与伪造报告回归测试。
- 关键结论：当前仍未执行真人现场矩阵，不能创建来源归档 tag 或关闭 #38；现场仍必须使用当前 clean install 和通过的 #37 hardware 报告。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/test/test_acceptance.py`、`worklog.md`。
- 验证：voice 包构建通过；voice 包 61/61 测试通过（`ROS_LOG_DIR=/tmp/dog_patrol_ros_log`）；`git diff --check` 通过。全仓测试在受限环境中仍有既有 ROS 日志/GPU/硬件相关失败。
- 后续：恢复 R818 和部署输入后，重新运行 #37，再运行 #38 三项真人矩阵；全部通过后才创建 annotated archive tag 并回填 issue/文档。

## 2026-08-07 16:58 - 实现 #38 现场验收入口并完成无人参与准备

- 目标：领取 #38；先完成不需要用户回应的验收入口、自动门禁和 clean-install 验证，再保留最终真人矩阵与来源归档。
- 完成：`perception_voice_acceptance` 新增 `--mode field`，固定首窗通过、首窗无应答后次窗通过、双窗无应答三种真实 R818/Vosk 结果形状；field 拒绝 fixture，先要求 #37 的完整成功 hardware 报告（3 个正常生命周期、9 项故障清理矩阵及无迟到结果），并逐项匹配 model、config、helper 和安装 `acceptance.py` 指纹后才会接管硬件。报告只保存结果、ROS 关联、清理和指纹；新增现场 runbook，并同步 package/领域/根目录状态。
- 关键结论：当前机器为 Orin，但 `adb devices -l` 无在线 R818，`/srv/dog-patrol` 也不存在；因此未执行新的 #37 硬件门禁、真人矩阵或来源 tag，且不能把历史自动结果当作当前 field 的前置。`field` 必须使用当前安装产物重新生成的 #37 成功报告。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/test/test_acceptance.py`、`docs/perception/voice/issue38_voice_field_acceptance.md`、`src/perception/dog_patrol_perception_voice/README.md`、`src/perception/requirements.md`、`README.md`、`worklog.md`。
- 验证：field CLI 的 TDD 定向测试通过；最终临时 clean install 六包 portable build 通过，安装 CLI help 通过；六包全量 `colcon test` 汇总 460 tests、0 errors/failures/skipped；环境检查器 17 项单测和 `git diff --check` 通过。field 的缺失 #37 报告 smoke 正确返回非零且未触发硬件。
- 后续：R818/ADB 和部署输入恢复后，先从当前 clean install 完整运行 #37 并保存成功报告；最后在用户在场时运行固定 field 矩阵。仅 field 报告通过后创建 `archive/dog-patrol-deployment-b979a7f-issue38` annotated tag，并回填来源 SHA、dog_patrol SHA 和报告摘要；失败时不归档。

## 2026-08-07 14:17 - 将 #37 验收收敛为必要测试

- 目标：按用户决定移除 33 次历史任务级重复循环，只保留必要生命周期与故障覆盖。
- 完成：默认 `--cycles` 改为 3，覆盖第一窗通过、第二窗通过和两窗拒绝；保留 9 个不同的 Prompt/窗口取消、state_seq/target_id 替换、stream/ADB/播放/恢复故障场景；第二窗故障 stage timeout 提高到 30 秒；同步本地验收文档、voice README 和 GitHub #37 acceptance criteria。
- 关键结论：3 个正常循环 + 9 个故障场景足以覆盖当前 fixture 注入模式下不同的生命周期形状；负例不会形成额外硬件生命周期。源仓 33 次仅保留为历史来源，不再是本票默认验收门槛。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/test/test_acceptance.py`、`docs/perception/voice/issue37_voice_hardware_acceptance.md`、`src/perception/dog_patrol_perception_voice/README.md`、GitHub #37。
- 验证：voice 包 `colcon test` 56/56；汇总 506 tests、0 errors/failures/skipped；真实 Orin 最小验收 3/3 正常循环、9/9 故障场景通过，环境/readiness/Vosk/ADB/R818 owner/残留/主机 PCM 全部通过；设备恢复为 `demo` 且无残留。
- 后续：若再需要历史 33 次回放，必须显式传入更大的 `--cycles` 和匹配 fixture，不属于默认验收。

## 2026-08-07 14:02 - 定位 #37 硬件长跑超时

- 目标：解释 Prompt 修复后 #37 完整硬件验收为何仍在长跑阶段超时。
- 完成：用单轮真实顺序和逐场景硬件故障矩阵复现；确认 `stream.start -> Prompt.play -> stream.close` 可完成，逐场景矩阵中 `prompt_cancel`、首窗取消、替换、stream/ADB/playback/restore 故障可正常清理；将 `second_window_cancel` stage timeout 临时从 10 秒提高到 30 秒后通过。
- 关键结论：之前的 480 秒是外层命令预算，不是验收代码的完整 33 轮预算；真实每轮包含 R818 接管、完整 Prompt 和恢复，33 轮之后还要跑 9 个硬件故障场景，因此 480 秒不足会在 CLI 写报告前被杀掉。`second_window_cancel` 原有 10 秒 stage 上限也不足以完成首窗、重试 Prompt 和进入第二窗，属于独立的矩阵参数误判，不是 R818 永久死锁。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/r818_stream.py`、`docs/perception/voice/issue37_voice_hardware_acceptance.md`。
- 验证：真实单轮生命周期完成；逐场景硬件矩阵输出 cleanup/owner/remote residue 均正常；`second_window_cancel` 使用 30 秒 stage timeout 返回 `passed=true`；诊断临时 fixture 已删除，设备恢复为 `demo` 且无 `arecord`/远端残留。
- 后续：把硬件矩阵 stage timeout 和部署验收总预算改为显式、可观测的参数后，再运行完整 33 轮；本回合未把诊断中的临时 timeout 作为代码修改提交。

## 2026-08-07 13:50 - 对齐源仓 Prompt 播放并修复现场超时

- 目标：核对源仓 Prompt 播放实现，解释现场能听到短声音但 #37 验收报告判定播放超时的问题。
- 完成：以 `moonshine_voice_commands/src/moonshine_voice_commands/speaker.py` 为行为基准修正主仓 Prompt 播放；保留当前取消接口，但改为一次性完成 WAV 输入与进程等待，加入源仓相同的 `acompressor` 和 mixer 超时；新增管道 EOF 回归测试。
- 关键结论：源仓和主仓使用同一 Prompt 文案、`plughw:CARD=Device,DEV=0` 和 `PCM=100%`；主仓原先 50 ms `Popen.communicate` 轮询在首轮写入未完成时丢弃后续 input，导致 `aplay` 播放短片段后等待 EOF 并超时。源仓完整 Prompt 可正常返回，修复后主仓安装版本也可正常返回。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/prompt.py`、`src/perception/dog_patrol_perception_voice/test/test_prompt.py`。
- 验证：安装后实际 Prompt 播放返回 `current_player_ok`；voice 包 `colcon test` 为 54/54；当前测试结果汇总为 504 tests、0 errors/failures/skipped；`git diff --check` 通过；无遗留 `aplay`/`ffmpeg` 播放进程。
- 后续：从修复后的安装产物重新执行 #37 的 33 次硬件验收；识别结果仍使用 fixture 注入，不能据此声明真实说话效果已验收。

## 2026-08-07 13:20 - 实现 #37 并完成 Orin 现场诊断

- 目标：领取并实现 GitHub #37，为 clean-installed Orin 提供无需用户说话的 voice 部署验收，并核对真实 ADB/R818/AC107 生命周期。
- 完成：新增严格 fixture provenance、统一感知环境门禁、真实硬件 R818/ALSA 故障矩阵、逐循环 vendor `demo` owner/远端残留/主机 PCM 检查、迟到 evidence grace、超时中止与清理报告；补充硬件报告诊断字段、安装运行时门禁和 CI 零 skipped/零测试防漏检；修正 AC107 `owner_pid   :` 解析及硬件失败场景 post-cleanup 报告语义。
- 关键结论：本机是 Jetson Orin `aarch64`，ADB、AC107/R818 和 USB `Device` 播放卡均在线；源仓 `moonshine_voice_commands/.venv` 确有 Vosk，当前系统 Python 也已安装 `vosk==0.3.45`。统一环境检查、Vosk/model preflight、ROS readiness、真实 R818 接管/恢复和残留清理均可通过，但实际 `plughw:CARD=Device,DEV=0` 的 `aplay` Prompt 播放在 10 秒超时，因此 #37 现场验收报告保持 `passed=false`，不能关闭 issue 或声称 33 次通过；fixture 注入识别结果仍不代表真实 Vosk 说话效果。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/r818_stream.py`、`.github/workflows/ci.yml`、`docs/perception/voice/issue37_voice_hardware_acceptance.md`、`src/perception/dog_patrol_perception_voice/README.md`。
- 验证：Orin 配置 `colcon build` 通过；clean-installed 全量测试 6 packages、503 tests，0 errors/failures/skipped；voice 包 53/53；统一环境门禁 PASS；真实单循环 readiness/ADB/R818 cleanup 通过但 Prompt playback 超时；短硬件矩阵的通过/失败均形成报告，最终设备无远端残留且 owner 恢复为 `demo`。
- 后续：确认 `Device` 播放链路的实际输出节点/ALSA 配置后，从安装产物重新执行 33 cycles 和完整硬件矩阵；在此之前 issue #37 保持 open。

## 2026-08-07 12:12 - 实现 #37 无人参与 Orin 语音验收

- 目标：领取并实现 GitHub #37，为 clean-installed 感知 Orin 提供不需要用户说话的真实部署验收入口。
- 完成：领取并指派 #37；新增安装后的 `perception_voice_acceptance`，支持部署机 fixture 注入识别结果、33 次任务循环、真实 readiness/provider/authorization ROS 生命周期、R818 远端残留检查和 Prompt/响应窗取消、state_seq/target_id 替换、stream/ADB/播放/恢复故障矩阵；补充严格 fixture schema、JSON 报告、安装 smoke、部署文档和 CI 检查。修复 `R818TaskSession.__enter__` 在 stream startup 失败时未调用 cleanup 的生命周期缺口。
- 关键结论：`--mode fixture` 只验证 ROS/任务合同和确定性故障矩阵；`--mode hardware` 才加载部署机 Vosk model、播放 Prompt、接管真实 ADB/AC107/R818，并逐任务检查 `/tmp` 临时节点和 `arecord` 清理。fixture 不包含 PCM/录音/model，硬件模式识别结果仍为 fixture 注入，因此不替代现场说话、FAR/FRR 或安全准入验收。
- 涉及文件：`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/acceptance.py`、`src/perception/dog_patrol_perception_voice/dog_patrol_perception_voice/adapter.py`、voice package/setup/tests、`.github/workflows/ci.yml`、`docs/perception/voice/issue37_voice_hardware_acceptance.md`、`README.md`、`src/perception/requirements.md`。
- 验证：六包 `colcon build` 通过；完整 `colcon test` 通过（466 tests，0 errors/failures/skipped）；voice 包 51/51；新增 acceptance `ament_flake8`/`ament_pep257`、compileall、package XML、git diff check 和安装后 `ros2 run ... perception_voice_acceptance --help` 通过。Orin 只读 preflight 能加载入口但当前 Vosk Python runtime、配置 ALSA 播放/mixer 和 ADB 设备均不可用，返回 ERROR；未执行真实 R818 接管。
- 后续：部署机补齐 `python3-vosk`、实际 ALSA/ADB 设备和 33 条源仓任务结果 fixture 后，按 `docs/perception/voice/issue37_voice_hardware_acceptance.md` 执行 `--mode hardware`；当前 issue 不应标记为现场验收完成。

## 2026-08-06 17:25 - 完成 #36 voice 历史 PCM A/B

- 目标：领取并实现 GitHub #36，使用冻结源部署提交与 clean-installed 主仓 voice module 对同一历史 PCM、Vosk model 和有效配置执行迁移前后 A/B。
- 完成：领取并指派 #36；在固定主仓提交 `f6f683dd52b2d23c3dbd0a8da494e7c122055575` 的 clean worktree 中构建临时 install prefix，通过现有 `InterleavedPcmTaskStream` seam 对源仓 `b979a7fd33aac5c9ced9591bb507e483faf4aef5` 与 installed main 逐窗回放 33 个 run、57 个窗口；新增逐 run/逐窗口/逐通道文本、决策时间、命中通道、vote counts 和 SHA-256 摘要报告。随后推送 `feat/issue-36-voice-ab`，创建 PR #43 并 squash merge；#36 已关闭，主仓合并提交为 `06e434cab39029e5832c9e1703cbe9097d3dcf45`。PCM、model、临时回放器和结果 JSON 均未进入主仓。
- 关键结论：两侧有效参数均为 `blue star`、`[unk]`、20 秒窗口、0.75 秒 vote guard、16 kHz 八通道输入、0–5 麦克风解复用和 960 帧（60 ms）回放块；57/57 窗口所有比较字段完全一致。任务级正例通过数为 18/20、负例拒绝数为 10/13，两侧相同；源仓已有 3 个负例误放行和 2 个标为正例但任务级拒绝的 run，主仓未新增或改变。该结论只覆盖离线迁移一致性，不替代现场硬件、实时性能、FAR/FRR 或生产 READY 验收。
- 涉及文件：docs/perception/voice/issue36_voice_ab_report.md、README.md、本地 worklog.md。
- 验证：报告内嵌命令完成 clean worktree build、PCM/model 清单强校验和隔离回放；两侧均 33 samples、57 windows、0 技术错误，逐窗口 JSON 比较 0 差异，回放后临时 build/install prefix 已清理；六包完整 ROS build/test 通过（463 tests，0 errors/failures/skipped）；bash 语法、git diff --check、资产聚合 SHA-256 和报告 33/57 行数检查通过；最终 Standards/Spec 双轴复审通过；PR #43 的 `build-and-test` 通过（2m34s），本地 `main` 已对齐 `origin/main` 且工作区干净。
- 后续：可在真实 R818/设备和受控部署环境继续现场语音效果、恢复、FAR/FRR、生产 READY 与整机联调验收。

## 2026-08-06 16:50 - 推送合并 #35 并收尾

- 目标：将 #35 的本地实现推送、合并并为下一票恢复干净工作区。
- 完成：直接推送 `main` 被仓库规则拒绝后，创建 `feat/issue-35-voice-readiness`，通过 PR #42 squash merge；删除远端和本地临时分支，issue #35 保持关闭。
- 关键结论：仓库 `main` 必须通过 PR 且要求 `build-and-test`；PR #42 的必需检查已通过，合并提交为 `f6f683d`。
- 涉及文件：本地 `worklog.md`；GitHub PR #42、issue #35。
- 验证：PR #42 已合并；`build-and-test` 成功；本地 `main` 与 `origin/main` 一致，工作区无未提交改动。
- 后续：可直接从干净的 `main` 开始下一票。

## 2026-08-06 16:25 - 实现 #35 voice readiness 与部署入口

- 目标：领取并实现 GitHub #35，为 voice 对每个 STARTUP sequence 发布真实 capability readiness，并接入安装/部署前检查。
- 完成：新增只读 `VoicePreflight`、sequence/generation 门禁 `VoiceReadinessController` 和 ROS `perception_voice_readiness`；新增 `voice.launch.py` 同时启动 readiness 与 evidence provider；统一环境检查覆盖 Vosk/model、config、安装 helper SHA-256、ADB `get-state`、FFmpeg flite filter、ALSA 播放设备/mixer 枚举和两个 voice executable；补齐 CI 安装产物 smoke、测试、README/requirements/issue35 文档。默认 config/helper 从安装产物解析，不回退源仓路径。
- 关键结论：readiness 使用现有 `CapabilityStatus` 的 reliable + transient-local QoS，并写入匹配的 `observed_startup_state_seq`；缺资产/设备为 `NOT_READY`，配置/模型/helper 校验错误为 `ERROR`，旧 sequence retained status 不匹配新 STARTUP。preflight 只做 Vosk load、ADB `get-state` 和工具查询，不创建 task、不执行 ADB push/shell、不接管 R818、不播放 Prompt、不修改 mixer。
- 涉及文件：`src/perception/dog_patrol_perception_voice/`、`src/perception/scripts/check_perception_environment.py`、`src/perception/test/test_check_perception_environment.py`、`.github/workflows/ci.yml`、`README.md`、`src/perception/README.md`、`src/perception/requirements.md`、`docs/perception/voice/issue35_voice_readiness.md`。
- 验证：六包 `colcon build` 通过；完整 `colcon test` 通过（463 tests，0 errors/failures/skipped）；voice 包 48/48；新增 readiness/preflight 测试 10/10；环境检查 unittest 17/17；新增文件 flake8/pep257、compileall、launch 描述解析和 `git diff --check` 通过。mypy/pyright 未安装；全包 lint 仍保留 #33 原有长行。
- 后续：真实 Vosk model、R818/ADB 恢复、音频设备、现场语音效果、FAR/FRR 和 face provider 接入仍需现场验收；实现已在 PR #42 squash merge，合并提交为 `f6f683d`。

## 2026-08-06 15:10 - 实现 #34 异步 voice evidence provider

- 目标：领取并实现 GitHub #34，以当前 `MissionState` 驱动生产 voice provider，并接入现有 `AuthorizationEvidence` 和 `perception_authorization`。
- 完成：新增 `perception_voice_provider` ROS 2 executable；只对未阻塞 `VERIFY_IDENTITY`、正数 `target_id` 创建任务，同一 `state_seq + target_id` 去重；独立 worker 串行管理硬件 session，覆盖两轮 evidence、错误、取消、generation/session 迟到结果门禁和会话替换。为 `R818TaskSession`/R818 stream/Prompt player 增加非阻塞 stop seam，实际恢复仍由 context cleanup 负责；补齐 fake hardware + 真实授权 adapter ROS 端到端测试、安装依赖、CI 和 voice provider 文档。
- 关键结论：不新增 ROS msg，不修改 manager 状态机、orchestrator 聚合规则或 `required_not_passed=2`；ROS callback 不执行 Prompt、响应窗或恢复；旧 session cleanup 完成前不会启动替换 session，任意时刻最多一个硬件 session；恢复失败发布 `ERROR` 并锁存 provider 硬件故障，不继续启动下一 session。provider 不发布 capability readiness，真实模型、R818/ADB 和现场效果仍需部署验收。
- 涉及文件：`src/perception/dog_patrol_perception_voice/`、`.github/workflows/ci.yml`、`README.md`、`src/perception/README.md`、`src/perception/requirements.md`、`docs/perception/voice/issue33_voice_import.md`、`docs/perception/voice/issue34_voice_provider.md`。
- 验证：六包 `colcon build` 通过；六包完整 `colcon test` 通过（452 tests，0 errors/failures/skipped）；voice 单包 provider/核心测试通过（37 tests）；`compileall`、package XML、provider/prompt/test `ament_flake8`/`ament_pep257` 通过；全包 flake8 仍有 #33 原有长行，未新增 #34 provider/prompt lint 问题。
- 后续：在真实 R818、ADB、Prompt、Vosk model 上完成恢复、识别效果、FAR/FRR、voice capability READY 和整机联调验收。

## 2026-08-06 14:28 - 实现 #33 可移植 voice 核心

- 目标：领取并实现 GitHub #33，按 #32 冻结 allowlist 将 R818/Vosk 生产任务级核心迁入主仓。
- 完成：新增 `dog_patrol_perception_voice` ROS Python package，提供 `R818VoiceAdapter`、单 task 八通道流的一次接管/恢复、两次独立响应窗、Prompt player、六麦受限 Vosk、最小 ADB/Base64 transport、配置和 `VoiceWindowResult`；迁入并审计 ARM64 静态 helper/C 源码；补齐 fake seam 测试、安装 LICENSE/requirements、voice 环境门禁和迁移 provenance/依赖许可证文档。已在 GitHub 将 #33 指派给当前账号。
- 关键结论：生产默认不保存 PCM，两个窗口结果不在核心内聚合；Prompt 期间持续消费并丢弃完整帧，Base64/16-byte frame 错误、Vosk/硬件错误直接暴露；clean install 不读取源仓或旧源仓，Vosk 保持 lazy import。`python3-vosk` 没有 rosdep 规则，package.xml 不声明该 key，固定 wheel 由安装产物 `requirements.txt` 提供。
- 涉及文件：`src/perception/dog_patrol_perception_voice/`、`docs/perception/voice/issue33_voice_import.md`、`src/perception/scripts/check_perception_environment.py` 及其测试、感知 README/requirements、`LICENSES/README.md`。
- 验证：voice 与环境检查单测 37/37；Ruff 通过；全量 ROS build/test 6 packages、438 tests，0 errors/failures/skipped；安装后模块路径、helper SHA-256 `c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52`、ELF64 LE AArch64 static、LICENSE/requirements/C/config/source isolation 均通过；PR #40 的 CI `build-and-test` 通过并 squash merge 为 `3a7372a`。
- 后续：真实 R818/ADB/设备恢复、Vosk 模型效果、FAR/FRR、ROS voice evidence provider/READY 和现场验收属于后续联调；#33 已评论并关闭。

## 2026-08-06 13:05 - 冻结语音部署候选基线

- 目标：完成 #32，从语音源仓提交 `b979a7f` 固定部署候选、迁入/排除边界和后续可复现验证基线。
- 完成：在本地 `moonshine_voice_commands` 保持 `main` 不变，创建 `deploy/dog-patrol-integration`；两分支均指向 `b979a7fd33aac5c9ced9591bb507e483faf4aef5`。主仓新增语音基线审计、逐文件资产清单和完整 Python 依赖冻结，记录 BSD-3-Clause 迁入边界、R818 task-level Base64、最小 ADB、六麦 Vosk、Prompt/helper 的 allowlist，排除 Moonshine、sherpa-onnx、通用 CLI、capture/evaluation/benchmark、origin.pcm/tmpfs/manual wakeup、模型、数据和构建状态。
- 关键结论：候选分支未发生代码或配置差异，因而不触发基线前后 PCM/model/config 行为比较；57 个历史 PCM、14 文件 Vosk model、产品配置、依赖版本、helper 和运行参数已记录 manifest/hash。依赖快照使用 `env -u PYTHONPATH`，避免 ROS overlay 污染。按用户要求移除 `Protect main` 的独立 review/approval 门禁，但保留 PR、squash、线性历史、CI、禁止删除和禁止强推；PR #39 已合并，#32 已关闭。源仓当前没有 Git remote，部署分支仍仅在本地。
- 涉及文件：`docs/issue32_voice_deployment_baseline_audit.md`、`docs/issue32_voice_deployment_manifest.sha256`、`docs/issue32_voice_deployment_dependency_freeze.txt`、`README.md`、本地 `worklog.md`；源仓仅新增本地 branch ref。
- 验证：源仓 `./tools/check.sh` 通过（143 tests、Ruff、formatter、config check）；源仓 `main...deploy/dog-patrol-integration` 无差异；主仓五包 build 通过，source `/opt/ros/humble` 与本仓 `install/setup.bash` 后完整 `colcon test` 通过（50/50 tracking CTest，manager 14、orchestrator 17 及接口包检查通过）。未 source 本仓 install 的首次整套测试曾使 mission integration 误报失败，显式 source 后单测和整套测试均通过。
- 后续：#33 从冻结分支按 allowlist 提炼可移植 voice package。不得整体迁入源仓，也不得将本机 PCM/model 提交进 Git。

## 2026-08-06 10:24 - 主仓全量回放旧视觉仓三段视频

- 目标：使用当前主仓 detection/tracking 对旧视觉仓 `datasets/orin_hik_h264_MOT/{01,02,03}` 三段视频做全量回放，导出可目检的叠加视频，并核对迁入后是否相对旧仓冻结基线回退。
- 完成：在独立目录以 Orin full-runtime Release 配置构建当前 `main`（`e264880`），用 production validator 加载原 TensorRT engine，显式采用当前 `bot_sort.yaml`、GMC off、tracker/SID light 完成 1136、947、539 帧回放；每段均导出 1280x1024、30 FPS、FFV1 无损 `eval_overlay.mkv` 及全套 CSV/JSON；三段视频均通过整段解码，代表帧目检可见检测框、track/semantic ID、primary 状态叠加正常。
- 关键结论：主仓与旧仓最接近的冻结 GMC-off 基线在三段视频上的总帧数、检测/跟踪阳性帧、LOCKED/OCCLUDED/LOST、主目标切换和 LOCKED-to-LOST 全部一致；`per_frame.csv`、`det_raw.csv`、`det_filtered.csv`、`identities.csv` 逐字节一致，其余行为日志只有最大 `1e-6` 的浮点格式差异，无文本、ID、状态或决策差异。因此未发现迁入导致的输出回退；这是相对冻结基线的一致性证明，不是带人工真值的绝对精度评测。主仓回放吞吐为 10.217/10.435/9.884 FPS，略低于旧基线 10.834/11.139/10.469 FPS。
- 涉及文件：本地 `worklog.md`；忽略目录 `data/eval_results/main_e264880_orin_hik_h264_010203_20260806_101420/` 下三段视频和评测明细。
- 验证：Orin full-runtime Release 构建通过；TensorRT engine production load 通过；三段回放返回 0；三段 FFV1 视频 `ffmpeg` 全量解码均通过；新旧 CSV 结构、逐单元文本与数值容差比对通过；去除数据集命名差异后 `identity_metrics.json` 完全一致；视频 SHA-256 分别为 `d15b124e...c11c3d7e`、`a9b1c78d...0c57fe7e`、`797f94ea...60c688d5`。
- 后续：用户可直接播放三段 `eval_overlay.mkv` 做真实效果目检；如需浏览器/手机兼容版本，可在保留无损原件的前提下另行转码 H.264 MP4。

## 2026-08-04 21:41 - 调整为仅冻结旧仓部署分支

- 目标：按用户确认取消旧仓 `main` 的冻结限制，并保留 `dev` 可写，只冻结部署基线。
- 完成：ruleset 20383216 重命名为 `Freeze deployment baseline` 并收窄到 `deploy/dog_patrol-integration`；短暂禁用规则完成两条基线 README 纠正后立即重新启用；旧仓文档明确 `main`、`dev` 和按需创建的 `research/*` 均可用于非生产研究；主仓稳定文档经 PR #31 修正并合并。
- 关键结论：旧仓只有部署分支不可更新、删除或强推；`main` 和 `dev` 不命中任何 ruleset。旧仓所有可写分支仍是非生产研究面，不能替代 `dog_patrol` 的正式实现和部署入口。
- 涉及文件：`README.md`、`docs/perception/tracking/issue3_spec_acceptance.md`、`docs/perception/tracking/issue15_authoritative_entry_archive.md`、本地 `worklog.md`；旧仓 `main` 与部署分支的 `README.md`；旧仓 GitHub ruleset 20383216。
- 验证：GitHub branch-rules API 对 `main`、`dev` 均返回空数组，仅部署分支返回 update/deletion/non-fast-forward；ruleset active、无 bypass；PR #31 CI 通过并 squash merge；两个仓库与远端同步且无 tracked 改动。
- 后续：非生产研究可直接使用旧仓 `main`、`dev` 或独立 `research/*`；正式交付继续整理到 `dog_patrol`。

## 2026-08-04 21:28 - 开放旧视觉仓库非生产研究分支

- 目标：在保持 `dog_patrol` 为正式 tracking 权威入口的同时，允许旧 `vision_demo_ws` 创建远端研究分支，并冻结旧仓生产基线。
- 完成：取消旧仓 GitHub archived 状态；在旧仓 `main` 和 `deploy/dog_patrol-integration` 分别更新研究治理告示；建立无 bypass 的 active ruleset `Freeze production baselines`，禁止两条基线更新、删除和强推；明确新实验统一走 `research/*`，产品化成果必须重新通过 `dog_patrol` PR、CI 和评审；主仓稳定文档经 PR #30 更新并合并。
- 关键结论：权威入口与仓库归档是独立概念。旧仓当前是非生产研究空间，不是构建、发布或部署来源；`main` 和部署分支不可写，`research/*` 可写。
- 涉及文件：`README.md`、`docs/perception/tracking/issue3_spec_acceptance.md`、`docs/perception/tracking/issue15_authoritative_entry_archive.md`、本地 `worklog.md`；旧仓两个基线分支的 `README.md`；旧仓 GitHub ruleset 20383216。
- 验证：旧仓 API `isArchived=false`；对 `main` 和部署分支的实际测试 push 均以 GH013 拒绝；临时 `research/governance-smoke` 可创建并已删除；PR #30 `build-and-test` 通过并 squash merge；两个本地仓库与对应远端基线同步且无 tracked 改动。
- 后续：旧仓新实验从最新适合的基线创建 `research/*`；正式采用时不要合并旧仓基线，而应按当前主仓接口整理独立 PR。

## 2026-08-04 21:08 - 将 worklog 改为仅本地文件

- 目标：纠正 `worklog.md` 被误提交到远端的问题，只在当前工作区保留日志。
- 完成：从 Git 索引删除 `worklog.md`，通过 PR #29 合并到 `main`；合并后从删除前版本恢复本地副本，继续由 `.git/info/exclude` 的 `/worklog.md` 规则忽略。
- 关键结论：ignore 规则只作用于未跟踪文件；此前 #4 实现绕过 ignore 首次加入后，后续提交持续跟踪。当前远端 `main` 已不再包含该文件，但旧提交历史仍保留曾提交的内容。
- 涉及文件：本地 `worklog.md`；远端删除 `worklog.md`。
- 验证：PR #29 CI `build-and-test` 通过并 squash merge；`origin/main` tree 与 GitHub Contents API 均确认文件不存在；本地文件存在、命中 exclude，工作区无 tracked 改动。
- 后续：若需从旧提交中彻底清除内容，必须另行执行历史重写；普通开发不得再强制添加该文件。

## 2026-08-04 20:14 - 完成 detection/tracking 接入父 Spec 整体验收

- 目标：独立验收 #3 的 Problem/Solution、实现与测试决定、Out of Scope 及 #4–#15 的实际关闭/合并证据，确认是否可以关闭父 Spec。
- 完成：逐项核对代码、稳定文档、12 个子票、主仓 PR #16–#27、旧仓 PR #95/#96、CI、116-commit 迁移 tag、当前 Orin/Hik 现场记录、旧仓 archive API 和分支锚点；首次 Spec 轴审查发现授权输出 seam 未交付，补充感知内部 `AuthorizationEvidence`、通用 authorization ROS adapter 和真实 DDS 外部-interface 测试；新增稳定验收矩阵，并修正 #15 文档中父票状态的历史表述；修复后 Standards/Spec 双轴复审均无 finding，提交 PR #28。
- 关键结论：detection/tracking 正式迁入、默认 light/可选 ONNX、orchestrator/interfaces、mission/standalone、crop/readiness、部署 requirements、真实 Orin/Hik 和权威仓库切换均已闭环。真实 face、voice 及其 evidence producer、导航算法和导航 Orin 整机验收仍明确保留为后续，不属于本 Spec 的完成声明；生产整体 READY 和授权结果仍不能绕过缺失 provider。
- 涉及文件：`README.md`、`src/perception/README.md`、`src/perception/requirements.md`、`src/perception/dog_patrol_perception_interfaces/`、`src/perception/dog_patrol_perception_orchestrator/`、`docs/perception/tracking/issue3_spec_acceptance.md`、`docs/perception/tracking/issue15_authoritative_entry_archive.md`、`worklog.md`。
- 验证：当前 `main=origin/main=19c4b65`；隔离 portable 五包最终 build 通过，400 tests、0 errors/failures/skipped；orchestrator 17 tests 通过；环境检查器 13 项单测通过；主仓 PR #16–#27 对应检查均为 SUCCESS；旧仓 `archived=true` 且默认/冻结分支锚点可读；仓库资产/submodule/Fake/demo 活动入口扫描无异常。
- 后续：等待 PR #28 CI，通过后合并，在 #3 留下可追溯结论并关闭；随后确认本地 `main` 与 `origin/main` 同步且工作区干净。

## 2026-08-04 19:59 - 切换 tracking 权威入口并归档旧视觉仓库

- 目标：完成 #15，使 dog_patrol 成为 tracking 后续开发、构建、测试和部署的唯一权威入口，并在保留历史、回退锚点和本机资产的前提下归档旧视觉仓库。
- 完成：先在旧仓冻结分支和默认分支提交并推送权威入口告示；主仓补齐感知 CODEOWNERS、Orin full-runtime 构建/检查/standalone 连续入口、许可证/来源/回退/归档证据和 face/voice/navigation 后续边界；首次 clean-clone 验收发现并修复 mission adapter 默认 DDS domain 污染，以及 integration fixture 首次 discovery 漏收 capability 后不重试的问题；PR #25/#26 均通过 CI 并 squash merge；最终 merged-main clean clone 验收后将 `HowAreAllWell/vision_demo_ws` 设为 GitHub archived。
- 关键结论：主仓最终验收 SHA 为 `c60ea3e028fd4abbeb83e21e14360d7041834385`；旧仓 `isArchived=true`，默认分支告示为 `599bdfc`、冻结分支告示为 `4f3df15`，历史仍可读取。fixture 只在 STARTUP 以 200 ms 有界周期重送当前测试 readiness 样本，直到权威 PATROL ack；生产 readiness 行为和 perception READY exactly-once 断言未改变。face、voice 和导航整机验收仍未完成，父 Spec #3 保持开放。
- 涉及文件：`.github/CODEOWNERS`、`README.md`、`docs/perception/tracking/issue15_authoritative_entry_archive.md`、`src/perception/requirements.md`、`src/perception/dog_patrol_perception_tracking/CMakeLists.txt`、`src/perception/dog_patrol_perception_tracking/test/mission_pipeline_integration_driver.cpp`、`worklog.md`；旧仓 `/home/user/workspace/vision_demo_ws/README.md`。
- 验证：环境检查器 13 项通过；Standards/Spec 双轴审查及两轮修复复审无剩余 blocking finding；20 个全新 ROS domain 的 mission integration 20/20 通过；远端最终 main 全新 clone 五包 portable build 通过，399 tests、0 errors/failures/skipped；PR #25/#26 CI 通过；归档后 API 验证 `isArchived=true` 且两条历史分支 SHA 可读；主仓和旧仓本地均与远端同步且无 tracked 改动，旧仓 ignored 的 MVS 日志、模型、engine、ReID 和数据目录仍在。
- 后续：继续由独立问题接入真实 face/voice provider，并在最终导航 Orin 完成精确环境与整机验收；不得因 #15 关闭父 Spec #3。

## 2026-08-04 18:44 - 完成 tracking Orin 硬件验收

- 目标：完成 #14 的迁入后 tracking 真实 Orin/Hik/semantic/crop 性能与稳定性验收。
- 完成：继承并修正迁移后 Orin 脚本路径和 MVS aarch64 库探测；新增 production-path TensorRT engine validator，消除 trtexec 在加载无关插件时崩溃导致的统一检查误报；修复统一检查器无条件要求 ReID ONNX 的 #13 回归；full-runtime Release 构建、统一环境检查和 434 项测试通过；完成真实 Hik clean capture、standalone graph 隔离、真人 detector/tracker/semantic primary、`TrackedTargetImage` 全字段与同人 crop、真人离场停发及三轮锁频资源对照，并回填 verified baseline。
- 关键结论：正式资源轮统一为 MAXN、CPU 2.2016 GHz、GPU 1.3005 GHz 锁频和默认 tracker/SID `light` backend。生命周期/正常/慢消费者轮合计 703 条 crop 的 target ID、源时间/帧、原图 bbox/尺寸、BGR8 行列/字节均 0 错误，跨三轮八张抽样为同一真人；离场后立即停止 crop，约 6.07 秒失效门禁后清空 primary，随后约 35 秒没有旧消息。无业务消费者/正常/500 ms 慢消费者三轮全程 `LOCKED`，稳态均值为 30.012/30.011/29.992 FPS，慢消费者自身仅收 109 条但探针仍约 8.27 Hz，未反压 tracking；三轮 acquisition failure/MVS lost packet 均为 0。
- 涉及文件：`README.md`、`docs/perception/tracking/issue14_tracking_hardware_acceptance.md`、`src/perception/requirements.md`、`src/perception/scripts/check_perception_environment.py`、`src/perception/test/test_check_perception_environment.py`、`src/perception/dog_patrol_perception_tracking/CMakeLists.txt`、`src/perception/dog_patrol_perception_tracking/src/tools/validate_tensorrt_engine.cpp`、`src/perception/dog_patrol_perception_tracking/scripts/bench_hik_mvs_camera.sh`、`src/perception/dog_patrol_perception_tracking/scripts/check_orin_env.sh`、`worklog.md`。
- 验证：当前分支 full Orin Release build 通过；默认 light + 空模型路径的统一环境检查 PASS；检查器 13 项 Python 单测通过；五包 434 tests、0 errors/failures/skipped；真实 capture 61/61 frames；真人 live 字段校验 703/703 条通过且跨三轮 8 张 crop 目检一致；三轮各 60 个 tegrastats 样本、topic hz/bw、camera/inference/queue 指标和历史 539 帧 Hik 回归通过；Python `py_compile`、shell `bash -n` 和 `git diff --check` 通过。
- 后续：提交并完成最终 Standards/Spec 复审、PR CI 和合并收尾；现场原始日志、crop、参数、模型与录像继续只保存在忽略目录。

## 2026-08-04 16:30 - 交付感知域部署检查

- 目标：完成 dog_patrol #13，为感知 Orin 和最终导航 Orin 提供整个感知域的部署 requirements 与统一 PASS/FAIL 检查。
- 完成：新增领域级 `requirements.md`，按 tracking/face/voice/orchestrator 记录 implemented/integrating/not-integrated 状态，固化两台真实 Orin 边界、平台/SDK 基线、Hik MVS 输入、本机 engine/ReID 资产和部署参数责任；新增统一 Python 检查器，覆盖架构、JetPack/L4T/CUDA/TensorRT/ROS/MVS 精确版本、USB 相机枚举、TensorRT engine 实际加载、ReID ONNX/external data 的同 OpenCV C++ runtime 实际加载、必需参数、full-runtime CMake/install/test 结果与模块状态，关键缺口统一汇总后非零退出；脚本逻辑测试接入 CI。
- 关键结论：当前感知 Orin 实机核验为 aarch64、JetPack 6.2.1、L4T R36.4.7、CUDA 12.6、TensorRT 10.3、ROS 2 Humble、MVS SDK `0x04080003`，且 `MV-CU013-A0UC` USB ID `2bdf:0001` 可枚举；支持面为 JetPack 6.2/L4T R36.4，最终导航 Orin 精确版本待现场检查。环境检查 PASS 只表示当前已实现范围的部署前置完整，不伪造 face/voice readiness，也不替代 mission 感知整体 READY。
- 涉及文件：`README.md`、`worklog.md`、`.github/workflows/ci.yml`、`src/perception/README.md`、`src/perception/requirements.md`、`src/perception/scripts/check_perception_environment.py`、`src/perception/test/test_check_perception_environment.py`、`src/perception/dog_patrol_perception_tracking/README.md`、`src/perception/dog_patrol_perception_tracking/CMakeLists.txt`、`src/perception/dog_patrol_perception_tracking/src/tools/validate_reid_onnx.cpp`。
- 验证：检查器 11 项 Python 单测、`py_compile`、`ament_flake8`、CLI help 和 `git diff --check` 通过；当前默认空资产参数 + portable install 的实机检查正确汇总 6 个关键缺口并返回 1；`validate_reid_onnx` 链接 production OpenCV DNN，对缺失 ONNX 返回 1 和可操作诊断；独立目录 full Orin runtime 五包构建通过，CMake 为 `TRACKING_ENABLE_ORIN_RUNTIME=ON`且 live executable 可执行，全量 434 tests、0 errors/failures/skipped。
- 后续：部署机提供不入库的真实 engine、ReID ONNX/external data、相机序列号和参数文件后保存首次完整 PASS 输出；最终导航 Orin 到位后用 `--target navigation-orin` 回填精确版本。人脸/语音实现与生产 provider 仍待后续问题接入。

## 2026-08-04 15:49 - 异步交付 standalone 主目标 crop

- 目标：完成 dog_patrol #12，在 `PrimaryTargetObservation` 上建立可供独立人脸进程消费的 ROS 2 crop transport，且慢消费者不阻塞 tracking 主链。
- 完成：新增感知内部 `TrackedTargetImage` 消息并生成 C++/Python 类型；mission 和 standalone 共用 `TargetImageRosAdapter`，以 best-effort、volatile、keep-last(1) QoS 和专用 worker 的有界丢旧队列发布同帧自持有 `bgr8` crop；新增可配置 crop 外扩、最大发布频率、队列容量和 topic；generation 门禁使无可信 observation 时同时清空排队值并取消已出队但尚未发布的旧 crop，publication-start 线性化握手保证失效返回后不会再开始新的旧 crop publish，同时不等待已开始的 DDS 工作完成；runtime monitor 输出发布、queue drop 和 rate-limit 指标；补齐故意变慢的独立 DDS 消费者与快速 observer frame-loop smoke；记录 Orin `topic hz/bw` 与 tracking FPS 对比入口。
- 关键结论：消息携带 semantic target ID、源相机时间/帧号、原图 bbox/尺寸、confidence、编码与 crop bytes，不携带 mission `state_seq` 或完整相机帧；ROS adapter 位于 tracking 算法之外，核心 observer 继续只依赖领域类型和 OpenCV；队列拥塞丢旧保新，tracking frame thread 不执行 ROS 序列化或 DDS publish。
- 涉及文件：`README.md`、`worklog.md`、`src/perception/dog_patrol_perception_interfaces/msg/TrackedTargetImage.msg`、`src/perception/dog_patrol_perception_interfaces/CMakeLists.txt`、`src/perception/dog_patrol_perception_interfaces/package.xml`、`src/perception/dog_patrol_perception_tracking/CMakeLists.txt`、`src/perception/dog_patrol_perception_tracking/README.md`、`src/perception/dog_patrol_perception_tracking/config/perception_tracking_params.yaml`、`src/perception/dog_patrol_perception_tracking/include/dog_patrol_perception_tracking/modules/primary_target_observer.hpp`、`src/perception/dog_patrol_perception_tracking/include/dog_patrol_perception_tracking/modules/target_image_ros_adapter.hpp`、`src/perception/dog_patrol_perception_tracking/src/modules/primary_target_observer.cpp`、`src/perception/dog_patrol_perception_tracking/src/modules/target_image_ros_adapter.cpp`、`src/perception/dog_patrol_perception_tracking/src/perception_tracking_node.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_primary_target_observer.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_target_image_ros_adapter.cpp`、`src/perception/dog_patrol_perception_tracking/test/target_image_ros_smoke.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_target_image_ros_smoke.sh`。
- 验证：五包 portable 构建通过；全仓汇总 414 tests、0 errors/failures/skipped；其中 tracking 50/50 CTest 通过，覆盖 crop 消息映射、外扩与边界裁剪、generation 失效取消、publication-start 线性化、慢 publish worker 非阻塞和故意变慢的独立 DDS 消费者/快速 observer frame-loop smoke；并发 adapter 单测连续 10 次、DDS smoke 连续 3 次通过；Python 生成类型导入及无 `state_seq` 断言、`git diff --check` 通过。
- 后续：当前环境无 Orin SDK/Hik 相机，未测真实相机帧下 crop 序列化带宽与人脸消费者负载；应按 tracking README 的性能入口保存正常/故意降速消费者对比，若 tracking FPS 或带宽超预算再评估共享内存 adapter。

## 2026-08-04 15:13 - 建立 tracking standalone observation seam

- 目标：完成 dog_patrol #11，使 Orin tracking 在无 mission、导航和激光雷达时仍能运行 production pipeline，并产生当前主目标 observation。
- 完成：新增 ROS-independent `PrimaryTargetObserver` / `PrimaryTargetObservation` / `PrimaryTargetObservationSink`，输出 semantic target ID、源帧元数据、clamped 可信 bbox、confidence 和深拷贝目标图；standalone live 使用线程安全 latest sink 消费每帧结果，无当前可信目标时显式清空。live 节点新增统一 runtime strategy 封装强类型 `mission|standalone` 模式共有的初始化失败策略、current-primary 和 process-frame 行为，并以独立 capability-status / observation-lifecycle collaborator 隔离模式专属职责；standalone 不创建 `MissionRosAdapter`，复用原 camera/detector/tracker/identity/primary/visualizer 链路；新增独立 launch、preview/record 开关、部署参数入口和 Orin 相机说明；portable CI 增加 live node translation-unit 编译检查；Standards/Spec 双轴审查发现的分支散落、输出 data clump、可信判定重复、observation 未消费和 semantic ID 回退问题均已修复；mission integration fixture 在发布一次性 capability 前同时等待真实 readiness subscriber，消除全套顺序运行的 DDS discovery 竞态。
- 关键结论：standalone observation 不携带也不要求 mission `state_seq`；未分配 semantic ID、低分、遮挡嫌疑、刚重获、association gate 失败、无效/越界当前框或源图元数据不一致均不会产生 observation；每个 live tick 在采集前失效旧 sink 值，因此相机/处理失败也不泄漏历史 observation；目标图为当前 frame ROI 的深拷贝，不复用相机 buffer。standalone 不创建 mission subscription/publisher、capability publisher 或导航/任务推进输出。
- 涉及文件：`README.md`、`src/perception/dog_patrol_perception_tracking/include/dog_patrol_perception_tracking/source_frame_metadata.hpp`、`src/perception/dog_patrol_perception_tracking/include/dog_patrol_perception_tracking/modules/primary_target_observer.hpp`、`src/perception/dog_patrol_perception_tracking/src/modules/primary_target_observer.cpp`、`src/perception/dog_patrol_perception_tracking/src/perception_tracking_node.cpp`、`src/perception/dog_patrol_perception_tracking/launch/dog_patrol_perception_tracking_standalone.launch.py`、`src/perception/dog_patrol_perception_tracking/test/test_primary_target_observer.cpp`、`src/perception/dog_patrol_perception_tracking/test/mission_pipeline_integration_driver.cpp`、`src/perception/dog_patrol_perception_tracking/README.md`、`worklog.md`。
- 验证：五包 portable 构建和全量测试通过；tracking 48/48 CTest 通过（含 6 项 deterministic observation interface 测试和真实 supervisor mission integration）；portable build 成功编译 live node translation unit；standalone launch Python 语法和 `git diff --check` 通过。
- 后续：未在当前无 Orin SDK/Hik 相机环境运行完整 runtime 或最终性能验收；应在 Orin 按 standalone 文档提供本机 engine/参数并执行相机启动及 preview/record smoke。

## 2026-08-04 14:40 - 贯通感知 capability readiness

- 目标：完成 dog_patrol #10，将 tracking 自身 readiness 与感知整体 READY 的所有权拆开并贯通。
- 完成：新增感知内部 `dog_patrol_perception_interfaces/CapabilityStatus`；tracking 从真实 detector/tracker 初始化和运行状态发布带 STARTUP sequence 的 detection/tracking 状态，初始化失败时保持 tracer 存活、运行异常时先发布 ERROR 再尝试后续帧恢复；移除 authorization placeholder 和整体 READY 聚合；orchestrator 新增纯 Python 三能力聚合核心及 `perception_readiness` ROS 节点，将 detection/tracking、face、voice 固定为 required capability，并仅在三者对当前 STARTUP sequence 全部 ready 时发布一次感知 READY；CI 和公共 mission integration 已纳入新包与真实 orchestrator，测试 adapter 提供 face/voice 的 ready/not-ready/error 状态。
- 关键结论：capability transport 使用 reliable + transient-local QoS；每个 provider 保留自身当前状态，晚启动 orchestrator 可恢复三者状态。face/voice 尚无生产 provider 时不会产生状态，因此不会通过 placeholder 假装整体 ready。tracking 的 TARGET_CONFIRMED、bbox、TARGET_LOST 和 TARGET_REACQUIRED 路径未改变。
- 涉及文件：`src/perception/dog_patrol_perception_interfaces/msg/CapabilityStatus.msg`、`src/perception/dog_patrol_perception_orchestrator/dog_patrol_perception_orchestrator/readiness.py`、`src/perception/dog_patrol_perception_orchestrator/dog_patrol_perception_orchestrator/readiness_node.py`、`src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp`、`src/perception/dog_patrol_perception_tracking/src/perception_tracking_node.cpp`、`.github/workflows/ci.yml`、`README.md`、`worklog.md`。
- 验证：五包 portable 独立构建通过；全仓汇总 385 tests、0 errors/failures/skipped；其中真实 supervisor + orchestrator mission integration、晚启动 transient-local transport、ready/not-ready/error、旧/重复/错误状态门禁及原 tracking 目标/bbox lifecycle 均通过；`git diff --check` 和 integration shell `bash -n` 通过。
- 后续：真实 face/voice provider 仍待后续问题实现并发布同一 `CapabilityStatus`；当前测试 adapter 不安装到生产 package。

## 2026-08-04 14:16 - 闭合 tracking 公共 mission 合同

- 目标：完成 dog_patrol #9，在主仓同一工作区用真实 mission supervisor 验证 tracking 公共任务链路。
- 完成：将导入的 lifecycle fixture 收口为本仓稳定 mission contract tracer；fixture 启动安装后的真实 `dog_patrol_manager mission_supervisor`，以 production primary/frame transaction/ROS adapter 跑通 READY、目标确认、fresh bbox、丢失阻塞、同 semantic 目标重获解阻、VERIFY 和下一目标选择；新增旧序列、等序列冲突、错误 target 和错误目标 evidence 的负向门禁，并显式断言 tracking 不发布授权结论；补齐 bbox 原图尺寸断言和稳定说明。
- 关键结论：普通 CI 的最高公共 seam 为无资产 `test_mission_pipeline_integration`；它只使用主仓构建产物，不需要旧视觉 overlay、模型、录制或 Orin SDK。受控 Orin 环境仍可将真实 detector/tracker/identity observations 输入同一组断言，但不是普通 CI 前置条件。
- 涉及文件：`README.md`、`worklog.md`、`docs/perception/tracking/mission_contract_integration.md`、`src/perception/dog_patrol_perception_tracking/README.md`、`src/perception/dog_patrol_perception_tracking/test/mission_pipeline_integration_driver.cpp`、`src/perception/dog_patrol_perception_tracking/test/test_mission_pipeline_integration.sh`。
- 验证：portable 模式构建 interfaces、manager、tracking 三包成功；聚焦真实 supervisor lifecycle CTest 通过；全仓四包汇总 394 tests、0 errors/failures/skipped；`git diff --check` 和 integration shell `bash -n` 通过。
- 后续：未运行显式 Orin/Hik/模型/录制 visual replay；该硬件路径沿用迁移前受控验收方式，不影响普通 CI 合同闭合。

## 2026-08-04 13:48 - 保留历史导入 tracking 并接入 CI

- 目标：完成 dog_patrol #8，将准备完成的 tracking package 及必要历史正式导入主仓，并保证普通环境可独立构建测试。
- 完成：按 #4 白名单过滤并导入 `dog_patrol_perception_tracking` 的 116 个相关历史提交；package 落入感知域，稳定文档和工具随迁；从整个导入历史排除退役路径、资产和本机状态，并匿名化私网、RTSP、个人路径及设备序列号；补齐 Apache-2.0 全文、组件范围和来源锚点；默认配置不再硬编码本机资产；CI 显式关闭 Orin runtime 并将 tracking 加入必跑 build/test。
- 关键结论：来源锚点为 `vision_demo_ws` 的 `7878d70e6d86ad2a283911f8719345171b1c1d2a`，过滤后 tip 为 `6faaed42bf0531239b0203885607a9ff318eedc7`；因主仓只允许 squash merge，116 个相关提交固定在 annotated tag `tracking-import/vision-demo-ws-7878d70`，SHA 因路径与内容清洗而变化。tracking 保持 Apache-2.0，主仓其余未另行标注内容继续使用 BSD-3-Clause。portable 核心无需旧视觉仓库、旧 overlay 或 Orin SDK。
- 涉及文件：`src/perception/dog_patrol_perception_tracking/`、`docs/perception/tracking/`、`assets/models/manifests/tracking_core_requirements.txt`、`LICENSES/`、`.github/workflows/ci.yml`、`.gitignore`、`README.md`、`src/perception/README.md`、`worklog.md`。
- 验证：不 source 旧视觉工作区，在独立 build/install/log 目录构建 4 个 package 成功；tracking portable 47/47 CTest 通过；全仓汇总 379 tests、0 errors/failures/skipped；导入提交图的排除路径、二进制扩展、私网/RTSP/个人路径、设备序列号和常见凭据规则扫描无命中；从本分支全新 clone 后再次执行同一构建测试，379 tests 全绿。
- 后续：未运行 Orin runtime、真实 Hik 相机、TensorRT engine、录制或视频 replay；这些仍需受控硬件和本机资产。人脸/语音及 authorization readiness 的真实接入继续由后续问题处理。

## 2026-08-04 12:47 - 退役 Fake 感知并建立 orchestrator

- 目标：完成 dog_patrol #7，将 Fake 联调 package 收敛为正式感知业务编排模块。
- 完成：ROS package 和 Python namespace 改名为 `dog_patrol_perception_orchestrator`；删除 Fake 节点、launch、控制服务、console executable 及专属测试；保留并补强纯 Python `AuthorizationCoordinator` 行为测试；同步 CI 和正式文档；在 PR #16 合并前保留 #4 新增的 tracking 基线审计信息并解决 README 冲突。
- 关键结论：orchestrator 当前只拥有 ROS-independent 授权会话规则，不安装 ROS 节点或 launch；readiness 聚合、真实人脸/语音结果 adapter 和公共事件映射留待后续问题接入。
- 涉及文件：`.github/workflows/ci.yml`、`README.md`、`src/perception/README.md`、`src/perception/dog_patrol_perception_orchestrator/`、`worklog.md`；删除 `src/perception/dog_patrol_perception/` 的正式源码与测试。
- 验证：package 根运行 `python3 -m pytest -q test`，8 项通过；全新临时 build/install 下构建 3 个 package 并运行 `colcon test`，22 项通过；`ros2 pkg executables dog_patrol_perception_orchestrator` 为空，安装目录无 Fake 文件；GitHub CI 通过。
- 后续：按父规格 #3 的后续子问题接入 readiness 聚合和真实 tracking、人脸、语音结果 adapter。

## 2026-08-04 12:24 - 冻结并审计 tracking 迁移基线

- 目标：完成 dog_patrol #4，在历史导入前固定可重复、可追溯且不夹带本机状态的 tracking 迁移起点。
- 完成：逐项记录并按用户明确授权清理 vision_demo_ws 的回退前改动；从干净提交 `380b44582c0c55e5e46d2eb862da7700f05349b3` 创建并推送 `deploy/dog_patrol-integration`；固定迁入白名单、提炼/排除范围、历史对象与敏感信息门禁、Apache-2.0 保留方式；迁移改造前完成构建和全部既有测试。
- 关键结论：不能直接导入整个源仓或 package 历史；后续必须在临时 clone 中按白名单过滤，参数化私网端点、RTSP userinfo 模板和本机绝对路径，并补齐 Apache-2.0 全文与组件映射后复扫。源 Git 历史没有 ≥1 MiB blob 或模型/录像二进制，但忽略目录的大资产和 `worklog.md` 历史必须排除。本票未改变 tracking、identity、mission 或输出行为。
- 涉及文件：`README.md`、`docs/issue4_tracking_baseline_audit.md`、`worklog.md`；源仓远端新增 `deploy/dog_patrol-integration` 分支。
- 验证：source ROS 2 和 dog_patrol overlay 后，源基线 `colcon build --packages-select vision_demo_host` 通过；`colcon test --packages-select vision_demo_host --return-code-on-test-failure` 为 53/53 CTest 通过，汇总 392 tests、0 errors/failures/skipped。dog_patrol 三个现有 package 的独立 worktree 构建通过，测试汇总 29 tests、0 errors/failures/skipped；历史对象和规则敏感信息扫描结果见审计文档。
- 后续：后续迁移票在临时 clone 中执行过滤历史导入，并对导入后的新 commit 图二次复扫；真实 TensorRT/Hik 现场验收不属于本票。
