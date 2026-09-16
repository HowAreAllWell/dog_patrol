# 巡检父任务生命周期轻量化设计与实现规范

## 0. 文档目的与状态

本文给出当前 `origin/merge` 导航、人员感知、身份核验和巡检中断点恢复能力之上的父任务
生命周期设计。目标是在不推翻现有业务状态机、不重命名现有状态、不增加不必要分布式事务
协议的前提下，实现：

本文是当前实现的权威契约。实现与测试已经覆盖下文的 READY、父任务命令、状态序号、认证
取消、路径屏障和 fail-closed 规则；若实现、测试辅助代码或其他说明文档出现冲突，以本文
的约束为准。

- 启动巡检；
- 暂停巡检；
- 继续巡检；
- 结束当前巡检并返航；
- 立即终止巡检；
- 正常完成、返航完成、终止和失败后的统一收口；
- 父任务与自动身份核验子任务的状态联动。

## 1. 设计结论

本方案采用两个职责明确、由同一个 `mission_supervisor` 原子维护的状态层次：

1. 现有 `/mission/state` 继续表示导航和感知结合后的当前业务阶段；
2. 新增 `/patrol_task/state` 表示父任务生命周期、结束结果和导航路径所有权。

现有业务状态名称和数值全部保留，只在末尾追加一个 `READY=7`：

~~~text
STARTUP=0
PATROL=1
CONFIRM_TARGET=2
APPROACH_TARGET=3
VERIFY_IDENTITY=4
TRACK_INTRUDER=5
RECOVER_PATROL=6
READY=7
~~~

`READY` 在逻辑上位于 `STARTUP` 与 `PATROL` 之间，但数值追加在末尾，避免改变现有接口
常量、分支判断、录包数据和测试期望。

父任务只有四个运行状态：

~~~text
TASK_NO_TASK=0
TASK_RUNNING=1
TASK_PAUSED=2
TASK_RETURNING=3
~~~

父任务命令只有五个：

~~~text
START
PAUSE
RESUME
RETURN
TERMINATE
~~~

READY 只用于整套业务首次启动或显式 reset 后的初始化握手。PAUSE、RESUME、RETURN、
TERMINATE 和正常完成都不重新执行 READY 握手。

### 1.1 唯一正式生产入口

父任务生产控制入口只有：

~~~text
/mission/task_command
dog_patrol_interfaces/srv/PatrolTaskCommand
~~~

正式上位任务管理器通过这个服务串行发送 START、PAUSE、RESUME、RETURN、TERMINATE。
`mission_supervisor` 是唯一处理者和状态写入者。

以下接口不是父任务生产命令入口：

- `/mission/event`：导航、感知内部上报业务事实；
- waypoint clicked point、delete、replace、clear：只编辑路线数据；
- `/waypoint_sequence/pause`、`resume`、`resume_from_current`：仅允许作为临时内部调试兼容
  接口，不进入生产控制链；
- `/mission/reset`：仅用于开发复位和整套故障恢复；
- `robot_console/app.py`：可以调用同一个 `/mission/task_command` 做调试，但不得直接改状态、
  直接发布父任务路径或建立第二套命令 Topic。

路线准备和父任务控制是两个不同入口：上位系统先通过现有 waypoint 数据接口准备路线，等待
WaypointSequenceState 表明路线非空且未完成，然后调用 START。发布路线本身永远不能让机器人
运动。

生产调用顺序必须是：读取当前 route_revision，串行发送一条路线编辑，等待 typed
WaypointSequenceState.route_revision 被 uint32 wrap-aware helper 判定为更新，再发送下一条；最后一次编辑确认后停止发送编辑，
再调用 START。START 请求发出后到父任务结束前不得再发送 clicked/delete/replace/clear。
RViz 直接点击只用于人工调试，不能与生产 START 并发。这条单控制方串行约束是接口正确性的
组成部分。

## 2. 适用范围和运行假设

### 2.1 当前范围

本方案覆盖机器人端实时控制，包括：

- waypoint 巡检；
- 人员检测和目标锁定；
- 目标接近；
- 人脸和动态口令身份核验；
- 未授权人员跟踪；
- 返回巡检中断点；
- 父任务返航；
- 路径仲裁和停止链。

外部任务系统或上位机负责：

- 生成和保存业务父任务 ID；
- 生成和保存自动核验子任务 ID；
- 生成核验事件 ID；
- 保存命令调用、状态历史、告警和截图；
- 页面上的父子任务关系和结果展示。

机器人总控不生成、不保存、不解释这些业务 ID。

### 2.2 明确的简化假设

本方案依赖以下已经确认的运行方式：

1. 正式父任务命令只有一个上位任务管理器串行调用；
2. 路线编辑和 START 由同一个调用方按顺序操作，不并发编辑和启动；
3. 导航、感知或 supervisor 发生进程级故障时，由外部进程管理器重启整套业务；
4. 整套业务重启后不恢复旧父任务、旧路线、旧目标或旧认证会话；
5. 普通短时规划失败、传感器抖动和目标丢失继续由现有模块重试或恢复；
6. PAUSE 是业务停止，不是模块重启；
7. RESUME 是恢复业务执行，不是重新初始化感知和导航。

由于不支持节点独立热恢复和多控制方并发，本方案不增加：

- `runtime_epoch`；
- `task_generation`；
- `command_id`；
- `expected_state_seq`；
- `route_epoch`；
- 用于跨任务绑定的 `route_state_seq`；路线编辑仅保留 WaypointSequenceState.route_revision
  作为本地处理确认；
- `route_clear_epoch`；
- `cleared_route_epoch`；
- `target_episode_id`。

## 3. 系统对象和职责边界

### 3.1 父任务

父任务表示一次从 START 到以下任一结束点的机器人执行：

- waypoint 全部完成；
- RETURN 到达 home；
- RETURN 失败；
- TERMINATE；
- 整套业务发生进程级故障并重启。

父任务拥有：

- 当前父任务状态；
- 最近一次结束结果；
- START 时保存的 `home_pose`；
- PAUSE 后的恢复方向；
- 当前路线是否允许执行；
- 当前导航路径所有权。

### 3.2 路线

路线由 waypoint publisher 唯一拥有，包括：

- waypoint 列表；
- 当前下标；
- 已完成数量；
- 是否全部完成；
- waypoint planner 的本地 `sequence_version`。

父任务只读取路线状态决定 START 是否允许，不复制 waypoint 列表，也不成为路线数据的
第二个权威所有者。

### 3.3 身份核验子任务

机器人端不创建第二个可独立控制的任务状态机。父任务进入 `TASK_RUNNING + PATROL` 后，
身份核验能力自动生效；其搜索、接近、核验、跟踪和返回过程直接由现有业务状态表示。

上位系统按以下规则投影核验子任务状态：

| 父任务/业务状态 | 核验子任务展示状态 |
|---|---|
| `TASK_RUNNING + PATROL` | 搜索中 |
| `CONFIRM_TARGET`、`APPROACH_TARGET` | 发现目标/接近中 |
| `VERIFY_IDENTITY` | 核验中 |
| `TRACK_INTRUDER` | 跟踪中 |
| `RECOVER_PATROL` | 返回巡检中断点 |
| `TASK_PAUSED` | 已暂停 |
| `TASK_RETURNING` | 父任务返航中 |
| 父任务完成或终止 | 已结束 |

子任务没有独立的启动、暂停、继续、返航、终止或人工放行命令。

## 4. 状态模型

### 4.1 现有业务状态

现有状态语义保持不变：

| 状态 | 含义 |
|---|---|
| `STARTUP` | 整套业务启动或 reset 后，等待感知与导航首次 READY |
| `PATROL` | 正在执行 waypoint 巡检并搜索人员 |
| `CONFIRM_TARGET` | 已选定目标，等待导航形成稳定目标位置 |
| `APPROACH_TARGET` | 已保存巡检中断点，正在接近目标 |
| `VERIFY_IDENTITY` | 已到达并停车，正在执行身份核验 |
| `TRACK_INTRUDER` | 未授权目标，正在跟踪 |
| `RECOVER_PATROL` | 正在返回巡检中断点 |
| `READY` | 感知和导航已初始化完成，当前不执行目标业务 |

READY 必须是安全待命状态：

- 不选择新目标；
- 不接受认证会话；
- 不执行 waypoint；
- 不执行目标接近、跟踪或巡检恢复；
- 默认不输出任何控制路径；
- `target_id=0`。

唯一运动例外是 `TASK_RETURNING + READY`：READY 仍不执行巡检/目标业务，但父任务层通过
PatrolTaskState.navigation_mode=MISSION 允许返航控制器发布 home 路径。任何只订阅
MissionState 的业务模块仍然必须保持停止。

READY 是本次整套业务进程的“首次初始化完成锁存”，不是持续健康状态：

- 双 READY 进入该状态后，普通 CapabilityStatus 抖动不自动把状态退回 STARTUP；
- 普通规划失败、短时传感器异常和目标业务错误继续由各模块现有重试/恢复处理；
- 关键进程崩溃由外部进程管理器停止并重启整套业务；
- 只有整套重启或显式 `/mission/reset` 才重新进入 STARTUP 并重新执行 READY；
- PAUSE、RESUME、RETURN、TERMINATE 和父任务正常完成都不能触发 readiness preflight。

术语必须区分：MissionEvent.READY 是感知或导航在 STARTUP 中上报的一次“能力已准备”事件；
MissionState.READY 是 supervisor 收到两侧 READY 事件后形成的“安全待命”状态。READY 状态
不能反过来要求模块继续发布 READY 事件。

### 4.2 父任务状态

| 状态 | 含义 | 路线编辑 | 运动许可 |
|---|---|---:|---:|
| `TASK_NO_TASK` | 当前没有活动父任务 | 允许 | 禁止 |
| `TASK_RUNNING` | 正在执行巡检父任务 | 禁止 | 由业务状态决定 |
| `TASK_PAUSED` | 父任务暂停，保留路线和进度 | 禁止 | 禁止 |
| `TASK_RETURNING` | 已结束原巡检，正在返回 home | 禁止 | 仅允许返航 |

### 4.3 父任务结束结果

结束结果与运行状态分离：

~~~text
TASK_OUTCOME_NONE=0
TASK_OUTCOME_COMPLETED=1
TASK_OUTCOME_RETURNED=2
TASK_OUTCOME_TERMINATED=3
TASK_OUTCOME_FAILED=4
~~~

任务结束后统一进入 `TASK_NO_TASK`，`last_task_outcome` 保留结束原因，直到下一次 START
被接受时清为 `TASK_OUTCOME_NONE`。

结束结果只能由以下权威事实写入，不能由 UI 或普通诊断事件直接设置：

| 结果 | 唯一形成条件 | 形成后是否仍有活动父任务 |
|---|---|---:|
| `TASK_OUTCOME_NONE` | 初始启动、reset，或 START 已被接受但尚未结束 | 初始/reset 为否，START 后为是 |
| `TASK_OUTCOME_COMPLETED` | 当前活动路线的非空快照满足 `waypoint_count>0 && sequence_done=true` | 否 |
| `TASK_OUTCOME_RETURNED` | `TASK_RETURNING + READY` 下接受 `RETURN_REACHED` | 否 |
| `TASK_OUTCOME_TERMINATED` | 接受 `TERMINATE` | 否 |
| `TASK_OUTCOME_FAILED` | `TASK_RETURNING + READY` 下接受 `RETURN_FAILED`，或 RETURNING watchdog 超时 | 否 |

进程崩溃属于外部业务会话异常结束：机器人端不会有机会发布一个新的 `FAILED` 快照，
上位任务系统应在确认整套重启后结束自己的父任务记录。它不能把重启前最后一份
`last_task_outcome` 猜测成 `FAILED`。

每个非 `NONE` 终态结果形成都必须在 supervisor 同一把状态锁内完成一次完整收口：设置
`task_state=TASK_NO_TASK`、`navigation_mode=NAVIGATION_STOP`、清除实时目标和
`home_pose`，递增一次 `state_seq`，再用同一个序号发布两条状态。周期重发同一终态不得
再次递增序号或重复清理。

### 4.4 唯一合法组合

| 父任务状态 | 允许的业务状态 |
|---|---|
| `TASK_NO_TASK` | `STARTUP`、`READY` |
| `TASK_RUNNING` | `PATROL`、`CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER`、`RECOVER_PATROL` |
| `TASK_PAUSED` | `READY` |
| `TASK_RETURNING` | `READY` |

以下组合禁止出现：

~~~text
TASK_NO_TASK + PATROL
TASK_RUNNING + STARTUP
TASK_RUNNING + READY
TASK_PAUSED + STARTUP
TASK_PAUSED + VERIFY_IDENTITY
TASK_RETURNING + PATROL
TASK_RETURNING + RECOVER_PATROL
~~~

其中 `TASK_PAUSED + READY` 表示任务仍存在但业务停止；`TASK_NO_TASK + READY` 表示系统
已经准备好但没有任务；`TASK_RETURNING + READY` 表示原巡检和核验业务已经退出，返航由
父任务导航接管。

父任务结果还有以下字段约束：

- `TASK_NO_TASK + STARTUP` 的结果必须为 `TASK_OUTCOME_NONE`；
- `TASK_RUNNING`、`TASK_PAUSED` 和 `TASK_RETURNING` 的活动阶段结果必须为
  `TASK_OUTCOME_NONE`；
- `TASK_NO_TASK + READY` 可以是 `NONE`（尚未开始下一任务），也可以保留四种终态结果；
- 接受 START 会把结果从任何旧终态清为 `NONE`，不能在活动任务期间继续暴露上一个任务的结果。

### 4.5 全局状态序号

`mission_supervisor` 内部只保留一个全局 `state_seq`。以下任一字段发生权威变化时递增：

- 父任务状态；
- 业务状态；
- 目标 ID；
- 已处理目标 ID；
- 最近结束结果；
- home pose 有效性或数值；
- 导航路径所有权。

`/mission/state` 和 `/patrol_task/state` 每次都使用同一个 `state_seq` 发布。两个 Topic 的
网络到达顺序不保证一致，但相同 `state_seq` 表示同一份内部权威快照。

state_seq 只在一次 mission_supervisor 进程会话内单调，不承担跨整套重启的身份作用。机器人
内部所有状态消费者随 supervisor 一起重启并重置本地序号基线。常驻上位任务系统必须从统一
watchdog/进程管理器获得“业务栈已重启”边界，或检测 supervisor publisher GID 重新连接，然后
清除旧 state_seq 基线并接受新的 `NO_TASK + STARTUP`。没有确认重启边界时，不能仅因收到较小
seq 就猜测新会话。

state_seq 使用 uint32，0 保留为“尚未观察到状态”，有效序号为 1 到 `0xFFFFFFFF`，随后回绕
到 1。所有状态订阅者必须使用无符号半区间比较，不能直接使用 Python/C++ 普通 `<`：

~~~text
delta = (candidate - current) & 0xFFFFFFFF
same  = delta == 0
newer = 0 < delta < 0x80000000
older = delta >= 0x80000000
~~~

MissionEvent 仍要求 observed_state_seq 与当前值严格相等。Python mux、waypoint、navigation
coordinator 和上位机都必须复用同一个 wrap-aware helper；当前 C++ mission sequence 的半区间
比较可以作为参考。

安全关键节点不得自行拼接两个不同序号的状态作控制决策。具体规则是：

- 感知和目标导航只按 `/mission/state` 执行业务；
- path mux 只按 `/patrol_task/state.navigation_mode` 选择路径；
- waypoint 只按 `/patrol_task/state` 决定父任务许可；
- UI 或记录器需要组合显示时，只组合 `state_seq` 相同的两份快照。

所有接口边界采用 fail-closed 校验：

- state_seq=0 的运行状态快照无效；
- 未知 MissionState、task_state、last_task_outcome 或 navigation_mode 不得映射为运行；
- 不在第 4.4 节合法组合中的同序号快照不得执行运动或目标处置；
- mux 对未知 navigation_mode 选择 STOP；
- waypoint 对未知 task_state/navigation_mode 设置本地 paused=true；
- navigation/perception 对未知业务状态清目标并停止；
- consumer 只记录限频诊断，不自行篡改或发布替代权威状态。

### 4.6 父任务命令矩阵

| 当前组合 | START | PAUSE | RESUME | RETURN | TERMINATE |
|---|---:|---:|---:|---:|---:|
| `NO_TASK + STARTUP` | 拒绝 | 拒绝 | 拒绝 | 拒绝 | 拒绝 |
| `NO_TASK + READY` | 满足路线/home 条件时接受 | 拒绝 | 拒绝 | 拒绝 | 拒绝 |
| `RUNNING + 任意合法业务状态` | 拒绝 | 接受 | 拒绝 | 接受 | 接受 |
| `PAUSED + READY` | 拒绝 | 拒绝 | 路线合法时接受 | 接受 | 接受 |
| `RETURNING + READY` | 拒绝 | 拒绝 | 拒绝 | 拒绝 | 接受 |

拒绝命令不得改变 state_seq、路线、home、目标、恢复上下文或导航模式。命令并发时以
supervisor 同一互斥锁中的实际线性顺序为准；正式客户端仍必须串行调用。

### 4.7 `target_id` 与 `handled_target_id`

两个字段职责不同：

- `target_id` 是当前实时目标上下文，只在正在处置目标或普通目标恢复时为非零；
- `handled_target_id` 是最近一次已经形成最终业务结果的目标 ID，用于恢复 PATROL 后通知
  感知暂时不要再次选择该目标；
- supervisor 只保存最近一次 handled ID；多个已处理目标及各自过期时间由感知侧
  PrimaryTargetManager 的内部排除表保存。

设置规则：

| 业务结果 | target_id | handled_target_id |
|---|---:|---:|
| 新 TARGET_CONFIRMED | 新目标 ID | 0 |
| AUTHORIZED | 当前目标 ID | 当前目标 ID |
| UNAUTHORIZED 进入 TRACK | 当前目标 ID | 0 |
| TRACK_INTRUDER 最终 TARGET_LOST | 当前目标 ID | 当前目标 ID |
| CONFIRM/APPROACH/VERIFY 提前 TARGET_LOST | 当前目标 ID，进入普通恢复时暂留 | 0 |
| 普通 EXECUTION_ERROR 进入恢复 | 当前目标 ID | 0 |
| 普通 RECOVER 完成进入 PATROL | 0 | 保留原值 |
| PAUSE | 0 | 保留原值 |
| RESUME 到 PATROL 或暂停恢复 RECOVER | 0 | 保留原值 |
| RETURN、TERMINATE、COMPLETED、FAILED、reset | 0 | 0 |
| 新父任务 START | 0 | 0 |

MissionState 合法字段组合：

| 业务状态 | target_id | handled_target_id |
|---|---:|---:|
| STARTUP | 0 | 0 |
| READY + NO_TASK/RETURNING | 0 | 0 |
| READY + PAUSED | 0 | 0 或暂停前已经成立的 handled ID |
| PATROL | 0 | 0 或最近处理完成的 ID |
| CONFIRM/APPROACH/VERIFY/TRACK | 必须大于 0 | 0 |
| 普通 RECOVER_PATROL | 必须大于 0 | 0 或等于 target_id |
| 暂停恢复 RECOVER_PATROL | 0 | 0 或暂停前已经成立的 handled ID |

感知进入新的 PATROL state_seq 时读取 `handled_target_id`。若大于 0，将其插入内部排除表；
周期重发相同 state_seq 不得重复创建新的巡检周期。排除表可以同时保存多个 semantic ID，选择
primary target 时跳过表中所有 ID。

插入必须是“仅不存在时新增”：如果同一个 handled ID 已在排除表中，PAUSE/RESUME 或其他
PATROL 重入不得覆盖其已有 absence 起始时间。目标过期后会从表中删除；将来同一 semantic ID
再次被完整处置时，才会作为新条目重新开始排除周期。

当前排除时限语义保持为“目标持续离开画面 60 秒”：目标仍可见时不开始或重置 absence timer；
只有持续不可见达到 `handled_ignore_absence` 后才从排除表删除并重新具备选择资格。这个计时属于
感知内部状态，不由 supervisor 修改 handled_target_id 来计时。

absence timer 只在 MissionState.PATROL 的有效感知帧中推进；READY、RECOVER_PATROL、目标活动
阶段以及无 MissionState 时冻结。PAUSE 持续时间不计入“离开画面 60 秒”，RESUME 回 PATROL 后
继续原有计时；这保证暂停不会在后台改变下一次目标选择资格。

必须区分“消息字段清零”和“感知排除表清空”：

- 新 TARGET_CONFIRMED、RETURN、TERMINATE、COMPLETED、FAILED 和新 START 会把 supervisor
  当前 `handled_target_id` 字段清零；
- 上述普通父任务边界不强制删除 PrimaryTargetManager 中尚未过期的多个排除项，它们继续按
  absence timeout 自然释放，避免立即开始的新任务重复选择刚处理且仍在现场的人；
- `/mission/reset` 进入 STARTUP 时必须显式清空感知排除表和当前 primary 上下文；
- 整套进程重启会自然清空内存排除表；
- 相同 READY/PATROL 快照的周期重发不得清空或重新计时排除表。

PAUSE/RESUME 时 supervisor 保留 handled_target_id，是为了暂停发生在已完成目标的
RECOVER_PATROL 或刚恢复到 PATROL 时，不因暂停丢失排除结论。它不代表恢复旧目标：
MissionState.target_id 仍为 0，face、voice、tracking 不得创建目标会话。

连续处理目标的示例：

~~~text
目标 87 处理完成 -> supervisor 发布 handled_target_id=87
回到 PATROL      -> 感知排除表加入 87
开始处理目标 99  -> supervisor 字段清为 0；感知表中的 87 仍保留
目标 99 处理完成 -> supervisor 发布 handled_target_id=99
回到 PATROL      -> 感知排除表加入 99

最终：supervisor 字段只保存 99
      感知排除表可以同时保存 87、99
~~~

### 4.8 READY 消费者契约

MissionState.READY 只说明“当前没有活动的巡检/目标业务阶段”，不能单独判断父任务是空闲、
暂停还是返航。各模块必须按下表消费：

| 模块 | 收到 MissionState.READY 的动作 | 是否还需要 PatrolTaskState |
|---|---|---|
| perception tracking | 清当前 primary，不选新目标，不发 bbox/目标事件 | 否 |
| face/voice/authorization | 取消活动会话，拒绝旧证据 | 否 |
| readiness nodes | 退出或抑制 STARTUP preflight，不发布新 READY | 否 |
| navigation target/fusion | 清实时目标和目标 planner，保持安全停止 | 否 |
| navigation coordinator | 先停止目标业务；RETURNING 时运行返航 controller | 是 |
| waypoint publisher | 不根据 MissionState.READY 自行判断暂停/清理/执行 | 是，唯一执行门控 |
| path mux | 不直接订阅 MissionState 选择 source | 是，只读 navigation_mode |
| UI/记录器 | 用相同 state_seq 组合显示 NO_TASK、PAUSED 或 RETURNING | 是 |

READY 本身绝对不能触发：

- waypoint 开始或恢复；
- 感知重新搜索人员；
- readiness 重新握手；
- 清空暂停任务路线；
- 返航开始。

这些动作分别只能由 `TASK_RUNNING + navigation_mode`、STARTUP、父任务终态和
TASK_RETURNING 驱动。

## 5. ROS 2 接口

### 5.1 `/mission/state`

类型继续为 `dog_patrol_interfaces/msg/MissionState`。现有常量和字段不修改，只追加：

~~~text
uint8 READY=7
~~~

最终接口为：

~~~text
uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5
uint8 RECOVER_PATROL=6
uint8 READY=7

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
string detail
uint32 handled_target_id
~~~

唯一发布者仍为 `mission_supervisor`。

字段顺序和已有数值不变，但 `handled_target_id` 的接口注释必须同步更新：它可以保留在
PAUSED+READY、暂停恢复型 RECOVER_PATROL 和随后 PATROL 中；在暂停恢复 RECOVER 中不要求等于
target_id（此时 target_id=0）。程序以第 4.7 节合法组合为准，不能继续沿用“RECOVER 时 handled
必须等于 target”的旧注释约束。

### 5.2 `/patrol_task/state`

新增 `dog_patrol_interfaces/msg/PatrolTaskState`：

~~~text
uint8 TASK_NO_TASK=0
uint8 TASK_RUNNING=1
uint8 TASK_PAUSED=2
uint8 TASK_RETURNING=3

uint8 TASK_OUTCOME_NONE=0
uint8 TASK_OUTCOME_COMPLETED=1
uint8 TASK_OUTCOME_RETURNED=2
uint8 TASK_OUTCOME_TERMINATED=3
uint8 TASK_OUTCOME_FAILED=4

uint8 NAVIGATION_STOP=0
uint8 NAVIGATION_WAYPOINT=1
uint8 NAVIGATION_MISSION=2

std_msgs/Header header
uint32 state_seq
uint8 task_state
uint8 last_task_outcome
uint8 navigation_mode
bool home_pose_valid
geometry_msgs/PoseStamped home_pose
string detail
~~~

说明：

- `navigation_mode` 是 supervisor 根据父任务和业务状态计算的控制许可；
- `home_pose` 在 RUNNING、PAUSED 和 RETURNING 中有效；
- `TASK_NO_TASK` 时 `home_pose_valid=false`；
- `last_task_outcome` 在 RUNNING、PAUSED、RETURNING 中必须为 `TASK_OUTCOME_NONE`；
  NO_TASK 下为 NONE 表示尚未开始下一任务，非 NONE 表示上一父任务的终态结果；
- READY 不需要额外的 `system_ready` 字段，`MissionState.state=READY` 已经表达就绪；
- `detail` 仅用于日志和显示，程序不得解析文本决定业务逻辑。

推荐 QoS：

~~~text
reliability: RELIABLE
durability: TRANSIENT_LOCAL
history: KEEP_LAST
depth: 1
~~~

### 5.3 `/mission/task_command`

新增 `dog_patrol_interfaces/srv/PatrolTaskCommand`：

~~~text
uint8 START=0
uint8 PAUSE=1
uint8 RESUME=2
uint8 RETURN=3
uint8 TERMINATE=4

uint8 command
---
bool accepted
bool changed
uint32 state_seq
uint8 task_state
uint8 mission_state
uint8 last_task_outcome
string error_code
string message
~~~

服务语义：

- `accepted=true` 表示命令通过校验并已在线性化点生效；
- `changed=true` 表示权威状态发生变化；
- 返回成功不表示 RETURN 已经到家；
- 所有拒绝均不得修改任何任务、路线、目标或版本状态；
- 调用超时后，客户端先读取两个状态 Topic，再决定是否重试；
- 不提供命令历史缓存和 `command_id`。

各命令成功响应的边界：

| 命令 | `accepted=true` 表示 | 不表示 |
|---|---|---|
| START | 已保存 home，并发布 RUNNING+PATROL | waypoint 已经产生路径或机器人已经运动 |
| PAUSE | 已发布 PAUSED+READY 和 STOP 权威状态 | 底盘已经物理停稳 |
| RESUME | 已发布 RUNNING+PATROL/RECOVER | 新路径已经到达 mux |
| RETURN | 已发布 RETURNING+READY，返航上下文已建立 | 已经到家 |
| TERMINATE | 已发布 NO_TASK+READY 和 STOP 权威状态 | 所有下游异步清理已经完成 |

需要观察物理停稳、路线已经清空或返航已经到达时，调用方分别读取运动反馈、
WaypointSequenceState 或最终 PatrolTaskState；不能从一次服务响应推断这些异步事实。

### 5.4 `/waypoint_sequence/state`

新增 `dog_patrol_interfaces/msg/WaypointSequenceState`：

~~~text
std_msgs/Header header
uint32 route_revision
uint32 waypoint_count
uint32 current_index
uint32 completed_count
bool sequence_done
bool paused
~~~

这个消息只表达路线当前内容修订和进度。它不携带父任务 ID、任务代次、路线实例 ID 或清理
确认字段。

推荐 QoS：可靠、transient-local、keep-last 1。supervisor 在 START 前读取最新快照。

该 Topic 只有 waypoint publisher 一个写入者。`completed_count`、`current_index` 和
`sequence_done` 的推进可以保持同一个 `route_revision`，因此 supervisor 不能只用
`route_revision` 的数值变化判断进度是否更新；它必须按这个单一 writer 的可靠消息顺序保存
最新完整快照，并在自己的状态锁内处理路线事实。`route_revision` 只用于确认内容编辑和
区分编辑前后的快照，不是进度消息的独立序号。

`route_revision` 是 waypoint publisher 进程内的路线内容修订号：

- 从 1 开始，按 uint32 wrap-aware 规则递增并跳过 0；
- 添加、移动、替换、删除、手工 CLEAR、任务终态清理和 reset 清理实际改变路线内容时递增；
- current_index 推进、completed_count 改变、paused 改变和周期重发不递增；
- 它只用于确认 Topic 形式的编辑已经被 waypoint publisher 处理，不是路线业务 ID；
- START 请求不携带 expected_route_revision，supervisor 只读取最新路线快照；单控制方在最后一次
  编辑确认后停止编辑，因此不需要乐观锁；
- waypoint publisher 进程重启时 revision 可以重新开始，因为关键节点重启按整套新会话处理。

本地 planner `sequence_version` 与公开 route_revision 职责不同：planner version 可以在暂停、
状态门控变化和进度推进时递增；route_revision 只表示路线内容被修改，二者不能混为同一协议。

现有 `/waypoint_sequence/status` String 可以继续用于人类日志显示，但 supervisor、上位机和测试
不得解析字符串决定路线是否可执行；所有程序判断只使用 WaypointSequenceState。

合法进度关系：

| 路线情况 | waypoint_count | current_index | completed_count | sequence_done |
|---|---:|---:|---:|---:|
| 从未配置或已清空 | 0 | 0 | 0 | false 或 true，均按“无路线”处理 |
| 有未完成点 | 大于 0 | 小于 waypoint_count | 等于 current_index | false |
| 非空路线全部完成 | 大于 0 | waypoint_count - 1 | waypoint_count | true |

实现必须按上述关系校验，而不是只检查每个字段各自落在整数范围内：

- `sequence_done=false` 时，`0 <= current_index < waypoint_count` 且
  `completed_count == current_index`；
- `sequence_done=true` 时，`current_index == waypoint_count - 1` 且
  `completed_count == waypoint_count`；
- `waypoint_count=0` 时，`current_index=0`、`completed_count=0`，无论
  `sequence_done` 为何值都只能解释为无路线；
- 任何不满足这些关系的快照都不能触发 COMPLETED，也不能作为 START/RESUME 的有效路线。

`completed_count` 不是“本次回调刚完成了几个点”的增量，而是当前路线快照中的持久累计进度。
删除、移动或替换 waypoint 后，publisher 必须先按原有完成语义重算这三个进度字段，再发布
新 `route_revision`；不能仅因为路线内容变了就把已完成点重新打开。

当前 publisher 清空路线时会得到 count=0、sequence_done=true；本方案不强制为了接口迁移修改
这个本地表示，但 supervisor 必须始终先判断 waypoint_count。只有
`waypoint_count>0 && sequence_done=true` 才是父任务正常完成，count=0 永远是 route_empty。

`paused` 表示 waypoint 执行门关闭，不等同于父任务 TASK_PAUSED：

- 只有 `TASK_RUNNING + NAVIGATION_WAYPOINT` 时为 false；
- NO_TASK、TASK_PAUSED、TASK_RETURNING 以及 RUNNING 下的 NAVIGATION_MISSION 时均为 true；
- supervisor 的 RESUME 前置条件不要求 paused 已经变为某个值，避免把它重新变成下游确认
  握手；真正执行仍由最新 PatrolTaskState 驱动。

移动 waypoint 不改变 completed_count；删除点时必须保持剩余点的完成语义，不能因为删除最后
一个点把前一个已完成点重新打开。WaypointSequenceState 的周期重发不改变这些字段。

### 5.5 `/mission/event`

现有事件名称和数值保持不变，追加：

~~~text
uint8 RETURN_REACHED=9
uint8 RETURN_FAILED=10
~~~

现有字段保持不变：

~~~text
std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 source
uint8 event
string detail
~~~

事件来源保持单一：

| 事件 | 允许来源 | 合法业务/父任务状态 |
|---|---|---|
| READY | 感知、导航 | `NO_TASK + STARTUP` |
| TARGET_CONFIRMED | 感知 | `RUNNING + PATROL` |
| TARGET_POSITION_READY | 导航 | `RUNNING + CONFIRM_TARGET` |
| ARRIVED_AND_STOPPED | 导航 | `RUNNING + APPROACH_TARGET` |
| AUTHORIZED、UNAUTHORIZED | 感知 | `RUNNING + VERIFY_IDENTITY` |
| TARGET_LOST | 感知 | RUNNING 下的 CONFIRM_TARGET、APPROACH_TARGET、VERIFY_IDENTITY 或 TRACK_INTRUDER |
| EXECUTION_ERROR | 感知、导航 | 任意当前业务状态，按第 13 节分类处理 |
| PATROL_RECOVERY_COMPLETE | 导航 | `RUNNING + RECOVER_PATROL` |
| RETURN_REACHED、RETURN_FAILED | 导航 | `RETURNING + READY` |

任何模块都不得代替另一个来源发布业务事件；父任务的五个外部命令不伪造成 MissionEvent。

新增事件规则：

- `RETURN_REACHED` 只能由导航在当前 `TASK_RETURNING + READY` 下发布；
- `RETURN_FAILED` 只能由导航在当前 `TASK_RETURNING + READY` 下发布；
- 两者都必须携带最新观察到的 `state_seq`，且 `target_id=0`；
- 迟到事件因 `observed_state_seq` 不等于当前 `state_seq` 被拒绝。

现有 `PATROL_RECOVERY_COMPLETE` 增加一条明确规则：

- 普通目标处置进入 RECOVER_PATROL 时，MissionState.target_id 保持原目标 ID，完成事件必须
  携带相同的非零 ID；
- PAUSE/RESUME 创建 RECOVER_PATROL 时，MissionState.target_id=0，完成事件也必须为 0；
- target_id=0 的 RECOVER_PATROL 只能由 `TASK_PAUSED + READY` 下的 RESUME 命令创建，普通
  目标事件不得创建该上下文；
- 两种恢复都必须严格匹配当前 state_seq。

### 5.6 父任务调试复位

保留 `/mission/reset`，仅用于开发、测试和整套业务复位，不作为正式父任务命令。

reset 执行：

1. 取消当前所有业务动作；
2. 清空父任务、target_id、handled_target_id、home、中断点和路线；
3. 清空感知和导航 READY 记录；
4. 设置 `TASK_NO_TASK + STARTUP`；
5. `last_task_outcome=TASK_OUTCOME_NONE`；
6. 重新等待首次 READY。

reset 路线清理不能依赖 last_task_outcome，因为 reset 明确把 outcome 设为 NONE。waypoint
publisher 需要额外观察 `/mission/state`，但仅用于识别 reset 边沿：

~~~text
已经保存第一份 STARTUP 基线
  -> 后续收到更大且相同 state_seq 的 TASK_NO_TASK + STARTUP 两份快照
  -> 清空路线、完成状态和 planner 请求
~~~

节点启动后收到的第一份 `TASK_NO_TASK + STARTUP` 不视为 reset，不自动删除从参数加载的预配置
路线；这些路线仍受 NO_TASK 禁止运动约束。waypoint 不得使用 MissionState 控制普通暂停、恢复
或路径所有权，生产执行门控仍然只来自 PatrolTaskState。

两条 Topic 到达顺序不确定，waypoint 必须分别缓存最新快照。第一份匹配的
`TASK_NO_TASK + STARTUP` 只建立基线；此后每个更大且匹配的 STARTUP state_seq 都表示一次 reset，
执行且只执行一次清理。单独收到其中一条不得猜测 reset。

## 6. 导航路径所有权

`navigation_mode` 的计算必须集中在 supervisor，不让 mux 同时订阅并拼接两个业务状态。

| 父任务/业务状态 | navigation_mode | 允许的路径来源 |
|---|---|---|
| `NO_TASK + STARTUP/READY` | STOP | 无 |
| `RUNNING + PATROL` | WAYPOINT | `/waypoint_global_path` |
| `RUNNING + CONFIRM_TARGET` | WAYPOINT | `/waypoint_global_path` |
| `RUNNING + APPROACH_TARGET` | MISSION | `/mission_global_path` |
| `RUNNING + VERIFY_IDENTITY` | MISSION | `/mission_global_path`，通常为空并停车 |
| `RUNNING + TRACK_INTRUDER` | MISSION | `/mission_global_path` |
| `RUNNING + RECOVER_PATROL` | MISSION | `/mission_global_path` |
| `PAUSED + READY` | STOP | 无 |
| `RETURNING + READY` | MISSION | `/mission_global_path` 中的返航路径 |

不新增 `/return_global_path`。目标接近、跟踪、巡检恢复和父任务返航均由 navigation mission
coordinator 独占发布 `/mission_global_path`，但同一时刻只允许其中一种 plan kind。

mux 的安全规则保持并强化为按状态版本建立屏障：

1. 初始选择 STOP；
2. 收到更大的 `state_seq` 时，即使 navigation mode 与前一状态相同，也立即发布空
   `/global_path`；
3. 清掉旧 source 缓存；
4. 等待新 source 在新状态之后发布的新路径；
5. 不重放旧 waypoint 或 mission path；
6. STOP 下收到任何非空路径都不转发。

按 state_seq 而不是只按 source 变化清空非常重要。例如 APPROACH_TARGET 和 RETURNING 都
使用 MISSION source，但前者的目标接近路径绝不能在 RETURN 后继续生效；PATROL 和
CONFIRM_TARGET 都使用 WAYPOINT source，但状态切换后也只允许新状态下重新发布的路径。

空路径屏障之后必须有对应的路径重发责任，否则系统虽然安全停止，却可能永久等不到新路径：

- `NAVIGATION_WAYPOINT`：waypoint publisher 收到新 PatrolTaskState 后清除旧缓存，从机器人
  当前位姿重新规划并发布；
- `APPROACH_TARGET`、`TRACK_INTRUDER`：目标导航按现有重规划周期继续发布当前状态的新路径；
- `RECOVER_PATROL`：恢复控制器在到达前周期重发当前有效恢复路径；
- `TASK_RETURNING`：返航控制器在到家前以配置周期从机器人当前 map 位姿重新规划到固定
  home pose，并周期发布最新有效返航路径；不能只更新时间戳后反复发布首次规划得到的旧路径；
- `VERIFY_IDENTITY`：navigation coordinator 周期发布空 mission path/停止要求，不发布非空
  目标路径。

普通 `nav_msgs/Path` 不携带 state_seq，因此 mux 使用状态时间戳建立最小接受门槛：

~~~text
barrier_stamp = PatrolTaskState.header.stamp
只接受 active source 且 path.header.stamp > barrier_stamp 的非空路径
~~~

waypoint 和 navigation coordinator 每次发布非空路径时都必须用同一个 ROS clock 将
`path.header.stamp` 更新为实际发布时间。零时间戳、明显未来时间戳以及不晚于 barrier 的路径
均不得驱动机器人。mux 必须对 waypoint path 和 mission path 应用相同规则，不能只保护其中
一路。

空路径必须继续向下游形成完整停止链：

mux 自行生成的空路径必须保留有效的全局坐标系 `header.frame_id=map` 和当前 ROS 时间戳，
不能只发送 `poses=[]` 而把 frame_id 留空。否则 RViz 可能因坐标变换失败保留旧路径显示，
导致 TERMINATE、PAUSE 或 heartbeat 超时后的停止状态与画面不一致；正常完成和立即终止
都必须发送可被下游接收的空路径。

~~~text
mux 发布空 /global_path
  -> RL/PRIEST 清除全局路径缓存、subgoal 和旧 /local_path
  -> 控制适配器取消 FollowPath
  -> 底盘速度命令归零并保持停止
~~~

下游不得在新 global path 到达前重放旧 local path 或旧速度命令。PAUSE、TERMINATE 和任何
navigation mode 切换的验收都必须同时观察 global path、local path 和最终底盘命令。

## 7. 完整生命周期流程

### 7.1 整套业务启动

初始快照：

~~~text
task_state = TASK_NO_TASK
mission_state = STARTUP
last_task_outcome = TASK_OUTCOME_NONE
navigation_mode = NAVIGATION_STOP
target_id = 0
home_pose_valid = false
~~~

感知和导航分别在 STARTUP 下执行现有 readiness 检查并发布 READY。第一个 READY 只在
supervisor 内部记录，不改变 `state_seq`。两个 READY 都成立后：

~~~text
TASK_NO_TASK + STARTUP
  -> TASK_NO_TASK + READY
  -> state_seq + 1
~~~

进入 READY 后不再处理 READY 事件。重复或迟到 READY 只能记录调试日志，不改变状态。

STARTUP readiness 必须全部绑定当前同一个 state_seq：

- detection tracking、face、voice 发布的 CapabilityStatus.observed_startup_state_seq 必须等于
  当前 MissionState.state_seq；
- perception readiness aggregator 只有在全部必需 capability 都为 READY 且序号相同时，才
  发布 SOURCE_PERCEPTION/READY；
- navigation READY 的 observed_state_seq 也必须等于当前 STARTUP state_seq；
- supervisor 收到第一侧 READY 后不递增 state_seq，使另一侧仍可使用同一 STARTUP 序号；
- reset 进入新的 STARTUP 时 state_seq 已递增，旧 transient-local CapabilityStatus 和旧 READY
  因序号不匹配全部失效；
- READY 状态本身不接收 capability 状态来重新开关执行许可。

### 7.2 无任务时准备路线

只有 `TASK_NO_TASK` 允许新增、移动、删除或清空 waypoint。

READY 下编辑路线时：

- waypoint publisher 更新列表和进度；
- 可以执行内部可视化和状态发布；
- 不允许向控制链发布可执行 waypoint path；
- `auto_start_on_click` 不得绕过父任务 START；
- 新路线初始 `sequence_done=false`；
- 添加路线不会创建父任务。

生产巡检默认要求 waypoint 参数 `loop=false`，此时最后一个 waypoint 形成
`sequence_done=true` 并正常完成父任务。若明确启用 `loop=true`，路线不会产生正常完成事实，
父任务将持续 RUNNING，直到外部发送 RETURN 或 TERMINATE；UI 和验收用例必须按这一语义展示，
不能同时期待循环路线自动 COMPLETED。

### 7.3 START

START 只从 `TASK_NO_TASK + READY` 接受。

接受条件：

1. 当前确实为 `TASK_NO_TASK + READY`；
2. 最新 waypoint 状态已经收到；
3. `waypoint_count > 0`；
4. `sequence_done=false`；
5. `current_index < waypoint_count`；
6. `completed_count <= waypoint_count`；
7. current index 与完成进度合法；
8. supervisor 能读取 `map -> base_footprint` 当前 TF；
9. TF 时间不在未来且年龄不超过配置的 `home_pose_max_age`；
10. home pose 坐标和四元数有限，四元数范数有效。

处理必须在同一把状态锁内完成：

~~~text
校验命令和路线快照
  -> 获取并保存当前 home_pose
  -> 清除旧 target_id 和 handled_target_id
  -> 清除旧 resume_behavior
  -> last_task_outcome = NONE
  -> task_state = RUNNING
  -> mission_state = PATROL
  -> navigation_mode = WAYPOINT
  -> state_seq + 1
  -> 发布两份状态快照
  -> 返回服务成功
~~~

waypoint publisher 只有看到新的 `TASK_RUNNING + NAVIGATION_WAYPOINT` 后才规划当前未完成
waypoint。

无路线、路线已完成、home 获取失败或状态不正确时拒绝 START，且不得清空或修改路线。

### 7.4 正常目标处置

父任务处于 TASK_RUNNING 时，现有目标流程保持：

~~~text
PATROL
  -> CONFIRM_TARGET
  -> APPROACH_TARGET
  -> VERIFY_IDENTITY
  -> AUTHORIZED: RECOVER_PATROL
  -> UNAUTHORIZED: TRACK_INTRUDER
  -> TARGET_LOST: RECOVER_PATROL
  -> PATROL_RECOVERY_COMPLETE: PATROL
~~~

导航在 CONFIRM_TARGET 中准备发布 TARGET_POSITION_READY 之前，必须先成功保存当前机器人
位置为 `patrol_interruption_pose`，再发布事件。不能等收到 APPROACH_TARGET 状态后才第一次
捕获：TARGET_POSITION_READY 与紧随其后的 PAUSE 可能使 keep-last 订阅者跳过中间 APPROACH
快照。先捕获再发事件可以保证 supervisor 一旦进入 APPROACH，导航本地已经有对应中断点。

中断点必须绑定产生 TARGET_POSITION_READY 时的 CONFIRM_TARGET state_seq 和 target_id；状态
或目标已改变时不得继续使用。TF 捕获失败时不发布 TARGET_POSITION_READY，保持 CONFIRM_TARGET
并按现有节流规则重试；最终由确认超时进入无中断点恢复。恢复完成、恢复超时或父任务结束后
清除该中断点。

身份核验通过、核验超时或目标丢失只结束一次核验事件，不结束父任务。核验事件结束后先回
巡检中断点，再继续原 waypoint 路线。

### 7.5 PAUSE

PAUSE 只从 TASK_RUNNING 接受，且不要求 READY、planner 或感知当前健康；暂停必须随时能
关闭运动许可。

supervisor 在加锁后先读取暂停前业务状态：

~~~text
PATROL 或 CONFIRM_TARGET
    resume_behavior = PATROL

APPROACH_TARGET、VERIFY_IDENTITY、TRACK_INTRUDER 或 RECOVER_PATROL
    resume_behavior = RECOVER_PATROL
~~~

随后原子更新：

~~~text
task_state = PAUSED
mission_state = READY
navigation_mode = STOP
target_id = 0
保留 handled_target_id
保留 home_pose
保留 resume_behavior
state_seq + 1
发布两份状态
~~~

模块动作：

- mux 立即清空输出路径并保持 STOP；
- waypoint 取消当前 planner、发布空 waypoint path、保留列表和进度；
- navigation coordinator 取消目标、恢复 planner 和 FollowPath；
- navigation coordinator 清空 mission path 和实时目标位置；
- 感知停止选择目标并清除实时 bbox；
- face、voice、authorization 取消当前会话；
- 取消不得产生 UNAUTHORIZED 或 TARGET_LOST 业务结果。

中断点规则：

- supervisor 在 PATROL 或 CONFIRM_TARGET 接受 PAUSE 时设置 resume_behavior=PATROL；
- supervisor 在 APPROACH_TARGET 或其后接受 PAUSE 时设置 resume_behavior=RECOVER_PATROL；
- navigation coordinator 进入 PAUSED+READY 时不得只根据自己最后收到的 MissionState 决定
  清除 pose，因为 keep-last 可能跳过短暂的 APPROACH_TARGET；
- 暂停期间可以保留由当前 CONFIRM_TARGET state_seq/target_id 绑定的候选 interruption pose，
  但它不是恢复许可；
- RESUME 最终状态为 PATROL 时，导航清除所有候选/陈旧 interruption pose 和 recovery latch；
- RESUME 最终状态为 RECOVER_PATROL 时，导航只使用与当前暂停链匹配的候选 pose；
- 从 APPROACH_TARGET 或其后暂停，保留已经保存的 `patrol_interruption_pose`；
- 从 RECOVER_PATROL 暂停，取消当前恢复 planner，只保留 interruption pose；旧
  `return_arrived`、completion-sent 和 request-in-flight latch 必须失效；
- 若原状态应有中断点但 TF 捕获曾失败，仍保存 `resume_behavior=RECOVER_PATROL`，RESUME 后
  由恢复控制器检测无有效中断点并安全转回 PATROL。

PAUSE 不执行以下动作：

- 不把业务状态设为 STARTUP；
- 不清除首次 READY 结论；
- 不要求模块重新发布 READY；
- 不修改 waypoint 完成状态；
- 不清除 home pose；
- 不执行返航。

### 7.6 快速 PAUSE/RESUME

命令服务和事件回调必须使用 supervisor 当前已有的同一把互斥锁，形成唯一线性顺序。

如果 PAUSE 先获得锁：

~~~text
RUNNING -> PAUSED + READY，state_seq=N
随后 RESUME -> RUNNING + PATROL/RECOVER，state_seq=N+1
~~~

如果 RESUME 先获得锁，则因当前不是 PAUSED 被拒绝，随后 PAUSE 正常执行。

不引入 READY 重新确认。即使调用方在 PAUSE 成功后立即 RESUME，两个状态也通过同一发布者
的递增 `state_seq` 明确排序。旧认证事件、目标事件和 planner 回调因观察到旧 state_seq
而失效。

状态是期望控制状态，不承诺 PAUSE 服务返回时底盘已经物理静止；PAUSE 的线性化点会发布
STOP 权威状态，mux 收到后立即建立空路径屏障，之后任何旧路径均不得重新进入控制链。正式
上位客户端应等待看到 `TASK_PAUSED + READY` 后再允许用户操作 RESUME。若产品将来要求
“底盘已经物理静止”的完成确认，应由独立运动状态/里程计反馈表达，不把它混入父任务状态机。

### 7.7 RESUME

RESUME 只从 `TASK_PAUSED + READY` 接受，不等待 READY，不进入 STARTUP，也不增加
RESUMING 状态。

RESUME 前必须读取 supervisor 已保存的最新 WaypointSequenceState：

- `waypoint_count > 0 && sequence_done=true`：不得重新打开旧路线；该完成事实由路线状态回调
  收口为 COMPLETED；
- `waypoint_count == 0`：视为异常路线丢失，拒绝 RESUME、保持 PAUSED，由上位机选择 RETURN
  或 TERMINATE，不能把空路线误记成正常完成；
- current index 或 completed count 不合法：拒绝 RESUME、保持 PAUSED并报告
  `invalid_route_progress`；
- 只有非空、未完成且进度合法的路线才能继续。

`RESUME` 服务本身永远不直接写入 COMPLETED；路线状态回调是完成事实的唯一来源。若
WaypointSequenceState 回调先在线性化点处理了 `sequence_done=true`，父任务已经收口，RESUME
按 `no_active_task` 拒绝。若 RESUME 先获得状态锁，它只会按当时看到的未完成路线进入
PATROL/RECOVER；随后到达的持久完成快照再由路线状态回调收口。两种顺序都必须让最终权威
状态成为 `TASK_NO_TASK + READY + TASK_OUTCOME_COMPLETED`，客户端应以状态 Topic 为准，不能
把 RESUME 的服务响应当成完成确认。

无中断点恢复要求：

~~~text
TASK_PAUSED + READY
  -> TASK_RUNNING + PATROL
  -> navigation_mode = WAYPOINT
~~~

存在中断点恢复要求：

~~~text
TASK_PAUSED + READY
  -> TASK_RUNNING + RECOVER_PATROL
  -> navigation_mode = MISSION
  -> target_id = 0
~~~

共同动作：

- 保留 home pose；
- 保留 waypoint 列表、完成状态和 current index；
- target_id 保持 0，不恢复旧人员目标；
- `state_seq + 1`；
- 发布两份状态；
- 清除 supervisor 内部 `resume_behavior`，其作用已经转化为当前业务状态。

进入 PATROL 后 waypoint 重新规划当前未完成点。进入 RECOVER_PATROL 后导航从机器人当前
位置重新规划到保存的中断点；到达或恢复超时后再进入 PATROL。

每次 RESUME 进入 RECOVER_PATROL 都创建新的本地 recovery generation，重置 return_arrived
和 completion-sent；即使 PAUSE 前已经到达但完成事件尚未被 supervisor 接受，也由新 generation
根据当前位置重新确认并发布当前 state_seq 的完成事件，不能重放旧事件。

这里的 `target_id=0` 是“暂停恢复上下文”，不是一个新的人员目标。导航发布
PATROL_RECOVERY_COMPLETE 时使用 0；状态机只要确认当前 RECOVER_PATROL 确实由 RESUME 创建、
event target_id 与当前 target_id 相等且 state_seq 匹配，就接受完成并回到 PATROL。face、
voice、tracking 不得因为这个恢复上下文重新创建目标会话。

### 7.8 RETURN

RETURN 可以从 TASK_RUNNING 或 TASK_PAUSED 接受。它结束原巡检，不是暂停后可继续的状态。

接受条件：

- 当前父任务为 RUNNING 或 PAUSED；
- `home_pose_valid=true`；
- home pose frame 为 map 且姿态有效。

原子状态变化：

~~~text
task_state = RETURNING
mission_state = READY
navigation_mode = MISSION
target_id = 0
handled_target_id = 0
清除 resume_behavior
保留 home_pose
state_seq + 1
发布两份状态
~~~

下游动作：

1. 感知、认证和跟踪全部停止；
2. 目标接近和巡检恢复 planner 全部取消；
3. waypoint publisher 立即清空原巡检路线和进度；
4. navigation coordinator 清空旧 mission path；
5. navigation coordinator 使用 PatrolTaskState.home_pose 发起返航规划；
6. 返航过程中按 `motion.return_replan_period` 从当前机器人位姿重新规划到同一个 home pose；
   新规划失败时立即清空旧返航路径并停车重试，不能继续执行可能已失效的缓存路径；
7. 返航到达位置容差由 `motion.return_position_tolerance` 独立配置，生产默认值为 `0.50 m`；
   它不能复用 `RECOVER_PATROL` 的 `motion.recovery_position_tolerance=0.25 m`；
8. mux 只接受新状态后的 mission path；
9. 到达 home 并稳定停车后发布 RETURN_REACHED。

RETURN_REACHED 被接受后：

~~~text
TASK_RETURNING + READY
  -> TASK_NO_TASK + READY
  -> last_task_outcome = RETURNED
  -> navigation_mode = STOP
  -> 清除 home_pose
  -> target_id = 0，handled_target_id = 0，清除 resume_behavior
  -> state_seq + 1，并发布同序号的两条终态快照
~~~

导航遇到明确不可恢复的返航 planner/controller 故障时发布 RETURN_FAILED。父任务的整体
返航超时由 supervisor 自己的 RETURNING watchdog 判定，不要求导航伪造超时事件。两种失败
入口都调用状态机同一个失败收口：

~~~text
TASK_RETURNING + READY
  -> TASK_NO_TASK + READY
  -> last_task_outcome = FAILED
  -> navigation_mode = STOP
  -> target_id = 0，handled_target_id = 0
  -> 清除 home_pose、resume_behavior 和所有任务上下文
  -> state_seq + 1，并发布同序号的两条终态快照
~~~

`RETURN_REACHED`、`RETURN_FAILED` 和 RETURNING watchdog 都必须在 supervisor 的同一把锁内
竞争。只有当前仍是 `TASK_RETURNING + READY` 且事件的 `observed_state_seq` 等于当前序号时，
返航事件才可形成结果；watchdog 也必须先重新检查这两个条件。先处理的一方完成唯一一次
收口，后到的一方因父任务已经是 `TASK_NO_TASK` 或序号已变化而无副作用。导航收到成功或
失败后的旧返航 planner 回调必须同时失效，不能再发布路径或再次发事件。

RETURNING 中收到 TERMINATE 时立即取消返航并按 TERMINATE 收口。

PatrolTaskState.home_pose 保存的是 START 时的 map 坐标和捕获时间。RETURN 每次提交
ComputePathToPose 时必须复制其位置和姿态，但把 planner goal 的 header.stamp 更新为当前 ROS
时间；不能把可能已经很旧的 START 时间戳直接用于 Nav2 TF 查询。home pose 的 map 坐标在整个
父任务中保持不变，PAUSE/RESUME 不重新捕获。

### 7.9 TERMINATE

TERMINATE 可以从 RUNNING、PAUSED 或 RETURNING 接受，不要求 READY、路线或 planner 可用。

原子状态变化：

~~~text
task_state = NO_TASK
mission_state = READY
last_task_outcome = TERMINATED
navigation_mode = STOP
target_id = 0
handled_target_id = 0
清除 home_pose
清除 resume_behavior
state_seq + 1
发布两份状态
~~~

所有节点取消当前工作，waypoint publisher 在处理该父任务状态回调时先清空旧路线，再重新
开放编辑。TERMINATE 不返航。

NO_TASK 下收到 TERMINATE 时返回 `accepted=false`、`error_code=no_active_task`，不改变状态。

### 7.10 waypoint 正常完成

最后一个 waypoint 到达时，waypoint publisher：

1. 设置 `sequence_done=true`；
2. 发布空 waypoint path；
3. 停止进一步规划；
4. 立即发布新的 WaypointSequenceState 持久快照。

supervisor 订阅 WaypointSequenceState，只要当前为 TASK_RUNNING 或 TASK_PAUSED，且收到最新
`waypoint_count > 0 && sequence_done=true`，就结束父任务。空路线状态不能触发 COMPLETED。
允许 PAUSED 是为了闭合“最后一点到达”和 PAUSE 并发的
情况：即使 PAUSE 回调先线性化，随后到达的持久完成事实仍然结束父任务，不留下已完成但无法
继续的暂停任务。完成检查不依赖当前业务状态：CONFIRM_TARGET 期间
waypoint 仍可能到达最后一点，而目标事件与路线状态的回调顺序不确定；无论当前仍是 PATROL、
已经进入 CONFIRM_TARGET，还是目标事件抢先推进到 APPROACH_TARGET，持久的路线完成快照最终
都会被处理。父任务完成优先，尚未完成的目标子任务被取消。

该判断必须使用 supervisor 已按 `route_revision` 接收的最新路线快照，并在同一状态锁内执行；
不能用一次性“最后一点到达”日志或旧的空路线快照代替。收到 `sequence_done=true` 后，先
使目标、planner 和 waypoint 执行上下文失效，再执行下方唯一一次 COMPLETED 收口。路线完成
不是新的运动许可，PAUSED 下接受它也不会重新打开 waypoint 执行。

同一条路线的 `sequence_done=true` 周期重发必须幂等：第一次完成父任务后 task_state 已经
变为 NO_TASK，后续相同快照不再触发收口。接受后：

~~~text
task_state = NO_TASK
mission_state = READY
last_task_outcome = COMPLETED
navigation_mode = STOP
target_id = 0
handled_target_id = 0
清除 home_pose 和 resume_behavior
state_seq + 1
发布同序号的两条终态快照
~~~

waypoint publisher 观察到活动任务结束后清空已完成路线。正常完成不自动返航、不自动重启，
下一次任务必须重新准备非空路线并再次 START。

结束竞争按 supervisor 的串行处理结果确定：

- PAUSE 先处理、完成快照后到：仍按 COMPLETED 收口；
- 完成快照先处理、PAUSE 后到：PAUSE 因 NO_TASK 被拒绝；
- RETURN 先处理、完成快照后到：RETURNING 下忽略路线完成，继续返航；
- 完成快照先处理、RETURN 后到：RETURN 因 NO_TASK 被拒绝；
- TERMINATE 先处理：后续完成快照忽略，结果保持 TERMINATED；
- 完成快照先处理：后续 TERMINATE 因 NO_TASK 被拒绝，结果保持 COMPLETED；
- RETURN_REACHED 与 TERMINATE 并发时同样以先进入互斥临界区的转换为准，后到事件因状态或
  state_seq 不匹配而无副作用。
- RETURN_FAILED 与 TERMINATE 并发时也以先进入互斥临界区的转换为准；先失败得到 FAILED，
  先终止得到 TERMINATED，后到的一方不改变结果。
- RETURN_REACHED 与 RETURN_FAILED 同时到达时只接受先处理的一条，结果分别为 RETURNED 或
  FAILED；不能因为两条事件都来自导航而合并或覆盖终态。
- RETURN_FAILED 与 supervisor RETURNING timeout 并发时，两者调用同一个 FAILED 收口；先处理
  的一次改变状态，后处理的一次因已非 RETURNING 而无副作用。

### 7.11 整套重启和 reset

进程级故障不走 PAUSE/RESUME：

~~~text
关键进程崩溃
  -> 外部进程管理器停止并重启导航、感知和 supervisor
  -> 所有内存中的路线、home、目标和认证会话消失
  -> TASK_NO_TASK + STARTUP
  -> 重新等待双 READY
  -> TASK_NO_TASK + READY
~~~

旧任务不自动恢复。由于旧发布者和旧消费者都随整套业务退出，不需要 runtime epoch 解决
跨进程热恢复。

生产进程管理必须与这一假设一致：mission supervisor、waypoint publisher、navigation
coordinator、path mux 以及感知/认证关键节点不得各自独立 respawn 后继续旧父任务。任一关键
节点退出时，统一 watchdog/launch supervisor 应停止控制输出并重启整套业务。重启窗口内，
控制适配器依靠现有 path/command timeout 将底盘速度归零。

单节点意外重启即使短暂发生，也必须默认 fail closed：在收到新的权威状态前不发布路径、不
选择目标、不恢复旧本地缓存。整套业务随后回到 NO_TASK+STARTUP，外部任务系统把旧父任务记录
为异常中止，不向机器人恢复其内存上下文。

## 8. 路线所有权和编辑边界

### 8.1 执行门控

waypoint publisher 必须直接订阅 `/patrol_task/state`，保存最新父任务状态和 navigation
mode。定时规划的必要条件为：

~~~text
task_state == TASK_RUNNING
navigation_mode == NAVIGATION_WAYPOINT
waypoint_count > 0
sequence_done == false
paused == false
~~~

仅仅添加 waypoint 不得触发可执行路径。

节点启动后、收到第一份 PatrolTaskState 之前必须默认 `paused=true` 并禁止发布控制路径；可以
加载或显示预配置 waypoint，但不得因为本地默认值或 `auto_start_on_click` 提前运动。

PatrolTaskState 是 waypoint 生产运行许可的唯一权威入口。收到更大的 state_seq 时，waypoint
publisher 在一个互斥临界区内执行：

~~~text
先使旧 planner request 失效
  -> 更新 task_state 和 navigation_mode
  -> 若进入 RETURNING 或任务终态，先清理路线
  -> RUNNING + WAYPOINT：paused=false，清除旧缓存并从当前位置重新规划
  -> 其他状态：paused=true，发布空 waypoint path
  -> 发布新的 WaypointSequenceState
~~~

navigation coordinator 不再向 waypoint publisher 发送生产用 pause/resume 命令。业务状态
从 PATROL 切到目标处置、从 RECOVER 回到 PATROL，都会使 supervisor 发布新的
PatrolTaskState.navigation_mode，waypoint publisher 直接按这个状态停启。

### 8.2 编辑门控

新增、移动、删除、撤销和手工 CLEAR 都必须满足：

~~~text
task_state == TASK_NO_TASK
~~~

RUNNING、PAUSED 和 RETURNING 中的编辑请求全部拒绝并发布诊断状态。暂停期间只能通过 RESUME
继续原路线，不能修改路线绕过父任务状态机。

现有 Empty Topic 的处理规则：

- `/waypoint_sequence/clear` 仅是 NO_TASK 下的路线数据编辑入口；
- `/waypoint_sequence/pause`、`resume`、`resume_from_current` 不属于正式父任务接口；
- 若为兼容内部调试暂时保留后三个 Topic，它们不得改变父任务状态，也不得绕过当前
  PatrolTaskState 门控；
- PAUSED 下收到调试 resume 必须拒绝，只有 `/mission/task_command` 的 RESUME 可以继续父任务；
- 生产 launch 和上位任务系统不得发布这三个调试 pause/resume Topic。

### 8.3 任务结束清理

不增加路线清理 epoch/ack 协议。waypoint publisher 在自己的互斥回调中按父任务状态边沿
清理：

| 边沿 | 动作 |
|---|---|
| `RUNNING/PAUSED -> RETURNING` | 立即清空路线 |
| `RUNNING/PAUSED/RETURNING -> NO_TASK` 且 outcome 非 NONE | 清空路线 |
| 第一份 STARTUP 基线之后出现更大的 `NO_TASK + STARTUP` state_seq | reset，清空路线 |
| 初始 `NO_TASK + NONE` | 不自动清空用户正在准备的新路线 |
| 相同状态周期重发 | 不重复清理 |

清理和重新开放编辑必须在 waypoint publisher 的同一个锁或 mutually-exclusive callback
group 内完成：先清理旧路线，再允许后续编辑回调。这样不会出现 TERMINATE 后的旧路线与新
路线混合。

正式上位客户端在任务结束后，应等待 `/waypoint_sequence/state.waypoint_count=0`，再发布
下一条路线。这个等待是调用流程约束，不是新的分布式事务协议。

### 8.4 planner 迟到结果

继续使用 waypoint publisher 已有的本地 `sequence_version`：

- 暂停、清理、当前目标改变和路线编辑时递增；
- 收到更大的 PatrolTaskState.state_seq 时，即使 task_state 和 navigation_mode 与上一份
  本地快照相同，也先使当前 planner 请求失效；这覆盖订阅者因 keep-last 只看到快速
  PAUSE/RESUME 最终快照的情况；
- planner 请求捕获当前 version；
- 回调 version 不匹配时直接丢弃；
- 丢弃时不发布路径、不修改进度。

该 version 是节点内部实现细节，不进入父任务 ROS 接口。

## 9. 暂停场景边界表

| PAUSE 前状态 | PAUSE 后 | 保留中断点 | RESUME 后第一状态 | 旧目标是否恢复 |
|---|---|---:|---|---:|
| PATROL | `PAUSED + READY` | 否 | PATROL | 否 |
| CONFIRM_TARGET | `PAUSED + READY` | 否 | PATROL | 否 |
| APPROACH_TARGET | `PAUSED + READY` | 是 | RECOVER_PATROL | 否 |
| VERIFY_IDENTITY | `PAUSED + READY` | 是 | RECOVER_PATROL | 否 |
| TRACK_INTRUDER | `PAUSED + READY` | 是 | RECOVER_PATROL | 否 |
| RECOVER_PATROL | `PAUSED + READY` | 是 | RECOVER_PATROL | 否 |

补充边界：

- PAUSE 到达时即使 planner action server 不可用，也必须接受并切 STOP；
- PAUSE 不把认证取消转换为 UNAUTHORIZED；
- PAUSE 不把跟踪取消转换为 TARGET_LOST；
- 已经成立的 `handled_target_id` 在 PAUSE/RESUME 中保留；
- 新父任务 START 时清除旧 handled target；
- pending planner 的旧结果只能被取消或丢弃，不能在 RESUME 后重放。

## 10. 事件和异步结果失效

### 10.1 跨节点业务事件

除首次 READY 外，所有 MissionEvent 必须满足：

~~~text
event.observed_state_seq == current state_seq
~~~

目标事件还必须满足：

~~~text
current task_state == TASK_RUNNING
event.target_id == current target_id
event source 允许发布该事件
event type 对当前 mission state 合法
~~~

任一条件失败都无副作用地拒绝。

`PATROL_RECOVERY_COMPLETE` 是 target_id 校验的唯一特例：它仍要求 event target_id 与当前
MissionState.target_id 相等，但允许二者同时为 0；0 只代表 PAUSE/RESUME 产生的中断点恢复。

### 10.2 READY 的特殊规则

READY 只在以下组合中处理：

~~~text
TASK_NO_TASK + STARTUP
~~~

READY 必须满足 `target_id=0`，来源只能是感知或导航。第一个 READY 不递增 state_seq；双 READY
到齐时才进入 READY 并递增一次。

在以下状态收到 READY 均不改变权威状态：

- `TASK_NO_TASK + READY`；
- `TASK_PAUSED + READY`；
- `TASK_RETURNING + READY`；
- 任意 TASK_RUNNING 状态。

因此 PAUSE/RESUME 不存在“旧 READY 还没有失效”或“重新 READY 还没有到”的竞争。

进入 READY 后继续到达的 CapabilityStatus 或 READY 事件只记录诊断，不改变 state_seq。若某个
能力故障严重到当前任务不能继续，应由模块发布既有 EXECUTION_ERROR 或由进程 watchdog 触发
整套重启，不能由 capability provider 自行修改 MissionState。

### 10.3 导航异步 planner

navigation planner 请求至少捕获：

~~~text
local_request_generation
state_seq
target_id
plan_kind
goal_pose
~~~

`plan_kind` 至少包括：

~~~text
TARGET_APPROACH
TARGET_TRACK
PATROL_RECOVERY
TASK_RETURN
~~~

回调应用前检查：

1. 本地 request generation 仍有效；
2. state_seq 与当前快照一致；
3. target_id 对目标请求一致；
4. plan kind 与当前任务/业务状态一致；
5. goal pose 仍是当前目标。

各 plan kind 的当前状态条件：

- TARGET_APPROACH：`TASK_RUNNING + APPROACH_TARGET` 且 target_id 匹配；
- TARGET_TRACK：`TASK_RUNNING + TRACK_INTRUDER` 且 target_id 匹配；
- PATROL_RECOVERY：`TASK_RUNNING + RECOVER_PATROL`，target_id 可以是普通恢复的非零值或暂停
  恢复的 0，但必须与当前 MissionState 相等；
- TASK_RETURN：`TASK_RETURNING + READY`、target_id=0、home pose 与当前 PatrolTaskState 一致。

现有 planner client 只匹配 target/recovery；实现返航时必须为 TASK_RETURN 增加独立匹配分支，
不能把返航伪装成 target plan。

每个被接受且确实改变权威状态的 PAUSE、RESUME、RETURN 和 TERMINATE 都会递增 state_seq；
RETURN_REACHED、RETURN_FAILED、路线 COMPLETED 和 reset 也一样。被拒绝的命令不递增，因此
旧 planner 结果只能依靠它所绑定的当前序号和本地 generation 判断，不需要额外 task generation。

### 10.4 认证异步结果

face、voice 和 authorization 会话继续绑定：

~~~text
observed_state_seq
target_id
stage
provider（仅用于证据接收方校验）
~~~

只有当前仍是 `TASK_RUNNING + VERIFY_IDENTITY`、state_seq、target_id、stage 和 provider 都匹配时
才能采纳。face、voice 和 authorization 只读取 MissionState；PAUSE、RETURN、TERMINATE 以及
父任务终态都会使 MissionState 进入 READY 或其他非 VERIFY 状态，接收方必须取消当前会话并
清空实时证据上下文。取消可以发布 `CANCELLED` 技术证据，但不得发布 `UNAUTHORIZED` 或
`TARGET_LOST`。state_seq=0、被 wrap-aware helper 判定为旧的状态、旧 target、旧 stage、未知
provider 和早于会话起点的证据全部丢弃。

## 11. 两条状态 Topic 的一致性

supervisor 在内存中只维护一份不可分割的快照，然后从该快照构造两条消息。状态更新流程：

~~~text
加锁
  -> 校验命令或事件
  -> 更新父任务和业务状态
  -> 计算 navigation_mode
  -> state_seq + 1
  -> 构造 MissionState(seq=N)
  -> 构造 PatrolTaskState(seq=N)
解锁前依次发布两条消息
返回服务结果
~~~

ROS 2 不保证不同 Topic 的到达先后，因此不能依赖“先发布哪一条”。本方案通过单 Topic 控制
依赖避免竞态：

- mux 只读取 PatrolTaskState.navigation_mode；
- waypoint 只读取 PatrolTaskState 的父任务许可；
- 感知、face、voice 只读取 MissionState；
- navigation coordinator 的目标业务读取 MissionState；返航业务缓存两条状态，只在相同
  state_seq 的 `TASK_RETURNING + READY` 同时成立后启动；
- navigation coordinator 对 RETURNING 的处理优先于普通 READY 行为；
- UI 只组合 state_seq 相同的快照。

所有状态订阅者还必须遵循统一版本规则：

- 同一 supervisor 进程会话内，被 wrap-aware helper 判定为 older 的快照直接丢弃；
- 等于本地最新 state_seq 的周期重发不重复触发副作用；
- 大于本地最新 state_seq 的快照先使旧异步上下文失效，再应用当前快照；
- 即使新快照的枚举值与本地相同，只要 state_seq 变大，也必须视为一个新的控制边界。

周期重发相同 state_seq 不重复执行业务副作用，但必须刷新本地状态 heartbeat 接收时间。安全
关键节点配置本地 `state_timeout`（应明显大于 supervisor 发布周期）：

- mux 超时：立即切 NO_SOURCE、发布空 global path；
- waypoint 超时：本地 paused=true、取消 planner、发布空 waypoint path；
- navigation coordinator 超时：清目标/返航输出并请求控制器停止；
- perception/authorization 超时：停止选目标并取消活动认证会话。

heartbeat 接收年龄使用进程单调时钟（steady/monotonic clock），不使用可能暂停或跳变的 ROS
仿真时间；路径与状态数据的新旧屏障仍使用统一 ROS clock 的消息时间戳。

heartbeat 超时只影响各节点本地安全门，不修改 supervisor 权威状态、不发布 READY、不把任务
转换为 PAUSED/FAILED。收到恢复的当前快照后，mux 和路径生产者先建立新空路径屏障并产生新
路径，不能直接重放超时前缓存。若确为进程崩溃，统一 watchdog 随后执行整套重启。

heartbeat 恢复时即使 state_seq 与超时前相同，也属于一次本地通信恢复边界：mux 更新 barrier
并等待新路径，waypoint/navigation 递增本地 request generation 后重新规划，感知重新打开当前
状态允许的处理门；但不得重新发布父任务事件、重置 handled absence timer 或重复改变业务状态。

最后一条保证快速 PAUSE/RESUME 时，即使 keep-last 订阅者只看到最终状态，也会取消旧请求并
从最终权威上下文重新开始，不会继续使用暂停前创建的 planner 或认证会话。

常驻外部消费者在已经确认整套业务重启后，必须先重置本地 latest state_seq，再应用新会话
规则；不得把上一进程的较大 state_seq 与新进程直接比较。机器人内部节点不实现跨会话恢复，
因为它们属于同一整套重启单元。

navigation coordinator 在业务状态从 RECOVER_PATROL 离开到 READY 时，不得仅凭 MissionState
立即删除中断点；它应保留中断点，直到收到以下任一确定边界：

- RECOVER_PATROL 成功或超时回到 PATROL；
- 父任务进入 RETURNING；
- 父任务进入 NO_TASK；
- 新 APPROACH_TARGET 覆盖为新的中断点；
- 整套 reset。

navigation coordinator 不根据 READY 单独决定是否恢复中断点。READY 期间只保存可能需要的
候选 pose；后续权威 MissionState=PATROL 时清除，MissionState=RECOVER_PATROL 时验证绑定关系
后使用，父任务 RETURNING/NO_TASK 或 reset 时无条件清除。这样即使中间 APPROACH 快照被
keep-last 跳过，也不会丢失应恢复的 pose，也不会在普通 PATROL RESUME 中误用它。

这使 PAUSE 两条 Topic 的到达顺序不会破坏恢复上下文。

## 12. 模块实现要求

### 12.1 `dog_patrol_interfaces`

修改：

- `msg/MissionState.msg`：追加 READY=7；
- `msg/MissionEvent.msg`：追加 RETURN_REACHED 和 RETURN_FAILED；
- 新增 `msg/PatrolTaskState.msg`；
- 新增 `msg/WaypointSequenceState.msg`；
- 新增 `srv/PatrolTaskCommand.srv`；
- CMake 增加 geometry_msgs、消息和服务生成；
- package.xml 增加 geometry_msgs 依赖。

禁止修改现有 MissionState 和 MissionEvent 常量数值。

### 12.2 `state_machine.py`

保留现有：

- `GlobalState` 名称和 0 到 6 数值；
- 目标事件转换表；
- state_seq 校验；
- target_id 校验；
- handled_target_id 规则；
- processed event 去重。

新增：

- `GlobalState.READY = 7`；
- `TaskState`；
- `TaskOutcome`；
- `NavigationMode`；
- 内部 `ResumeBehavior`；
- 父任务命令数据和结果；
- 路线可执行性快照；
- home pose；
- `handle_task_command()`；
- `handle_waypoint_state()`；
- `begin_paused_patrol_recovery()`；
- `handle_return_reached()`；
- `handle_return_failed()`；
- `_finish_task()` 公共收口。

建议扩展 `MissionSnapshot`，但仍保持一个纯 Python、无 ROS 依赖的确定性状态机。home pose
可以在 ROS 层完成校验后以不可变数据传入，避免纯状态机直接依赖 TF。

所有转换集中在 state machine；ROS node 不允许在回调里绕过状态机直接拼装下一状态。

### 12.3 `mission_supervisor.py`

新增：

- PatrolTaskState publisher；
- WaypointSequenceState subscriber；
- PatrolTaskCommand service；
- TF buffer/listener；
- map、robot frame 和 TF timeout 参数；
- home_pose_max_age 参数；
- RETURN timeout；
- 对新增事件的适配。

命令、事件、路线状态和 timeout 回调全部使用同一个 `RLock`。服务响应使用状态机返回的
同一快照，不能在解锁后再次读取并拼接。

状态发布继续周期重发，但相同快照不得递增 state_seq。
发布周期同时供下游做本地 heartbeat；supervisor 参数必须与各消费者 state_timeout 配套，默认
建议 10 Hz 发布、0.5 到 1.0 秒超时，真机根据调度抖动验证后确定。

### 12.4 `global_path_seq_publisher.py`

新增：

- PatrolTaskState subscriber；
- MissionState subscriber，仅用于识别非初始的 STARTUP reset 边沿；
- WaypointSequenceState publisher；
- task state 变化处理；
- START 前的编辑/执行门控；
- 正常完成状态发布；
- 任务结束自动清理。

所有路线内容编辑必须递增 route_revision 并在同一临界区完成内容更新，再发布新
WaypointSequenceState；拒绝的编辑和纯进度变化不得递增。route_revision 使用与 state_seq
相同的 uint32 wrap-aware 比较规则。

每次收到更大的 PatrolTaskState.state_seq 都必须更新执行门控并使旧 planner request
失效；不能只比较 task_state 或 navigation_mode 枚举值。

保留现有 clicked point、删除、替换、interactive marker、单段规划、到达判断和
`sequence_version` 逻辑。

`auto_start_on_click` 只能改变路线准备状态，不能在 TASK_NO_TASK 下产生控制路径。若保留该
参数，实际执行仍必须受 TASK_RUNNING + NAVIGATION_WAYPOINT 门控。

### 12.5 `navigation_policy.py`

在现有枚举末尾增加 READY。READY policy：

~~~text
safe_stop = true
clear_navigation_path = true
reset_target = true
allow_target_fusion = false
allow_target_motion = false
~~~

STARTUP 继续用于 readiness；READY 只用于已初始化后的安全待命，不触发 readiness 事件。

`navigation_policy` 不再拥有 waypoint pause/resume 控制权。现有 `pause_patrol`、
`resume_patrol` 字段可以在迁移时删除，或暂时保留但 navigation coordinator 不再根据它们
发布 Empty 命令；waypoint 的生产门控只来自 PatrolTaskState。

### 12.6 `navigation_mission_coordinator.py`

新增 PatrolTaskState 订阅和返航控制器，复用现有 planner client 与 mission path publisher。

现有 `/waypoint_sequence/pause`、`/waypoint_sequence/resume` 和
`/waypoint_sequence/resume_from_current` 不再由 navigation coordinator 在生产流程中发布。
恢复完成时，导航只发布 PATROL_RECOVERY_COMPLETE；supervisor 接受后切回 PATROL 并发布
`NAVIGATION_WAYPOINT`，由 waypoint publisher 自己重新规划当前未完成点。

navigation coordinator 分别缓存 MissionState 和 PatrolTaskState。父任务返航只在两者
state_seq 相同且组合为 `TASK_RETURNING + READY` 时创建/重试 RETURN planner；单独收到
RETURNING 或 READY 只清旧目标动作并等待另一份快照。这样无论 Topic 到达顺序如何，READY
的安全清理都发生在返航 planner 启动之前，不会取消刚创建的返航请求。

`_navigation_ready_reason()` 的 strict readiness 检查必须同步删除对 waypoint pause/resume
subscriber 数量的要求；否则生产 launch 移除这些 Topic 后导航永远无法 READY。waypoint publisher
是否在线以及是否有可用路线，由 supervisor 是否收到 `/waypoint_sequence/state` 判断：没有
状态快照时系统仍可完成基础 STARTUP->READY，但 START 返回 `route_unavailable`，不得运动。

处理优先级：

~~~text
相同 state_seq 的 TASK_RETURNING + READY
  -> tick return controller

两条状态尚未匹配
  -> safe stop，等待当前完整快照

其他状态且 MissionState.STARTUP
  -> tick startup readiness

MissionState.READY
  -> safe stop，不发布 READY

RECOVER_PATROL
  -> tick recovery controller

其余现有目标行为
  -> 原逻辑
~~~

父任务从 NO_TASK+READY 进入 RUNNING+PATROL 时不在导航节点重新捕获 home；home 由 supervisor
在 START 服务临界区内捕获并通过 PatrolTaskState 发布。

返航 controller 需要：

- 当前 state_seq；
- home pose；
- planner request generation；
- 到达距离阈值；
- 线速度和角速度阈值；
- 稳定停车时间。

整体返航超时只由 supervisor 的 RETURNING watchdog 持有，避免导航 controller 和
supervisor 各维护一套相互竞争的任务超时。

返回路径仍发布到 `/mission_global_path`。

巡检中断点在 navigation coordinator 内部保存为不可变上下文，而不只是一个裸 pose：

~~~text
pose
source_confirm_state_seq
source_target_id
local_generation
~~~

TARGET_POSITION_READY 发布前创建该上下文。进入 PAUSED+READY 时可以暂存；RESUME 到
RECOVER_PATROL 时创建新的 recovery generation 并使用 pose，RESUME 到 PATROL、父任务进入
RETURNING/NO_TASK 或 reset 时清除。任何旧 planner callback 都必须同时匹配 local_generation
和当前 state_seq。

`_publish_status()` 必须增加 READY 分支，不能继续落到 “unsupported mission state”：

- NO_TASK/PAUSED + READY：TargetNavigationStatus 使用 WAITING_TARGET、target_id=0、
  distance_valid=false；
- RETURNING + READY：目标导航状态仍不伪装成人员目标，父任务返航状态和结果以
  PatrolTaskState 为权威，detail 可显示返航诊断；
- READY 下不得沿用暂停前的目标距离和 target status。

### 12.7 `navigation_path_mux.py`

将权威选择输入从 MissionState 改为 PatrolTaskState.navigation_mode：

~~~text
STOP -> NO_SOURCE
WAYPOINT -> WAYPOINT_SOURCE
MISSION -> MISSION_SOURCE
~~~

保留当前 fail-closed、状态版本检查、空路径和等待新路径机制。每次收到更大的 state_seq
都建立一次空路径屏障，即使 source 没变；切换 MISSION plan kind 时 navigation coordinator
也必须先清空 mission path，再发布新任务的路径。

mux 分别维护 waypoint 和 mission 的最小路径时间戳。非空路径只有在 source 匹配且发布时间
晚于当前 PatrolTaskState 屏障时才转发；生产者按第 6 节要求重新规划或周期重发，保证屏障后
既不会使用旧路径，也不会因一次跨 Topic 到达乱序永久停住。

### 12.8 感知 tracking

MissionState READY 映射为安全非活动状态：

- 不选择 primary target；
- 清空活动 target；
- 不发布 bbox；
- 不发布 TARGET_LOST；
- 不把 READY 当成 PATROL；
- 不启动新的身份核验。

现有 0 到 6 状态映射保持不变。

`mission_coordinator.hpp` 必须在现有枚举末尾增加 `kReady=7`；
`MissionPhaseFromMessage()` 必须把 MissionState.READY 映射到 kReady，所有处理 phase 的 switch
都显式把 kReady 当成不接受 bbox、不选择目标的状态，不能让它落入 unknown message 分支。

`MissionFromMessage()` 的字段校验必须按第 4.7 节调整。实现不得把 RECOVER_PATROL 一律视为
必须有活动目标，并且只允许 `handled_target_id == target_id`，这会拒绝以下两个新合法快照：

~~~text
TASK_PAUSED + READY
target_id = 0
handled_target_id > 0

TASK_RUNNING + RECOVER_PATROL（暂停恢复）
target_id = 0
handled_target_id >= 0
~~~

修改后的校验要求：

- STARTUP、READY、PATROL 必须 `target_id=0`；
- CONFIRM、APPROACH、VERIFY、TRACK 必须 `target_id>0`；
- RECOVER_PATROL 允许 `target_id>0` 的普通目标恢复，也允许 `target_id=0` 的暂停恢复；
- STARTUP 必须 `handled_target_id=0`；
- READY 和 PATROL 允许 `handled_target_id>=0`；
- 目标活动阶段必须 `handled_target_id=0`；
- 普通 RECOVER 中 handled 只能为 0 或等于 target_id；
- target_id=0 的暂停恢复中 handled 可以为 0 或暂停前保留的正值。

MissionRosAdapter 不订阅 PatrolTaskState，因此无法单独判断 READY 是 NO_TASK 还是 PAUSED；它只
按上述 MissionState 字段规则验证。`READY + handled_target_id>0` 是否只发生在 PAUSED，由唯一
状态权威 supervisor 保证。

进入 PATROL 时，PrimaryTargetManager 继续调用
`ResetForPatrolCycle(mission.handled_target_id)`。该方法只能重置当前 primary 选择上下文并把
新的正 ID 插入 handled 排除表，不得清空表中其他尚未过期的 ID。相同 state_seq 的周期状态
重发不能再次调用 ResetForPatrolCycle，避免反复重置 absence timer。

handled 排除表必须使用等价于 `try_emplace()` 的“缺失才插入”行为；PAUSE/RESUME 或其他
PATROL 重入不得用 `insert_or_assign()` 覆盖同一 ID 的已有 absence timer。

MissionState 首次进入 STARTUP 或 state_seq 变大后重新进入 STARTUP 时，感知必须调用独立的
`ResetMissionHistory()`（名称可按实现调整）清空当前 primary、handled 排除表和 absence timer；
不能复用 `ResetForPatrolCycle(0)`，因为后者按现有语义不会清空多个历史排除项。

`PrimaryTargetManager::UpdateForMission()` 必须按 phase 明确分流：

| phase | primary target 行为 | handled 排除表 |
|---|---|---|
| STARTUP | 清除当前 primary，不选择目标 | 清空 |
| READY | 清除当前 primary，不选择目标 | 保留 |
| PATROL | 按面积和资格选择目标 | 先插入当前 handled ID，再按 absence 规则维护 |
| CONFIRM/APPROACH/VERIFY/TRACK | 只维持当前目标上下文，不选择新的 patrol primary | 保留 |
| RECOVER_PATROL | 清除当前 primary，不选择目标 | 保留，等待回到 PATROL 应用 handled ID |

实现必须按上述 phase 显式分流。READY 和暂停恢复只清当前 primary，不调用普通 PATROL
Update；用于“清当前 primary 但保留 handled 表”的方法必须与 `ResetMissionHistory()` 分开，
避免 PAUSE 误删已经成立的排除结论。当前实现已经按此规则分流并覆盖多 handled history。

### 12.9 face、voice 和 authorization

现有会话成立条件保持为 VERIFY_IDENTITY + 有效 target_id。收到 READY（包括 PAUSE、RETURN、
TERMINATE 和父任务终态对应的 READY 快照）或任何其他非 VERIFY_IDENTITY 状态时：

- 取消当前工作；
- 发布 CANCELLED 只能作为技术记录，不能转成业务未授权；
- 丢弃随后到达的旧证据；
- 不执行 readiness 握手。首次 readiness 仍只由 STARTUP 驱动。

face、voice 和 perception aggregate readiness controller 继续以 MissionState.STARTUP 作为唯一
preflight 触发状态。观察到 READY 或其他非 STARTUP 状态时，必须增加本地 generation 或清除
desired startup，使正在运行但尚未返回的旧 preflight 结果不再发布；不能因为 READY 的枚举值
是新值而把它当成未知状态后保留旧 readiness 任务。face、voice 和 aggregate readiness 的
控制器及真实 ROS 节点测试已经覆盖 STARTUP preflight 尚未结束即进入 READY、旧 STARTUP
快照和序号回绕场景。

### 12.10 robot console 和上位任务系统

内部 console 可以增加五个调试按钮，但正式业务任务仍由外部任务系统管理。按钮可用性：

| 当前状态 | 可用操作 |
|---|---|
| `NO_TASK + STARTUP` | 无，等待 READY |
| `NO_TASK + READY` 且路线合法 | START |
| RUNNING | PAUSE、RETURN、TERMINATE |
| PAUSED | RESUME、RETURN、TERMINATE |
| RETURNING | TERMINATE |

外部系统必须保存父任务 ID、自动子任务 ID、命令历史和状态历史；这些 ID 不进入机器人实时
控制接口。

外部系统还必须订阅统一 watchdog/进程管理器的业务栈重启通知，或跟踪 supervisor publisher
GID。确认重启后结束旧父任务记录、重置本地 state_seq 基线，并等待新 STARTUP->READY；不能
在旧业务 ID 下自动发送 RESUME。

console 只调用 `/mission/task_command`，不发布 MissionState、PatrolTaskState 或运动路径。
它只显示最新且合法的父任务快照；未知状态、非法组合或旧 state_seq 时禁用全部父任务按钮，
并按服务响应串行发送命令。

### 12.11 launch 和生产配置

生产 launch 必须显式配置以下内容，不能依赖当前 standalone 默认值：

- global_path_sequence_publisher 的 `auto_start_on_click=false`；
- global_path_sequence_publisher 的 `publish_pure_pursuit_plan=false`，只发布私有
  `/waypoint_global_path`；
- global_path_sequence_publisher 的 `use_static_goals=false`；生产重启后不得从参数、缓存或
  latched 外部命令自动重放旧父任务路线；
- 有限巡检任务使用 `loop=false`；若业务明确选择循环巡检，按第 7.2 节持续运行语义处理；
- mission supervisor：`state_topic=/mission/state`、`task_state_topic=/patrol_task/state`、
  `event_topic=/mission/event`、`waypoint_state_topic=/waypoint_sequence/state`、
  `task_command_service=/mission/task_command`，并配置 map/robot frame、TF timeout、
  home_pose_max_age 和 return timeout；
- waypoint publisher：`task_state_topic=/patrol_task/state`、`mission_state_topic=/mission/state`
  （后者只用于 reset 边沿）、`waypoint_state_topic=/waypoint_sequence/state`；
- navigation coordinator：`mission_state_topic=/mission/state`、
  `task_state_topic=/patrol_task/state`、`mission_event_topic=/mission/event`；
- navigation path mux：`task_state_topic=/patrol_task/state`，以及现有 waypoint/mission/output
  path Topic；
- mux、waypoint、navigation coordinator、perception/authorization 配置与 supervisor
  `state_publish_rate` 匹配的 `state_timeout`，生产值由真机调度抖动测试确定；
- navigation coordinator 不再接收或发布生产用 waypoint pause/resume topic；
- navigation_path_mux 的权威控制订阅改为 PatrolTaskState，不再从 MissionState 推导 source；
- navigation readiness strict checks 删除 waypoint pause/resume subscriber 条件；
- `/global_path` 仍只有 mux 一个发布者。

以下现有配置已经按本契约清理，后续修改不得重新加入：

- `m20_patrol_navigation.yaml` 中的 `pause_patrol`、`resume_patrol`、
  `resume_patrol_from_current`；
- navigation mission coordinator launch 中对应参数；
- `navigation.launch.py` 和 `priest_external_nav.launch.py` 中仅用于 coordinator 生产恢复链的
  waypoint resume 参数。
- `priest_external_nav.launch.py` 中 global_path_sequence_publisher 的独立 `respawn=True`；
  关键节点退出应交给统一整套 watchdog 处理。

如需保留手工调试 Topic，应只在 waypoint publisher 自己的调试参数开启时创建，并继续受
PatrolTaskState 门控；生产 launch 默认关闭。节点启动到收到第一份 PatrolTaskState 期间，mux
和 waypoint publisher 都必须 fail closed。

Nav2 core 的 respawn/lifecycle 恢复策略也必须与统一 watchdog 配合：如果 planner/controller
重启会丢失当前 FollowPath 上下文，则不得让父任务继续假装 RUNNING；统一 watchdog 应结束本次
业务运行会话并重新进入 STARTUP。

整套重启进入 READY 后，必须由上位任务系统明确重新发布一条路线并再次 START。调试环境可以
显式启用 static goals，但必须标记为调试行为，不能作为生产任务恢复机制。

### 12.12 包依赖、测试工具和模拟节点

接口和代码改动必须同步更新依赖：

- `dog_patrol_interfaces`：增加 geometry_msgs 的 CMake/package 依赖；
- `dog_patrol_manager`：增加 geometry_msgs 和 tf2_ros 依赖，用于 START 捕获 home pose；
- `move`：增加 dog_patrol_interfaces 依赖，用于 PatrolTaskState、WaypointSequenceState；
- `robot_console`：增加 dog_patrol_interfaces 依赖，用于 PatrolTaskCommand 和状态显示；
- `dog_patrol_navigation` 已有 dog_patrol_interfaces、geometry_msgs、tf2_ros，继续复用。

以下测试辅助代码也属于接口迁移范围，不能只修改生产节点：

- `test_state_machine.py` 的启动 helper：双 READY 后先断言 READY，再显式调用 START，不能继续
  假设自动进入 PATROL；
- navigation policy/mux 测试：增加 READY safe-stop、PatrolTaskState navigation_mode 和同 source
  新 state_seq 屏障；
- perception tracking C++ 测试：增加 kReady 枚举、READY/暂停恢复字段组合和 handled 表计时；
- face、voice、aggregate readiness 测试：已覆盖离开 STARTUP 后迟到 preflight 结果被抑制、
  旧 state_seq/target/stage evidence 被丢弃以及取消不产生 UNAUTHORIZED；
- `tools/fake_integration`：fake navigation 仍只负责 READY/导航事件，测试控制端必须准备
  WaypointSequenceState 并调用 `/mission/task_command` 的 START；
- `tools/navigation_coordinator/mock_mission_events.py`：READY 状态下不伪造目标事件，只有显式
  START 进入 PATROL 后才按原逻辑工作。

完整测试不能以“看到 PATROL”作为 readiness 成功条件；新的正确启动断言是：

~~~text
双 READY -> NO_TASK + READY，且无路径、无目标
显式 START -> RUNNING + PATROL，随后才允许 waypoint 和人员搜索
~~~

## 13. 错误处理

服务建议错误码：

~~~text
invalid_command
invalid_state_transition
system_not_ready
route_unavailable
route_empty
route_already_complete
invalid_route_progress
home_pose_unavailable
invalid_home_pose
no_active_task
return_not_allowed
internal_error
~~~

错误码只描述服务在该次线性化点看到的前置条件，不代表后续异步动作已经完成：

- `route_empty` 表示 `waypoint_count=0`；
- `route_already_complete` 只用于 START 看到非空且已完成的路线；
- `invalid_route_progress` 表示路线字段关系不满足第 5.4 节，而不是路线暂时不可用；
- `no_active_task` 表示命令线性化时已经是 `TASK_NO_TASK`，包括终态快照已经先收口的情况；
- `return_not_allowed` 表示有活动父任务但当前命令不满足 RETURN 的条件（例如 home 无效）；
- `system_not_ready` 只表示仍处于 STARTUP，不把 planner、控制器或感知的异步故障伪装成
  父任务终态。

错误分类：

- 命令前置条件不满足：拒绝命令，无副作用；
- TASK_RUNNING 的 CONFIRM/APPROACH/VERIFY/TRACK 阶段发生普通异常：进入
  RECOVER_PATROL，父任务继续，handled_target_id 保持 0；
- 已在 RECOVER_PATROL 时收到 EXECUTION_ERROR：记录或限频诊断，不重复进入恢复、不清中断点、
  不结束父任务，由既有恢复 timeout 决定回 PATROL；
- PATROL、STARTUP、READY、TASK_PAUSED 下的 EXECUTION_ERROR：只记录诊断，不自动改变父任务；
- TASK_RETURNING 的明确返航失败必须使用 RETURN_FAILED，普通 EXECUTION_ERROR 不能被误当成
  RETURN_REACHED、TERMINATE 或身份未授权；
- 返航规划持续失败或超时：父任务 FAILED 收口；
- 关键进程崩溃：外部整套重启，不尝试旧任务恢复。

## 14. 测试要求

### 14.1 纯状态机测试

至少覆盖：

1. 初始 `NO_TASK + STARTUP`；
2. 单侧 READY 不改变状态和 state_seq；
3. 双 READY 进入 `NO_TASK + READY`；
4. READY 下重复 READY 无状态变化；
5. READY 后普通 capability 抖动不会退回 STARTUP；
6. 只有 reset 才重新进入 STARTUP 并要求新 READY；
7. reset 后旧 startup state_seq 的 capability/READY 不能推动新 STARTUP；
8. STARTUP 下 START 被拒绝；
9. READY 但无路线时 START 被拒绝；
10. 过期、未来或非法四元数的 home TF 使 START 无副作用地拒绝；
11. 有效 START 进入 `RUNNING + PATROL` 并保存 home；
12. 现有目标处置全部转换保持不变；
13. 各业务状态 PAUSE 后均为 `PAUSED + READY`；
14. PATROL/CONFIRM 暂停后 RESUME 进入 PATROL；
15. APPROACH/VERIFY/TRACK/RECOVER 暂停后 RESUME 进入 RECOVER_PATROL；
16. 暂停恢复型 RECOVER 使用 target_id=0，完成事件 target_id=0 可以被接受；
17. 普通目标恢复继续要求相同的非零 target_id；
18. PATROL_RECOVERY_COMPLETE 与恢复 watchdog timeout 两种顺序都只回 PATROL 一次；
19. PAUSE 和 RESUME 不处理 READY；
20. PAUSE 后旧事件因 state_seq 失效；
21. RETURN 从 RUNNING 和 PAUSED 均可接受；
22. RETURN_REACHED 进入 RETURNED；
23. RETURN_FAILED 进入 FAILED；
24. RETURNING 中 TERMINATE 进入 TERMINATED；
25. 当前任务收到 `sequence_done=true` 进入 COMPLETED；
26. `sequence_done` 与目标事件两种回调顺序都只能完成一次父任务；
27. PAUSE 与最后 waypoint 到达并发时，持久完成快照仍能从 PAUSED 完成父任务；
28. RESUME 不会重新打开 `sequence_done=true` 的路线；
29. AUTHORIZED 和 TRACK 最终 TARGET_LOST 设置 handled_target_id；
30. 提前 TARGET_LOST 和 EXECUTION_ERROR 不设置 handled_target_id；
31. PAUSE/RESUME 保留已经成立的 handled_target_id；
32. RETURN、TERMINATE、COMPLETED、FAILED 和 reset 清除 handled_target_id；
33. state_seq 从 `0xFFFFFFFF` 回绕到 1 时，各状态消费者仍接受新状态并拒绝真正旧状态；
34. 所有结束状态清除 home 和实时目标；
35. `RETURN_REACHED`、`RETURN_FAILED` 和 RETURNING watchdog 的先后竞争只形成一次终态，
    且每次终态只递增一次 state_seq；
36. RETURN_REACHED 与 RETURN_FAILED 同时到达时只接受一条，不能覆盖已形成的 RETURNED/FAILED；
37. RESUME 与 `sequence_done=true` 的路线回调按同一把锁线性化，服务本身不伪造 COMPLETED；
    路线回调先到时 RESUME 得到明确的 `no_active_task`，RESUME 先到时后续路线回调仍只形成
    一次 COMPLETED；
38. 相同路线完成快照的周期重发不再次清理路线、不再次发布终态；
39. 活动父任务的 `last_task_outcome` 始终为 NONE，四种非 NONE 结果只出现在 NO_TASK 终态。

### 14.2 waypoint 测试

- NO_TASK 下编辑不产生可执行路径；
- START 后才开始规划；
- PAUSE 保留列表、current index 和完成状态；
- START/RESUME 拒绝空路线、已完成路线和非法进度，且拒绝不修改路线或父任务；
- RECOVER_PATROL 期间不发布 waypoint path；
- 恢复到 PATROL 后重新规划当前未完成点；
- RETURN、TERMINATE、COMPLETED 后路线清空；
- 初始 NO_TASK 状态不误清新路线；
- 非初始 reset 进入 STARTUP 时清空路线和 planner 请求；
- 在首次 READY 前再次 reset，也能根据更大的 STARTUP state_seq 清空已准备路线；
- 重复父任务状态不重复清理；
- 任务活动期间编辑请求被拒绝；
- 每次实际改变路线内容的编辑使 route_revision 恰好递增一次，拒绝编辑、清空已空路线和进度
  推进不递增；
- 上位客户端等待 route_revision 确认最后一次编辑后再 START；
- 调试 pause/resume Topic 不能绕过 PatrolTaskState；
- planner 旧 sequence_version 结果被丢弃；
- 最后 waypoint 发布持久的 `sequence_done=true`，周期重发不重复结束任务。

### 14.3 mux 和停止链测试

- 启动、READY、PAUSED 和 NO_TASK 下始终 STOP；
- PATROL/CONFIRM 只接受 waypoint path；
- 目标处置、恢复和返航只接受 mission path；
- 每个新 state_seq 都先发布空路径，即使 source 没变；
- state_seq 回绕到 1 时仍建立新屏障，不把它误判为旧状态；
- 切换后旧 source 路径不能恢复；
- waypoint 和 mission path 都拒绝不晚于状态屏障的非空路径；
- 屏障后当前生产者重新发布路径，系统能够继续运动；
- 快速 PAUSE/RESUME 后只转发新状态产生的新路径；
- TERMINATE 后 global path、local path 和底盘命令最终清零。
- supervisor 状态 heartbeat 超时后，即使最后状态是 RUNNING，各节点也本地 fail closed；
- heartbeat 恢复时先清旧缓存并等待新路径，不直接恢复超时前路径。

### 14.4 导航测试

- 移除 waypoint pause/resume Topic 后，strict readiness 不因此永久阻塞 READY；
- READY 状态发布 WAITING_TARGET/无目标诊断，不落入 unsupported state；
- CONFIRM_TARGET 发布 TARGET_POSITION_READY 前先保存并绑定中断点；
- 中断点 TF 捕获失败时不进入 APPROACH_TARGET，并由确认超时安全恢复；
- PATROL/CONFIRM 暂停后 RESUME 到 PATROL 时清除候选/陈旧中断点；
- TARGET_POSITION_READY 后 APPROACH 快照被跳过再 PAUSE 时，RESUME 到 RECOVER 仍能使用已绑定
  候选中断点；
- 各目标状态暂停时取消 planner；
- RECOVER 暂停时保留中断点；
- RESUME 后能重新规划中断点；
- RECOVER 即将完成时 PAUSE，使旧 completion 失效；RESUME 后以新 state_seq 重新确认完成；
- 无中断点时恢复控制器安全回 PATROL；
- target_id=0 的暂停恢复能发布并完成 PATROL_RECOVERY_COMPLETE；
- RETURN 使用 START 保存的 home；
- PatrolTaskState.RETURNING 和 MissionState.READY 两种到达顺序都只启动一次返航 planner；
- 两条状态 state_seq 未匹配时不启动返航并保持停止；
- RETURN planner goal 使用当前 ROS 时间戳，不复用 START 时的旧 header stamp；
- RETURN 不先回巡检中断点；
- 到家稳定停车后只发布一次 RETURN_REACHED；
- 明确不可恢复的返航故障发布 RETURN_FAILED，整体返航超时由 supervisor 收口；
- RETURNING 中 TERMINATE 使旧返航结果失效。

### 14.5 感知和认证测试

- READY 下不选择目标；
- MissionRosAdapter 接受 READY+target_id=0+handled_target_id>0；
- MissionRosAdapter 接受暂停恢复 RECOVER+target_id=0，并保留 handled_target_id；
- MissionRosAdapter 继续拒绝活动目标阶段 target_id=0；
- MissionRosAdapter 继续拒绝普通 RECOVER 中与 target_id 不相等的正 handled ID；
- PrimaryTargetManager 在 READY 和 RECOVER_PATROL 下不选择 primary target；
- PAUSE 进入 READY 时清当前 primary 但保留 handled 排除表；
- 连续处理多个目标后，PrimaryTargetManager 同时排除所有尚未过期的 handled ID；
- handled 目标持续可见时保持排除，持续离开画面达到配置时间后才重新允许选择；
- READY/RECOVER/目标活动阶段冻结 handled absence timer，PAUSE 时长不计入过期；
- 相同 PATROL state_seq 周期重发不重置 handled absence timer；
- PAUSE/RESUME 造成 PATROL 重入时，已有 handled ID 的 absence timer 不被重新置零；
- 普通任务结束和新 START 不强制清空尚未过期的感知排除表；
- `/mission/reset` 进入 STARTUP 时清空感知排除表和 absence timer；
- STARTUP preflight 尚未结束即进入 READY 时，迟到 readiness 结果不再发布；
- PAUSE 进入 READY 时取消当前认证；
- 取消不会产生 UNAUTHORIZED；
- PAUSE 后旧 bbox、face、voice 结果被拒绝；
- RESUME 不恢复旧目标；
- PATROL 后重新进入搜索；
- 父任务 RETURN/TERMINATE 时核验子任务正确结束。

### 14.6 集成验收顺序

1. 启动整套系统，确认 `STARTUP -> READY` 且机器人不运动；
2. 发布 waypoint，确认仍不运动；
3. START 后开始当前 waypoint；
4. 在 PATROL 中 PAUSE/RESUME；
5. 在 APPROACH、VERIFY、TRACK、RECOVER 中分别 PAUSE/RESUME；
6. 验证暂停后不重新 READY，继续直接进入 PATROL 或 RECOVER；
7. 验证正常路线完成后回 READY、路线清空；
8. 从 RUNNING 和 PAUSED 分别 RETURN；
9. 验证返航成功和返航失败；
10. 从 RUNNING、PAUSED、RETURNING 分别 TERMINATE；
11. 验证每个结束场景后只能重新准备路线并 START；
12. 重启整套业务，确认旧任务不恢复并重新经历 STARTUP/READY。
13. 分别终止 supervisor、waypoint、mux、navigation coordinator 和关键感知节点，确认统一
    watchdog 停止控制输出、整套重启且不会单节点恢复旧任务。
14. 上位任务系统保持运行而机器人业务栈重启，确认它重置 state_seq 基线、结束旧任务记录并
    接受新会话的 STARTUP/READY。
15. 整套重启后不自动加载或重放旧 waypoint，必须重新准备路线并 START 才能运动。

## 15. 实施顺序

建议按以下顺序落地，每一步先完成对应测试：

1. 扩展接口包：READY、PatrolTaskState、WaypointSequenceState、PatrolTaskCommand 和新增事件；
2. 扩展纯 Python state machine，先实现 STARTUP/READY 和五个父任务命令；
3. 修改 supervisor 发布两条同序号状态并接入命令服务和 home TF；
4. 修改 navigation policy、tracking 和认证节点，使 READY 成为安全待命；
5. 修改 waypoint publisher，加入父任务门控、状态发布、完成状态和终态清理；
6. 将 mux 改为只按 PatrolTaskState.navigation_mode 仲裁；
7. 实现 navigation coordinator 的父任务返航；
8. 完成暂停中断点保留和 RESUME 恢复；
9. 清理旧 pause/resume 参数并更新生产 launch 的 fail-closed 配置；
10. 补齐 manager、move、robot_console 的 package 依赖；
11. 更新 fake integration、mock events 和所有 READY 相关测试 helper；
12. 接入 robot console 调试入口；
13. 编译完整依赖闭包并执行单元、集成和真机测试。

不要同时大规模重构现有目标处置代码。每一步只修改当前接口所需的最小范围，保证当前
PATROL 到 RECOVER_PATROL 流程持续通过测试。

## 16. 最终不变量

实现完成后必须始终满足：

1. 现有 MissionState 0 到 6 的名称、数值和业务含义不变；
2. STARTUP 只表示首次初始化或 reset 后等待 READY；
3. READY 表示模块已初始化且当前业务安全待命；
4. 没有 START 就不能执行 waypoint 或选择人员目标；
5. PAUSE 进入 `TASK_PAUSED + READY`，不重新执行 READY 握手；
6. RESUME 直接进入 PATROL 或 RECOVER_PATROL；
7. PAUSE/RESUME 不恢复旧目标和认证会话；
8. 已形成中断点的目标阶段暂停后，RESUME 先恢复中断点；
9. 暂停恢复型 RECOVER 的 target_id 为 0，普通目标恢复继续使用原非零 target_id；
10. handled_target_id 只表示最近形成最终结果的目标，多个排除目标由感知内部表保存；
11. PAUSE/RESUME 保留 handled_target_id，但不恢复实时目标或认证会话；
12. RETURN、TERMINATE、COMPLETED、FAILED、reset 和新 START 清除消息中的 handled_target_id；
13. 普通任务边界不清空感知内部未过期排除表，STARTUP reset 和整套重启必须清空；
14. RETURN 结束原路线并只去 home pose；
15. TERMINATE 随时停止且不返航；
16. 机器人端形成的任务完成、返航完成、终止和失败收口都进入 `TASK_NO_TASK + READY`；进程
    崩溃由外部会话管理单独记录，不伪造这一快照；
17. 只有整套启动或 reset 才回 `TASK_NO_TASK + STARTUP`；
18. `/mission/state` 和 `/patrol_task/state` 只有 supervisor 一个发布者；
19. 两条状态使用同一个 state_seq；
20. state_seq 的新旧判断使用 uint32 wrap-aware helper，不能直接比较整数大小；
21. `/mission/task_command` 是唯一父任务生产命令入口；
22. waypoint 运行许可只来自 PatrolTaskState，不来自调试 pause/resume Topic；
23. mux 只按 PatrolTaskState.navigation_mode 选择路径；
24. 每个新 state_seq 都建立空路径屏障，当前路径生产者必须随后重发；
25. `/global_path` 只有 mux 一个发布者；
26. waypoint 列表只有 waypoint publisher 一个权威所有者；
27. route_revision 只确认路线内容编辑，不能充当任务 ID 或跨任务路线身份；
28. `sequence_done` 是父任务完成的持久事实，不能只用一次性事件表达；
29. 父任务活动或暂停期间禁止路线编辑；
30. 旧 state_seq、target_id 或本地 planner generation 的结果不能生效；
31. 机器人端不保存业务父任务 ID、子任务 ID和事件 ID；
32. 进程级故障通过整套重启处理，不热恢复旧任务；
33. 当前版本不引入 runtime、task、route epoch 或路线清理握手。
