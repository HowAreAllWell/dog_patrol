# ReID confirm 去重闸门 — tracking 侧改动说明

基线：`merge` 分支 `47f7d48`（PR #50 第五轮，2026-10-11 复审 N06/T01 返工）。
在 confirm_target 决策链上集成短时 ReID 去重，按评审 R04/R05/R06-A + N01-N06 方案：
**谁算"处置完成"由 supervisor 决定，ReID 只把该结论按外观延续**（换
track、重启后仍认得）；被去重者从选人资格中排除并让出 primary。
处置资格**当日有效**（契约：处置过的人当天免重复处置，跨天/reset 失效）。
所有改动仅在本仓库 `src/perception/dog_patrol_perception_tracking` 内，
不改变 merge 已有的任务生命周期/状态校验/心跳逻辑。

## 改动清单

| 文件 | 改动 |
|---|---|
| `src/modules/reid_confirm_gate.cpp` + hpp | **新增**。异步问询闸门：非阻塞 Allow、pending hold、超时/服务不可用 fail-open、结论锁存（跨天失效）、排除队列、handled 转发（带 crop 兜底）；**N02**：取消代次+请求号，reset/跨天/超时/被替代的旧响应一律丢弃；**N03**：`SyncDay()` 每帧独立处理跨天（清锁存/排除队列/在途问询） |
| `src/modules/primary_target_manager.cpp` + hpp | **新增 ReID 排除表**（R04）：仅影响巡逻选人资格，**当日有效**（N04 契约，不随不可见释放；业务 handled 表的 60s 释放规则不变）；**N01**：巡逻选人边界重评当前 primary 资格，被排除的在位者让位重选；**N03**：`ClearReidDedupExclusions()` 跨天清理 |
| `include/.../mission_frame_transaction.hpp` | 转发器 `ExcludeForReidDedup` / `ClearReidDedupExclusions`（inline） |
| `src/modules/mission_ros_adapter.cpp` + hpp | ProcessFrame 可选第 4 参 `confirm_gate`（默认空=原行为，既有测试零改动）；新增 `ExcludeForReidDedup` / `ClearReidDedupExclusions`（持 mission_mutex_） |
| `src/modules/mission_tracking_runtime.{hpp,cpp}` | **新增**（评审 N06/T01）：MissionTrackingRuntime 与运行时公共抽象从 perception_tracking_node.cpp 拆出、编入 `_ros_adapter`（OFF 配置可链接可回归）；**N06**：handled 转发按"新的权威处置结果"去重——① 观察到 handled_target_id 相对上一观察快照的转换（0→X 即新完成；同值 heartbeat/RECOVER→PATROL 携带/跨天保留旧快照不触发）＋② 活动处置阶段重武装（跳帧兜底）；STARTUP 重入清转发上下文 |
| `src/perception_tracking_node.cpp` | mission 模式：reid.* 参数、构造 gate 与 runtime；运行时实现已拆至 modules/mission_tracking_runtime |
| `test/test_reid_dedup_exclusion.cpp` | **新增**（评审 R18/N01-N04）：manager 级 7 例——排除释放在位 primary、全员排除进入等待、非 primary 不扰动、活动目标不换人、排除不随 60s 不可见释放、跨天只清 ReID 表、STARTUP 全清 |
| `test/test_reid_confirm_gate.cpp` | **新增**（N02/N03/T01）：真实 gate + 受控 mock service（Reentrant 回调组 + 多线程执行器，支持乱序应答）7 例——reset 作废在途响应、超时 fail-open 与 Q2 成功分开断言（Q2 在自身超时预算内验收，删响应即失败）、Q2 allow 先回/Q1 suppress 迟到不覆盖、Q2 suppress 先回/Q1 allow 迟到不解锁、跨天清理不依赖 Allow、跨天作废在途问询、未启用恒放行 |
| `test/test_reid_handled_forwarding.cpp` | **新增**（N06 生产 runtime 回归）：真实 MissionRosAdapter（StoreMissionState 直喂合法快照）+ 真实 gate + 订阅 handled 通道计数 5 例——同 ID 次日新完成再转发、heartbeat/携带结论不重复、跨天保留旧快照不重新授予、不同 ID 转发、reset 后再处置转发 |
| `CMakeLists.txt` / `package.xml` | find_package + depend `dog_patrol_reid_provider`；gate 源文件进 `${PROJECT_NAME}_ros_adapter`；依赖走 ament_target_dependencies（评审 R15 修正，不用 rosidl_get_typesupport_target）；两个新 gtest 进 colcon test |

## 数据流

```
帧处理(串行, 不等 ReID)              MissionRosAdapter / PrimaryTargetManager
  ├─ N03 跨天：gate.SyncDay()==true ──► ClearReidDedupExclusions（选人层同步失效）
  ├─ R04 注入：gate.DrainExclusions ──► ExcludeForReidDedup ──► 选人跳过已去重者
  │            （N01：UpdateForPatrol 每帧重评在位 primary 资格，被排除者让位重选）
  ├─ R06/N06 转发：观察到 handled_target_id 的新权威结论（值转换或重武装）
  │            ──► gate.NotifyHandled ──► /perception/reid/handled
  │            重回 STARTUP(reset)  ──► gate.NotifyHandledReset（清 provider handled + 本地锁存 + 在途响应作废）
  └─ confirm 事件 ──► gate.Allow(id, crop)   【非阻塞】
        ├─ 已锁存 seen+handled → false 抑制
        ├─ 问询在途(pending)   → false hold 本帧（靠事务层 100ms 重发再试）
        ├─ 放行窗口内          → true 发布
        └─ 无结论 → 异步发起问询（helper 线程收响应，携带代次+请求号；
          仅当仍属当前代次/条目仍 pending/请求号匹配时更新锁存，
          决策永远发生在下一帧的当前状态上下文里 → R05/N02 结构性成立）

ReID 节点：/perception/reid/handled 落库（track 绑定优先，crop 特征兜底，
均须过身份阈值不误标）→ 同人换 track 再问询时响应 handled=true
```

## 行为约定

1. **fail-open**：服务不在线 / 问询在途超过 `reid.timeout_ms`（默认 3s）/
   ERROR → 放行 confirm，退回无去重原行为；**Allow() 从不阻塞**（R05：
   mission_mutex 内无 service 等待）。
2. **抑制条件 = is_seen && handled**（R06-A）：仅"见过"未处置的人照常走
   处置流程；处置资格来自 supervisor（handled 结论经 gate 转发落 ReID 库）。
3. **处置资格当日有效**（N04 契约）：seen+handled 按语义 ID 持续抑制并
   进入选人排除表；排除**不随持续不可见释放**（业务 handled 表的 60s 释放
   是 supervisor 结论的 merge 原规则，两层各自维护、互不覆盖）；跨天时
   gate 锁存、排除队列、在途问询与选人层排除表由 `SyncDay()` 统一作废
   （N03，不依赖 confirm 事件触发）；STARTUP/reset 全清。
4. **旧响应丢弃**（N02）：reset、跨天、超时 fail-open、被新问询替代之后
   到达的响应整体丢弃，不得重建条目或排除，避免污染新业务上下文。
5. **handled 转发按新结论去重**（N06）：转发不按 ID 记账，而按"新的权威
   处置结果"——观察到 handled_target_id 相对上一帧快照发生变化（0→X 即
   新完成），或该 ID 经历过活动处置阶段后结论再次出现。同 ID 跨天重新
   处置完成会再次转发；同值 heartbeat、RECOVER→PATROL 携带、跨天保留的
   旧快照不会重复授予。已知理论残留：若帧处理把整个处置相位全部跳过
  （转换与活动相位一个都没观察到），同 ID 新结论会漏发——心跳 Hz 级、
   处置相位持续秒级，正常帧率下不构成现实路径。
6. **问询在途 hold**：pending 期间 confirm 事件不发布（事务层 100ms 重发
   自愈）；新人首次确认延迟约一个问询往返 + 一次重发周期。
7. 正常日志：`[reid_gate] ... 新人/见过但未处置，放行`、`... 已见过且已
   处置，抑制 confirm_target`、`[reid_gate] 跨天 ...`、`[handled] ... 已标记处置（按 track 绑定/
   特征检索）`。

## 参数（tracking 节点新增，仅 mission 模式声明）

```yaml
reid.enable: true                  # false = 完全旁路（gate 为空，零行为变化）
reid.service_name: /perception/reid/dedup
reid.handled_topic: /perception/reid/handled
reid.timeout_ms: 3000              # 问询在途超时：fail-open
reid.retry_interval_sec: 2.0       # 放行结论窗口，过后可重问
reid.max_tracked: 64
```

## 与评审验收项的对应

- R04/N01：已处置的 A 持续可见时让出 primary，B 新人被选为 primary 并发布
  确认（`UpdateForPatrol` 每帧重评在位 primary 资格）；全员被排除 → 无
  primary 无事件无问询；活动目标阶段（APPROACH/VERIFY/TRACK）仍只绑定
  supervisor.target_id，不因排除换人（`test_reid_dedup_exclusion` 覆盖）。
- R05：ReID 慢/冷加载/超时期间状态接收与帧处理不被阻塞；迟到结论只更新
  锁存，不凭旧 state_seq 补发事件。
- R06：确认失败/接近失败/核验前暂停或终止的人（未拿到 handled）再次出现
  仍会 confirm；跨狗仅"见过"（无 handled）不推定处置完成。
- N02：reset 清缓存后放回旧响应，entries/exclusions 与候选资格不变；旧
  请求晚于新请求返回、超时后重问的迟到结论均丢弃
  （`test_reid_confirm_gate` 覆盖）。
- N03：所有可见人员都被 ReID 排除时，日期推进到次日由每帧 `SyncDay()` 独
  立清理旧排除（锁存/排除队列/在途问询/选人层四处置同步失效），不依赖
  Allow 被调用；业务 handled 表不受影响。
- N04：契约对齐——处置资格当日有效：选人层 ReID 排除与 gate 锁存、
  provider handled 三层同生命周期（当日/跨天/reset），不随持续不可见释放；
  评审原探针"61 秒后应重新具备资格"的期望按新契约反转（见
  `ExclusionSurvivesAbsenceBeyondHandledIgnoreWindow`）。
- N06：同 ID 次日重新处置完成再次转发 handled；heartbeat/RECOVER→PATROL
  携带不重复；跨天保留旧快照不重新授予今日资格；不同 ID/reset 后正常
  （`test_reid_handled_forwarding` 用生产 MissionTrackingRuntime 覆盖）。
- T01：Q2 成功断言在自身超时预算内验收（删 Q2 响应即失败），乱序应答
 （Q2 先回、Q1 迟到）双向覆盖（`test_reid_confirm_gate`）。

## 构建与联调

1. ReID 节点包位于本仓库 `src/perception/dog_patrol_reid_provider`，随仓库
   一起进 colcon 工作空间。
2. `colcon build --packages-select dog_patrol_reid_provider dog_patrol_perception_tracking`
3. 先起 ReID 节点（model_path 必填，device_id 唯一），再起感知链路；
   `reid.enable:=false` 可随时旁路对比。
4. 接口字段与降级语义见 provider 包内 `docs/reid_dedup_interface_for_detection.md`；
   联调客户端 `ros2 run dog_patrol_reid_provider sim_detection_client.py`。
