# ReID PR #50 / #51 详细评审意见

## 最新验收结论：2026-10-12，PR #51，88fa528

**结论：在用户确认的运行范围和已接受限制下，可以接受本 PR。** 用户于 2026-10-12
完成基本实机测试，反馈主要行为正常，并明确接受下述低概率漏标。本轮不再将其作为合并
阻断项；之前的 N06、原 T01 仍保持关闭。

当前判断针对已构建并测试的 `88fa5286dd3fb04720ae4b193fe74ec7d60b1bc0`，依据包括前轮
代码审查、主工作区五包构建、manager 70 项、provider 2/2 CTest、tracking 最终完整
58/58 CTest、真实 TensorRT 服务验证，以及用户本次实机反馈。这不扩大为所有现场人员
精度和任意故障恢复场景均已验收的承诺。

### 本轮确认的范围和已接受限制

- **运行前提：** 用户明确 tracking 与 ReID 一起执行，不会出现所讨论的单侧离线或独立
  重启。因此，相关故障恢复用例不作为当前待修项，不围绕它们增加重传或额外协议。
- **正常处理规则：** 已处理结论由 supervisor 产生，ReID 缓存当天多个人的结论；后者
  不是 supervisor 最近一个 handled_target_id 字段的逐值镜像。开始处理 B 后该字段为 0、
  ReID 仍保留此前已处理 A 的记录，是预期行为。未处理时暂停不会凭空新增 handled。
- **已接受的低概率漏标：** AUTHORIZED 已被 supervisor 接受后，如果下一次图像帧处理前
  TERMINATE 等状态变化已将最新 handled_target_id 清零，帧路径可能漏转发这次完成结果。
  主要后果是以后可能重复核验；这条漏发本身不会把未处理人员标成已处理。
- **用户决定：** “漏了都没事，因为这个太不容易发生了”“我刚测试基本上问题不大，可以接受了”。
  保留已知限制和复现证据，不把该用例失败表述为已修复或测试全绿，也不继续作为接收阻断。

在线补充检查使用当前生产 runtime、adapter、gate 和实际 gallery API，接收器只按消息
执行标记/清理；没有启动真实模型或接触域 42。四项对照中，正常在线完成与未处理暂停、在线
reset、完成结果已被帧消费后再终止，均通过；完成与终止落在同一帧间隔的用例复现了上述
漏标。输入 `VERIFY(seq=6) → AUTHORIZED/RECOVER(seq=7, handled=7) → TERMINATE/READY
(seq=8, handled=0)` 已通过真实纯状态机验证为合法顺序，两个状态均被 adapter 接受。

材料位于 `build/pr51-review-88fa528/consistency-review/`、
`consistency_online.log` 和 `consistency_online.xml`。较早的 `consistency_review.xml`
包含已被用户排除的离线/独立重启场景，不能据其失败数量增加本轮合并阻断意见。

2026-10-12 先按用户要求完成本地合并，随后用户明确授权接收远端 PR 并同步分支。现已将
合并提交 `1cffd2406ac9a97767d8433a3ddfc0c2ceace1e9` 推送到 `origin/merge`，GitHub API
确认 PR #51 为 closed、merged=true，目标分支为 merge，PR head 仍为已验收的 `88fa528`。
合并保留双父提交，业务代码与测试版本一致；本地 `feature/reid-dedup-gate` 分支保留。
验收记录和实机指南已归档，后续工作区使用 merge 分支。

## 历史复审：2026-10-11，PR #51，88fa528

**当时结论：N06 和原 T01 已修复，本轮未发现新的 P1/P2 问题，可以进入实机验收。**
这是对当前 head 的代码审查结论，不表示真实人员识别精度、多人现场表现或整机巡检已经验收。
审查阶段没有合并 PR、发布 GitHub review 或覆盖工作区生产 install。后续工作区测试准备见
[PR #51 实机测试说明](../integration/reid_pr51_robot_test_guide.md)。

### 当前范围与版本

| 项目 | 内容 |
| --- | --- |
| 新 PR | [PR #51](https://github.com/HowAreAllWell/dog_patrol/pull/51)，open、非 draft |
| 目标分支 | `merge`，已通过 GitHub API 确认，不再是此前的 main 目标 |
| 当前 head | `88fa5286dd3fb04720ae4b193fe74ec7d60b1bc0` |
| 比较基准 | merge `47f7d487925911093ddab262b69ffb675d6a9124`，merge-base 与该提交一致 |
| 上次已审查版本 | `1aea964cd770aec0a65efd5ac22656c2168ccc7b` |
| 本轮增量 | 相比 1aea964 改动 7 个文件；相对 merge 共 49 个改动文件、1 个新增提交 |
| 审查引用 | `origin/pr-51-review-20261011`，旧版引用保留 |
| 审查时工作分支 | `merge`，审查过程中未切换到 PR 分支 |
| 实际测试产物 | `/mnt/nvme/workspace/dog_patrol/build/pr51-review-88fa528/artifacts` |
| 兼容测试路径 | `/tmp/dog_patrol_pr51_20261011.0YZ1Xq`，现为指向上述 NVMe 目录的符号链接 |

本轮逐项检查七个增量文件：runtime 头文件/实现、生产 tracking 节点、CMake、gate 测试、
handled 转发测试和 REID_GATE_NOTES。provider、MissionRosAdapter、PrimaryTargetManager、
ReidConfirmGate 的业务源码以及旧任务/导航/认证模块相对上一版未变，仍重跑相关回归。
用于验证的 49 个 PR 改动文件与当前 head 逐文件比较一致；独立探针和变异测试放在单独目录，
未修改 PR 原始业务源码或原始测试。

### N06：同 ID 次日新处置漏转发，已关闭

当前实现位于
[mission_tracking_runtime.cpp:79](https://github.com/HowAreAllWell/dog_patrol/blob/88fa5286dd3fb04720ae4b193fe74ec7d60b1bc0/src/perception/dog_patrol_perception_tracking/src/modules/mission_tracking_runtime.cpp#L79)。
作者把 `MissionTrackingRuntime` 从生产节点拆成公共模块，编入 ROS adapter 库，使 ON/OFF
配置都能直接测试同一份生产实现。核心转发逻辑不再只记住“上次转发的 ID”：现在观察
handled_target_id 的转换及活动目标上下文，识别同 ID 的新处置结果。

本轮没有只依赖作者新增用例：上一轮的独立边界复现 `runtime_review` 原样保留断言，新版
1/1 通过；由真实 MissionStateMachine 生成完整合法序列的 `runtime_lifecycle_review`
新版 3/3 通过。测试适配仅将原来的包含生产节点 cpp 改为包含新公开头文件并链接生产库，
没有改动“同 ID 次日必须再次转发”等期望。

| 验收场景 | 当前结果 |
| --- | --- |
| 同 ID 次日重新经历完整处置并形成 AUTHORIZED 结果 | 再次转发，原漏发问题不再出现 |
| 同序号 heartbeat，以及 RECOVER→PATROL 携带同一结论 | 不重复转发 |
| 跨天仍保留昨日的 handled 快照 | 不重新授予今日已处理资格 |
| 不同 ID 的新处置结果 | 正常转发 |
| STARTUP reset 后再次完成同 ID | 清除转发上下文并正常发送新结果 |

作者新增的 `test_reid_handled_forwarding` 5/5 通过，使用真实 runtime、adapter、gate 和
ROS handled 通道，覆盖方向合理。它们是状态接收/转发边界测试，直喂快照不等于完整状态机
或真实识别闭环；上面的独立完整序列验证补足了这一层区别。

### T01：Q2 成功断言的确定性假通过路径，已关闭

当前测试位于
[test_reid_confirm_gate.cpp:242](https://github.com/HowAreAllWell/dog_patrol/blob/88fa5286dd3fb04720ae4b193fe74ec7d60b1bc0/src/perception/dog_patrol_perception_tracking/test/test_reid_confirm_gate.cpp#L242)。
作者将 mock 改为 Reentrant 回调组和多线程执行器，能在 Q1 仍挂起时接收并返回 Q2；请求
等待有界，正向排除采用条件等待。Q1 超时放行与 Q2 正常响应的成功断言分开，新增两种相反
结论的乱序返回测试。当前 gate 正式测试 7/7 通过。

本次再次把原测试复制到独立文件，**只删除 Q2 的 Respond 调用**，保留 timeout、sleep、
helper 和断言，再编译运行 `TimeoutFailsOpenAndNewResponseAcceptedWithinBudget`：
`AllowedWithinBudget(7, 60)` 明确失败，退出码为 1。正常原测试通过、删响应对照失败，证明
上一版“无需 Q2 响应也能靠第二次超时通过”的确定性路径已消除。这次故意制造的失败是测试
有效性的证据，不是 PR 新业务缺陷。

另用上一轮独立的 reentrant mock 验证 Q2 allow→Q1 suppress、Q2 suppress→Q1 allow，
以及 ERROR、服务缺失回退和四种活动目标阶段不换人，5/5 通过。没有为了通过作者测试而
修改旧探针断言。

测试仍使用毫秒级等待，helper 的预算按轮询次数估算。后续可改为 steady_clock 的实际剩余
预算并给 DDS 调度留出更大余量，以改善繁忙机器上的稳定性；不将这项改进与旧版必然超时的
假通过问题混为一谈，也不作为本轮新增业务阻断项。

### 历史问题的当前复核状态

| 历史意见 | 当前判断及本轮依据 |
| --- | --- |
| R10～R15 构建/链接 | 保持关闭；系统 colcon 完成 TRT/Orin ON 两包构建，另完成 tracking OFF 构建和运行测试 |
| R01/R02/R03/R08/R09 生命周期入口 | 保持关闭；原 MissionRosAdapter 19/19 通过，READY、暂停恢复、heartbeat、冲突和零序号规则继续有效 |
| R04/N01 候选被去重者挡住 | 保持关闭；正式选人 7/7、独立 gate/primary 7/7 及活动目标阶段绑定检查通过 |
| R05/N02 阻塞与旧异步结果 | 保持关闭；非阻塞状态接收、reset、跨天、替代请求和乱序测试通过 |
| R06/N06 处置资格与反馈 | 见过且已处置才豁免的判定保留，原跨日漏转发也已关闭；失败/取消不授予 handled 的旧 manager 测试通过 |
| R07/N03 跨天失效 | 保持关闭；每帧 SyncDay、选人表清理、全员排除与旧回调失效的独立检查通过 |
| N04 当日策略 | 按用户确认保持关闭；已处置人员当日免重复处理，不要求改回基础 60 秒规则 |
| R16/R17 持久化 | 保持关闭；对当前 provider 真实源码再次运行五项独立异常/恢复测试，5/5 通过 |
| R18/T01 测试 | 注册和原缺口已修复；新增 19 项正式用例执行，删 Q2 响应反向验证按预期失败 |
| N05 模型部署/Python 接口生成 | 保持关闭；正式 TensorRT 引擎再次 GPU 推理成功；m20_nav 导入新接口并创建实际 rclpy client 成功 |
| B01 基础 absence 计时 | 本轮涉及的基础实现未变；仍是 merge 既有问题，保留文末记录，不算 PR 新回归 |

### 实际测试结果

| 检查 | 结果 |
| --- | --- |
| 系统 colcon，provider + tracking，TRT/Orin runtime ON | 2 包构建成功 |
| 系统 colcon，tracking runtime OFF | 构建成功；磁盘环境处理后重试成功，见下文 |
| ON provider 正式 CTest | 2/2 通过 |
| ON tracking 正式 CTest | 57/58 通过；唯一失败为既有 visualizer 的临时 cwd/数据根重叠 |
| 同一 ON visualizer binary 从真实工作区执行 | 12/12 通过；原 CTest 内部失败的是 4 个 gtest，用例源码未被本 PR 修改 |
| OFF 新增三组 + 原 adapter | 4/4 CTest 通过，即新增 19 项与原 adapter 19 项，共 38 个 gtest |
| 新增正式测试 | 选人 7 + gate 7 + handled 转发 5 = 19 项通过 |
| 原 Python 回归，m20_nav | 369 passed：face 87、voice 64、orchestrator 49、manager 70、navigation 68、waypoint 12、console 10、fake 7、mock events 2 |
| 上轮独立 gate/primary、异步、持久化、重入、完整 runtime、边界 runtime 探针 | 7 + 5 + 5 + 1 + 3 + 1 = 22 项通过 |
| 作者新版测试删 Q2 响应的反向对照 | 1 项按预期失败；正常原测试通过，证明原 T01 路径关闭 |
| 当前 C++ TensorRT FeatureExtractor 加载正式引擎 | 推理成功，1280 维、有限值、L2 norm=1；模型未重新转换 |
| 新 ReID Python 绑定及原父任务接口 | m20_nav 导入及创建 client 成功，READY=7、返航事件=9/10、TASK_RUNNING=1、Request.START=0 |

不把以上结果写成“整个 colcon test 全绿”，也不把 CTest 包装层与 gtest XML 重复累计成
业务用例总数。现有 visualizer 环境失败与已复现的产品缺陷分别处理。

### 环境、材料与尚未验证的内容

补充 OFF 构建首次遇到系统盘 `/` 已满，日志明确为 ENOSPC，而非源码编译错误。本轮将自己
创建的新旧 PR 临时审查目录搬到工作区已忽略的 build 子目录，保留原 /tmp 符号链接；未删除
用户数据或 stash。OFF 编译临时文件也改放 NVMe，之后构建与四组测试通过。这些路径变化
不涉及业务实现或测试期望的修改。

本轮材料位于上述 NVMe artifacts 目录，主要包括：

- `build_console.log`、`test_console.log`、`test_result.log`、`build/*/test_results`：ON 构建和正式测试。
- `off_build_console.log`：首次 ENOSPC；`off_build_retry.log`、`off_test_console.log`：恢复后的 OFF 构建/测试。
- `python_regression.log/xml`：369 项既有 Python 回归。
- `runtime_review.log/xml`、`runtime_lifecycle_review.log/xml`：N06 的独立前后对照，当前全部通过。
- `gate_review`、`async_gate_review`、`persistence_review`、`identity_reentry_review` 的 log/xml：旧问题独立复测。
- `review/gate_test_no_q2_response.cpp`、`mutation_no_q2.log/xml`：新版 T01 删响应对照。
- `visualizer_workspace.log/xml`、`model_review.log`、`model_feature.f32`：环境复核和实际 GPU 输出。
- `review/`、`review-build/`、`review_evidence_summary.json`：探针源码、可执行文件和结果索引。

仍未完成真实人员同人/跨人标注数据的阈值标定、多人交叉录像精度对比、完整相机/认证/导航
运动验收，以及真实云端多狗同步；后者当前 cloud.enable=false，不在本轮范围。若整个目标
处置过程都未被帧路径观察到，仅凭最后一份相同 handled 快照无法区分新旧结果；当前实现
没有扩大协议来解决任意长停顿下的历史事件恢复，不能宣称具备这种保证。

PR #51 的正文还沿用“ReID 排除 60 秒不可见释放”和仅 OFF 验证等旧说明，建议作者同步为
已确认的当日策略及当前测试状态。这是说明同步建议，不重新打开 N04/N05，也不把已恢复
的构建错误继续列为当前缺陷。

## 历史复审：2026-10-11，PR #50，1aea964

以下为该版本当时的意见；N06、原 T01 已按文首 PR #51 的证据关闭。

**当时结论：仍建议 Request changes，处理 N06 及测试缺口 T01 后再合并。** 当时重新核对
R01～R18、N01～N06，没有因为之前标记过“关闭”就跳过。N01～N03 的实现通过独立复测；
N04 按用户明确确认的“处理过的人当天免重复处置”规则验收，不再要求 ReID 排除在 60 秒后
释放；N05 再次通过实际 TensorRT GPU 推理验证。

本轮新增一个已复现的 P2 问题：同一 semantic ID 在次日形成新的处置结果后，生产 runtime 的
handled 转发仍只按 ID 去重，漏掉今天的新结果。本次补充审查还确认 T01：新增超时测试的
Q2 成功断言可以靠超时放行通过，删除 Q2 响应后仍然全绿。当前业务待修项为 N06，测试待修项
为 T01；后面的旧版评审保留用于追溯，不把已经关闭的意见重复列为当前缺陷。

### 当前 PR 的修复清单与重新验收条件

此清单适用于已审查的 head `1aea964`，供作者落实本轮修改；历史章节中的构建阻断和旧版
修复顺序不再代表当前待办。R06 的基本判定已恢复，仍有 N06 的生产反馈问题；R18 的测试
注册已恢复，仍有 T01 的验证缺口，不能将这两项笼统标记为完全关闭。

| 待办 | 需要修改的内容 | 重新验收条件 |
| --- | --- | --- |
| N06，P2，业务问题 | 修正生产 runtime 的 handled 转发去重，识别同一 semantic ID 的新权威处置结果；不能只比较 ID，也不能在跨天后直接重发昨日结论 | 同 ID 次日新完成必须再次转发；同序号 heartbeat、RECOVER→PATROL 的同一结论不重复转发；跨天旧快照不重新授予今日已处理资格；不同 ID、reset 后新完成正常 |
| T01，P2，测试缺口 | 修改超时用例，使 Q2 正常响应的成功断言不能被 timeout fail-open 满足；增加可乱序响应的 mock 和有界请求等待；补入 N06 的 runtime 回归 | 正常 Q2 响应通过；删除 Q2 响应时对应成功断言必须失败；Q2 allow→旧 Q1 suppress、Q2 suppress→旧 Q1 allow 两种顺序都不被旧响应覆盖；N06 完整合法流程通过 |

**可直接用于当前 PR 的总评：** 当前版本已经恢复此前被破坏的 READY、暂停恢复、heartbeat、
同序号冲突和零序号校验，生产配置构建成功，369 项既有 Python 回归通过。新增选人测试总体
合理，但超时测试存在已证实的假通过路径，且生产 runtime 仍漏发同 ID 次日新处置的 handled。
请修复 N06、补强 T01 并保留原回归测试后重新提交验证。N04 按已确认的当日策略关闭，N05
按已完成的 TensorRT 部署关闭；既有 visualizer 临时目录失败和 B01 基础计时问题单独记录，
不列为本 PR 新增缺陷。当前仍建议 Request changes。

### 当前规则和审查范围

| 项目 | 内容 |
| --- | --- |
| 用户本轮确认 | “就是按当天处理，之前的是一个基础版本” |
| 本轮采用规则 | 已处置人员当天免重复处置；跨天重建当日资格；未处置/失败/取消不能仅凭见过免处置 |
| 当前 PR head | `1aea964cd770aec0a65efd5ac22656c2168ccc7b` |
| 比较基准 | merge：`47f7d487925911093ddab262b69ffb675d6a9124` |
| 上轮 head | `e08b320580653de0bf11a99c94a8a04faebb0936` |
| 本地审查引用 | `origin/pr-50-review-20261011`，旧引用保留 |
| 本轮验证目录 | `/tmp/dog_patrol_pr50_20261011.jwZjJ1` |
| 正式模型引擎 | `/home/orin/reid_clipreid_vitb16_after.fp16.engine`，本轮再次用 PR 实际 C++ 提特征实现加载并推理通过 |

**N04 的关闭依据是用户确认了新的业务规则。** 作者将 ReID 排除改为当日有效，并把旧的
“61 秒后允许重新处理”测试改为“当日仍排除”，在本轮新规则下合理。不是仅凭反转断言就宣称
旧规则已经修复。基础业务 handled 表仍保留原来的机制，ReID 表采用新的当日策略。

权威设计文档目前仍主要描述基础版本的 60 秒规则。建议 PR 同步补充启用 ReID 时的当日策略、
跨天/reset 和基础排除表之间的关系；这里属于已确认需求的文档同步，不再把旧 60 秒期望用于
阻断本轮实现。

### 构建、测试与已关闭意见

| 检查 | 结果 |
| --- | --- |
| 系统 colcon：provider + tracking，TRT/Orin runtime ON | 两包成功构建，未经修改的 PR 源码 |
| provider 正式 CTest | 2/2 通过 |
| tracking 正式 CTest，串行执行 | 56/57 通过；唯一失败是既有 visualizer 测试在 /tmp 构建 cwd 下的数据根重叠 |
| visualizer 从真实工作区单独执行 | 12/12 通过；不计为 PR 新增回归 |
| tracking 原版 MissionRosAdapterTest | 19/19 通过 |
| 新增 test_reid_dedup_exclusion | 7/7 通过 |
| 新增 test_reid_confirm_gate | 5/5 通过 |
| 上轮独立 gate/primary 探针，按当前接口和确认的新规则调整 | 7/7 通过 |
| ReID Python 生成接口和实际 rclpy client 创建 | 通过；上轮缺少生成器的问题已修复 |
| 本轮新增生产 runtime 跨日反馈探针 | 1 项失败，N06 |
| 既有 Python 回归：face / voice / authorization-readiness | 87 + 64 + 49 = 200 项通过，包含节点级取消、旧证据和迟到 preflight 测试 |
| 既有 Python 回归：manager / navigation / waypoint | 70 + 68 + 12 = 150 项通过；数量按当前源码重跑结果计 |
| 既有 Python 回归：robot_console / fake integration / mock mission events | 10 + 7 + 2 = 19 项通过；Python 合计 369 passed |
| 新增独立异步/活动目标探针 | 5/5 通过；包含 Q2 先于 Q1 返回、ERROR 和服务缺失回退，以及四种活动目标阶段不换人 |
| 新增独立持久化探针 | 5/5 通过；替换前进程退出、rename 失败、临时库恢复、时间重锚和 handled/track 恢复语义 |
| 用真实状态机生成输入的生产 runtime 联动探针 | 2/3 通过；同 ID 次日新处置漏发仍失败，heartbeat/旧快照和不同 ID/reset 对照通过 |
| N06 的同 semantic ID 重入前提 | 1/1 通过；实际 IdentityManager 在 2000 个缺失帧后可恢复原 ID，基础 handled 到期并清当日 ReID 排除后可重新被选中 |
| 新增超时测试的删响应对照 | 只删 Q2 Respond 后仍通过，确认 T01；这项“通过”是测试缺口的证据，不是业务验收通过 |
| 正式 TensorRT 引擎再次推理 | 1280 维、全部有限、L2 norm=1；未重新转换模型 |

原始整包 colcon 结果仍有上述环境相关失败，因此不写成“所有 colcon 测试全绿”。相关源码
未被 PR 修改，且同一个新编译的 visualizer 可执行文件从工作区执行通过，已明确区分验证环境
与 ReID 行为问题。

本次使用 PR 快照明确指定 `--base-paths` 重跑两个包，provider 2 个 CTest、tracking 57 个
CTest 均实际发现并执行；tracking 中 56 个目标通过，1 个 visualizer 目标失败。这个失败目标
内部是 4 个 gtest 用例失败，从真实工作区执行则 12/12 通过。`colcon test-result` 同时统计
CTest 包装层和 gtest XML，不能把它的汇总数字直接当成不重复的业务用例数。

### R01～R18 全量复查矩阵

以下状态均针对 `1aea964`，比较对象仍是 merge `47f7d48`。本次还逐文件核对了 PR 改动的
46 个文件，测试快照与该 head 内容完全相同；临时复现和删响应对照没有修改 PR 原始业务源码
或原始测试。未改动的 tracking identity/appearance 链路也随完整 tracking CTest 一起运行。

| 原意见 | 当前判断 | 本次证据与边界 |
| --- | --- | --- |
| R10 srv 分隔符 | 已修复 | 原文件存在真实 `---`；rosidl 生成成功，实际 Python client 创建成功，响应含 handled |
| R11 package group | 已修复 | 当前 package 元数据被系统 colcon/ament 接受，两包构建成功 |
| R12 Humble service QoS | 已修复 | 使用 `get_rmw_qos_profile()`；实际 reid_node 构建通过 |
| R13 引用对象错误访问 | 已修复 | 云端代理目标也在 provider 构建中成功编译链接，不是仅跳过该文件 |
| R14 TensorRT 编译/工厂参数 | 已修复 | `REID_BUILD_TENSORRT=ON` 实际编译；本轮实际加载正式引擎并推理成功 |
| R15 外部 typesupport 链接 | 已修复 | `TRACKING_ENABLE_ORIN_RUNTIME=ON` 构建生产 tracking 节点及 ROS adapter 成功，原 adapter 19/19 通过 |
| R01 READY | 已修复 | READY/READY+handled 映射恢复；原 adapter、primary/frame、认证节点的 READY 行为测试通过 |
| R02 暂停恢复 target_id=0 | 已修复 | 原 adapter 合法组合测试通过；manager 的暂停恢复测试继续通过 |
| R03 heartbeat | 已修复 | `MissionStateHeartbeatTimeoutFailsClosed` 和既有 frame 状态可用性测试通过，超时 API 已恢复 |
| R04 同屏新人被挡 | 已修复 | 作者 7 项选人测试及独立探针通过；已排除 primary 让位、全员排除 idle；四种活动目标阶段仍绑定权威 ID |
| R05 锁内同步等待 | 原阻塞问题已修复 | 实际 gate 非阻塞查询；旧探针确认状态接收不等待 service；原 adapter 旧帧失效和忙帧回调测试通过；异步失效另见 N02/T01 |
| R06 见过被误当成已处置 | 判定本体已修复，生产反馈仍有 N06 | gate 只采纳成功响应的 seen+handled；seen-only/ERROR 放行；manager 未完成/失败/取消不授予 handled 的测试通过；不能因此忽略跨日漏转发 |
| R07 跨天缓存失效 | 原问题已修复 | SyncDay 在每帧选人前调用；全员排除仍能跨天清表；gate 代次、待注入队列和选人 ReID 表一起失效，基础 handled 表不被误清 |
| R08 同序号冲突 | 已修复 | 两种原 adapter 冲突锁存测试通过；冲突后原快照不能恢复业务，更新序号可恢复 |
| R09 state_seq=0 | 已修复 | 原 adapter 入口拒绝断言通过，未恢复接受零序号的旧回归 |
| R16 先删旧库再替换 | Linux 原问题已修复 | 独立测试在真实 SaveToFile 的 rename 前 `_exit`，旧正式库仍可读；注入 EIO 后 Save 返回 false、dirty 保留、旧文件不丢；不是物理断电验证 |
| R17 跨开机 steady_clock | 已修复 | 独立测试以旧大时间戳落盘，加载到当前真实 steady 基线后，新身份不再被容量淘汰；不是用不相容的假 now 去测试真实重锚逻辑 |
| R18 测试注册/覆盖 | 注册已修复，覆盖仍需补齐 | provider 2 个 CTest 及新增 12 个 gtest 实际执行；T01 存在假通过路径，生产 runtime 的 N06 未被作者测试覆盖，不能笼统标记完全关闭 |

N01～N05 的关闭不是根据“作者说已修”得出的；除 N04 依据明确确认的需求变更外，均有当前
源码、实际构建或独立测试支撑。B01 是基础版本已有计时边界，单独放在本文末尾，不记成 PR
新增缺陷，也不作为推翻已确认当日策略的理由。

| 意见 | 本轮关闭结果 |
| --- | --- |
| N01 被排除者占 primary | 已修复；UpdateForPatrol 重评当前 primary 资格，新人可接替，活动任务不换人 |
| N02 reset 后旧回调重建排除 | 已修复；generation + request_id + pending 状态校验，旧响应不重建条目 |
| N03 全员排除时跨天无法清理 | 已修复；每帧先 SyncDay，再清选人层 ReID 排除和 Drain，跨天回调一并失效 |
| N04 60 秒释放与当日策略冲突 | 按用户确认的新规则关闭；独立探针已按当日有效验证 |
| N05 模型部署 | 保持关闭；使用已转换的 TensorRT FP16 引擎 |
| ReID Python 接口缺失 | 已修复；新增 rosidl_default_generators 和 runtime 导出，实际 client 创建成功 |

### N06 — [P2] 同一 semantic ID 次日重新处置完成后，handled 结论未再次转发

**已复现。评论位置：**
[perception_tracking_node.cpp:163](https://github.com/HowAreAllWell/dog_patrol/blob/1aea964cd770aec0a65efd5ac22656c2168ccc7b/src/perception/dog_patrol_perception_tracking/src/perception_tracking_node.cpp#L163)。

**原因：** MissionTrackingRuntime 用 `last_forwarded_handled_` 这个单独的 ID 判断是否已经转发。
它只在进入 STARTUP 的 reset 路径清零；SyncDay 不更新该值，观察到 handled_target_id=0 的
新核验过程也不更新该值。因此，“昨天的 A”和“今天重新处理完成的同一 semantic ID A”被当成
同一份已经转发过的结论。

**具体场景：**

1. 第一天人员 A（semantic_id=7）处理完成，runtime 发送 `semantic-7` 的 handled，记住 ID=7。
2. 进入第二天，SyncDay 清理 gate 当日缓存/排除，provider 的当日库也已经更新日期。
3. A 在第二天重新进入核验：`VERIFY_IDENTITY, target_id=7, handled_target_id=0`。
4. supervisor 接受本次完成结果，发布新的 `RECOVER_PATROL, target_id=7, handled_target_id=7`。
5. runtime 判断 `7 == last_forwarded_handled_`，不再发送今天的 handled。

**触发前提与选人资格：** 新的一天必须确实形成同 ID 的新处置结果，且期间未通过 STARTUP
reset 或另一个 ID 的处置改变转发去重值。不是说零点一过所有旧目标都会立刻被重选：仍在
基础 handled 排除表中的可见目标继续按基础规则排除。

本次额外调用实际 IdentityManager 和 PrimaryTargetManager 验证了这个前提：使用默认配置和
可控外观特征，A 缺失 2000 帧（选人时钟推进约 68 秒），换 raw track 返回后恢复原 semantic
ID；基础 handled 的 60 秒已经到期，调用跨日路径的 ClearReidDedupExclusions 后，A 确实
重新具备资格并成为 primary。该测试 1/1 通过，证明没有把“次日一定分配新 semantic ID”当成
隐含保证。可控特征用于验证状态流转，仍不代表真实录像的识别精度。

**影响：** 今天的 ReID 身份记录可能一直是 handled=false。以后同人换 raw/semantic track
重新出现时，外观库虽然认出见过，却不能延续“今天已经处理完成”的结论，造成当天重复核验。
这与刚确认的当日免重复处置目标直接相关，不是要求回到旧 60 秒规则。

**测试证据：** 本轮直接包含 PR 的生产 perception_tracking_node.cpp，使用其中真实的
MissionTrackingRuntime，创建实际 MissionRosAdapter/ReidConfirmGate，并在隔离 ROS 域订阅
handled 通道。没有复制该判断逻辑来模拟，也没有启动相机或驱动机器人。

| 输入给真实 runtime 的快照 | 期望 handled 消息累计数 | 实际累计数 |
| --- | ---: | ---: |
| 第一天 seq=100，RECOVER，target=7，handled=7 | 1 | 1 |
| 第二天 seq=101，VERIFY，target=7，handled=0 | 1 | 1 |
| 第二天 seq=102，RECOVER，target=7，handled=7 | 2 | 1 |
| 对照：随后 ID=8 形成新的处置结果 | 3 | 2，ID=8 能正常发送 |

对照证明同一通道可以正常发送另一 ID 的结果，失败点是同 ID 的新处置结果被旧去重值吞掉。
本测试只验证消息转发边界，不把它夸大为完整相机识别或真实人员精度测试。

**本次对复现测试本身的补强：** 为避免只用手工快照遗漏真实转换条件，新增
`generate_runtime_scenarios.py`，直接运行 merge/PR 共同使用且未被 PR 改动的
MissionStateMachine。每个命令和事件都断言 accepted，生成 STARTUP、双 READY、显式 START、
CONFIRM、APPROACH、VERIFY、AUTHORIZED→RECOVER、恢复 PATROL 的完整序列，再交给真实
MissionTrackingRuntime 执行。第二天重复完整同 ID 流程仍复现漏发。

这组 `runtime_lifecycle_review` 共 3 项：同 ID 次日完成失败；相同 heartbeat 与跨天旧快照
不重复下发通过；不同 ID 和 STARTUP reset 后再次处理通过。期望由 supervisor 的实际
AUTHORIZED 结果产生，不依赖日志文字或复制 `last_forwarded_handled_` 判断。没有启动物理
相机，空人员观测仅用于隔离 handled 转发边界，因此仍不能称为整机闭环识别测试。

**修复要求：**

- 识别“新的权威处置结果”，再为结果去重，不能只用 semantic ID 是否变化判断。
- 至少覆盖新日期中同 ID 从未处置阶段到处置完成的过程；结合 state_seq、phase 和本地已观察
  上下文区分新结果与周期重发，保持相同 heartbeat 不重复下发。
- 不能简单跨天清零变量后无条件重发仍保留的旧 handled 快照，否则可能把昨天的处置结果
  重新标成今天已经处理。需要区分旧快照延续和今天的新完成结果。
- 不要求增加全系统 task_generation、command_id 等协议，优先在现有权威状态和本地转发
  上下文中完成。

**验收：** 同 ID 次日重新完成能再次转发；同序号 heartbeat 不重复转发；跨天仍保留的旧快照
不重新授予当日已处理资格；不同 ID 正常转发；reset 后再次处理同 ID 正常转发。还应检查在
keep-last 接收跳过部分中间快照时，处理结论是否仍能正确识别。

### 新增测试合理性审查

**总体判断：新增测试方向合理，不能把“12 项全绿”当成全部问题已经处理正确。**

| 新增测试 | 合理之处 | 目前能够证明和不能证明的范围 |
| --- | --- | --- |
| `test_reid_dedup_exclusion.cpp`，7 项 | 通过选人公共接口验证 primary 让位、全员排除、不扰动其他 primary、活动目标绑定、当日排除、局部清理和 STARTUP reset | 有效覆盖 N01 和选人层的 N03/N04；直接调用 Clear API，不能证明生产 runtime 必定正确调用，也没有验证 supervisor→handled→provider 整条反馈 |
| `test_reid_confirm_gate.cpp`，reset 用例 | 使用真实 gate 和 ROS mock service；旧请求确已到达后再 reset，新请求能产生排除 | 能检测旧响应在 reset 后重建排除；固定 sleep 的完成同步仍可改进 |
| 同文件，跨天两个用例 | 不调用 Allow 就调用 SyncDay，以及在途旧响应跨天作废，符合“全员排除时也必须换日”的场景 | 能验证 gate 自身清理；不能代替生产选人和 handled 转发的跨日联动测试 |
| 同文件，disabled 用例 | 显式关闭 gate 恒放行，符合旁路语义 | 仅是 gate API；原有任务/认证不回归还须保留原 adapter 等测试 |
| 同文件，超时/替代请求用例 | Q1 超时、建立 Q2 后丢弃 Q1 suppress 的负向断言有价值 | Q2 的成功响应没有被可靠验收，见 T01；mock 串行回调不能构造 Q2 先返回、Q1 后返回 |

把“61 秒后重新可选”改为“当日仍排除”的新增断言，在用户已经确认的新规则下合理。这里没有
把需求变更误写成修复旧规则，也没有要求作者为了通过历史测试而把当日去重改回基础版本。

### T01 — [P2，测试缺口] 超时用例不能证明 Q2 响应生效，删掉 Q2 响应仍通过

**已用对照测试确认。评论位置：**
[test_reid_confirm_gate.cpp:217](https://github.com/HowAreAllWell/dog_patrol/blob/1aea964cd770aec0a65efd5ac22656c2168ccc7b/src/perception/dog_patrol_perception_tracking/test/test_reid_confirm_gate.cpp#L217)。

`TimedOutQueryRejectsLateAndReplacedResponses` 把 timeout 设为 100ms；发出 Q2 后，先等待
150ms 才给 Q2 新人响应。随后 `UntilAllowed` 只检查 Allow 返回 true，没有区分“消费 Q2
成功响应”与“Q2 也超时后 fail-open”。因此最后两条断言可以在没有 Q2 响应的情况下成立。

本次把原测试复制到独立临时文件，**只删除** `mock_->Respond(1, false, false)`，不改 gate、
timeout、sleep 或断言，然后单独编译运行该用例：仍然 1/1 通过。日志出现两条 timeout，分别
对应 Q1 和 Q2；没有 Q2 正常响应的放行日志。证据为 `mutation_no_q2.xml` 及临时复制源码。

这不表示原用例的所有断言都无效：Q1 的迟到 suppress 没有进入排除队列仍是有效检查。
问题是不能据此声称“新请求响应处理正确、所有替代/乱序场景均已覆盖”。尤其测试 mock 使用
SingleThreadedExecutor 和默认互斥 service 回调，Q1 未释放时 Q2 无法进入回调，不具备真正
控制 Q2 先返回的能力。

**建议修改：**

1. 拆开“超时 fail-open”“旧请求失效”“新请求成功处理”三种结果，避免共用一个 true 断言。
2. 对 Q2 的成功处理，在它的剩余超时预算以内等待并断言；删除 Q2 响应时这一成功断言必须
   失败，不能靠第二次超时放行满足。
3. 增加可并发/可延迟发送响应的 mock，分别覆盖：Q2 allow 后 Q1 suppress 迟到；Q2 suppress
   后 Q1 allow 迟到。后者应正向观察 Q2 的排除事件，再确认 Q1 不能解除该结论。
4. `AwaitRequest` 使用有界等待并明确断言请求到达。当前无限 wait 只会等外层 CTest 杀进程，
   请求丢失时定位困难；固定 sleep 的正向断言改为有界条件等待，减少调度负载引起的误判。
5. 保留并补入 N06 的生产 runtime 跨日反馈回归；仅测试独立 gate/PrimaryTargetManager
   无法发现生产转发变量遗漏日期/新处置上下文。

**独立复核结果：** 本次新写的 `async_gate_review` 使用 reentrant service 和多线程 mock，
实际让 Q2 先返回、Q1 后返回，上述两种相反结论的顺序均通过。另有 ERROR 即使携带
seen+handled 也不得抑制、服务缺失回退，以及四种活动目标阶段不因排除换人，共 5/5 通过。
所以 N02 的实现修复可以关闭，T01 仍应作为 PR 内测试需要补强的意见，二者不能混为一谈。

还对补强测试做了相同的删响应对照：删除独立测试的 Q2 响应后，它在 Q2 超时前的成功断言
明确失败，结果见 `async_mutation_no_q2.xml`。原测试删响应仍通过、补强测试删响应会失败，
两者的区别是确实验收了新响应，不能把这项故意制造的失败记为 PR 新增业务缺陷。

### 本轮复现材料、命令和限制

临时目录 `/tmp/dog_patrol_pr50_20261011.jwZjJ1` 保留：

- `build_console.log`、`log/`：两包真实 ON 构建结果。
- `test_console.log`、`test-log/`、`build/*/test_results`：正式测试证据。
- `gate_review.xml`：按当前规则重跑上一轮独立探针，7/7 通过。
- `visualizer_workspace_cwd.xml`：既有环境失败的工作区重跑结果。
- `review/runtime_review.cpp`、`runtime_review.xml`：N06 直接调用生产 runtime 的失败复现。
- `review/CMakeLists.txt`：探针构建。生产节点 main 在测试编译时改名，未实例化物理相机节点，
  未修改 PR 源文件。
- `full_audit_build.log`、`full_audit_ctest_verified.log`、`full-audit-test-log-verified/`：本次明确
  指向 PR 两包重新构建/测试的记录；`full_audit_test_result.log` 保留原始汇总及环境失败。
- `python_full_audit.log`、`python_full_audit.xml`：m20_nav 运行 369 项既有 Python 回归的结果。
- `gate_full_audit.xml`、`async_gate_review.xml`、`persistence_review.xml`：7 + 5 + 5 项独立检查。
- `review/gate_test_no_q2_response.cpp`、`mutation_no_q2.xml`：只删 Q2 响应后仍通过的对照。
- `review/async_gate_no_q2_response.cpp`、`async_mutation_no_q2.xml`：补强测试删响应后按预期失败。
- `review/generate_runtime_scenarios.py`、`review/runtime_scenarios.hpp`、
  `review/runtime_lifecycle_review.cpp`、`runtime_lifecycle_review.xml`：合法完整序列复现及对照。
- `review/identity_reentry_review.cpp`、`identity_reentry_review.xml`：同 semantic ID 长缺失后
  恢复并重新可选的前提验证。
- `review/baseline_absence_review.cpp`、`baseline_absence_merge.xml`、`baseline_absence_pr.xml`：
  B01 在 merge 与当前 PR 的同源对照；两版均复现，不计为 PR 新增回归。
- `visualizer_full_audit_workspace.xml`：同一新编译 binary 从真实工作区重跑，12/12 通过。
- `model_full_audit_feature.f32`：本次正式 TensorRT 引擎实际推理输出，1280 维 float32。

```bash
source /opt/ros/humble/setup.bash
source /mnt/nvme/workspace/dog_patrol/install/local_setup.bash
source /tmp/dog_patrol_pr50_20261011.jwZjJ1/install/local_setup.bash

/usr/bin/python3 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review/generate_runtime_scenarios.py
cmake -S /tmp/dog_patrol_pr50_20261011.jwZjJ1/review \
      -B /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build
cmake --build /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build \
      --target gate_review runtime_review runtime_lifecycle_review async_gate_review \
               persistence_review identity_reentry_review gate_test_no_q2_response \
               async_gate_no_q2_response baseline_absence_merge baseline_absence_pr -j2

export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dog_patrol_pr50_20261011.jwZjJ1/review-ros-logs
ROS_DOMAIN_ID=187 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/gate_review
# 以下两项预期通过：独立异步/活动目标检查及持久化检查。
ROS_DOMAIN_ID=201 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/async_gate_review
/tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/persistence_review
/tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/identity_reentry_review
# N06 的旧边界探针和完整合法序列探针均预期复现漏发失败。
ROS_DOMAIN_ID=192 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/runtime_review
ROS_DOMAIN_ID=200 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/runtime_lifecycle_review
# 原测试删 Q2 响应后仍通过；这是 T01 的复现。
ROS_DOMAIN_ID=197 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/gate_test_no_q2_response \
  --gtest_filter=ReidConfirmGateTest.TimedOutQueryRejectsLateAndReplacedResponses
# 补强测试删 Q2 响应后应失败；这是有意设置的测试有效性对照。
ROS_DOMAIN_ID=204 /tmp/dog_patrol_pr50_20261011.jwZjJ1/review-build/async_gate_no_q2_response \
  --gtest_filter=AsyncGateReview.NewResponseAllowsBeforeItsDeadlineEvenWhileOldResponseIsHeld
```

本轮没有变更 PR 业务代码、没有切换 merge 分支或合并提交。模型提特征实现与上一轮一致，
TensorRT 引擎没有重复转换，已再次加载并运行。仍未对带身份标签的真实现场数据做精度标定，也未启动完整巡检
运动验收；这些限制不被表述为已经完成的测试。

## 历史复审：2026-10-10，e08b320

下面的 N01～N04 是上一版本的意见。按本文最新状态表均已关闭，尤其 N04 不再作为当前待修项。

**当时建议：Request changes（针对 e08b320）。** 该版本已完成生产配置构建，原有 READY、暂停恢复、heartbeat 和
同序号冲突逻辑已恢复；这些旧问题不再作为当前 head 的未修复意见。本轮仍复现了新去重链路的
四项行为问题（N01～N04）。模型已经完成 TensorRT FP16 转换并通过实际加载和推理验证，
原 N05 按已确定的 TensorRT 部署方式关闭，不再列为 PR 问题。本节只记录该版本的历史结论；
当前待办与验收条件以文首 1aea964 的最新复审为准。后面的 2026-10-09 内容保留为旧版证据。

### 本轮范围

| 项目 | 内容 |
| --- | --- |
| 当前 head | `e08b320580653de0bf11a99c94a8a04faebb0936` |
| 当前提交 | `perception: ReID 端侧去重节点 + confirm_target 前去重闸门（评审 R01-R18 返工）` |
| 基准 merge | `47f7d487925911093ddab262b69ffb675d6a9124`，未变化 |
| PR 历史变化 | 作者重新整理了提交历史；当前 head 相对 merge 为一个新增提交 |
| 本轮本地审查引用 | `origin/pr-50-review-20261010`；旧 `origin/pr-50` 保留用于历史对照 |
| 模型 | `/home/orin/reid_clipreid_vitb16_after.onnx`，344753233 字节 |
| 模型 SHA-256 | `905c3072d73dfd4d8cf92884c9a90af83ca7ac4d5fab9ddcc14369d07c3ec98f` |
| 验证目录 | `/tmp/dog_patrol_pr50_latest.5ddvsI`，源码由 git archive 提取，未修改 PR 业务代码 |

本轮继续遵守“增强 ReID，保留 merge 既有任务和认证行为”的约束。未合并 PR、未覆盖生产
install，原始 ONNX 文件保持不变。验证通过的正式引擎已保存为
`/home/orin/reid_clipreid_vitb16_after.fp16.engine`，转换记录位于同名 `.engine.json` 文件；
构建日志和其它测试材料保留在临时验证目录。

### 构建与模型验证结果

**构建：** 系统 `/usr/bin/colcon`，`REID_BUILD_TENSORRT=ON`、
`TRACKING_ENABLE_ORIN_RUNTIME=ON`、`BUILD_TESTING=ON`，provider 和 tracking 两包均成功构建。
旧版的 srv 分隔符、package group、Humble QoS、引用成员访问、TRT 编译和外部 typesupport
target 错误均未再次出现。日志中的 OpenCV/cv_bridge 版本警告记录在案，不把警告本身认定为 PR
新增的业务错误。

**模型：**

| 检查 | 实际结果 |
| --- | --- |
| ONNX checker / 图接口 | 通过；opset 17；输入 `image_rgb_f32: [1,3,256,128] float32`；输出 `feature: [1,1280] float32`；无外部权重文件 |
| m20_nav 的 ONNX Runtime 1.23.2 CPU | 零输入推理成功，输出有限值，L2 norm 约 1 |
| PR 实际 C++ OpenCV 4.8.0 CPU 后端 | 原 ONNX 导入失败；保留为未采用后端的兼容性记录，不作为当前 TensorRT 部署的阻断项 |
| TensorRT 10.3 FP16 引擎构建 | 在本机 Orin GPU 上成功，测试引擎约 168 MiB |
| PR 原始 C++ TensorRT FeatureExtractor | 加载和推理成功；1280 维、全部有限、L2 norm=1；该次暖态提特征约 9.4ms |
| TRT 与 ORT 的相同零输入输出对比 | 余弦约 `0.99999124`，最大元素绝对误差约 `0.00062738` |
| 正式 TensorRT FP16 引擎，优化级别 3 | 已保存到 home；175199364 字节，约 168 MiB；实际 GPU 加载和推理通过，1280 维、全部有限、L2 norm=1 |
| 正式引擎与 ORT 的相同零输入输出对比 | 余弦约 `0.99998617`，最大元素绝对误差约 `0.00048701` |

首次复核兼容性使用 `builderOptimizationLevel=1` 构建临时 FP16 引擎，随后按用户要求使用
优化级别 3 生成并验证正式引擎。一次零输入推理的耗时不能当作长期性能基准；零输入对比也不能
证明现场同人/跨人识别准确率或 0.70 阈值已经标定通过。

**真实 provider service：** 在隔离 `ROS_DOMAIN_ID=189` 运行 PR 原始 reid_node 和 reid_query，
使用测试引擎和同一张合成灰度 crop 验证数据链路：

| 步骤 | is_seen | handled | 结果 |
| --- | --- | --- | --- |
| 首次 query `review-a` | false | false | 正常建库 |
| 再 query `review-a` | true | false | 见过未处置不授予免处置结论 |
| 发布 handled `review-a`，换 `review-b` 查询同一 crop | true | true | 外观命中延续 handled |
| 发布 clear，再换 `review-c` 查询 | true | false | 已处理标记清除，外观事实保留 |

该测试证明真实模型接口、库命中和 handled/clear 通道基本可工作，合成图片不代表真实人员
识别精度。测试 provider 已发送 SIGINT 正常停止，没有重启用户节点。

### 旧意见逐项关闭状态

| 原意见 | 本轮状态 | 依据 / 剩余问题 |
| --- | --- | --- |
| R10～R15 构建错误 | 已修复 | 两包 ON 生产配置实际构建成功 |
| R01 READY | 已修复 | 原版 adapter 测试通过，映射和清理恢复 |
| R02 暂停恢复 target_id=0 | 已修复 | 原版合法组合测试通过 |
| R03 heartbeat | 已修复 | state_timeout/API 和既有超时测试恢复 |
| R08/R09 冲突和零序号 | 已修复 | 入口校验恢复，原版 adapter 19/19 通过 |
| R04 多人候选被挡住 | 未修完整 | 增加了排除表，但已锁定 primary 不检查该表，N01 已复现 |
| R05 锁内同步等待 | 阻塞部分已修复；异步失效未完成 | 查询不阻塞状态接收，但 reset 后旧响应仍写入排除，见 N02 |
| R06 见过不等于处置完成 | 首次/未处置分支已修复；有效期仍冲突 | service 和 gate 加 handled 判定，基本链路通过；60 秒释放仍被永久缓存/标记覆盖，见 N04 |
| R07 跨天失效 | 未修完整 | Allow 增加日期判断，但已注入选人层的排除未同步清理，见 N03 |
| R16 先删旧库再替换 | Linux 上原子替换窗口已修复 | POSIX rename 直接覆盖并有 LoadLatest；实际断电持久性承诺仍未做硬件验证 |
| R17 重启后时间基线 | 已修复原问题 | 加载时重建时间锚点，新增库测试通过 |
| R18 测试注册和真实 gate 覆盖 | 注册已修复；gate 覆盖仍不足 | provider 2 个 CTest 被执行；仓库未新增相应 gate/候选/取消测试，本轮额外探针 7 项中 4 项失败 |
| N05 模型部署路径 | 已关闭 | 已转换正式 TensorRT FP16 引擎并验证 GPU 推理；按下方参数部署，不再要求将 OpenCV ONNX 兼容性作为本 PR 修复项 |

### N01 — [P1，R04 未修完整] 排除表没有释放已锁定的 PATROL primary

**已复现。评论位置：**
[primary_target_manager.cpp:86](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_perception_tracking/src/modules/primary_target_manager.cpp#L86)。

`ExcludeForReidDedup()` 只向表中 try_emplace。当前 primary 的优先保持分支仍在
UpdateInternal 第 383 行直接接受可见原目标，没有按 PATROL 候选资格检查该 ID 是否已经被排除。

**具体业务场景：** A 是已经处理过的人，B 是尚未处理的新人，两人同时出现在画面中。

1. 巡检阶段先把面积较大的 A 选为 primary，并查询 ReID。
2. ReID 返回 A 已见过且已处置，下一帧 DrainExclusions 把 A 加入选人排除表。
3. A/B 都仍可见，supervisor 的权威状态仍为同一 PATROL 快照。
4. 当前 primary 保持逻辑继续使用 A，没有重新检查 A 已失去选人资格。
5. gate 反复抑制 A 的 TARGET_CONFIRMED，B 一直没有机会被选择和处置。

**正确行为：** 在 PATROL 中，A 被有效排除后应让出主目标，由 B 或其他有效候选接替。
如果所有人都已排除，应进入无有效 primary 的等待状态。

**本轮探针：** A 框 80×80、B 框 60×60；先选 A，再调用 ExcludeForReidDedup(A)。
IsMissionEligible(A) 已返回 false，但下一帧 primary 仍为 A，期望 B，测试失败。

**影响：** 此项仍会使已处置人员持续挡住同屏新人。当前新增表并没有完成 notes 中声称的
“被去重者不再占用 primary”。

**修复要求：** 在当前权威状态允许的 PATROL 候选重评边界释放已经失去资格的 primary，并选择
其他有效候选。不要无条件在 Exclude API 中清 primary：迟到排除可能发生在活动目标阶段，必须
继续保留 supervisor.target_id 的唯一绑定。

具体要区分两条路径：

- **巡逻选人：** 重新检查当前 primary 的 IsMissionEligible，失去资格后清理该 primary 的选择
  上下文并重选，不能只向表里插入一个 ID。
- **接近/核验/跟踪：** 继续只绑定 supervisor.target_id，不能为了修复巡逻重选而让另一位
  更大的人抢走已经进行中的目标任务。

**验收：** 同屏 A 已处理/B 新人、全员排除、不同 primary 被排除、活动 APPROACH/VERIFY/TRACK
阶段不被其他人抢占均覆盖。本轮“活动 target 不因排除换人”的对照探针通过，应继续保留。

### N02 — [P1，异步结果失效] reset 清缓存后，旧响应能重新建立排除

**已复现。评论位置：**
[reid_confirm_gate.cpp:79](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L79)。

OnResponse 只携带 target_id，通过 `entries[target_id]` 无条件创建/覆盖条目，没有绑定本次
请求代次、日期或 reset 边界，也不要求条目仍是同一份 pending。NotifyHandledReset 清空表并没有
使在途回调失效。

**具体业务场景和复现顺序：**

1. 旧业务上下文对 A 发起查询 Q1，响应尚未到达。
2. 用户执行 reset，NotifyHandledReset 已清空本地 entries 和 exclusions，并下发 handled 清除。
3. 新的业务上下文开始使用已经清空的缓存。
4. Q1 的旧响应随后到达，携带 seen=true、handled=true。
5. OnResponse 通过 entries[A] 重新创建条目，并把 A 再次写进 exclusions。
6. 下一次 DrainExclusions 将旧结论注入新的选人上下文。

本轮测试在第 5 步之后期望排除队列为空，实际再次取到了 A。**正确行为是 Q1 已被 reset
作废，旧响应应直接丢弃，不能重建缓存或改变新上下文的人员资格。**

**影响：** reset 后新的业务上下文可能再次被旧结果排除。容量清表、跨日清表、超时后重新发起
同 ID 查询，也存在同类旧结果覆盖新结论的路径。

“回调不直接发布业务事件”不能证明结果已经失效，因为回调写入的资格结论会影响后续新帧的选人。

**修复要求：** 给查询绑定本地 request generation/取消边界；reset、日期变化和被替代请求使旧
generation 无效。回调在修改条目/队列之前验证它仍属于当前 pending，并检查当前业务关联；
失效结果不得用 operator[] 重新创建条目。这不需要增加全系统 runtime_epoch 或 task_generation。

有效性检查必须发生在修改 entries/exclusions 之前，至少确认：请求属于当前取消代次、对应
条目仍存在、条目仍等待同一份请求、日期和业务关联没有失效。已超时并被新请求替代的 Q1，
不能晚于 Q2 返回后再覆盖 Q2 的结论。只清表或只判断 target_id 相同都不足以解决这个问题。

**验收：** reset 后迟到响应、旧请求晚于新请求返回、超时后重问、跨午夜返回，以及 PAUSE/RETURN/
TERMINATE 后旧资格结果的处理均测试。异步查询不阻塞状态接收的测试本轮已经通过，要同时保持。

### N03 — [P2，R07 未修完整] 跨天失效没有覆盖已经注入的选人排除表

**已复现。评论位置：**
[reid_confirm_gate.cpp:140](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L140)；关联
[perception_tracking_node.cpp:140](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_perception_tracking/src/perception_tracking_node.cpp#L140)。

日期检查仅在 Allow() 内进行，只清 entries，不清 PrimaryTargetManager 的 reid_dedup 表。
当可见人员都已被该表排除时，没有 primary、没有 TARGET_CONFIRMED，自然不会调用 Allow，
连 gate 内部的日期检查都不会执行。

**具体业务场景：**

1. 前一天，画面中的人员已被 ReID 判定为 seen+handled，排除结论已经注入选人层。
2. 这些人持续可见，所以不会触发“持续不可见到期释放”。
3. 到第二天，provider 的当日身份库已经清空，但选人层仍保留昨天的排除项。
4. 所有人都被排除，没有 primary，也没有 TARGET_CONFIRMED。
5. 没有确认事件就不会调用 Allow，放在 Allow 内的日期检查也无法执行。

**正确行为：** 日期切换应独立推进，使昨天的 ReID 排除结论失效；不能要求先出现一名可选人员，
才能解除“所有人员都不可选”的状态。

本轮复现用 A 持续可见，将可注入日期从 10 日推进到 11 日，再按生产每帧 Drain→选人的顺序
处理。实际 A 仍不可选、primary=-1。测试推进了日期源，没有实际等待到午夜。

即使还有其他人触发 Allow，gate 也没有通道清理已经注入的旧 reid_dedup 表。日期变化时未清
exclusions 队列和未失效的在途回调也需要一并处理。

**修复要求：** 在每帧/周期边界独立处理日期变化，不依赖存在确认候选。同步清理 ReID 缓存、
旧排除队列及选人层的 ReID 排除项，失效旧查询。原业务 handled 表仍按原有业务规则维护，
不能为了清 ReID 当日记录而误清其他权威业务历史。

日期失效要覆盖四处状态：gate.entries、待 Drain 的 exclusions、PrimaryTargetManager 的
ReID 排除表，以及上一日期发起的在途查询。只清 entries 会留下已经生效的选人排除；只清
选人表则可能被旧队列或旧回调立即重新排除。

**验收：** 全员排除跨午夜、部分人排除跨午夜、待 Drain 的旧结果跨午夜、在途查询跨午夜，
在保持 semantic_id 不变且节点不重启时验证。

### N04 — [P1，R06 有效期仍冲突] 60 秒资格释放后 gate/provider 仍保留永久 handled

**已复现 gate 层，provider 标记生命周期已代码确认。评论位置：**
[reid_confirm_gate.cpp:152](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L152)；
[reid_gallery.cpp:273](https://github.com/HowAreAllWell/dog_patrol/blob/e08b320580653de0bf11a99c94a8a04faebb0936/src/perception/dog_patrol_reid_provider/src/reid_gallery.cpp#L273)。

选人表按持续不可见 60 秒删除排除项，但 gate 的 kSuppress 没有同类释放机制，始终返回 false。
这次返回也不会重新投递排除队列。provider 的 handled=true 则保留到 clear_all/跨天，持久化也保存
这个 bool，没有单个人员处置资格过期的反馈。

**具体业务场景和复现顺序：**

1. A 已完成处置，选人层、gate 和 provider 都保存了相关排除/handled 结论。
2. A 持续离开画面超过默认 60 秒，本轮探针使用 61 秒。
3. 选人层按旧规则删除 A 的排除项，IsMissionEligible(A) 确认重新具备资格。
4. A/B 再次可见，选人层可以重新选中 A。
5. gate.Allow(A) 仍因旧 kSuppress 返回 false，A 不能重新进入处置流程。
6. 即使只清 gate 缓存，重新查询 provider 仍会拿到长期保存的 handled=true，再次被排除。

**正确行为：** 按原业务规则到期后，A 应重新具备确认和处置资格。外观库仍然可以知道“今天
见过 A”，但这不能自动延长 A 的免处理资格。实际配置 handled_ignore_absence 不为 60 秒时，
各层也应遵守同一份配置和有效期规则，不能各自使用不同的固定时限。

**影响：** A 不能按原规则重新处置；在 N01 修复后，这条路径仍可能让 A 再次占 primary，
因为缓存抑制不再次产生排除通知。单纯清 gate 缓存也不够：重新查询 provider 会再次拿到永久
handled=true，从而重新排除 A。

**修复要求：** 明确一个完整的处置资格有效期，并让选人层、gate、provider 的授予/释放一致。
本轮用户要求保留旧业务，不能把“持续不可见 60 秒释放”悄悄改成“当天处理一次就永久免处置”。
外观特征库可以继续保留，外观见过的事实不必与业务资格同时删除。

需要把单个人员资格的到期/撤销传递到所有会再次阻止确认的状态中。只改 primary 表、只给 gate
加缓存超时，或重新查询仍永久 handled 的 provider，都不能完成释放。也不能用 clear_all
清掉所有人的处理记录来代替单个人员到期，否则会破坏其他仍应排除的人员资格。

**验收：** 原 semantic_id 在 60 秒后再出现、新 semantic_id 识别为同人、短时间离开后回来、
持续可见及 reset/重启各有明确期望，并覆盖资格释放后仍能选择同屏新人。

### 当前剩余意见的修复顺序与验收

优先处理 **N01、N02**：先保证新人不会被其他人挡住，且旧异步结果不能污染新业务上下文。
随后将 **N03、N04** 作为同一组有效期一致性问题处理，确保日期清理、持续不可见释放和查询
缓存失效不会互相覆盖或重新建立过期排除。

| 意见 | 修复后必须直接观察到的结果 |
| --- | --- |
| N01 | A 被有效去重且仍可见时，B 新人能够成为 primary 并发布确认；活动目标阶段仍不换人 |
| N02 | reset 后放回旧响应，entries/exclusions 和后续候选资格均不发生旧结果引起的变化 |
| N03 | 所有可见人员都被 ReID 排除时，日期推进到次日也能独立清理旧排除，不依赖 Allow 被调用 |
| N04 | 单个人员达到配置的不可见释放时限后，各层均允许其按旧规则重新确认；其他未到期人员仍保持排除 |

四项需要补入仓库可自动执行的 gate/集成测试，不能只在说明中宣称已满足。本轮临时探针已经
复现四项失败，可据此建立修复前失败、修复后通过的回归。**N05 已关闭**，不属于这组待修改项；
**B01 属于旧 merge 的既有计时问题**，仍在文末单列，不混入 PR 新增问题。

### 模型部署说明：TensorRT 转换和推理验证已完成

原 N05 已关闭，不再作为 PR 评审问题。当前确定使用本机转换并验证过的 TensorRT 后端：

```yaml
device: tensorrt
model_path: /home/orin/reid_clipreid_vitb16_after.fp16.engine
```

引擎由提供的 ONNX 使用 TensorRT 10.3.0.30、FP16、优化级别 3 构建，输入/输出保持 FP32。
实际调用 PR 的 TensorRT FeatureExtractor 验证了加载、1280 维输出、有限值和 L2 归一化。

转换记录：`/home/orin/reid_clipreid_vitb16_after.fp16.engine.json`。
引擎 SHA-256：`0d7a307f7a5dea7e97380e3ea9e648695229bb22e828daff3a6f2e7317ef9d9f`。

OpenCV 对原 ONNX 的导入失败仅保留在前面的兼容性验证记录中；本次按上述 TensorRT 参数部署，
不再把该兼容性差异列为 P1 或要求作者修复。本项关闭不改变 N01～N04 的业务行为复现结果。

### 本轮完整测试与限制

| 验证 | 结果 |
| --- | --- |
| 两包 ON 生产配置 build | 2 packages finished |
| provider 正式 CTest | 2/2 通过，库 164/164 断言、仿真 34/34 断言 |
| tracking 原版 MissionRosAdapterTest | 19/19 通过，无改测试绕过 |
| tracking 7 组相关 core CTest | 7/7 通过 |
| tracking 整包正式 CTest 初跑 | 53/55 测试目标通过；visualizer 和 target image smoke 初跑失败 |
| visualizer 单独从工作区重跑 | 12/12 通过；初跑因 /tmp 构建 cwd 下，默认相对数据根与测试使用的 /tmp 输出根重叠；相关源码未改 |
| target image ROS smoke 单独重跑 | 通过；初跑失败原因未完全归结，不能声称原整包 colcon 已全部绿灯 |
| 新补真实 gate/primary 探针 | 7 项：3 通过、4 失败，对应 N01～N04 |
| 真实 TensorRT provider + C++ query/handled/clear | 通过基本数据链路，临时节点已停止 |

本轮新探针使用实际 PR 类和 ROS mock service，补充源在临时目录 `review/gate_review.cpp`。
通过的三项分别是活动 target 不被排除改选、查询不阻塞状态接收、见过但未处置可以放行。
这些通过项也说明本次不是简单沿用旧评审的否定结论。

其它仍需明确的限制：

- 当前 provider 的 Python ROS 接口没有生成，实际 `from dog_patrol_reid_provider.srv import
  ReIDDedup` 失败；本轮使用 C++ 客户端完成联调。package.xml 声明生成器不等同于 CMake 已
  显式加载 rosidl_default_generators，标准 Python/ROS CLI 的可用性需要补充验证。
- 未使用带身份标签的真实人体数据集验证准确率、跨人误命中率及 0.70 阈值；模型单次输出正常
  不等于现场识别效果提升已经被证明。
- 未启动完整巡检/导航或驱动机器人运动。B01 是 merge 原有问题，最新代码没有专门修复它，
  仍不计为本次 PR 新增缺陷。
- 旧 ReID/MOT 特征关联链仍未接入新的 CLIP embedding；当前增强重点依旧是确认前的当日查询。
- Linux rename 原子覆盖已修复旧窗口；没有做真实拔电和文件/目录持久性验证，不把库单测通过
  扩大为断电承诺已经验收。

### 本轮复现材料与命令

临时目录 `/tmp/dog_patrol_pr50_latest.5ddvsI` 中保留：

- `log/`、`test-log/`：未经修改的 PR 构建、正式测试日志。
- `gate_review.xml`、`review/gate_review.cpp`：7 项 gate/选人复现及结果。
- `visualizer_workspace_cwd.xml`：从工作区单独重跑结果。
- `review/model_review.cpp`：调用 PR 原始 FeatureExtractor 的模型探针。
- `trtexec_build.log`、`reid_review_fp16.engine`：测试 GPU 转换产物。
- `model_zero_reference.npy`、`cpp_trt_feature.bin`：数值接口比较材料。

```bash
# 生产配置编译（输出仅在 /tmp）
source /opt/ros/humble/setup.bash
source /mnt/nvme/workspace/dog_patrol/install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/colcon --log-base /tmp/dog_patrol_pr50_latest.5ddvsI/log build \
  --base-paths /tmp/dog_patrol_pr50_latest.5ddvsI/src/perception/dog_patrol_reid_provider \
               /tmp/dog_patrol_pr50_latest.5ddvsI/src/perception/dog_patrol_perception_tracking \
  --build-base /tmp/dog_patrol_pr50_latest.5ddvsI/build \
  --install-base /tmp/dog_patrol_pr50_latest.5ddvsI/install --symlink-install \
  --cmake-args -DREID_BUILD_TENSORRT=ON -DTRACKING_ENABLE_ORIN_RUNTIME=ON -DBUILD_TESTING=ON

# 新的 gate/选人复现；需要本机 DDS 通信权限
source /tmp/dog_patrol_pr50_latest.5ddvsI/install/local_setup.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dog_patrol_pr50_latest.5ddvsI/ros-logs
cmake -S /tmp/dog_patrol_pr50_latest.5ddvsI/review \
      -B /tmp/dog_patrol_pr50_latest.5ddvsI/review-build
cmake --build /tmp/dog_patrol_pr50_latest.5ddvsI/review-build -j3
/tmp/dog_patrol_pr50_latest.5ddvsI/review-build/gate_review

# 已转换并验证的正式 TensorRT 引擎；GPU 操作需相应权限
/tmp/dog_patrol_pr50_latest.5ddvsI/review-build/model_review tensorrt \
  /home/orin/reid_clipreid_vitb16_after.fp16.engine \
  /tmp/dog_patrol_pr50_latest.5ddvsI/cpp_trt_feature.bin
```

## 历史评审记录：2026-10-09，2214cd1

下面的 R01～R18、测试输出和旧路径属于上一版 head 的历史记录；其当前关闭状态以本文首部表格
为准。已经关闭的旧错误不再对新版重复提出。

评审日期：2026-10-09。建议评审结论：**Request changes**。

本文件可作为 PR 总评及逐行评论使用。代码链接固定到本次审查的提交，避免后续修改导致行号含义变化。

第二轮复核约束：用户明确要求本 PR 增强 ReID 效果，并保留 merge 已有业务实现。因此下面以
“模型/外观能力可以增强，任务生命周期、处置资格和原有身份跟踪约束必须保留”为集成条件。
本轮还直接检查了 merge 的生产调用链，并重跑其既有测试；第九部分单列发现的 merge 自身问题，
不把它算到 PR 作者的新增缺陷中。

## 一、评审范围和依据

| 项目 | 内容 |
| --- | --- |
| PR | #50 |
| 比较基准 | 当前 `merge`：`47f7d487925911093ddab262b69ffb675d6a9124` |
| PR head | `2214cd1c2e2f1f0dfec41401c3aa99ab6810562b` |
| 提交一 | `27662449d7dc6029f8dc3bdda5cc6a6bac4a7f97`，`perception: add reid dedup node + confirm_target gate` |
| 提交二 | `2214cd1c2e2f1f0dfec41401c3aa99ab6810562b`，`reid: 当日库磁盘持久化（重启恢复，跨天作废）` |
| 权威业务契约 | [patrol_task_lifecycle_design.md](../contracts/patrol_task_lifecycle_design.md) |
| 比较方式 | `git diff merge...origin/pr-50`；merge-base 恰为当前 merge，新增提交数为 2 |

PR 提交目标为 main 的情况由本次请求提供。本次只评审相对于 merge 的这两个新增提交，不把 merge 已有的巡检生命周期、MobileFaceNet 等历史改动当作本 PR 新增内容。

优先级和证据标记：

- **P1**：构建阻断或正常巡检流程的功能回归，合并前需要解决。
- **P2**：在明确条件下触发的功能错误或验证缺口，需要修复并补充验证。
- **已复现**：实际编译或执行了对应生产代码/隔离测试，后文给出结果。
- **代码确认**：调用链和状态变化可直接从代码确认，本轮没有执行相应完整场景。
- **契约待对齐**：实现改变了当前权威业务规则；须先明确产品语义，不能仅按模型命中率评价对错。

### 1.1 “效果更好”需要分层提供证据

建议作者在 PR 中明确目标并提供两类验证：

1. **旧身份跟踪链路：** 同一录像、同一检测输入下，对比 raw/semantic ID 切换、身份碎片、
   出画面重入恢复、多人交叉/遮挡、错误接管和活动 target 连续性。若未接入新特征，该层不能
   仅凭 provider 的模型名字认定效果提升。
2. **新当日去重链路：** 真实人体裁剪的同人/跨人分布、重复处置减少情况、新人漏选、失败/取消后
   重试、跨 semantic ID/跨狗/跨天/重启行为，以及首帧冷加载和服务响应耗时。

增强外观特征时要核对新旧模型的预处理、输出维度、归一化、特征库更新和阈值语义。不能只把新
模型路径塞进旧 OSNet 预处理，也不能为放宽匹配而删除已有任务状态与 bbox 可信性检查。

合并验收首先要求新功能关闭或服务不可用时，旧业务入口与 merge 保持一致；打开后只允许已明确
定义的外观/去重增益，不改变旧任务命令、目标所有权和认证结果语义。

## 二、历史总评（针对 2026-10-09 的 2214cd1）

这个 PR 同时引入了 ReID 查询、目标确认门控和当日库持久化，但当前实现有多项合并阻断：

1. 新 provider 的 ROS service、package 元数据、Humble service API、云端代理源码、TensorRT 源码及 tracking typesupport 链接存在独立构建错误。当前原版 PR 无法完成生产构建，修复一个错误之后还会暴露后续错误。
2. `MissionRosAdapter` 删除了 merge 已有的 READY 映射、暂停恢复字段支持、heartbeat 超时和同序号冲突处理。PAUSE、RETURN、TERMINATE 及终态都会进入 READY，tracking 却拒绝新状态并保留旧状态。这些回归在关闭 ReID 时同样存在。
3. ReID 只抑制确认事件，却保留被去重者的 primary 锁定，导致同屏其他新人无法被选择。
4. ReID 查询在 mission_mutex 内同步等待，默认最长 3 秒，会阻塞 tracking 的状态接收和帧处理。独立 helper executor 不会解除这把主业务锁的阻塞。
5. provider 的“今天见过”在首次查询时就成立，gate 又将其直接解释为免处置。这会跳过首次确认失败、接近失败或取消后尚未完成核验的人，需要与当前 handled 业务规则对齐。
6. gate 的正命中缓存没有跨天失效机制；身份库每日清零不能让这个缓存自动失效。
7. 持久化采用先删除旧库再改名，在替换中断时会丢整份已落盘历史；恢复旧 steady_clock 时间也会在机器重启后造成容量淘汰顺序错误。

验证中，相同的 6 项生命周期回归在 merge 全部通过、在 PR 全部失败；PR 的多人去重和锁等待复现另有 2 项失败；持久化异常复现有 2 项失败。provider 自带 134 条库断言和 31 条逻辑仿真断言通过，但它们没有运行实际 tracking gate，也未注册到 CTest，不能覆盖上述问题。

请先恢复 merge 已有的生命周期实现，再修正去重候选选择、异步边界和持久化，并提交真实 colcon 构建、节点测试及业务场景验收结果。建议在这些问题处理完成后重新评审。

## 三、逐项评审意见：构建阻断

以下问题相互独立。前面的错误会遮挡后面的错误，不能只修首次 colcon build 报出的那一处就认为构建完成。

### R10 — [P1] ReIDDedup.srv 缺少真实请求/响应分隔符

**引入提交：** `2766244`。**证据：** 原版 colcon build 已复现。

**评论位置：** [ReIDDedup.srv:13](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/srv/ReIDDedup.srv#L13)。

第 13 行只有响应说明注释，ROS service 必须在请求和响应之间有单独一行 `---`。注释不能替代这个分隔符。

**实际错误：**

```text
rosidl_adapter.parser.InvalidServiceSpecification:
Could not find separator '---' between request and response
```

**影响：** 新接口无法生成；tracking 新增硬依赖该包，因此生产工作空间也无法完成构建。

**修复要求：** 在请求字段之后、响应字段之前加分隔符。同步修正对接文档中复制的 srv 定义示例，避免下游照抄同一个错误。

**验收：** 干净的独立 build/install 目录能生成 service；Request 包含 crop、track_id 等请求字段，Response 包含 status、is_seen 等响应字段，Python/C++ 生成类型均可导入/编译。

### R11 — [P1] 接口 package 的 member_of_group 拼写不符合 Humble 要求

**引入提交：** `2766244`。**证据：** 临时补上 R10 后，colcon 配置阶段已复现。

**评论位置：** [package.xml:23](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/package.xml#L23)。

当前写成 `ros_idl_interface_packages`，应为 `rosidl_interface_packages`。

**实际错误：**

```text
Packages installing interfaces must include
'<member_of_group>rosidl_interface_packages</member_of_group>'
in their package.xml
```

**修复要求：** 使用正确 group。检查 package.xml 的接口生成、运行时和依赖声明与真实构建一致。

**验收：** 修复 R10 后，新包不再在 rosidl_generate_interfaces 的 group 检查阶段退出。

### R12 — [P1] create_service 的 QoS 参数与 ROS 2 Humble API 不兼容

**引入提交：** `2766244`。**证据：** 临时修复 R10/R11 后实际编译失败。

**评论位置：** [reid_node.cpp:233](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/reid_node.cpp#L233)。

当前调用将 `rclcpp::ServicesQoS()` 直接传入带 callback group 的 create_service。当前 Humble 的该重载要求 `const rmw_qos_profile_t&`。

**实际错误：**

```text
cannot convert rclcpp::ServicesQoS to const rmw_qos_profile_t&
```

**修复示例：**

```cpp
rclcpp::ServicesQoS().get_rmw_qos_profile()
```

也可用对应的原生 service QoS profile。保留单独的 service callback group，不能为绕过编译而改变云端等待所需的回调并发关系。

**验收：** 在项目实际使用的 Humble 上编译真实 reid_node，并完成一次请求/应答；不能只验证纯标准库测试可编译。

### R13 — [P1] handle_operation 中的引用对象不能用指针语法访问

**引入提交：** `2766244`。**证据：** 实际编译失败。

**评论位置：** [reid_cloud_proxy_node.cpp:205](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/reid_cloud_proxy_node.cpp#L205)，同函数第 207 行。

handle_operation 的参数是 `const ReIDRequest& msg`，这两行却使用 `msg->min_similarity` 和 `msg->max_results`。

**实际错误：**

```text
base operand of '->' has non-pointer type 'const ReIDRequest'
```

**修复要求：** 改为 `msg.min_similarity` 和 `msg.max_results`，检查该函数其余字段访问类型一致。

**验收：** reid_cloud_proxy 实际目标可编译；search、upsert、search_and_upsert 分别验证，不用纯 CloudLibrary 仿真代替真实节点目标编译。

### R14 — [P1] TensorRT 后端包含头文件和工厂函数参数类型错误

**引入提交：** `2766244`。**证据：** 在本机 TensorRT/CUDA/OpenCV 头文件下做语法编译，已复现。

**评论位置：** [feature_extractor_trt.cpp:195](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/feature_extractor_trt.cpp#L195)、[feature_extractor_trt.cpp:245](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/feature_extractor_trt.cpp#L245)。

两个独立编译问题：

1. 调用 cv::dnn::blobFromImage，却没有包含 opencv2/dnn.hpp。
2. 命名空间内的 CreateTrtFeatureExtractor 使用 `const Config&`。Config 是 FeatureExtractor 的嵌套类型；这里没有类作用域或别名，必须限定为 `const FeatureExtractor::Config&`。头文件声明已经使用了限定形式。

**实际错误：**

```text
'cv::dnn' has not been declared
'Config' does not name a type
```

**影响：** REID_BUILD_TENSORRT 默认 ON，目标 Orin 上检测到 TensorRT/CUDA 后会编译这个文件。只构建 OFF 后端不足以验证默认生产构建。

**修复要求：** 补足所需头文件，使工厂声明与定义一致，包括保留的非 TRT 占位分支。

**验收：** OFF 和 ON 两种配置都能完整编译。ON 的编译通过之后，再用目标机生成的真实 engine 验证加载、提特征和输出；本轮只确认编译错误，没有验证推理精度。

### R15 — [P1] tracking 不能把外部接口包名传给本包 typesupport target 查询

**引入提交：** `2766244`。**证据：** 在临时修复并安装 provider 后，tracking 生产配置已复现。

**评论位置：** [tracking/CMakeLists.txt:208](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/CMakeLists.txt#L208)。

`rosidl_get_typesupport_target(reid_provider_typesupport dog_patrol_reid_provider rosidl_typesupport_cpp)` 需要第二个参数是当前 CMake 构建中生成接口的 target。tracking 并未创建名为 dog_patrol_reid_provider 的生成 target；find_package 引入的是该包导出的接口目标。

**实际错误：**

```text
dog_patrol_reid_provider is not a CMake target.
Maybe rosidl_generate_interfaces was given a different target name?
```

**修复要求：** 使用外部包导出的依赖/目标。当前已把 dog_patrol_reid_provider 加入 ament_target_dependencies，应先验证该依赖能否提供所需类型支持，避免额外调用错误的本包 target 查询。

**验收：** provider 安装后，`TRACKING_ENABLE_ORIN_RUNTIME=ON` 的 tracking 配置、编译、链接全部成功。OFF 不经过这段代码，不能用 OFF 构建替代生产构建验证。

## 四、逐项评审意见：运行行为与业务契约

### R01 — [P1] 恢复 READY 状态映射，避免暂停和终态后继续使用旧业务状态

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:87](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L87)，`MissionPhaseFromMessage()` 的 switch 结束位置。

**问题和原因：**

当前 diff 删除了原来的 READY -> kReady 分支。收到 MissionState.READY 时，MissionFromMessage 返回 nullopt，StoreMissionState 返回 false，原 latest_mission_ 被保留。

READY 是巡检生命周期的正常状态，覆盖初始化完成后的待命、PAUSE、RETURN、TERMINATE 和父任务终态。拒绝 READY 会使 tracking 与 supervisor 的业务状态不一致。

**最小触发步骤：**

1. tracking 接受 `PATROL, state_seq=10, target_id=0`。
2. 当前帧有可信人员，primary 被选中。
3. supervisor 接受 PAUSE，发布 `READY, state_seq=11, target_id=0`。
4. tracking 拒绝该快照。
5. 下一帧仍按 seq=10 的 PATROL 处理，primary 保留，并可继续产生 TARGET_CONFIRMED。

如果暂停前是 APPROACH/VERIFY/TRACK，tracking 会保留旧活动目标上下文。其他消费者仍有独立状态门控；本项已证实的是感知状态和输出回归，不能据此宣称整机一定继续运动。

**期望：**

- READY 必须映射到显式 kReady。
- 接受 READY 后清当前 primary，不选择新目标，不发目标 bbox，不发 TARGET_LOST。
- `READY + target_id=0 + handled_target_id>0` 是暂停后保留已处理记录的合法状态。
- READY 不能触发 STARTUP readiness 重跑。

**修复要求：** 恢复 merge 已有映射和 READY 处理路径。不能通过设置 `reid.enable=false` 代替修复，因为状态映射不受该开关控制。

**验收：**

- `AcceptsReady` 和 `PauseClearsPrimaryAndEvents` 回归通过。
- 分别从 PATROL 和目标活动阶段进入 READY，验证清 primary、停止 bbox/事件。
- 补充 RETURN、TERMINATE、COMPLETED、FAILED 后收到 READY 的覆盖。

**契约依据：** 第 4.8 节、第 12.8 节及第 14.5 节。

### R02 — [P1] RECOVER_PATROL 必须允许暂停恢复的 target_id=0

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:108](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L108)。关联逻辑为该文件第 21 行和第 25 行。

**问题和原因：**

新增 HasActiveTarget() 把 RECOVER_PATROL 一律归为必须 target_id>0 的阶段。随后 handled 校验又只允许 RECOVER 中 handled_target_id 等于 target_id。这拒绝了两个合法快照：

```text
RECOVER_PATROL, target_id=0, handled_target_id=0
RECOVER_PATROL, target_id=0, handled_target_id=42
```

**触发场景：** 在 APPROACH、VERIFY、TRACK 或恢复阶段 PAUSE，再 RESUME。supervisor 按设计进入暂停恢复型 RECOVER_PATROL，target_id=0，导航使用保存的中断点恢复巡检，不应创建人员目标会话。

**期望字段规则：**

| 状态 | target_id | handled_target_id |
| --- | ---: | --- |
| STARTUP | 0 | 0 |
| READY、PATROL | 0 | 合法非负 ID |
| CONFIRM、APPROACH、VERIFY、TRACK | 正 ID | 0 |
| 普通 RECOVER | 正 ID | 0 或等于 target_id |
| 暂停恢复 RECOVER | 0 | 0 或暂停前保留的正 ID |

MissionRosAdapter 只读 MissionState，不能在这里额外要求获取父任务状态才能接受已经由 supervisor 形成的合法快照。

**修复要求：** 恢复按 phase 和 target_id 分情况校验的实现。目标活动阶段继续要求非零 target_id，暂停恢复不创建人员会话。

**验收：** `AcceptsPausedRecovery` 通过；target_id=0 的两种 handled 组合均接受；普通 RECOVER 中 handled=其他人员 ID 仍拒绝。

**契约依据：** 第 4.7、7.7、12.8 节。第 12.8 节已经明确列出不能将所有 RECOVER 视为活动目标的原因。

### R03 — [P1] 保留感知状态 heartbeat，避免断联后无限执行旧状态

**引入提交：** `2766244`。**证据：** 已复现；同时造成既有测试编译失败。

**评论位置：** [mission_ros_adapter.cpp:195](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L195)、[mission_ros_adapter.cpp:272](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L272)。

**问题和原因：**

PR 删除 Config.state_timeout、last_state_received_ 和 MissionStateAvailableLocked()。CurrentMission() 无条件返回 latest_mission_，ProcessFrame() 无条件使用旧快照。

**复现：**

1. 接受一个 PATROL 快照。
2. 不发送任何 heartbeat，等待 800ms，超过当前 merge 默认 750ms 接收年龄限制。
3. 处理一个含可信人员的帧。

merge 的 CurrentMission 失效，输出无 primary、无确认；PR 仍返回旧状态，选中 primary 并产生确认事件。若旧状态是目标活动阶段，同样存在继续用旧目标上下文的路径。

**额外构建影响：** 既有 test_mission_ros_adapter.cpp 第 205 行仍设置 config.state_timeout，原版测试目标实际编译失败，错误为 `Config has no member named state_timeout`。

**修复要求：**

- 恢复可配置的 state_timeout 和 steady/monotonic 接收时间。
- 相同合法序号的 heartbeat 刷新接收时间，但不重做业务副作用。
- 超时使感知本地处理门关闭，不替 supervisor 修改父任务状态。
- 恢复通信后重验快照，不能使用已取消的旧查询/目标结果。
- 为 ReID 耗时移除超时检测会掩盖断联，应修复 R05 的阻塞结构。

**验收：** 既有 heartbeat 测试能编译且通过；补充过期前 heartbeat 刷新、过期后无输出、恢复后仅按当前权威状态处理的测试。

**契约依据：** 第 11 节的周期状态与本地 state_timeout 规则。

### R04 — [P1] 去重拒绝必须更新候选资格，否则会阻塞同屏其他新人

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:275](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L275)。

**问题和原因：**

frame_transaction_.Update() 已经选定并锁定 primary；gate=false 只跳过 publish。没有把该人员移出候选，也没有释放当前 primary。PrimaryTargetManager 对持续可见的当前 primary 优先保持，因此后续仍只尝试同一个人。

**已执行的复现：**

- 人员 A：semantic_id=1，框 80×80，gate 拒绝。
- 人员 B：semantic_id=2，框 60×60，gate 允许。
- 两者始终可信且可见，连续处理 5 帧，帧间 source_time 增加 150ms，超过确认重试的 100ms 间隔。
- 结果：gate 仅被询问 A，从未询问 B。

**用户可见影响：** 一个已见人员在前景持续停留，可能让整个巡检一直跳过旁边尚未处置的人。只增加一次事件过滤不能实现多目标去重后的正常选择。

**修复要求：** 把有效的去重/处置资格结论送回候选选择边界，跳过被排除的 A 后继续选择 B。缓存资格必须有明确有效期和失效规则。活动目标阶段仍绑定 supervisor.target_id，不能因这项修复切换活动任务人员。

**验收：**

- 已见 A 始终可见时，新人 B 仍能得到确认。
- 所有人均被排除时无确认、无反复查询。
- A 的排除失效后可重新成为候选。
- 已进入目标活动阶段后，B 变大不能抢走 A 的活动任务。

### R05 — [P1] 不要持有 mission_mutex 同步等待 ReID service

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:272](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L272)、[reid_confirm_gate.cpp:46](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L46)。

**实际调用关系：**

```text
帧处理 -> ProcessFrame
         -> 获取 mission_mutex_
         -> frame_transaction_.Update
         -> confirm_gate
         -> future.wait_for(timeout_ms)，默认 3000ms
         -> publish
         -> 释放 mission_mutex_

状态接收 -> StoreMissionState -> 等待同一 mission_mutex_
```

helper 节点有独立 executor，能让 service response 到达，但 StoreMissionState 仍被主业务锁阻塞。主节点的独立 mission callback group 也不能绕过该锁。

**已执行的复现：** gate 模拟等待 250ms。等待开始后，另一线程调用 StoreMissionState，传入合法的新 RECOVER 状态。接收函数约等到 gate 返回后才完成，100ms 内返回的检查失败。这里用合法 RECOVER 状态隔离了 R01 的 READY 解析错误。

**影响：** ReID 慢、冷加载或超时期间，tracking 无法及时应用 PAUSE、RETURN、TERMINATE、reset，串行帧流水线也暂停。等待结束时仍可能发布使用旧 state_seq 和旧 source stamp 的确认；supervisor 可以拒绝旧事件，但这不能代替感知及时取消。

**修复要求：**

- ReID 查询异步进行，不能在 mission_mutex 内阻塞等待。
- 关联本地查询上下文，例如当前 PATROL state_seq、semantic_id、来源帧信息和本地取消代次。
- 使用返回结果时重验当前状态、序号、目标及有效期；已离开 PATROL 的结果丢弃。
- 超时/ERROR 的降级是否放行按明确业务策略处理，同时保持状态接收和帧处理响应性。
- 不要求为此增加 runtime_epoch、task_generation 等全系统协议字段。

**验收：** ReID 等待期间分别送 PAUSE、RETURN、TERMINATE、reset；状态应及时更新，旧查询结果不发业务事件、不恢复旧目标。补充服务不可用、慢响应、超时和退出时的覆盖。

### R06 — [P1，原业务语义回归] 不能直接将“今天见过”当成“已经完成处置”

**引入提交：** `2766244`。**证据：** 代码确认，与当前权威设计存在直接冲突；没有执行真实模型的完整处置流程。

**评论位置：** [reid_node.cpp:403](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/reid_node.cpp#L403)、[reid_confirm_gate.cpp:120](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L120)。

**现有顺序：**

1. 新人首次作为候选，tracking 查询 ReID。
2. provider 未命中，立刻 gallery.create 入库。
3. provider 返回 is_seen=false，tracking 才发布 TARGET_CONFIRMED。
4. supervisor 随后执行位置确认、接近、身份核验或跟踪。

入库发生在第 4 步业务完成之前。provider 命中历史外观就返回 is_seen=true，没有区分已通过核验、已完成跟踪、刚刚看到但未处理、失败或已取消。

**明确的错误场景：**

1. 人员 A 首次查询已经入库。
2. CONFIRM_TARGET 位置确认超时，或 APPROACH 失败，或核验前 PAUSE/RETURN/TERMINATE。
3. A 尚未形成最终业务结果，supervisor.handled_target_id 仍为 0。
4. 回到巡检或重新开始任务后，再对 A 发起真正的 ReID 查询。
5. 同日外观库命中，gate 将其锁存为抑制确认，A 无法按原设计重新处置。

gate 有默认 2 秒的未抑制缓存，因此不是每一次立刻重试都会查询 provider；本项指缓存窗口之后的再次查询，或同人新的 semantic_id 触发的查询。

**契约冲突：** 第 4.7 节规定 AUTHORIZED 或 TRACK_INTRUDER 最终 TARGET_LOST 才形成 handled；提前目标丢失和普通 EXECUTION_ERROR 不应授予免处置资格。现有排除表还采用持续不可见 60 秒释放的规则，PR 的当天见过直接抑制也改变了这一规则。

provider 对接文档第 4 节明确说明 is_seen 提供出现事实，是否抑制由检测侧/总控策略决定。因此本项评审的是 gate 的业务解释，不能据此指称 ReID 外观查询本身实现错误。

**本轮复核结论：** 用户已经明确“增强 ReID，之前的实现不能被弄坏”。因此本轮按现有 handled
和失败后重试规则验收，该问题应作为需要修复的业务回归处理，不默认接受当天见过就免处置的新策略。

**修复要求：**

- 以当前权威设计为准时，外观身份库与业务已处理结论分开维护，仅具有有效已处理资格的人员才被排除。
- 未处置、失败、取消的首次目标应保留重新确认资格。
- 跨狗 seen 命中也不能凭一个 bool 推定其他狗已经处置完成。
- “当天出现过就免处置”属于不同的业务策略。如团队以后要引入，应单独评审并修订权威设计，不混入本次保留旧功能的 ReID 增强。

**验收：** 首次位置确认失败后重新出现、接近失败后重新出现、核验前暂停再继续、核验前终止后重开、同人换 semantic_id、其他狗只见过但未处理等场景都必须有明确预期和测试。

### R07 — [P2] gate 的正命中缓存需要与每日清库同步失效

**引入提交：** `2766244`。**证据：** 代码确认；本轮未运行真实跨午夜场景。

**评论位置：** [reid_confirm_gate.cpp:101](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/reid_confirm_gate.cpp#L101)。

**问题：** entries 中 seen=true 时直接 false 返回，不再 Query。条目没有日期、有效期、清库通知或主动失效路径。provider 的 check_daily_reset 只清 provider 自己的库。

**触发条件：** tracking 跨午夜持续运行；某 semantic_id 在前一天被 gate 判定见过，次日仍沿用同一 ID。即使 provider 已清空当天库，gate 也不会再次查询，仍保留昨天的抑制结论。

max_tracked 超限清空发生在后续查询路径，不能保证跨天失效，也不能把容量清空当作业务日期策略。

**修复要求：** 按明确的每日有效期同步清理 gate 缓存。可使用本地日期判断或有效期策略，但应与 provider 的日期口径一致；不能仅补 gallery 的跨日测试。

**验收：** 用可注入日期推进到次日，在 semantic_id 不变、tracking 不重启的情况下，旧 seen 结论失效并重新询问。正常同日缓存仍应抑制重复查询。

### R08 — [P2] 恢复同序号冲突锁存，不能继续使用存在冲突的旧快照

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:173](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L173)。

**问题：** 原实现收到同序号不同内容时，锁存该序号冲突并关闭本地状态门，等待新序号恢复。PR 只返回 false，latest_mission_ 仍有效，之后重发原内容也会重新被接受。

**复现序列：**

```text
PATROL(seq=30, handled_target_id=42) -> 接受
PATROL(seq=30, handled_target_id=43) -> 拒绝，但旧快照仍有效
PATROL(seq=30, handled_target_id=42) -> PR 又接受
```

同一个 state_seq 必须表示同一份权威业务快照。存在冲突时继续选用第一个值会隐藏协议错误，并影响已处理人员资格等业务判定。

**修复要求：** 恢复冲突标记；对同序号的字段冲突、同序号非法状态快照均按既有规则处理。更新的合法序号可以解除冲突；同序号重发不得解除冲突。

**验收：** `SameSequenceConflictClosesCurrentState` 及既有 `RejectsConflictingDispositionAtTheSameStateSequence`、`InvalidSameSequenceSnapshotLocksOutOriginalSnapshot` 等测试通过。

### R09 — [P2] 保留 state_seq=0 的入口拒绝规则

**引入提交：** `2766244`。**证据：** 已复现。

**评论位置：** [mission_ros_adapter.cpp:97](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_perception_tracking/src/modules/mission_ros_adapter.cpp#L97)。

**问题：** PR 删除 message.state_seq==0 的拒绝条件。序号 cursor 首次接收时本身可以接受 0，因此入口删除校验会让未初始化/非法序号被当作业务状态。

**复现：** MissionFromMessage 接收 `PATROL, state_seq=0, target_id=0`。merge 返回 nullopt，PR 返回有效快照。

**修复要求：** 恢复边界层的非零序号校验，保留现有 wrap-aware 新旧判断，不用普通数值大小比较替代。

**验收：** STARTUP、PATROL、READY、目标状态的 seq=0 均拒绝；正常相同 heartbeat 与合法 uint32 回绕行为保持正确。

## 五、逐项评审意见：第二个提交的持久化

### R16 — [P2] 保存文件应原子覆盖，不能先删除上一份有效库

**引入提交：** `2214cd1`。**证据：** 注入进程中断的测试已复现；没有做真实硬件断电试验。

**评论位置：** [reid_gallery.cpp:341](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/reid_gallery.cpp#L341)。

**实际顺序：**

```cpp
std::remove(path.c_str());
if (std::rename(tmp.c_str(), path.c_str()) != 0) {
  std::remove(tmp.c_str());
  return false;
}
```

写临时文件已经成功，并不能使这段替换成为原子操作。旧文件删除后、新文件改名前，library.bin 不存在。此时进程退出或机器断电，LoadFromFile 只打开正式路径，返回 kMissing；它不会从 .tmp 恢复。rename 失败的路径还会删除 .tmp，进一步失去两份数据。

**已执行的复现：**

1. 成功保存包含旧身份 A 的正式库。
2. 开始保存含新增身份 B 的第二份库。
3. 包装 remove 调用，在旧正式库删除后注入子进程退出，阻止后面的 rename。
4. 新建 Gallery，从正式库路径加载。
5. 实际返回 kMissing；期望至少上一份已提交的有效库仍可加载。

测试调用的是 PR 的真实 SaveToFile/LoadFromFile，注入点只控制中断时刻。

**影响：** 与 README 的“异常断电最多丢一个 10 秒防抖窗口内的更新”承诺不符，会失去此前已成功保存的整份当天历史。

**修复要求：**

- 在 Linux/Orin 上使用同目录临时文件直接 rename 原子覆盖正式文件，不先 unlink 正式文件。
- 如果文档保留断电持久性承诺，落实文件和目录的必要同步及恢复行为；ofstream.flush 只到用户态/系统缓存边界，不能单独证明断电持久性。
- Windows 的替换策略单独处理，不为兼容性削弱目标 Linux 的原子替换。
- 写入或替换失败时，保留上一份成功快照；临时文件恢复策略应明确并测试。

**验收：** 正常重复保存、写失败、替换失败、写临时文件中断、正式替换边界中断、残留 .tmp 的加载路径均有测试。不能只做正常 round-trip。

### R17 — [P2] 不能跨机器重启原样恢复 steady_clock 时间用于淘汰

**引入提交：** `2214cd1`。**证据：** 使用真实保存/加载/淘汰函数，模拟重启时间基线重置已复现。

**评论位置：** [reid_gallery.cpp:409](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/src/reid_gallery.cpp#L409)。关联保存位于第 331 行，淘汰比较位于第 290 行。

first_seen/last_seen 来自 steady_clock::now().time_since_epoch()。这个值适合单次机器启动期间的相对时间比较，不能作为跨机器重启的持久化时间基准。PR 将这两个数直接保存并原样恢复，evict_if_needed 又按 last_seen 最小值淘汰。

**已执行的复现：**

```text
容量：1
旧机器启动期间：A.last_seen=10000，保存成功
同日机器重启：新的 now=2
恢复 A，原 last_seen 仍为 10000
创建新人 B，B.last_seen=2
超过容量，淘汰最小 last_seen -> B 被当场删除，A 留存
```

本轮用容量 1 使结果直接可观察；生产默认容量较大时，达到容量后发生同样的比较错误。仅重启进程通常共享同一个系统单调时钟基线，因此现有“构造新 Gallery 再加载”的测试不会自然覆盖关机充电后开机。

**影响：** 新人无法稳定入库，再次出现会反复当作新人；淘汰策略偏向保留旧启动期间的记录，破坏去重稳定性。

**修复要求：** 持久化改用可跨重启比较的时间/访问顺序，或加载时将已有记录重建到同一可比较基线。运行时 TTL 和 heartbeat 仍应使用单调时钟，不能因持久化调整而改用可能跳变的墙钟。

**验收：** 同日进程重启、同日整机重启、容量已满后的新增和命中刷新都覆盖。若采用墙钟时间，还需定义系统时间修正时的排序行为。

## 六、逐项评审意见：测试接入

### R18 — [P2] 把新增自测注册到 CTest/colcon，并补真实 gate 和节点测试

**引入提交：** `2766244`，第二个提交扩展了这些未注册的自测。**证据：** CTest 发现测试数量为 0。

**评论位置：** [provider/CMakeLists.txt:100](https://github.com/HowAreAllWell/dog_patrol/blob/2214cd1c2e2f1f0dfec41401c3aa99ab6810562b/src/perception/dog_patrol_reid_provider/CMakeLists.txt#L100)。

当前 test_reid_gallery 和 sim_reid_e2e 只有 add_executable、include、install，没有 add_test 或 ament 测试注册。

**实际结果：**

```text
ctest --test-dir <provider build dir> -N
Total Tests: 0
```

手动执行它们时分别有 134/134 和 31/31 断言通过，但 colcon test 不会自动执行这些断言。sim_reid_e2e 还通过手写 encounter/fan_out 镜像节点编排，未运行真实 MissionRosAdapter、ReidConfirmGate 或 ROS provider 回调，所以不能验证状态锁、任务取消、多目标候选资格和真实节点构建。

**修复要求：**

- 注册可由 colcon test 自动执行的库测试和仿真，失败时能在 test-result 中体现。
- 保留并通过现有 tracking 生命周期测试；不能删掉因回归失败的测试来获得绿灯。
- 为真实 gate 增加 service 不可用、ERROR、慢响应、timeout、重复查询、跨天、取消后迟到结果测试。
- 为实际 provider 节点和云端代理增加最小 ROS 请求/应答测试。纯库通过不能替代真实目标可编译和真实回调可工作。
- 增加 R04 的多人候选、R06 的失败后重新处置和 R16/R17 的持久化异常测试。

**验收：** `colcon test-result --verbose` 显示新增测试且能报告故意注入的失败；提交每类测试的执行命令、数量和结果。

## 七、现有验证证据及其边界

### 7.1 执行环境和隔离方式

- 本机项目的 ROS 2 Humble、系统 `/usr/bin/colcon`、系统编译器及本机 OpenCV/TensorRT/CUDA 头文件。
- PR 源码通过 git archive 提取到临时目录，未切换当前分支。
- 生命周期复现使用两版实际 MissionRosAdapter，与两版相同且未改动的 tracking core 链接。
- ROS 复现使用 `ROS_DOMAIN_ID=187`、`ROS_LOCALHOST_ONLY=1`，测试业务话题为 `/pr50_review/...`；未控制机器人。
- 沙箱首次阻止 DDS socket，随后在获准的沙箱外环境重跑，记录的正式结果来自该次运行。

### 7.2 测试结果

| 测试/检查 | merge | PR | 说明 |
| --- | --- | --- | --- |
| AcceptsReady | 通过 | 失败 | READY 及 READY+handled |
| AcceptsPausedRecovery | 通过 | 失败 | 暂停恢复 target_id=0 |
| PauseClearsPrimaryAndEvents | 通过 | 失败 | READY 后仍保留 primary/事件 |
| HeartbeatExpiryClosesFrameOutput | 通过 | 失败 | 800ms 无 heartbeat 后仍输出 |
| SameSequenceConflictClosesCurrentState | 通过 | 失败 | 冲突未关闭当前快照 |
| RejectsZeroSequence | 通过 | 失败 | 接受 seq=0 PATROL |
| DedupOfLargestPersonDoesNotStarveNewPerson | 不适用 | 失败 | A 被去重后 B 始终未被询问 |
| DedupDoesNotBlockMissionStateReceiver | 不适用 | 失败 | 250ms gate 阻塞状态接收 |
| PreviousCommittedSnapshotSurvivesInterruptedReplacement | 不适用 | 失败 | 正式库路径丢失，kMissing |
| NewEntrySurvivesCapacityEvictionAfterMachineReboot | 不适用 | 失败 | 新人被当场淘汰 |
| provider 自带库断言 | 不适用 | 134/134 通过 | 手动执行 |
| provider 自带逻辑仿真断言 | 不适用 | 31/31 通过 | 手动执行，镜像编排 |
| provider CTest 发现 | 不适用 | 0 项 | 未注册新增测试 |
| 原版 provider colcon build | 不适用 | 失败 | R10 首先阻断 |
| tracking 生产 ON 配置 | 不适用 | 失败 | R15 |
| 原版 tracking adapter 测试目标 | 不适用 | 编译失败 | Config.state_timeout 被删除 |

合计：相同的 6 项生命周期回归，merge 6/6 通过、PR 0/6 通过；PR probe 另加 2 项，共 8 项失败；持久化 probe 2 项失败。

### 7.2.1 第二轮直接针对 merge 的补充验证

本轮重新查询远端 PR head，仍为 `2214cd1c2e2f1f0dfec41401c3aa99ab6810562b`。

| 本轮验证 | 结果 | 解释 |
| --- | --- | --- |
| merge 的 7 组相关 CTest/gtest | 84 个用例通过 | identity 24、primary 34、mission coordinator 4、frame transaction 7、appearance service 2、MOT config 11、MOT observability 2 |
| merge 的完整原有 MissionRosAdapterTest | 19/19 通过 | 直接运行工作区原有测试目标，含 ROS 回调、状态和 bbox/事件测试 |
| merge 的 manager 测试 | 70 通过 | 状态机、命令、事件和相关适配逻辑 |
| merge 的 navigation 测试 | 68 通过 | 当前实际数量，以本轮 pytest 输出为准 |
| PR 的同源原有 adapter 测试，gate=nullptr | 15 通过、3 失败 | READY/暂停恢复和两种同序号冲突失败 |
| PR 原有 heartbeat 测试 | 原样仍无法编译 | Config.state_timeout 被删除；之前的独立 heartbeat probe 已确认回归 |
| merge 新补长暂停计时探针 | 1 项失败 | B01：merge 自身既有缺陷，明确不算本 PR 新增回归 |

前四行共重跑 **241 个既有用例，全部通过**。这不包含最后一行新补的边界用例，也不表示 merge
所有行为都已经无缺陷。

PR 旁路复核没有改业务代码，只在临时复制的测试源码中排除了那一项已知无法编译的 heartbeat API
测试，剩余 18 项与 merge 的原测试相同。实际调用 ProcessFrame 的默认 gate=nullptr，相当于
该边界没有接入去重门，仍复现 3 项失败。这证明 R01/R02/R08 属于入口回归，无法通过关闭 ReID 修复。

本轮通过的原有测试包括：权威 target_id 绑定、普通短时缺失、同 raw 几何恢复、不同 raw 不误接管、
多个 handled ID 排除、READY 清 primary、reset 清历史、bbox 原图坐标及来源时间、帧处理中状态
回调响应、旧帧结果失效等。新模型集成后应保留这些测试和行为。

补充结果文件仍位于临时材料目录：

- `merge_adapter_full.xml`：19 项原 adapter 测试。
- `merge_manager_navigation.xml`：70+68 项 Python 测试。
- `pr_existing_bypass.xml`：18 项 PR 旁路原测试，3 项失败。
- `merge_boundary_probe.xml`：B01 的失败复现。
- `review-probes/pr_existing_runtime_tests.cpp`、`review-probes/merge_boundary_probe.cpp`：本轮临时测试源码。

本轮基线验证命令：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dog_patrol_pr50_review.O9eoJE/second-pass-ros-logs

ctest --test-dir build/dog_patrol_perception_tracking \
  -R '^test_(identity_manager|primary_target_manager|mission_coordinator|mission_frame_transaction|appearance_feature_service|mot_tracker_config|mot_tracker_observability)$' \
  --output-on-failure -j2

./build/dog_patrol_perception_tracking/test_mission_ros_adapter \
  --gtest_output=xml:/tmp/dog_patrol_pr50_review.O9eoJE/merge_adapter_full.xml

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH="$PYTHONPATH:src/orchestration/dog_patrol_manager:src/navigation/fast_livo_dog/navigation/dog_patrol_navigation" \
  /usr/bin/python3 -m pytest -q \
  src/orchestration/dog_patrol_manager/test \
  src/navigation/fast_livo_dog/navigation/dog_patrol_navigation/test \
  --junitxml=/tmp/dog_patrol_pr50_review.O9eoJE/merge_manager_navigation.xml
```

### 7.3 构建诊断的解释

为了观察被前置错误遮挡的后续错误，仅在临时副本逐步修改了以下四处：

1. 加 srv 分隔符。
2. 修正 package group。
3. 修正 Humble service QoS 实参。
4. 修正代理函数的引用成员访问。

这样处理后 OFF provider 才能安装，用于继续验证 tracking 的外部类型支持配置。这个临时安装成功不能当作原 PR 构建成功。

四处临时源码修改已经撤销，并逐文件验证 PR 副本与 fetched head 一致。TensorRT 的两个错误通过直接语法编译独立确认。

### 7.4 复现材料

临时材料根目录：`/tmp/dog_patrol_pr50_review.O9eoJE`。该路径属于本轮机器现场，可能被系统清理；可长期转交的判断、触发步骤和验收要求均已写在本文件中。

| 材料 | 位置 |
| --- | --- |
| 生命周期、多候选、锁等待测试源码 | `review-probes/mission_probe.cpp` |
| 两版编译配置 | `review-probes/CMakeLists.txt` |
| 持久化异常测试源码 | `review-probes/persistence_probe.cpp` |
| merge 回归 XML | `baseline_probe.xml` |
| PR 回归 XML | `pr_probe.xml` |
| 持久化回归 XML | `persistence_probe.xml` |
| 原版构建错误 | `colcon-log/latest_build/dog_patrol_reid_provider/stderr.log` |
| 修分隔符后的 group 错误 | `colcon-log-after-separator/latest_build/dog_patrol_reid_provider/stderr.log` |
| 进一步暴露的 C++ 编译错误 | `colcon-log-after-group/latest_build/dog_patrol_reid_provider/stderr.log` |

当前现场可重跑隔离 probe：

```bash
source /opt/ros/humble/setup.bash
source /mnt/nvme/workspace/dog_patrol/install/local_setup.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dog_patrol_pr50_review.O9eoJE/ros-logs

cmake -S /tmp/dog_patrol_pr50_review.O9eoJE/review-probes \
  -B /tmp/dog_patrol_pr50_review.O9eoJE/review-probes-build
cmake --build /tmp/dog_patrol_pr50_review.O9eoJE/review-probes-build -j2

/tmp/dog_patrol_pr50_review.O9eoJE/review-probes-build/baseline_probe
/tmp/dog_patrol_pr50_review.O9eoJE/review-probes-build/pr_probe
/tmp/dog_patrol_pr50_review.O9eoJE/persistence_probe
```

原版 provider 的独立构建复现可使用新的临时输出目录：

```bash
source /opt/ros/humble/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=2 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/colcon --log-base /tmp/pr50_reproduce_log build \
  --base-paths /tmp/dog_patrol_pr50_review.O9eoJE/src/perception/dog_patrol_reid_provider \
  --build-base /tmp/pr50_reproduce_build \
  --install-base /tmp/pr50_reproduce_install \
  --cmake-args -DREID_BUILD_TENSORRT=OFF
```

上述原版命令预期先在 R10 失败。修复前置错误后继续重跑，才能看到后面的错误。

### 7.5 未验证事项

- 未验证真实 ReID 模型的同人/跨人准确率和 0.70 阈值是否适合现场。
- 未完成真实 GPU 推理、真实 cloud provider 联调或整机运动验收。
- 跨天 gate 和业务已处理语义项属于代码确认/契约分析，没有伪称已执行真实跨午夜或完整核验流程。
- 持久化测试注入的是进程中断及时间基线变化，没有伪称实际拔电验证。

上述边界不影响对已复现的构建和生命周期回归给出 Request changes。

## 八、建议修复顺序与重新评审条件

1. 将 PR 的目标分支对齐到团队拟集成的 merge，并保留能清晰区分本次 ReID 修改与既有历史的提交范围。
2. 先处理 R01、R02、R03、R08、R09，恢复 merge 已有的 MissionRosAdapter 业务约束。`reid.enable=false` 时也必须通过既有生命周期回归。
3. 修复 R10～R15 和既有 adapter 测试编译问题，提交目标 Humble/Orin 环境的干净构建结果。
4. 按本轮用户要求保留 R06 涉及的现行业务语义，解决 R04 的候选资格反馈和 R05 的异步取消边界。
5. 修复 R07 的日期有效期和 R16/R17 的持久化语义。
6. 完成 R18 的测试注册，让新增和既有回归进入标准 colcon test 流程。
7. 最后进行真实模型和业务集成验收：显式 START、PAUSE/RESUME、RETURN、TERMINATE、未处理新人、失败后重试、多人同屏、断联、跨天和重启。

重新提交评审时，请提供：

- 更新后的 commit/head 和逐项修复对应关系。
- provider OFF/ON、tracking 生产 ON 的构建命令和结果。
- 相关包 colcon test/test-result 的真实结果。
- 上述复现与新增验收场景的测试结果。
- 对 R06 如何保持现有已处理资格、失败和取消后重试语义的说明。
- 尚未验证的模型/硬件限制。

本次工作产出为本地评审稿；未发送 GitHub review、未创建修复提交、未合并 PR。

## 九、merge 自身既有边界：不计为 PR 新增缺陷

### B01 — 长时间 PAUSE 仍可能被计入 handled absence 到期时间

**归属：** 已存在于 merge，当前 PR 沿用这一 absence 计时行为。**证据：** 历史纯状态探针
已复现；2026-10-11 又用同一测试源码分别链接 merge 与当前 PR 的真实 core，均在恢复 PATROL
后错误释放排除的同一断言失败。此项单独记录，不要求作者为本 PR 承担一个原本就存在的问题。

**源码位置：**

- [merge: primary_target_manager.cpp:109](https://github.com/HowAreAllWell/dog_patrol/blob/47f7d487925911093ddab262b69ffb675d6a9124/src/perception/dog_patrol_perception_tracking/src/modules/primary_target_manager.cpp#L109)，按 now-start 计算 absence。
- [merge: primary_target_manager.cpp:325](https://github.com/HowAreAllWell/dog_patrol/blob/47f7d487925911093ddab262b69ffb675d6a9124/src/perception/dog_patrol_perception_tracking/src/modules/primary_target_manager.cpp#L325)，READY/RECOVER 清 primary，但不调整已有 absence 时间基线。

**现有逻辑：** 非 PATROL 时确实不调用 UpdateHandledIdentityAbsence，所以暂停中不会立即删除排除项。
但是恢复到 PATROL 后仍使用暂停前保存的绝对单调时间差，已经过去的暂停时长会被一起计入。

**已执行复现：**

```text
t=0：PATROL，已处理 A 在表内，A 可见
t=1：A 不可见，开始记录 absence_start=1
t=10：进入 READY，暂停
t=120：仍 READY，A 仍在排除表（此时尚未调用到期维护）
t=121：RESUME 回 PATROL，A 仍不可见
计算 now-start=120 秒 -> A 立即解除排除
```

按设计第 4.7 节，PAUSE 时长不应计入 60 秒持续不可见预算；上述有效 PATROL 时间尚不足 60 秒。
现有测试覆盖了排除表保留、try_emplace 不覆盖旧起点、正常 PATROL 中的 60 秒释放，但没有覆盖
“已开始 absence 后经历长时间 PAUSE 再恢复”的时间基线。

**后续修复方向：** 单独处理被冻结期间的时间预算，例如累计有效 PATROL absence 时长，或在
非活动到活动边界调整已有计时基线。READY、RECOVER、活动目标阶段及无有效状态期间的预算规则
应按权威设计统一测试；不能简单清表或重新授予免处置资格。

本轮保持业务源码不变，只记录基础计时问题。按用户本轮确认，启用 ReID 的已处置人员采用
当日去重策略；该策略与这里的基础 60 秒预算属于不同层，不能把 B01 算成 PR 新回归，也不
因此重新打开已按新需求关闭的 N04。
