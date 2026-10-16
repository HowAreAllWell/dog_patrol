# PR #52：仅核验通过授予 handled 的评审

## 接收记录：2026-10-16

用户完成本轮审查确认后授权接收 PR，并要求保留本地已调整的语音超时。
已先以 `6931f8b` 单独提交每轮响应窗 10 秒的设置，再通过 `--no-ff` 合入 PR #52。
合并提交为 `cf296509ac19658b9e1f32de10e1a54e4c0f25da`，双父分别为 `6931f8b` 与
已审查的 PR head `96f3597`，PR 的两个原始提交均保留。没有更改其他功能分支。

语音的三份 YAML、VoiceConfig 默认值和 YAML 缺省回退值均为 10 秒，voice 现有测试
64 项通过，资产校验 31 项通过。此参数仅影响每轮 Prompt 之后的响应窗，其他硬件恢复
超时和认证阶段规则未改。部署仍须执行下文的旧 handled reset，不自动操作现场任务或身份库。

合并后已用系统 colcon 在主工作区重新构建 dog_patrol_interfaces、dog_patrol_manager 和
dog_patrol_perception_voice，3 包成功；正式测试为 manager 71 项、voice 64 项，均无失败。
ROS 接口和 supervisor 导入通过。记录为审查目录下的 `accepted_workspace_build.log` 与
`accepted_workspace_tests.log`。本次构建没有重启现场节点。

## 结论

2026-10-16 对 [PR #52](https://github.com/HowAreAllWell/dog_patrol/pull/52) 的
`96f35979a0a0a282153fcc8c1ce9088116e8845e` 完成代码、测试和下游链路审查。
**未发现本次改动引入、需要阻止合并的 P1/P2 问题，代码审查层面可以接受。**

这是有意调整业务定义：原规则允许 `TRACK_INTRUDER` 最终 `TARGET_LOST` 授予 handled，
新规则只允许 `VERIFY_IDENTITY` 接受有效 `AUTHORIZED` 授予。核验失败后仍进入跟踪，
跟踪丢失后仍恢复巡逻；改变的是后续同人再次出现时不再因这次跟踪结束被免核验。
不是删除入侵者跟踪，也不是在 TRACK 阶段直接启动第二个核验会话。

上线需按 PR 已提供的步骤显式清理旧 handled，见本文部署说明。该步骤是旧数据语义迁移，
不是本次发现的新代码缺陷。审查阶段未合并 PR、发布远端 review、修改生产代码或重启现场节点；
随后用户授权接收，合并和超时设置记录见上节。

## 版本与范围

| 项目 | 核实结果 |
| --- | --- |
| 目标分支及比较基准 | `merge`，`76715bfbe0f8d7b32f73e66ff3d55ad0a13733b5` |
| merge-base | 与上述 merge 提交相同 |
| PR head | `96f35979a0a0a282153fcc8c1ce9088116e8845e` |
| 提交 | `dacb4b5` 业务修复；`96f3597` 补齐环境检查测试的 ReID 模块预期 |
| 文件范围 | 11 个文件：状态机、状态机测试、MissionState 注释、6 份文档、CI、环境检查测试 |
| 真实业务源码变化 | 仅 `dog_patrol_manager/state_machine.py` |
| GitHub 状态 | open、非 draft、mergeable=true；检查时 `build-and-test` 为 completed/success |
| 本地审查引用 | `origin/pr-52-review-20261016` |
| 隔离审查材料 | `build/pr52-review-96f3597/` |

tracking、ReID provider、感知 bringup/orchestrator、导航源码相对本地 merge **没有变化**。
MissionState 只有注释变化，字段、顺序、枚举和值均未改动。之前统一感知启动、四项 capability
READY、父任务命令、导航路径屏障、认证会话门控没有被回退。

## 逐项检查

### 1. handled 的唯一新增入口正确

PR 的 `state_machine.py:286` 在合法 AUTHORIZED 分支设置当前 target ID。
此前已经检查事件来源、当前 state_seq、合法状态转移和目标 ID；AUTHORIZED 仅允许感知
来源，且转移表中只有 VERIFY_IDENTITY 接受该事件。旧序号、错误目标、错误来源、其他阶段
的 AUTHORIZED 不能绕过校验写入 handled。

`begin_patrol_recovery():325` 删除 `target_handled` 参数，通用失败恢复入口将 handled 清为 0。
`_handle_target_lost():477` 不再按 TRACK_INTRUDER 状态授予资格。
检索仓库内全部调用后，supervisor 的确认超时入口和其他调用均未遗留被删除的关键字参数。

### 2. 已核验通过的结果没有被通用恢复入口误清

有效 AUTHORIZED 通过转移表直接进入 RECOVER_PATROL，不调用
`begin_patrol_recovery()`，因此不会被该方法里的赋零覆盖。
恢复完成和恢复 watchdog 只清活动 target，保留已经成立的 handled。
RECOVER 阶段的 EXECUTION_ERROR 仍走忽略分支，不能覆盖已通过结果。
重复事件仍按既有事件键去重，不会形成第二次业务完成。

### 3. 下游排除与 ReID 转发能消费新的零值

`PrimaryTargetManager::UpdateForMission()` 在新的 PATROL 快照消费显式
handled_target_id；它没有按“前序阶段是 TRACK”自行推断已处理。
`ResetForPatrolCycle(0)` 只清当前 primary，不插入新的排除，也不清除其他已通过人员的记录。

`MissionTrackingRuntime` 仅对正 handled ID 的新权威结论调用 NotifyHandled。
核验失败、跟踪丢失、未完成时暂停或终止的零值快照，不会向 provider 发送人员 handled。
ReID 仍使用 `is_seen && handled` 判断免重复处置，“见过但未通过”仍可被放行。
本次没有修改任何特征模型、匹配阈值、身份合并或当日库的生命周期。

| 流程 | 新行为及验证结果 |
| --- | --- |
| 核验失败 → TRACK → 丢失 → 导航恢复完成 | handled 保持 0，同 ID 可再次被选择并核验 |
| 核验失败 → TRACK → 丢失 → 恢复 watchdog | 与正常恢复相同，不授予 handled |
| 同一人员再次核验并通过 | 正常授予 handled，随后排除并转发 ReID |
| A 已通过，B 核验失败并跟踪结束 | A 的历史排除保留，B 仍可选择；不会因 B 的零值删除 A |
| 核验通过 → 恢复中暂停/继续 → PATROL | 已通过结果保留，A 继续被排除 |
| 未通过 → 跟踪中暂停/继续 | 不新增 handled，恢复巡逻后仍可核验 |
| 未通过 → TERMINATE | 不新增 handled |
| READY 后显式 reset | 清除 tracking 排除和 ReID handled，保留 ReID 身份及外观模板 |

### 4. PR 新回归有效，未用改断言掩盖原问题

新增/调整的测试覆盖两种恢复完成路径、相同 ID 重新进入核验并最终通过，以及恢复期间
感知/导航错误、重复结果和恢复 watchdog。
原来 AUTHORIZED 的正向断言仍保留；TRACK 丢失的期望改为 0，符合本 PR 明确调整的契约。

独立将 PR 版本的 `test_state_machine.py` 放到原 merge 状态机上执行，得到
**4 failed、39 passed**：两项失败来自核验失败后的导航/超时恢复，两项来自 TRACK 结束后的
错误/恢复保护检查；实际值都是原实现的正 handled，期望为 0。
同一文件在 PR 实现上全部通过，说明这些测试能检出原误标行为。

作者用例中的“再次核验”直接向 supervisor 发送合法事件，本身不能证明感知一定重新选人。
本轮因此另做真实 runtime/排除选择/消息转发/身份库的串联探针，结果见下节。

### 5. 配套 CI 与文档变化合理

CI Build 补入 tracking 已依赖的 `dog_patrol_reid_provider`，属于补齐现有依赖。
环境检查测试增加 `reid: implemented`，与 merge 已有 requirements 表一致，没有降低门禁。
接口注释、权威父任务设计、导航/感知合同、内部状态文档和实机指南均同步为“仅核验通过”。
旧 PR 评审中的历史结论保留原含义，不将其当成当前业务契约。

## 本轮验证证据

| 验证 | 结果 |
| --- | --- |
| PR manager 全套、授权规则、环境检查测试 | 108 passed |
| 新状态机回归对照旧 merge | 4 failed、39 passed，属于预期旧行为复现 |
| 独立编译 PR dog_patrol_interfaces / dog_patrol_manager | 系统 colcon，2 packages finished |
| 上述独立安装的 colcon test-result | 71 tests，0 errors、0 failures、0 skipped |
| 从 PR 源码重新编译主目标 C++ 测试 | 34 passed |
| 真实 tracking runtime 与 ReID 身份库串联 | 7 组流程、95 个权威快照，全部通过 |
| PR diff 空白检查 | 通过 |

108 项已经包含 manager 的 71 项，不能相加当作不同测试。没有在本机重跑作者整套 CI 的所有包；
GitHub CI 成功状态已通过 API 核实，相关结果与本地验证分开记录。

串联探针由 **PR 实际 MissionStateMachine** 生成合法快照，不手工制造“TRACK 丢失已处理”
等旧业务结果。它复用当前已构建且本 PR 未改动的 tracking 生产 core/ROS adapter 库，执行
真实 MissionRosAdapter、PrimaryTargetManager、MissionTrackingRuntime、ReidConfirmGate；
接收器收到实际 ReIDHandled 消息后调用 PR 的真实 IdentityGallery API。
逐快照检查两人的 handled 和 PATROL primary，额外等待投递延迟以检出误发通知。
ReID provider 的模型进程未启动，未用该探针宣称真实人员识别精度或 GPU 推理已经验收。

主要证据：`python_review.log`、`baseline_against_pr_regressions.log`、`build.log`、
`colcon_test.log`、`primary.log`、`primary.xml`、`runtime_probe.log`，均位于审查材料目录。
`generate_traces.py`、`runtime_probe.cpp` 和 `traces/` 保留复现输入与探针源码。

## 部署时需要执行的旧数据迁移

旧 ReID 持久化库的 handled 只有布尔值，不能分辨“旧规则下跟踪结束”与“确实核验通过”。
仅更新代码或重启可能恢复旧误标；这是本次语义变化的实际部署边界，PR 指南已明确说明。

1. 在没有活动巡检任务时，更新并重新构建相关接口和 manager，统一使用新版本重启。
2. 正常启动整组感知，tracking 和 ReID 同时运行；等待 tracking 已处理 READY 状态及图像帧。
3. 显式调用 `/mission/reset`，令正在运行的 tracking 观察到重入 STARTUP。可使用
   `ros2 service call /mission/reset std_srvs/srv/Trigger '{}'`。
4. 确认 ReID 日志出现“任务 reset：清除 handled 标记”；这会同时让 tracking 清历史排除、
   provider 清 handled，保留外观身份库。reset 后路线需重新准备，并显式 START。
5. 此次清理也会取消此前真正通过人员的旧免核验记录，他们首次再遇时可能需要重新核验；
   新规则下再次通过后正常累计新记录。不能只清一端，或把“刚重启已是 STARTUP”当成已完成此步骤。

本轮 7 组串联验证包含 READY 后 reset，确认两人的 handled 被清除、身份仍存在、重新 START
后可选人。未执行现场 reset、删除 `data/reid` 或调整模型资产。
既往已接受的“完成快照在下一帧前被终止快照覆盖，可能漏转发”的低概率限制没有被本 PR
修复，也没有扩大成本次新增阻断意见；仍按 PR #51 已确认的范围处理。

## 审查现场保留

审查阶段工作分支保持 merge，HEAD 为 `76715bf`，PR 未合并，生产 install 未覆盖；
当时用户现有四个 voice 配置/代码文件的未提交修改原样保留。
审查阶段仅新增本评审文档，PR 源码和测试产物均放在忽略的 build 审查目录。
用户随后授权保留该超时设置并接收 PR，已形成上节记录的独立配置提交和合并提交。
