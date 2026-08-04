# Issue #3 detection/tracking 接入整体验收

## 结论

2026-08-04 对父 Spec #3 及子票 #4–#15 完成独立整体验收。父 Spec 的交付重点——将
detection/tracking 正式迁入 `dog_patrol`、建立后续 face/voice seam、提供 standalone 与 mission
运行边界、完成当前感知 Orin/Hik 验收并切换唯一权威仓库——均已有合并代码、自动测试、现场证据和
远端状态支撑，可以关闭。

真实人脸、真实语音、导航算法和最终导航 Orin 整机验收没有完成，也不在本 Spec 的完成声明中。
orchestrator 当前拥有 readiness 聚合、ROS-independent 授权会话规则及通用结果 adapter；真实
face/voice provider 后续通过 `AuthorizationEvidence` 接入，不得用测试 provider 或生产 placeholder
伪造 evidence、整体 READY 或授权结论。

## 父 Spec 核对

| 范围 | 代码或稳定证据 | 结论 |
| --- | --- | --- |
| tracking 正式迁入并保留历史 | `src/perception/dog_patrol_perception_tracking/`；annotated tag `tracking-import/vision-demo-ws-7878d70` 固定 116 个过滤提交，tip `6faaed42` | 通过；无 submodule、旧 overlay 或旧仓构建依赖 |
| portable / Orin 构建边界 | tracking `CMakeLists.txt` 的 `TRACKING_ENABLE_ORIN_RUNTIME`；PR #18、#24 | 通过；portable 核心与 production runtime 共用实现，开启硬件 runtime 时依赖缺失会在配置期失败 |
| 默认 light / 可选 ONNX | tracking 默认参数、ReID backend 规范化和感知环境检查器 | 通过；tracker/SID 默认 `light` 且空模型路径合法，仅 `osnet_onnx` 及其 alias 要求并实际加载 ONNX |
| orchestrator 与 Fake 退役 | `AuthorizationCoordinator`、`perception_authorization`、`ReadinessCoordinator` 和 `perception_readiness`；PR #16、#20 及本次闭环 | 通过；授权证据只在匹配且未阻塞的 VERIFY_IDENTITY 会话映射为 `AUTHORIZED`、`UNAUTHORIZED`、`EXECUTION_ERROR`，正式安装面无 Fake/placeholder |
| 感知内部 interfaces | `AuthorizationEvidence.msg`、`CapabilityStatus.msg`、`TrackedTargetImage.msg` | 通过；与公共 `dog_patrol_interfaces` 分离，可生成 C++/Python 类型 |
| mission 公共合同 | `mission_contract_integration.md` 和真实 `mission_supervisor` 集成测试；PR #19 | 通过；覆盖确认、fresh bbox、丢失/重获和错误输入门禁，tracking 不发布授权结论 |
| standalone 与 observation | `PrimaryTargetObservation`、standalone launch 和 runtime strategy；PR #21 | 通过；不创建 mission adapter，不要求导航/激光雷达，不伪造公共任务状态 |
| crop transport | `TargetImageRosAdapter` 和 `TrackedTargetImage` transport/smoke；PR #22 | 通过；同帧、自持有、可配置、有界异步丢旧，失效后停发，慢消费者不反压 |
| capability readiness | tracking capability publisher、orchestrator readiness node 和 test-only provider；PR #20 | 通过；tracking/face/voice 全部匹配当前 STARTUP 才至多发布一次整体 READY |
| 部署 requirements | `src/perception/requirements.md`、统一环境检查器；PR #23 | 通过；模块状态、精确平台/SDK、资产、参数、full-runtime 和 PASS/FAIL 入口单一且可执行 |
| 当前感知 Orin/Hik | `issue14_tracking_hardware_acceptance.md`；PR #24 | 通过；full-runtime、真实 1280×1024@30 FPS、真人 semantic primary、crop、离场停发、资源与慢消费者均有现场证据 |
| 权威入口与归档 | `issue15_authoritative_entry_archive.md`；PR #25–#27 | 通过；主仓为唯一入口，旧仓 `archived=true`，默认/冻结分支锚点仍可读 |

Problem/Solution 中列出的仓库漂移、demo/Fake 命名、readiness 所有权、跨语言 crop seam、双运行模式、
portable CI、部署门禁和两台 Orin 验收分工均由上表闭合。Implementation Decisions 与 Testing Decisions
未发现代码或稳定文档冲突；公共消息枚举和状态机业务流程未因迁移改变，模型、engine、录像、隐私数据、
凭据及本机部署状态未进入 Git。

## 子票和远端闭环

| 子票 | 合并或关闭证据 | 状态 |
| --- | --- | --- |
| #4 基线冻结审计 | dog_patrol PR #17，merge `5a3110f` | closed |
| #5 portable/full runtime 拆分 | 旧仓 PR #95，merge `b6d57fb`；随后由主仓过滤历史固定 | closed |
| #6 正式 package 改名 | 旧仓 PR #96，merge `7878d70`；随后由主仓过滤历史固定 | closed |
| #7 Fake 退役/orchestrator | dog_patrol PR #16，merge `b8d72e2` | closed |
| #8 保留历史导入/CI | dog_patrol PR #18，merge `4f442b2` | closed |
| #9 mission 合同 | dog_patrol PR #19，merge `6846c21` | closed |
| #10 readiness | dog_patrol PR #20，merge `b71a9ba` | closed |
| #11 standalone observation | dog_patrol PR #21，merge `290c6bf` | closed |
| #12 crop transport | dog_patrol PR #22，merge `d75023d` | closed |
| #13 requirements/checker | dog_patrol PR #23，merge `bc145b5` | closed |
| #14 Orin/Hik 验收 | dog_patrol PR #24，merge `c91996e` | closed |
| #15 权威入口/归档 | dog_patrol PR #25–#27，merge `fd23749`、`c60ea3e`、`19c4b65` | closed |

上述 dog_patrol PR 的 `build-and-test` 均为 `SUCCESS`。旧仓已归档，REST API 返回
`archived=true`；默认分支为 `599bdfc`，冻结分支 `deploy/dog_patrol-integration` 为 `4f3df15`。

## 本次独立复验

- 当前复验基点：`main` / `origin/main` 均为 `19c4b659d4af7a9efe3c26f3368406ac26959990`。
- 独立 `/tmp` build/install/log、未 source 旧视觉工作区：portable 五包 build 通过；400 tests，
  0 errors、0 failures、0 skipped。
- 感知环境检查器 13 项 Python 单测通过；覆盖默认 light 空模型路径、ONNX alias 与实际加载失败门禁、
  模块状态事实源及 PASS/FAIL 行为。
- orchestrator 的真实 DDS 外部-interface 测试覆盖旧 sequence、错误 target、早于当前会话的结果时间、
  两轮未通过、通过、技术错误、取消、阻塞和重复 evidence；只产生 `UNAUTHORIZED`、`AUTHORIZED`、
  `EXECUTION_ERROR` 三个预期事件。
- 仓库扫描未发现 submodule、被提交的模型/engine/录像/rosbag，活动代码与正式运行面未发现
  `vision_demo_host`、`vision_demo_node`、Fake provider 或 authorization placeholder。

## 明确保留的后续状态

- face：`not-integrated`；真实识别、模型、白名单、阈值策略和 evidence producer 待后续问题。
- voice：`not-integrated`；真实识别、音频设备、口令流程、模型和 evidence producer 待后续问题。
- orchestrator：`integrating`；不得在 face/voice provider 缺失时发布生产整体 READY。
- navigation：实现尚未迁入；最终导航 Orin 的精确环境、雷达/TF/外参、资源竞争、完整任务和长稳验收待后续问题。

这些项目是 #3 明确的 Out of Scope 或既定后续集成边界，不是 detection/tracking 接入未完成，也未被本次
错误标记为完成。
