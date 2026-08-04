# worklog

## 2026-08-04 14:40 - 贯通感知 capability readiness

- 目标：完成 dog_patrol #10，将 tracking 自身 readiness 与感知整体 READY 的所有权拆开并贯通。
- 完成：新增感知内部 `dog_patrol_perception_interfaces/CapabilityStatus`；tracking 从真实 detector/tracker 初始化和运行状态发布带 STARTUP sequence 的 detection/tracking 状态，移除 authorization placeholder 和整体 READY 聚合；orchestrator 新增纯 Python 三能力聚合核心及 `perception_readiness` ROS 节点，将 detection/tracking、face、voice 固定为 required capability，并仅在三者对当前 STARTUP sequence 全部 ready 时发布一次感知 READY；CI 和公共 mission integration 已纳入新包与真实 orchestrator，测试 adapter 提供 face/voice 状态。
- 关键结论：capability transport 使用 reliable + transient-local QoS；每个 provider 保留自身当前状态，晚启动 orchestrator 可恢复三者状态。face/voice 尚无生产 provider 时不会产生状态，因此不会通过 placeholder 假装整体 ready。tracking 的 TARGET_CONFIRMED、bbox、TARGET_LOST 和 TARGET_REACQUIRED 路径未改变。
- 涉及文件：`src/perception/dog_patrol_perception_interfaces/`、`src/perception/dog_patrol_perception_orchestrator/`、`src/perception/dog_patrol_perception_tracking/`、`.github/workflows/ci.yml`、`README.md`、`src/perception/README.md`、`worklog.md`。
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
