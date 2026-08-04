# worklog

## 2026-08-04 13:48 - 保留历史导入 tracking 并接入 CI

- 目标：完成 dog_patrol #8，将准备完成的 tracking package 及必要历史正式导入主仓，并保证普通环境可独立构建测试。
- 完成：按 #4 白名单过滤并导入 `dog_patrol_perception_tracking` 的 116 个相关历史提交；package 落入感知域，稳定文档和工具随迁；从整个导入历史排除退役路径、资产和本机状态，并匿名化私网、RTSP、个人路径及设备序列号；补齐 Apache-2.0 全文、组件范围和来源锚点；默认配置不再硬编码本机资产；CI 显式关闭 Orin runtime 并将 tracking 加入必跑 build/test。
- 关键结论：来源锚点为 `vision_demo_ws` 的 `7878d70e6d86ad2a283911f8719345171b1c1d2a`，过滤后 tip 为 `6faaed42bf0531239b0203885607a9ff318eedc7`；主仓通过 merge parent 保留相关演进，SHA 因路径与内容清洗而变化。tracking 保持 Apache-2.0，主仓其余未另行标注内容继续使用 BSD-3-Clause。portable 核心无需旧视觉仓库、旧 overlay 或 Orin SDK。
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
