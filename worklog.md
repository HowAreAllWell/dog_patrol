# worklog

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
