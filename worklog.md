# worklog

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
