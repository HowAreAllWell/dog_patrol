# dog_patrol

机器狗巡逻项目的主仓库。当前仓库负责主任务状态机、跨模块 ROS 2 接口和联调约定；导航与感知实现按团队目录独立演进。

## 当前状态

- `dog_patrol_interfaces`：已实现，保存主状态机、任务事件、目标框和导航状态消息。
- `dog_patrol_perception_interfaces`：已实现，保存感知内部 capability、授权证据和主目标 crop 合同。
- `dog_patrol_manager`：已实现，包含 ROS-independent 状态机和 `mission_supervisor` ROS 2 节点。
- `navigation/`：已迁入 fast_livo_dog 完整导航链，并提供 2D 导航协调器。
- `dog_patrol_perception_tracking`：已从视觉准备仓保留必要历史导入；普通环境构建
  portable tracking 核心和 ROS adapter，Orin runtime 由部署端显式开启。
- 本仓是 tracking 正式开发、构建、测试和部署的唯一权威入口；旧 `vision_demo_ws` 是非生产
  研究仓库，只有 `deploy/dog_patrol-integration` 是冻结部署基线；`main`、`dev` 和按需创建的
  `research/*` 均可继续研究。研究成果必须重新通过本仓 PR、CI 和评审进入正式实现；旧仓不是
  构建或部署依赖。
- tracking 公共 mission tracer 已接入同工作区安装后的真实 `mission_supervisor`，无资产路径纳入普通
  CI，覆盖目标确认、fresh bbox、丢失/重获及无效状态输入门禁。
- tracking 已提供不创建 mission ROS adapter 的正式 standalone Orin 启动方式，并通过
  ROS-independent `PrimaryTargetObservation` 返回当前可信语义主目标及自持有目标图像；mission 与
  standalone 均通过有界异步 ROS adapter 以 `TrackedTargetImage` 向独立人脸进程交付同帧 crop。
- `dog_patrol_perception_orchestrator`：已实现 capability readiness、分阶段授权命令与最终任务事件
  adapter。授权流程固定为“初始仅人脸；未通过后人脸与语音并行第一轮；两者均未通过后并行第二轮”；
  任一 provider 通过即授权，技术错误单独上报，只有第二轮两者均未通过才判未授权。
- `dog_patrol_perception_face`：已迁入真实算法（detector/recognition/verifier/whitelist）与
  `perception_face_provider`、真实 readiness 和 launch。模型与白名单由同一 face YAML 或 ROS 参数提供；
  provider 严格消费 tracking 最新主目标 crop，并按授权阶段发布 evidence。真实 TensorRT 模型、会话取消、
  跨会话隔离与 readiness 已在当前 Orin 验证。
- `dog_patrol_perception_voice`：已迁入 R818/Vosk 核心、真实 readiness 和异步 provider。MissionState 只负责
  会话门禁；provider 仅响应 orchestrator 发出的第一轮/第二轮并行授权命令，每条命令运行一个语音窗，
  状态替换或取消会抑制迟到结果，默认不落盘 PCM。既有硬件与真人语音验收记录保留在
  [`docs/perception/voice/`](docs/perception/voice/)；本次以整条感知流程的最终现场验收为准。
- 语音部署候选源仓 `moonshine_voice_commands` 已以 `b979a7fd33aac5c9ced9591bb507e483faf4aef5` 冻结；#33 只从该版本选择性提炼，不把源仓作为构建或运行依赖。允许/排除清单与复现证据见 [`docs/issue32_voice_deployment_baseline_audit.md`](docs/issue32_voice_deployment_baseline_audit.md)，迁入说明见 [`docs/perception/voice/issue33_voice_import.md`](docs/perception/voice/issue33_voice_import.md)。
- 感知域已提供整体部署 requirements 和统一 Orin 环境检查，显式区分 tracking、face、voice 和 orchestrator 当前状态。
- tracking 已在当前感知 Orin 完成 full-runtime build/test、真实 Hik 30 FPS、standalone 隔离、
  真人 semantic primary、主目标 crop/离场停发和慢消费者不反压验收；verified baseline 见
  [`docs/perception/tracking/issue14_tracking_hardware_acceptance.md`](docs/perception/tracking/issue14_tracking_hardware_acceptance.md)。
- detection/tracking 接入父 Spec #3 已完成独立整体验收；子票、合并、CI、现场证据与明确后续边界见
  [`docs/perception/tracking/issue3_spec_acceptance.md`](docs/perception/tracking/issue3_spec_acceptance.md)。
- 人脸 bbox 已通过轻量 `FaceOverlay` 回传，在 tracking 既有 `VisualizerRecorder` 的同一 canvas/window
  上绘制；错目标、未来帧和过期结果不显示。最终还需用户参加一次仅导航 fake 的感知整流程现场验收，
  同时记录阶段耗时、CPU/GPU/RAM、温度及 tracking/face 指标。
- 目标公开远程：`https://github.com/HowAreAllWell/dog_patrol`

## 目录

导航已经迁入 src/navigation/fast_livo_dog，完整运行边界和 app.py 的三组生命周期见
 src/navigation/README.md。感知组合启动入口位于
 src/perception/dog_patrol_perception_bringup。

```text
src/contracts/dog_patrol_interfaces/       # 两团队共同维护的 ROS 2 合同
src/orchestration/dog_patrol_manager/      # 主状态机和 supervisor
src/orchestration/robot_console/           # 系统级 Qt 总控入口
src/navigation/                            # fast_livo_dog 导航模块
src/perception/                            # 感知业务编排入口
src/perception/dog_patrol_perception_interfaces/ # 感知内部 ROS 2 合同
src/perception/dog_patrol_perception_face/  # face 算法、provider、readiness 与配置
src/perception/dog_patrol_perception_voice/ # voice 核心、异步 evidence provider 与生产 Adapter
src/perception/dog_patrol_perception_tracking/ # tracking 核心与可选 Orin runtime
docs/contracts/                             # 可评审的接口协议
docs/perception/tracking/                    # tracking 稳定说明与迁移验收证据
docs/perception/voice/                        # voice 迁入和部署边界
docs/workflows/                             # 业务流程参考文档
```

ROS 2 package 名称保持 `dog_patrol_` 前缀；上层目录是所有权和代码组织目录，不是额外的 ROS 2 package。

## 运行和验证

完整系统由 fast_livo_dog 的 app.py 管理三类生命周期：manager 在 app 启动时常驻；
2D 导航按钮管理 Nav2/move 和导航协调器；感知任务按钮独立管理 tracking、face、voice
及感知编排。导航或感知关闭时通过 /mission/reset 清除旧任务会话，manager 只在 app
退出时关闭。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_perception_interfaces \
  dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  dog_patrol_perception_face \
  dog_patrol_perception_voice \
  dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=OFF
source install/setup.bash
colcon test --packages-select \
  dog_patrol_interfaces dog_patrol_perception_interfaces \
  dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  dog_patrol_perception_face \
  dog_patrol_perception_voice \
  dog_patrol_perception_tracking \
  --event-handlers console_direct+
colcon test-result --verbose
```

模型、录制视频、相机日志、人脸白名单、特征向量和现场配置不进入公开仓库；它们必须通过本机部署或受控资产目录提供。

根目录代码默认使用 BSD-3-Clause；从视觉准备仓迁入的 tracking 组件保持
Apache-2.0。具体范围和许可证副本见 [`LICENSES/README.md`](LICENSES/README.md)。

## 核心协作约定

- `dog_patrol_interfaces` 是跨团队共享合同，不属于导航或感知任一实现目录。
- 主状态机只编排业务状态，不实现检测、跟踪、人脸或语音算法。
- 导航和感知可以通过 ROS 2 直接交换数据，但不得依赖对方的私有代码 Module。
- `TARGET_LOST` 和 `TARGET_REACQUIRED` 由感知发布；导航发现 bbox 过期时先本地停车并发布导航 `BLOCKED` 状态。
- 感知内部的授权流程只向主状态机映射最终授权、未授权或技术错误结果。

详细合同见 [`docs/contracts/perception_navigation_interface.md`](docs/contracts/perception_navigation_interface.md)，业务流程见 [`docs/workflows/机器狗巡逻与可疑目标处置流程（更新后）.docx`](docs/workflows/机器狗巡逻与可疑目标处置流程（更新后）.docx)。

感知编排模块说明见 [`src/perception/README.md`](src/perception/README.md)。
人脸算法迁入、主目标 crop、mission 会话和唯一预览规则见
[`src/perception/dog_patrol_perception_face/COLLABORATION.md`](src/perception/dog_patrol_perception_face/COLLABORATION.md)。
仅导航 fake 的最终感知整流程与性能记录入口见
[`tools/fake_integration/README.md`](tools/fake_integration/README.md)。
tracking 的 portable/Orin 构建、运行和配置说明见
[`src/perception/dog_patrol_perception_tracking/README.md`](src/perception/dog_patrol_perception_tracking/README.md)。
voice provider 的 ROS 参数、取消/会话门禁和验证边界见
[`docs/perception/voice/issue34_voice_provider.md`](docs/perception/voice/issue34_voice_provider.md)。
旧仓研究治理、追溯和回退锚点见
[`docs/perception/tracking/issue15_authoritative_entry_archive.md`](docs/perception/tracking/issue15_authoritative_entry_archive.md)。
父 Spec 的整体验收矩阵见
[`docs/perception/tracking/issue3_spec_acceptance.md`](docs/perception/tracking/issue3_spec_acceptance.md)。

## 协作方式

日常修改通过短分支和 Pull Request 合并到 `main`。模块所有权由 `.github/CODEOWNERS` 管理，合并前必须通过 CI 和对应 owner 审查。
