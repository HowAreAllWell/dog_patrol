# 感知模块

感知团队的实现入口。tracking、face、voice、readiness 与授权 orchestrator 均已接入；最终整体验收
只替换导航为 fake，其余感知和主状态机节点使用真实实现。
整个感知域的 Orin 平台、SDK、资产、参数、模块状态和统一环境检查见
[`requirements.md`](requirements.md)。该入口不替代各 ROS package manifest 或模块内部配置。

感知运行资源统一放在 `dog_patrol_perception_assets_20260813/`，与各感知 ROS package
同级，不依赖工作空间外的资源目录。运行时可通过 `DOG_PATROL_ASSETS_ROOT` 覆盖默认位置。

`dog_patrol_perception_tracking` 提供相机、检测、tracking、semantic identity、主目标选择、
mission ROS 2 adapter，以及录制和离线评估工具。普通开发和 CI 显式关闭 Orin runtime，
只构建可移植核心；CUDA、TensorRT、Hik MVS 和 FFmpeg runtime 由 Orin 部署显式开启。

`dog_patrol_perception_voice` 提供按需调用的 R818/Vosk 任务级核心。MissionState 只建立/取消授权会话；
`perception_voice_provider` 只响应 `DUAL_FIRST` 或 `DUAL_SECOND` 命令，每次运行一个独立响应窗并发布
带阶段的 `AuthorizationEvidence`。另有 `perception_voice_readiness` 对每个 STARTUP sequence 执行
只读部署 preflight 并发布 `CapabilityStatus`。它不提供常驻监听或落盘 PCM。生产 Adapter、provider 参数、
安装资产和迁入边界见 [`dog_patrol_perception_voice/README.md`](dog_patrol_perception_voice/README.md)、
[`../../docs/perception/voice/issue34_voice_provider.md`](../../docs/perception/voice/issue34_voice_provider.md)
以及 [`../../docs/perception/voice/issue33_voice_import.md`](../../docs/perception/voice/issue33_voice_import.md)。

`dog_patrol_perception_face` 包含 YOLO/TensorRT 检测、sface 识别、白名单匹配、生产 provider、
readiness 和 launch。它只消费 tracking 的 latest-only `TrackedTargetImage`；MissionState 负责会话门禁，
`AuthorizationCommand` 决定当前算法窗口。bbox 通过 `FaceOverlay` 返回并叠加到 tracking 的既有
`VisualizerRecorder`，不创建第二个相机或预览窗口。详见
[`dog_patrol_perception_face/README.md`](dog_patrol_perception_face/README.md)。

`dog_patrol_perception_interfaces` 是感知团队内部 ROS 2 interface package，当前提供：

- `CapabilityStatus`：表达 capability 名称、ready/not-ready/error、诊断信息和关联的
  `STARTUP state_seq`；
- `TrackedTargetImage`：向独立人脸进程交付同帧 semantic 主目标 crop；
- `AuthorizationCommand`：orchestrator 向 provider 下发初始人脸、两轮并行或取消命令；
- `AuthorizationEvidence`：将 provider 的单阶段结果与 `state_seq + target_id + stage` 关联；
- `FaceOverlay`：把人脸 bbox、匹配状态和原 crop 来源元数据送回既有 tracking canvas。

Capability topic 使用 reliable + transient-local QoS，使晚启动的 orchestrator 可以获得各真实
provider 保留的当前状态。授权证据是离散、reliable、volatile 事件，不在节点重启后重放旧结果。

`dog_patrol_perception_orchestrator` 提供 ROS-independent 的感知业务编排和 readiness ROS adapter：

- `AuthorizationCoordinator`：维护与 `state_seq + target_id` 绑定的分阶段授权会话；
- 初始仅人脸；失败后人脸/语音并行第一轮；两者均失败后并行第二轮；任一通过立即授权，任一技术错误
  结束为技术错误，只有第二轮两者均未通过才判未授权；
- 不依赖 ROS 2、具体人脸算法或语音算法的纯 Python 测试面。
- `perception_authorization` 节点：发布阶段命令，只在未阻塞的 `VERIFY_IDENTITY` 会话接受匹配 evidence，将最终结果
  映射为 `AUTHORIZED`、`UNAUTHORIZED` 或 `EXECUTION_ERROR`；`CANCELLED` 不发布任务事件。
- `ReadinessCoordinator`：将 `detection_tracking`、`face`、`voice` 固定为 required capability；
- `perception_readiness` 节点：只在三者状态都匹配当前 STARTUP sequence 时发布一次
  `SOURCE_PERCEPTION/READY`。

tracking 只发布自身 `detection_tracking` 状态，不再聚合或发布整体 READY。face/voice provider 只发布
各自 evidence，readiness 节点只发布各自 capability，因此整体 READY 仍由现有 required capability
流程决定。与主状态机
交互只使用 `dog_patrol_interfaces`，感知内部 capability transport 使用
`dog_patrol_perception_interfaces`。

## 构建和测试

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

## 授权编排 Interface

`AuthorizationCoordinator` 是纯 Python Module，不依赖 ROS 2 或具体算法。调用方通过
`observe(AuthorizationSession)` 对齐当前 `state_seq + target_id`，再通过
`record_observation(AuthorizationObservation)` 提交 provider、stage 和结果。

- `observe` 对新会话下发 `INITIAL_FACE`，替换/退出会话时先下发 `CANCEL`；
- 初始人脸未通过后下发 `DUAL_FIRST`，第一轮两者均未通过后下发 `DUAL_SECOND`；
- 任一预期 provider 通过或报错立即结束；第二轮两者均未通过才结束为未授权；
- 非当前会话、错误 stage、重复或已取消结果都被忽略，不能影响新会话；
- `perception_authorization` ROS adapter 订阅内部 `AuthorizationEvidence`，并将完成结果映射为公共
  任务事件；该职责不下沉到人脸或语音算法。
