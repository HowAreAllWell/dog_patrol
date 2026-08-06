# 感知模块

感知团队的实现入口。tracking 和可移植 voice 核心已正式迁入本仓；人脸实现尚未建立，voice
的真实 provider/evidence producer 尚未接入。
整个感知域的 Orin 平台、SDK、资产、参数、模块状态和统一环境检查见
[`requirements.md`](requirements.md)。该入口不替代各 ROS package manifest 或模块内部配置。

`dog_patrol_perception_tracking` 提供相机、检测、tracking、semantic identity、主目标选择、
mission ROS 2 adapter，以及录制和离线评估工具。普通开发和 CI 显式关闭 Orin runtime，
只构建可移植核心；CUDA、TensorRT、Hik MVS 和 FFmpeg runtime 由 Orin 部署显式开启。

`dog_patrol_perception_voice` 提供按需调用的 R818/Vosk 任务级核心：一个 task session 只接管和恢复
一次 R818，Prompt 期间丢弃音频，最多运行两个独立响应窗；它不提供常驻监听、落盘 PCM 或通用 CLI。
生产 Adapter、安装资产和迁入边界见 [`dog_patrol_perception_voice/README.md`](dog_patrol_perception_voice/README.md)
以及 [`../../docs/perception/voice/issue33_voice_import.md`](../../docs/perception/voice/issue33_voice_import.md)。

`dog_patrol_perception_interfaces` 是感知团队内部 ROS 2 interface package，当前提供：

- `CapabilityStatus`：表达 capability 名称、ready/not-ready/error、诊断信息和关联的
  `STARTUP state_seq`；
- `TrackedTargetImage`：向独立人脸进程交付同帧 semantic 主目标 crop；
- `AuthorizationEvidence`：将 provider 的最终证据与 `state_seq + target_id` 会话关联。

Capability topic 使用 reliable + transient-local QoS，使晚启动的 orchestrator 可以获得各真实
provider 保留的当前状态。授权证据是离散、reliable、volatile 事件，不在节点重启后重放旧结果。

`dog_patrol_perception_orchestrator` 提供 ROS-independent 的感知业务编排和 readiness ROS adapter：

- `AuthorizationCoordinator`：维护与 `state_seq + target_id` 绑定的授权会话；
- 两轮未通过、立即通过、技术错误、取消和旧会话结果拒绝规则；
- 不依赖 ROS 2、具体人脸算法或语音算法的纯 Python 测试面。
- `perception_authorization` 节点：只在未阻塞的 `VERIFY_IDENTITY` 会话接受匹配证据，将最终结果
  映射为 `AUTHORIZED`、`UNAUTHORIZED` 或 `EXECUTION_ERROR`；`CANCELLED` 不发布任务事件。
- `ReadinessCoordinator`：将 `detection_tracking`、`face`、`voice` 固定为 required capability；
- `perception_readiness` 节点：只在三者状态都匹配当前 STARTUP sequence 时发布一次
  `SOURCE_PERCEPTION/READY`。

tracking 只发布自身 `detection_tracking` 状态，不再聚合或发布整体 READY。人脸和 voice 当前没有
生产 evidence provider，因此保持 not-ready；测试通过独立 adapter publisher 注入它们的状态。与主状态机
交互只使用 `dog_patrol_interfaces`，感知内部 capability transport 使用
`dog_patrol_perception_interfaces`。

## 构建和测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_perception_interfaces \
  dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  dog_patrol_perception_voice \
  dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=OFF
source install/setup.bash
colcon test --packages-select \
  dog_patrol_interfaces dog_patrol_perception_interfaces \
  dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  dog_patrol_perception_voice \
  dog_patrol_perception_tracking \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 授权编排 Interface

`AuthorizationCoordinator` 是纯 Python Module，不依赖 ROS 2 或具体算法。调用方通过
`start(AuthorizationSession)` 启动当前 `state_seq + target_id` 会话，再通过
`record(session, result)` 提交 `PASSED`、`NOT_PASSED`、`ERROR` 或 `CANCELLED`。

- `PASSED`、`ERROR` 和 `CANCELLED` 立即结束当前会话；
- 第一次有效 `NOT_PASSED` 返回 `None`，第二次返回最终未通过结果；
- 非当前 `state_seq + target_id` 的旧结果返回 `None`，不能影响新会话；
- `perception_authorization` ROS adapter 订阅内部 `AuthorizationEvidence`，并将完成结果映射为公共
  任务事件；该职责不下沉到人脸或语音算法。真实 provider 尚未接入，因此生产环境当前不会产生
  evidence 或授权事件；测试 provider 只存在于测试代码，不安装。
