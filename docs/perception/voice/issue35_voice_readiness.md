# Issue #35：voice readiness 与部署入口

## 入口和状态合同

`dog_patrol_perception_voice` 提供两个相互独立的 ROS executable：

- `perception_voice_provider` 在 `VERIFY_IDENTITY` 中运行两轮任务级语音验证，发布
  `AuthorizationEvidence`；
- `perception_voice_readiness` 订阅 transient-local `MissionState`，只对有效 STARTUP sequence
  做一次 preflight，并向 transient-local `/perception/capability_status` 发布 capability `voice`。

每条 readiness 的 `observed_startup_state_seq` 必须等于触发它的 STARTUP sequence。状态映射为：

| 结果 | `CapabilityStatus` | 含义 |
| --- | --- | --- |
| 所有门禁通过 | `READY` | 当前安装、模型、设备和播放路径可供 voice provider 使用 |
| 部署输入或外部设备缺失 | `NOT_READY` | 需要部署机补齐模型、helper、ADB 或音频设备 |
| 配置、模型加载或 helper 校验失败 | `ERROR` | 当前资产/运行时不可信，需要排错或更换 |

旧 sequence 的 retained status 不会满足新的 STARTUP；非 STARTUP 和 `state_seq == 0` 不触发发布。
`ReadinessCoordinator` 仍负责整体 `detection_tracking + face + voice` 聚合，voice 节点不发布整体
`MissionEvent`。

## 只读 preflight 边界

preflight 从安装产物读取默认 config 和 ARM64 helper，加载部署机提供的 Vosk model，并检查：

- Vosk runtime/model 是否可加载；
- helper 是否存在、可执行且 SHA-256 为
  `c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52`；
- ADB `get-state` 是否返回 `device`；
- FFmpeg 是否包含 flite filter；
- `aplay -L` 是否枚举配置的播放设备，`amixer scontrols` 是否枚举配置的 mixer control。

这些操作不创建 `R818VoiceAdapter` task，不执行 ADB push/shell，不停止 `vtn_init`/demo，不接管
AC107，不写 R818 运行数据，不修改 mixer，也不播放 Prompt。通过 preflight 不代表真实语音识别效果、
R818 恢复、现场设备接管或 FAR/FRR 已验收。

## 安装后启动

推荐从安装产物启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dog_patrol_perception_voice voice.launch.py \
  model_dir:=/absolute/path/to/vosk-model \
  config_file:=/absolute/path/to/voice.yaml
```

统一部署检查还需要显式传入同一个 `--voice-model-dir` 和可选的 voice config/helper 路径；命令见
[`src/perception/requirements.md`](../../../src/perception/requirements.md)。它同时核对 voice package
的安装 marker、两个 executable、完整 ROS build/test 结果和上述 voice 运行时门禁。

模型、PCM、录音、白名单、凭据和现场音频配置不进入仓库；不应通过源仓虚拟环境或源仓模型目录提供
隐式运行依赖。
