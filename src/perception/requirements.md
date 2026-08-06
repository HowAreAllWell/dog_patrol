# 感知域部署 requirements

本文档是 **整个感知域** 的部署入口，供感知 Orin 和导航 Orin 联调时使用。它不是
`dog_patrol_perception_tracking` 的内部依赖清单，也不替代任何 ROS package 的
`package.xml`。各 package 的 ROS 构建/运行依赖以各自 manifest 为唯一事实来源；算法默认值和
阈值继续由所属模块的配置文件拥有，本文档不复制它们。

## 部署目标和当前状态

真实部署目标只有两台：当前感知 Orin 和最终导航 Orin。本项目不引入机群、节点注册或远程资产分发抽象。

| 模块 | 状态 | 平台与部署输入 | 验收边界 |
| --- | --- | --- | --- |
| tracking | `implemented` | Jetson Orin；CUDA/TensorRT、Hikrobot MVS SDK、相机、本机 engine、ROS 参数；仅 `osnet_onnx` 后端需要 ReID ONNX | 完整 Orin build/test；相机可枚举；live 节点加载 engine 并稳定输出 tracking 指标 |
| face | `not-integrated` | 预期从 `TrackedTargetImage` 消费同帧主目标 crop；实现、模型和白名单尚未进入本仓 | 当前只验证 crop transport；不得把测试 provider 当作生产 readiness |
| voice | `implemented` | `dog_patrol_perception_voice` 的 R818/Vosk 核心、异步 evidence provider、Prompt player、安装 helper 和受控配置；设备与 Vosk 模型由部署机提供 | provider ROS 端到端 fake hardware 验证已通过；真实设备恢复、模型效果和生产 `voice` READY 仍待现场验收 |
| orchestrator | `integrating` | ROS 2 Humble；已有 readiness 聚合、ROS-independent 授权规则和通用授权事件 adapter | tracking/face/voice 对当前 STARTUP sequence 都 ready 才能发布感知整体 READY；真实 face/voice provider 尚未接入 |

环境检查中的最终 `PASS` 表示“当前已实现范围的部署前置条件完整”，不会把
`not-integrated` 模块伪装成 ready，也不等价于 mission 中的感知整体 `READY`。

## 平台与 SDK 基线

- 支持系列：JetPack 6.2 / L4T R36.4，aarch64，CUDA 12.6、TensorRT 10.3、ROS 2
  Humble 和 MVS SDK 4.8。JetPack 6.0/6.1 等其他组合尚未验收，不属于当前支持面。
- 当前感知 Orin 已核验：JetPack `6.2.1`、L4T `R36.4.7`、CUDA `12.6`、TensorRT `10.3`、ROS 2
  `Humble`、Hikrobot MVS SDK `0x04080003` (4.8.0.3)。tracking 的 JetPack 6.2.1 engine
  导出工作流已验证；机器上的精确系统事实以 L4T 版本为准，不从 marketing 版本反推。
- 最终导航 Orin：目标为同一 JetPack 6.2 / L4T R36.4 兼容面；精确 L4T、CUDA、TensorRT、ROS 2
  和 MVS 版本均为 **待现场核验**，必须用下方统一命令留存结果，不默认与感知 Orin 一致。

## 设备、资产和参数

tracking 当前唯一 live 输入是 Hikrobot MVS `MV-CU013-A0UC` USB3 Vision 相机（USB
ID `2bdf:0001`）。已验收模式为 `BayerGB8` (`0x0108000a`)、`1280x1024@30 FPS`，
转换为自持有 BGR8 帧后进入 detector。多台同型设备并存时必须配置序列号。

以下内容必须由部署机提供，不进入 Git：

- `detector.runtime_path`：YOLO26n FP16 TensorRT `.engine`。engine 必须在目标 Orin 上用
  `dog_patrol_perception_tracking/scripts/export_yolo26n_engine_orin_jp621.sh` 生成；不跨设备、
  JetPack 或 TensorRT 版本复用。
- `tracker.reid_backend` 和 `sid.reid_backend` 默认均为 `light`，此时模型路径应留空且不需要
  ReID ONNX。仅对应 backend 为 `osnet_onnx`（也包括 runtime 规范化到它的 `onnx`、`osnet`
  和 `true_reid` 别名）时，`tracker.reid_model_path` 或 `sid.reid_model_path` 才必须指向已批准的
  ReID ONNX；如该 ONNX 使用 external data，对应 `.onnx.data` 必须同时存在。
- `camera.mvs_serial`：实际相机序列号；`camera.mvs_model` 必须为上述已支持型号。
- 部署专用 ROS 参数文件和 tracker YAML。参数文件不得携带 RTSP userinfo、凭据、白名单、
  特征向量、录像或本机临时目录。

face 的模型、隐私数据与设备需求要在实现接入时由各自模块补充；voice 的 Vosk 模型、ADB serial 和
音频设备同样由部署机提供，不进入 Git。voice 的 provider 参数和验收边界见
[`../../docs/perception/voice/issue34_voice_provider.md`](../../docs/perception/voice/issue34_voice_provider.md)，
迁入边界与安装验证见 [`../../docs/perception/voice/issue33_voice_import.md`](../../docs/perception/voice/issue33_voice_import.md)。

## 统一环境检查

### Full-runtime 构建和测试

当前感知 Orin 直接从本仓根目录构建，不 source 或引用旧视觉工作区：

```bash
cd /absolute/path/to/dog_patrol
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_perception_interfaces \
  dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  dog_patrol_perception_voice \
  dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=ON
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

### 部署门禁

在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
python3 src/perception/scripts/check_perception_environment.py \
  --target perception-orin \
  --params-file /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --install-prefix /absolute/path/to/dog_patrol/install \
  --build-base /absolute/path/to/dog_patrol/build
```

最终导航 Orin 将 `--target` 改为 `navigation-orin`。命令统一检查架构/L4T、CUDA、
TensorRT、ROS 2、MVS SDK、USB 相机枚举、detector/ReID 资产、必需参数、tracker YAML、
colcon 安装结果、完整 Orin runtime 构建开关、测试结果和上表模块状态。engine 由安装后的
`validate_tensorrt_engine` 使用 production detector 的同一 TensorRT 反序列化、binding 校验和 CUDA
分配路径实际加载；只有配置选择 `osnet_onnx` 后端时，ReID ONNX 才由同一 OpenCV C++ runtime
实际加载。默认 `light` 后端和空模型路径应通过 ReID 资产门禁。全部关键检查通过时最后一行为
`PERCEPTION ENVIRONMENT: PASS`；任一关键依赖缺失时输出可操作的 `[FAIL]` 说明，最终为
`PERCEPTION ENVIRONMENT: FAIL` 并返回非零状态。

对 `navigation-orin` 的第一次成功检查只证明落在已支持系列；还需将输出中的精确版本
回填到本文档，经现场验收后才能从“待核验”改为“已核验”。

### 启动完整 tracking runtime

统一检查为 `PASS` 后，从本仓安装产物启动 production standalone tracking；它使用完整 Hik
camera、TensorRT detector、tracker、semantic identity、primary observation 和异步 crop transport，
但不要求尚未接入的 face/voice provider、mission supervisor、导航或激光雷达：

```bash
cd /absolute/path/to/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dog_patrol_perception_tracking \
  dog_patrol_perception_tracking_standalone.launch.py \
  params_file:=/absolute/path/to/orin_tracking.yaml \
  tracker_config:=/absolute/path/to/bot_sort.yaml
```

运行后以 production 节点的 `runtime_monitor` 日志、`ros2 topic hz /perception/tracked_target_image` 和
`ros2 topic bw /perception/tracked_target_image` 核验相机、推理、tracking FPS 和 crop transport。
需要 mission 模式时改为运行 `dog_patrol_perception_tracking_node`，并同时启动主仓内的 supervisor、
orchestrator 和 `perception_voice_provider`；在 face provider、voice capability readiness 和现场
部署输入均未完成前，感知整体 READY 仍应保持未就绪。

## Tracking verified baseline

2026-08-04，当前感知 AGX Orin 已在 MAXN、CPU 2.2016 GHz 和 GPU 1.3005 GHz 锁频下完成
tracking 硬件验收：production 默认 tracker/SID `light` backend，真实 Hik
`BayerGB8` 1280×1024@30 FPS，真人 detector/tracker/semantic primary 和 standalone
`TrackedTargetImage` 字段、同人 crop、离场停发均通过。无业务消费者、正常消费者和 500 ms
慢消费者三轮各 60 秒的 tracking 稳态均值分别为 30.012、30.011 和 29.992 FPS；慢消费者未反压
tracking。三轮 acquisition failure/MVS lost packet 均为 0，估计 drop/non-contiguous 均为 2/2
且仅见于启动阶段。完整阶段分位数、资源、topic 和 queue-drop 基线见
[`../../docs/perception/tracking/issue14_tracking_hardware_acceptance.md`](../../docs/perception/tracking/issue14_tracking_hardware_acceptance.md)。
