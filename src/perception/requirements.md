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
| voice | `not-integrated` | 实现、语音 SDK/模型、音频设备和生产 provider 尚未进入本仓 | 当前不能发布生产 `voice` READY |
| orchestrator | `integrating` | ROS 2 Humble；已有 readiness 聚合和 ROS-independent 授权规则 | tracking/face/voice 对当前 STARTUP sequence 都 ready 才能发布感知整体 READY；真实 face/voice 结果 adapter 尚未接入 |

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

face 和 voice 的模型、隐私数据与设备需求要在实现接入时由各自模块补充；在此之前不伪造占位资产。

## 统一环境检查

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
