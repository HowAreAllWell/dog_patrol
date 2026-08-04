# Issue #14 tracking 硬件验收（待补现场目标）

## 结论

2026-08-04 在当前感知 AGX Orin 和主仓 `fix/issue-14-tracking-hardware-acceptance`
工作树完成了平台、full-runtime build/test、真实 Hik 采集、standalone 隔离、暗场 live 吞吐和
历史 Hik 录像的 detector/tracker/semantic identity 补充回归。**本票尚未通过验收**：60 秒 baseline
时相机视场为无人暗场，2026-08-04 18:15 复查时已有照明但仍无人；真实 live run 始终为
`det=0/tracks=0/IDLE`，因此不能据此声称完成真实目标、
`TrackedTargetImage` 内容、目标离场停止发布、正常/慢消费者资源对照或队列 drop 的硬件验收。

模型、录像、原始日志、抓帧和带相机序列号的部署参数只保存在本机忽略目录，不提交 Git。

## 平台与资产

- Jetson AGX Orin Developer Kit，aarch64；JetPack 6.2.1、L4T R36.4.7。
- CUDA 12.6、TensorRT 10.3、ROS 2 Humble、MVS SDK `0x04080003`。
- `nvpmodel` 为 MAXN。旧 OSNet baseline 采样时 CPU/GPU 曾由现场锁频；18:24 重跑默认 light
  baseline 时 CPU policy 为 729,600–2,201,600 kHz 动态调频，当前账号没有免密 sudo，不能在本轮
  恢复 `jetson_clocks`。后续三轮 crop 资源对照必须使用同一明确锁频状态，不得把两轮资源值直接横比。
- Hikrobot `MV-CU013-A0UC`，USB ID `2bdf:0001`，SDK 枚举到唯一设备；序列号不公开，
  以 `sha256[:12]=ca65abbb68b2` 标识。
- 相机模式：`BayerGB8` (`0x0108000a`)、1280×1024@30 FPS、payload 1,310,720 bytes，
  `balanced` Bayer conversion、smoothing=false，输出自持有 BGR8。
- YOLO26n FP16 engine：8,365,176 bytes，SHA-256
  `92477cc6dceb1d8c737646469e07bebd2f387ae7ea050f93cd012781dcccb8a8`。
- 本次 production live 使用仓库默认 `tracker.reid_backend=light` 和
  `sid.reid_backend=light`，不加载 ReID ONNX。现场参数曾为两个空闲的 model path 填入同机
  单文件 ONNX，但没有把 backend 改为 `osnet_onnx`；统一检查器此前无条件加载该 ONNX 属于 #13
  回归，现已改为按对应 backend 条件检查。该 ONNX 检查不能写成 live OSNet 验收证据。

## 已完成证据

### 构建、环境和测试

- Release full Orin runtime 构建成功，CMake cache 为
  `TRACKING_ENABLE_ORIN_RUNTIME=ON`，安装产物包含 production live 节点和硬件工具。
- 新增 `validate_tensorrt_engine`，以 production `PreprocessInfer` 的同一反序列化、YOLO output
  binding 校验、execution context 和 CUDA buffer/stream 分配路径加载 engine。统一环境检查最终为
  `PERCEPTION ENVIRONMENT: PASS`。这避免 `trtexec` 在读取 plan 前初始化无关标准插件并于本机
  `LLVM ERROR: out of memory` 退出造成的误报；production 节点和新 validator 均成功加载同一 engine。
- 五包 full-runtime 测试汇总：434 tests，0 errors、0 failures、0 skipped；另有统一检查器
  13 项 Python 单测通过。包含 target-image adapter 的有界队列/失效门禁、故意变慢的 publish
  fixture、ROS transport smoke 和真实 mission supervisor integration，但这些确定性测试不能替代
  本票要求的 live crop 资源测量。

### 真实相机和 standalone

- 2 秒 clean FFV1 capture：captured/written/dropped/write-error/camera-gap=
  `61/61/0/0/0`；`ffprobe` 为 FFV1、bgr0、1280×1024、30000/1001、61 decoded frames。
- 修正 #13 门禁后重新完成 60 秒 standalone live：effective config 明确为 tracker/SID
  `reid_backend=light` 且 model path 为空；稳态 57 个 FPS 样本均值 30.023，范围
  29.73–30.45 FPS；最终 1,745 frames，acquisition failures=0、estimated drops=4、
  non-contiguous=2、MVS lost packets=0。旧 `standalone_baseline_60s.log` 是人为覆盖
  `osnet_onnx` 的历史运行，不作为默认 production baseline。
- 默认 light run 最终 acquisition p50/p95/p99=`7.963/12.510/13.935 ms`，conversion=
  `10.023/10.246/10.373 ms`，copy=`0.622/0.684/0.745 ms`，inference total=
  `13.673/20.702/22.693 ms`。
- 默认 light run 的 60 个 tegrastats 样本：RAM 平均/最大 3,612/3,625 MiB；12 核总容量口径
  CPU 平均 19.87%，单核峰值 65%；GPU 平均/最高 50.88%/97%；CPU/GPU 最高
  50.22/45.38°C。该轮为动态调频且视场无人，只证明默认 light 的真实相机无 crop 吞吐，不能替代
  锁频后的三轮含人 crop 资源对照。
- ROS graph 中只有 `/dog_patrol_perception_tracking_node`；它不订阅 mission state，只发布
  `/perception/tracked_target_image`（加 ROS 参数/日志基础 topic）。进程表无 mission supervisor、
  navigation 或 lidar，证明 standalone 启动不依赖这些组件。

### 补充回归（不作为 live crop 替代）

当前主仓安装产物回放同机历史 Hik H.264 的 539 帧：det-positive/tracks-positive 均为 539/539，
LOCKED/OCCLUDED 为 358/177，primary switch=0。该结果证明迁入后的 detector、tracker 和 semantic
identity 能处理已有真人场景，但输入不是当前实时相机，且 offline path 不创建
`TargetImageRosAdapter`，所以不关闭下方现场缺口。

## 必须补齐的现场验收

需要用户提供一个可控现场时段：打开照明，让至少一名真人进入当前 Hik 视场，保持可跟踪后完整离场。
在同一次 current-main standalone run 中必须保存：

1. 真人进入后的 detector、track、semantic primary 和 `observation_current=true` live 日志。
2. 独立 subscriber 收到的 `TrackedTargetImage`：非空 `target_id`、源时间/帧号、原图 bbox/尺寸、
   `bgr8`、crop 行列和 byte 数一致；抽样 crop 视觉上对应 bbox 中同一人。
3. 真人完全离场并超过当前目标失效门禁后，topic 停止新增旧目标消息。
4. 使用 production 默认 light backend 完成无消费者、正常消费者和故意慢消费者三轮同长度对照；分别记录 tracking FPS、topic hz/bw、
   CPU/GPU/RAM、温度/功耗和 `target_image_metrics`，确认慢消费者不反向阻塞并记录实际 queue drop。

在这些证据齐全前，#14 应保持打开，顶层 requirements 不应写入“tracking 硬件验收通过”。
