# Issue #14 tracking 硬件验收

## 结论

2026-08-04 在当前感知 AGX Orin 和主仓 `fix/issue-14-tracking-hardware-acceptance`
工作树完成了平台、full-runtime build/test、真实 Hik 采集、standalone 隔离、真人 live
detector/tracker/semantic identity、`TrackedTargetImage` 内容与离场门禁，以及三轮锁频资源对照。
Issue #14 的 tracking 硬件验收通过。

模型、录像、原始日志、抓帧和带相机序列号的部署参数只保存在本机忽略目录，不提交 Git。

## 平台与资产

- Jetson AGX Orin Developer Kit，aarch64；JetPack 6.2.1、L4T R36.4.7。
- CUDA 12.6、TensorRT 10.3、ROS 2 Humble、MVS SDK `0x04080003`。
- 三轮正式对照前统一核验 `nvpmodel=MAXN`；三个 CPU policy 的 min/max/current 均为
  2,201,600 kHz，GPU min/max/current 均为 1,300,500,000 Hz。当前账号不能运行 root-only 的
  `jetson_clocks --show`，但 sysfs 和每秒 `tegrastats` 均确认 CPU/GPU 在三轮保持上述锁频状态。
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

### 真人、crop 和离场门禁

- 真人进入后 live 节点稳定为 `det=1/filtered=1/tracks=1`、`LOCKED`、semantic
  `primary_id=1` 和 `observation_current=true`；默认 tracker/SID backend 均为 `light`。
- 独立 best-effort subscriber 在生命周期、正常消费者和慢消费者轮分别收到 104、490 和 109 条
  `TrackedTargetImage`，合计 703 条。每条都通过：正
  `target_id`、非零且严格递增的源时间、可用且严格递增的源帧号、
  `hik_camera_optical_frame`、1280×1024 原图、正且不越界的 bbox、`bgr8`、
  `crop_step=crop_width*3`、`crop_bytes=crop_height*crop_step`，以及 crop 覆盖 bbox 且不超过原图。
  三轮均为 `target_id=1`，0 项字段校验错误。production observer/adapter 的确定性测试另行覆盖
  源元数据、原图 bbox 与同帧深拷贝 crop 的映射关系。
- 跨生命周期、正常和慢消费者轮目检 8 张分布于早期/中期/后期的无损 PNG，均为同一名穿深蓝
  上衣和浅色短裤的真人，人体内容、姿态变化与连续 bbox crop 一致；图片、逐消息 SHA-256 和
  原始字节仅保存在忽略目录。
- 真人完全离场后，节点先于 18:37:08 进入 `OCCLUDED` 并立即令
  `observation_current=false`，超过 180 帧失效门禁约 6.07 秒后进入 `IDLE`、清空 primary。
  subscriber 最后一条消息为源帧 916；随后约 35 秒无新消息，节点的 submitted/published/drop/
  rate-limited 计数保持不变，未发布旧目标。

### 同长度资源和反压对照

三轮均为 60 秒、MAXN + 同一 CPU/GPU 锁频、真实 Hik 1280×1024@30 FPS、真人稳定
`LOCKED` 和默认 `light` backend。`topic hz/bw` 是三轮一致的测量探针；首轮在探针启动前保存
`Subscription count: 0`，因此“无业务消费者”没有 face/crop 业务订阅者。表内 target image
四项为各自严格 60 秒窗口的计数增量，不包含启动预热。

| 场景 | tracking FPS（稳态均值/范围） | topic hz / bw | CPU 均值 / 单核峰值 | GPU 均值/峰值 | RAM 均值/峰值 MiB | CPU/GPU 峰温 °C | GPU-SOC / CPU-CV / 5V 平均功耗 mW | target image submitted/published/drop/rate-limit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 无业务消费者 | 30.012 / 29.91–30.16 | 8.434 Hz / 9.31 MB/s | 12.43% / 67% | 26.03% / 52% | 3866.3 / 3873 | 53.34 / 48.78 | 6440 / 2683 / 5824 | 1795 / 508 / 13 / 1265 |
| 正常消费者 | 30.011 / 29.94–30.10 | 8.601 Hz / 9.22 MB/s | 13.31% / 68% | 26.20% / 55% | 3829.0 / 3836 | 53.16 / 48.28 | 6464 / 2703 / 5829 | 1798 / 524 / 19 / 1255 |
| 慢消费者（回调阻塞 500 ms） | 29.992 / 29.07–30.10 | 8.265 Hz / 9.48 MB/s | 13.21% / 62% | 26.85% / 55% | 3826.9 / 3836 | 53.28 / 48.53 | 6468 / 2715 / 5829 | 1792 / 510 / 30 / 1252 |

正常消费者 60 秒收到 490 条；故意慢消费者只收到 109 条，但独立探针仍收到约 8.27 Hz，tracking
仍为 29.992 FPS，较无业务消费者均值仅低 0.020 FPS（0.067%）。队列实际执行丢旧（13/19/30），且慢消费者
没有把 tracking 帧线程或 DDS publisher 反向阻塞。三轮 camera acquisition failure 和 MVS lost
packet 均为 0；估计 drop/non-contiguous 均为 2/2，均只发生在启动阶段。

三轮最终阶段 p50/p95/p99（ms）：

| 场景 | acquisition | Bayer conversion | BGR copy | inference total |
| --- | --- | --- | --- | --- |
| 无业务消费者 | 22.811/24.246/24.435 | 3.412/3.500/3.561 | 0.351/0.387/0.409 | 4.976/7.702/8.702 |
| 正常消费者 | 22.291/22.835/22.907 | 3.387/3.447/3.637 | 0.356/0.411/0.550 | 5.390/7.853/8.418 |
| 慢消费者 | 22.453/22.885/22.957 | 3.463/3.703/3.867 | 0.370/0.585/0.617 | 5.090/7.538/8.066 |

### 补充回归

当前主仓安装产物回放同机历史 Hik H.264 的 539 帧：det-positive/tracks-positive 均为 539/539，
LOCKED/OCCLUDED 为 358/177，primary switch=0。该结果证明迁入后的 detector、tracker 和 semantic
identity 能处理已有真人场景。该离线结果仅作回归补充，正式结论以上述当前实时相机验收为准。

## 证据边界

可提交文档只保留聚合结果、非敏感资产摘要和验证结论。现场参数、相机序列号、逐消息 JSON、crop
图片、`tegrastats`、ROS topic 输出和 live 原始日志位于本机
`log/issue14_acceptance/live_20260804_1834/`，由 `.gitignore` 排除；模型和录像同样不进入 Git。
