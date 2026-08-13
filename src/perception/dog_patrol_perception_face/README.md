# dog_patrol_perception_face

人脸验证算法在主仓中的正式接入目录，包含 YOLO/TensorRT 检测、sface 识别、白名单匹配、
`perception_face_provider`、`perception_face_readiness`、统一 launch、配置和测试。

核心输入是 tracking 发布的
`dog_patrol_perception_interfaces/msg/TrackedTargetImage`，默认 topic 为
`/perception/tracked_target_image`；算法不得自行读取相机或重新选择主目标。
MissionState 只建立、替换或取消授权会话；`perception_face_provider` 只在收到匹配的
`AuthorizationCommand` 后处理当前 `state_seq + target_id` 的最新 crop，并把一次窗口结果连同 stage
发布到 `/perception/authorization_evidence`（`provider=face`）。输入队列始终丢旧保新，推理结束时还会
拒绝过期、被新 crop 替代或来自旧命令的结果。

`perception_face_readiness` 从与 provider 相同的 YAML 读取 engine 和白名单路径；它实际加载两个
TensorRT engine、分别执行一次最小推理并读取白名单后才发布 `face=READY`。显式 ROS 参数可覆盖
YAML 中的 `detector_engine`、`recognition_engine` 和 `whitelist_dir`。

人脸 bbox 通过 best-effort、keep-last(1) 的 `FaceOverlay` 发布，消息保留原 crop 的 source stamp、
frame number 和 target id。tracking 只在这些来源元数据仍与当前 canvas 匹配且不超过 0.5 秒时，在
既有 `VisualizerRecorder` 上绘制 bbox；人脸节点不创建第二个窗口或相机 reader。

部署时复制 `config/face.yaml`，填入三个资产路径后启动：

```bash
ros2 launch dog_patrol_perception_face face.launch.py \
  config_file:=/absolute/path/to/face.yaml
```

`face_metrics` 定期及退出时记录输入、队列丢弃、过期丢弃、窗口和推理耗时；最终整流程脚本同时保存
CPU/GPU/RAM、温度、阶段耗时和 tracking preview 指标。更详细的模块边界见
[`COLLABORATION.md`](COLLABORATION.md)。
