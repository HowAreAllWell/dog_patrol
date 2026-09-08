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

## 注册白名单人员

白名单注册使用与生产 provider 完全相同的 face detector、五点对齐和 SFace TensorRT recognition
模型，不需要重新训练。每个身份保存为 `whitelist/<name>/*.npy` 多模板目录；模板文件名携带人脸
像素尺寸，运行时按当前 `size_sigma` 对余弦相似度做尺度加权后取该身份的最高模板分数。

先停止感知任务，准备只包含一个人的清晰图片、视频或图片目录。使用统一虚拟环境执行：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
source /mnt/nvme/venv/m20_nav/bin/activate

python3 -m dog_patrol_perception_face.enrollment \
  --name person_001 \
  --source ~/person_001.mp4 \
  --report /tmp/person_001_enrollment.json
```

工具默认从 `DOG_PATROL_ASSETS_ROOT` 读取资产；未设置时使用当前工作区的
`dog_patrol_perception_assets_20260813`。也可通过 `--assets-root`、`--config`、
`--detector-engine`、`--recognition-engine` 和 `--whitelist-dir` 显式覆盖。

视频默认每 5 帧检查一次，要求每个候选画面只有一张脸，并检查置信度、人脸尺寸、亮度和清晰度；
从合格候选中按人脸像素尺度分布选择 12 个模板，另留 6 帧不参与注册。保留帧会同时与所有现有
身份竞争，必须全部匹配到新身份且超过 `similarity_threshold`，否则不写白名单。单张图片可以注册，
但没有独立保留帧验证，可靠性低于视频或多图目录。

同名身份默认拒绝覆盖。确认更新已有人员时使用：

```bash
python3 -m dog_patrol_perception_face.enrollment \
  --name person_001 \
  --source ~/person_001_new.mp4 \
  --replace
```

旧模板会移动到白名单同级的 `whitelist_backups/<name>/<timestamp>/`，staging 目录也位于白名单
目录外，避免 readiness/provider 在写入中途加载半套模板。注册完成后重新启动感知，使 face readiness
和 provider 重新加载白名单。`.npy` 是生物特征数据，不应提交到公开仓库。

`face_metrics` 定期及退出时记录输入、队列丢弃、过期丢弃、窗口和推理耗时；最终整流程脚本同时保存
CPU/GPU/RAM、温度、阶段耗时和 tracking preview 指标。更详细的模块边界见
[`COLLABORATION.md`](COLLABORATION.md)。
