# dog_patrol_perception_face

人脸验证算法在主仓中的正式接入目录，包含 YOLO/TensorRT 检测、SFace 或 MobileFaceNet 识别、白名单匹配、
`perception_face_provider`、`perception_face_readiness`、统一 launch、配置和测试。

核心输入是 tracking 发布的
`dog_patrol_perception_interfaces/msg/TrackedTargetImage`，默认 topic 为
`/perception/tracked_target_image`；算法不得自行读取相机或重新选择主目标。
MissionState 只建立、替换或取消授权会话；`perception_face_provider` 只在收到匹配的
`AuthorizationCommand` 后处理当前 `state_seq + target_id` 的最新 crop，并把一次窗口结果连同 stage
发布到 `/perception/authorization_evidence`（`provider=face`）。输入队列始终丢旧保新，推理结束时还会
拒绝过期、被新 crop 替代或来自旧命令的结果。

只有 `TASK_RUNNING + VERIFY_IDENTITY + target_id>0` 才能建立人脸会话。READY（包括 PAUSE、
RETURN、TERMINATE 后的 READY）或其他非 VERIFY 状态会取消会话；取消可记录 `CANCELLED`，
不会发布 `UNAUTHORIZED`。旧 `state_seq`、target、stage 和 provider evidence 会被丢弃。

`perception_face_readiness` 从与 provider 相同的 YAML 读取 engine 和白名单路径；它实际加载两个
TensorRT engine、分别执行一次最小推理并读取白名单后才发布 `face=READY`。显式 ROS 参数可覆盖
YAML 中的 `detector_engine`、`recognition_engine` 和 `whitelist_dir`。
识别后端由 `recognition_model` 和 `recognition_embedding_dim` 明确声明，运行时不根据文件名猜测
预处理。`sface` 使用既有 raw-BGR 输入和 128 维特征；`mobilefacenet_arcface` 使用 RGB、
`pixel / 127.5 - 1`、供应方 ArcFace 五点模板和 512 维特征。模型、维度与 `alignment_profile`
必须成套匹配；readiness 会校验 engine 输出与白名单维度，任何不一致均拒绝 READY。

readiness 只由当前 `MissionState.STARTUP` 触发。离开 STARTUP 后，迟到 preflight 或旧 STARTUP
快照不能再次发布 `face=READY`；新的 readiness 必须等待 supervisor reset 后的新序号。
provider 和 readiness 默认使用 `state_timeout=0.75 s`；MissionState heartbeat 超时会取消当前
人脸会话并使在途 preflight 结果失效。

人脸 bbox 通过 best-effort、keep-last(1) 的 `FaceOverlay` 发布，消息保留原 crop 的 source stamp、
frame number 和 target id。tracking 只在这些来源元数据仍与当前 canvas 匹配且不超过 0.5 秒时，在
既有 `VisualizerRecorder` 上绘制 bbox；人脸节点不创建第二个窗口或相机 reader。

`config/face.yaml` 是生产MobileFaceNet默认配置。部署时填入三个资产路径后启动：

```bash
ros2 launch dog_patrol_perception_face face.launch.py \
  config_file:=/absolute/path/to/face.yaml
```

## 注册白名单人员

白名单注册使用与生产 provider 完全相同的 face detector、五点对齐和 TensorRT recognition
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

## MobileFaceNet 接入

供应方 ZIP 中的 `w600k_mbf.onnx` 是 InsightFace `buffalo_sc` 原始 MobileFaceNet。先在目标
Orin 上构建与本机 TensorRT 版本匹配的 FP16 engine：

```bash
cd /mnt/nvme/workspace/dog_patrol
bash src/perception/dog_patrol_perception_face/scripts/build_mobilefacenet_engine.sh \
  /home/orin/迁移orin_MobileFaceNet_原始版.zip
```

脚本同时从供应方 ZIP 提取 `face_detection_yunet_2023mar.onnx`。YuNet 通过 OpenCV DNN
直接加载，不需要转换成 TensorRT engine。

新模型不能读取旧 SFace 白名单。为每个授权人员准备单人图片或视频，并使用供应方兼容注册命令：

相机驱动发布 `/left_camera/image_raw` 时，先保持原始1280x1024比例录制30秒MP4：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dog_patrol_perception_face perception_face_record_enrollment \
  --topic /left_camera/image_raw \
  --output "$HOME/video/person_001.mp4" \
  --fps 10 \
  --duration 30
```

录制期间只允许注册人员一人出现在画面中，缓慢左右转头、轻微抬头低头，并覆盖近、中、远距离；
不要裁成正方形或把1280x1024拉伸成其他比例。MP4通过FFmpeg使用通用H.264编码；如需无损
原始质量，输出文件改为 `.avi` 即会使用FFV1。按 `Ctrl-C` 可以提前结束，已有文件默认不会覆盖。

录制完成后执行注册：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source /mnt/nvme/venv/m20_nav/bin/activate
python3 -m dog_patrol_perception_face.supplier_enrollment \
  --name person_001 \
  --source "$HOME/video/person_001.mp4"
```

该命令保留供应方 `manage_whitelist_trt.py` 的注册算法：YuNet 人脸框和五点、ArcFace 112
对齐、亮度和清晰度过滤、按清晰度取最多 80 帧，然后把归一化特征求均值。每个人最终只有一个
中心特征。存储层改为 provider 可直接读取的
`face/whitelist_mobilefacenet/<name>/g0_0px_f0.npy`；同名人员会先备份旧中心特征再更新。
供应方原始的单文件 `gallery_mobilefacenet_original.npz` 不会生成，也不能直接交给 provider。

所有人员注册完成后直接启动完整感知；生产bringup默认选择MobileFaceNet engine和新白名单：

```bash
ros2 launch dog_patrol_perception_bringup perception_stack.launch.py \
  assets_root:=/mnt/nvme/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813
```

新白名单为空时face readiness会保持NOT_READY，不会退回旧SFace或无白名单运行。确需诊断旧模型时，
必须显式指定 `face_sface.yaml`、`recognition.engine` 和 `whitelist`。

配置中的 `0.3274939358` 仅是供应方 9-5 协议下 FAR 不高于 1% 的参考值。上线前必须使用现场
摄像头、距离和光照数据重新标定 `similarity_threshold`；本接入默认关闭旧的脸框尺寸相似度加权，
避免把模型余弦分数与采集尺度人为相乘。

`face_metrics` 定期及退出时记录输入、队列丢弃、过期丢弃、窗口和推理耗时；最终整流程脚本同时保存
CPU/GPU/RAM、温度、阶段耗时和 tracking preview 指标。更详细的模块边界见
[`COLLABORATION.md`](COLLABORATION.md)。
