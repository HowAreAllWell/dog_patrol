# dog_patrol_perception_face

人脸验证算法在主仓中的正式接入目录，包含 YOLO/TensorRT 检测、MobileFaceNet 识别、白名单匹配、
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
不会发布 `UNAUTHORIZED`。旧会话和错误目标的 evidence 会被丢弃；同一联合核验的人脸结果跨口令轮次有效。

`perception_face_readiness` 从与 provider 相同的 YAML 读取 engine 和白名单路径；它实际加载两个
TensorRT engine、分别执行一次最小推理并读取白名单后才发布 `face=READY`。显式 ROS 参数可覆盖
YAML 中的 `detector_engine`、`recognition_engine` 和 `whitelist_dir`。
识别契约固定为 `mobilefacenet_arcface`：输入使用 RGB、`pixel / 127.5 - 1`、供应方 ArcFace
五点模板，输出固定为 512 维特征。模型、维度和对齐方式不再由 YAML 切换；readiness 会校验
engine 输出与白名单维度，任何不一致均拒绝 READY。

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
模型，不需要重新训练。每个身份保存为 `whitelist_mobilefacenet/<name>/*.npy` 目录；模板文件名携带人脸
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

为每个授权人员准备单人图片或视频，并使用供应方兼容注册命令：

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

新白名单为空时 face readiness 会保持 `NOT_READY`，不会回退到其他识别模型或无白名单运行。

配置中的 `0.3274939358` 仅是供应方 9-5 协议下 FAR 不高于 1% 的参考值。上线前必须使用现场
摄像头、距离和光照数据重新标定 `similarity_threshold`；本接入默认关闭旧的脸框尺寸相似度加权，
避免把模型余弦分数与采集尺度人为相乘。

`face_metrics` 定期及退出时记录输入、队列丢弃、过期丢弃、窗口和推理耗时；最终整流程脚本同时保存
CPU/GPU/RAM、温度、阶段耗时和 tracking preview 指标。更详细的模块边界见
[`COLLABORATION.md`](COLLABORATION.md)。

## 巡检子任务平台配置

生产模型固定使用迁移包中的 `yolo26n_face_det.onnx`、`w600k_mbf.onnx` 和
`face_detection_yunet_2023mar.onnx`。构建脚本会先校验三个源模型的 SHA-256，再在目标 Orin
上生成匹配当前 TensorRT/JetPack 的 FP16 engine；engine 不从其他机器复制。

运行白名单继续采用现有的“每人一个目录、目录内一个或多个 512 维 `.npy`”格式。provider
启动时一次性加载到内存，运行期间不会逐次读取磁盘。该格式便于平台按人员增删和保留多个现场
样本；不要直接把旧的单文件 `.npz` 当作 `whitelist_dir` 使用。

人脸 provider 支持《巡检子任务配置》的纯人脸核验时长：

```yaml
window_timeout_seconds: 3.0            # 旧配置兼容回退值
initial_face_window_seconds: 3.0       # 纯人脸核验时长

evidence_photo_enabled: true
evidence_photo_root: "/data/dog_patrol/photos/face" # 单独启动时的回退目录
evidence_photo_format: "jpg"
evidence_photo_jpeg_quality: 90
evidence_photo_retention_seconds: 604800
evidence_photo_max_files: 10000
```

`initial_face_window_seconds` 为 `0` 时使用原 `window_timeout_seconds`。联合核验没有独立的人脸
截止时间：人脸从首次口令阶段持续识别，跨越播报、等待和后续轮次；任一模块通过、口令次数耗尽、
目标丢失或任务取消时由编排/任务状态统一结束。后续口令命令不重置人脸模型或在途推理。
旧 YAML 中的 `dual_verification_window_seconds` 仍可读取以兼容部署，但不再参与计时，应从配置中移除。
配置由平台生成或更新后，在下次启动 provider 时生效。完整感知启动会把统一 `photos_root` 传给人脸节点，人脸
照片固定写入 `<photos_root>/face/`；`evidence_photo_enabled` 仍是人脸模块独立开关，而总入口的
`photos_enabled=false` 会统一关闭各模块照片输出。

人脸算法的唯一图像输入仍是 tracking 发布的主目标 crop，本模块所称结果“帧”就是未经再次
裁剪或绘制的该输入 crop。人脸通过时保存实际通过帧及该帧的 `similarity`；人脸未通过时保存
本阶段第一次成功检测到人脸的 crop 及该次检测对应的 `similarity`，而不是后续帧的最高值。
全程未检测到人脸时不返回照片且 `similarity=0.0`。保存路径以
`file://` URL 写入 `AuthorizationEvidence.image_url`。文件名以源拍摄时间开头并包含目标 ID；
碰撞时追加序号，节点重启也不会覆盖保留期内的旧照片。同时返回匹配的 `person_id` 和
`similarity`；照片写入失败只追加诊断，不改变已经得到的人脸业务结果。

口令内容、正确回答、口令次数、目标跟踪/丢失、总核验超时及重复核验忽略时长不由人脸 provider
维护；这些配置由 voice、tracking、处置/总控读取。人脸 provider 只按命令阶段运行并返回证据。
