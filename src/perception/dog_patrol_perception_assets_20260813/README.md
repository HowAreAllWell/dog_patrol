# 感知部署资产包

本目录是交付给导航团队的感知部署资产包模板。它描述目录结构、配置入口和校验方式；模型、白名单、设备序列号和现场参数不放入 Git。

## 目录结构

将本目录复制到目标 Orin 后，按以下结构填入受控资产：

```text
perception_assets/
├── runtime/
│   ├── face.yaml
│   └── reid.yaml
├── tracking/
│   ├── yolo26n_fp16_640.engine
│   ├── perception_tracking.yaml
│   └── bot_sort.yaml
├── reid/
│   ├── reid_clipreid_vitb16_after.onnx
│   ├── reid_clipreid_vitb16_after.fp16.engine
│   ├── reid_clipreid_vitb16_after.fp16.engine.json
│   └── README.md
├── face/
│   ├── detector.engine
│   ├── mobilefacenet_fp16.engine
│   ├── face_detection_yunet_2023mar.onnx
│   └── whitelist_mobilefacenet/<person>/*.npy
├── voice/
│   ├── vosk-model-small-en-us-0.15/
│   └── voice.yaml
└── SHA256SUMS
```

`SHA256SUMS` 应由资产提供方在填充真实文件后生成，不包含白名单。供应方注册每人写入一个归一化的 512 维中心特征 `g0_0px_f0.npy`；备用注册工具可写入多模板，文件名携带采集时的人脸尺寸。

## 必需资产

- tracking：目标 Orin 本机生成的 YOLO26n FP16 TensorRT engine、相机序列号参数和 tracker YAML。
- reid：CLIP ViT-B/16 after 原始 ONNX、已在当前 Orin 验证的 FP16 engine 和转换记录；用于当日已处置人员去重，输出 1280 维。
- face：YOLO/TensorRT detector engine、MobileFaceNet recognition engine、YuNet 注册模型，以及 512 维 MobileFaceNet whitelist embedding 目录。
- voice：Vosk 模型目录、目标 R818 的 ADB serial、实际 ALSA 播放设备和 mixer 配置。

Tracking engine 必须在目标 JetPack/TensorRT 组合上生成或验证，不能只按文件名跨设备复用。白名单属于生物特征数据，应通过受控渠道分发。

## 配置

从本仓模板复制后填写现场值：

```bash
cp src/perception/dog_patrol_perception_tracking/config/perception_tracking_params.yaml tracking/perception_tracking.yaml
cp src/perception/dog_patrol_perception_tracking/config/bot_sort.yaml tracking/bot_sort.yaml
cp src/perception/dog_patrol_perception_face/config/face.yaml runtime/face.yaml
cp src/perception/dog_patrol_perception_voice/config/voice.yaml voice/voice.yaml
```

至少修改：

- `tracking/perception_tracking.yaml` 的 `camera.mvs_serial` 和 `detector.runtime_path`；默认 `light` ReID 不需要 ONNX。
- `runtime/face.yaml` 的 `detector_engine`、`recognition_engine` 和 `whitelist_dir`；生产默认是MobileFaceNet。
- `runtime/reid.yaml` 的 `model_path` 和 `data_dir`；本机已配置为本资产包下的 TRT engine，以及 NVMe 工作区的 `data/reid`。
- `voice/voice.yaml` 的 `device_serial`、`prompt_device`、`prompt_mixer_card` 和 `prompt_mixer_control`。

不要把这些现场配置提交回公开仓库。

## 校验和启动

在仓库根目录构建并安装全部感知包，然后运行统一环境检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/perception/scripts/check_perception_environment.py \
  --target perception-orin \
  --params-file /path/to/perception_assets/tracking/perception_tracking.yaml \
  --tracker-config /path/to/perception_assets/tracking/bot_sort.yaml \
  --voice-model-dir /path/to/perception_assets/voice/vosk-model-small-en-us-0.15 \
  --voice-config-file /path/to/perception_assets/voice/voice.yaml \
  --install-prefix "$PWD/install" \
  --build-base "$PWD/build"
```

检查通过后，用 `tools/fake_integration/README.md` 的真实节点命令进行感知整流程验收。导航接入时保持合同中的 topic、QoS、`state_seq`、`target_id` 和 bbox 新鲜度约束。

ReID 服务已随完整感知 launch 自动启动，读取 `runtime/reid.yaml`；用法见 [reid/README.md](reid/README.md)，完整现场操作
见 [PR #51 测试指南](../../../docs/integration/reid_pr51_robot_test_guide.md)。模型资源整理不会
自动重启服务；当日身份库继续保存在工作区 `data/reid`，不放入本静态资产目录。

生成校验和示例：

```bash
cd /path/to/perception_assets
find runtime tracking reid face voice -type f \
  -not -path '*/whitelist_mobilefacenet/*' \
  -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
```

白名单应由受控资产系统单独记录校验和，不应公开提交。

新增或更新白名单人员使用 face 包内的统一注册工具，不生成 `gallery.npz`：

```bash
source /opt/ros/humble/setup.bash
source /mnt/nvme/workspace/dog_patrol/install/setup.bash
source /mnt/nvme/venv/m20_nav/bin/activate
python3 -m dog_patrol_perception_face.enrollment \
  --name person_001 \
  --source ~/person_001.mp4 \
  --assets-root /mnt/nvme/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813
```

工具只在质量检查和保留帧竞争验证全部通过后，写入 `face/whitelist_mobilefacenet/<name>/*.npy`。更新已有身份
必须显式增加 `--replace`；操作前停止感知，完成后重新启动以加载新白名单。

MobileFaceNet 先在目标 Orin 上从交付 ZIP 构建 engine：

```bash
bash src/perception/dog_patrol_perception_face/scripts/build_mobilefacenet_engine.sh \
  /home/orin/迁移orin_MobileFaceNet_原始版.zip \
  /mnt/nvme/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813
```

脚本还会提取供应方白名单注册使用的 YuNet ONNX；该模型由 OpenCV 直接执行，无需转换。
然后用供应方兼容流程为每个身份重新注册：

```bash
source /mnt/nvme/venv/m20_nav/bin/activate
python3 -m dog_patrol_perception_face.supplier_enrollment \
  --name person_001 \
  --source /absolute/path/person_001.mp4
```

该流程把每个人的合格特征求均值，最终中心特征写入
`face/whitelist_mobilefacenet/<name>/g0_0px_f0.npy`，可由现有 provider 直接加载。
完整感知bringup默认使用 `runtime/face.yaml`、`mobilefacenet_fp16.engine` 和
`whitelist_mobilefacenet`。白名单为空时 readiness 会保持 `NOT_READY`；系统没有旧模型回退路径。
