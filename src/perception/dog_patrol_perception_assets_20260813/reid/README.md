# CLIP ReID 部署资源

本目录保存 PR #51 使用的人体 ReID 模型与原始转换记录：

| 文件 | 用途 |
| --- | --- |
| `reid_clipreid_vitb16_after.onnx` | 供应方原始 ONNX，供后续重新生成 engine |
| `reid_clipreid_vitb16_after.fp16.engine` | 当前 Orin 使用的 TensorRT FP16 engine |
| `reid_clipreid_vitb16_after.fp16.engine.json` | 原始转换记录，包含来源/engine SHA-256、TensorRT 版本、IO 和验证结果 |

本机配置入口为 `../runtime/reid.yaml`，其 model_path 已指向本目录。转换记录中的 home
路径保留构建时的来源信息，不是服务启动时使用的配置路径。

模型输入为 RGB float32 `[1,3,256,128]`，输出为 1280 维 float32 特征；engine 内部启用
FP16，TensorRT 版本为 10.3.0.30。当前文件由已验证的原件复制，未重新转换；原 ONNX 和
engine 的 SHA-256 分别为：

```text
905c3072d73dfd4d8cf92884c9a90af83ca7ac4d5fab9ddcc14369d07c3ec98f  reid_clipreid_vitb16_after.onnx
0d7a307f7a5dea7e97380e3ea9e648695229bb22e828daff3a6f2e7317ef9d9f  reid_clipreid_vitb16_after.fp16.engine
```

文件校验使用资产包根目录的 SHA256SUMS。更换目标设备或 TensorRT 版本时，需重新验证或
从本目录 ONNX 生成匹配的 engine。模型文件沿用仓库的 `*.onnx`、`*.engine` 忽略规则。

从仓库根目录启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/navigation/fast_livo_dog/config/fastdds_udp_only.xml"
export DOG_PATROL_PYTHON=/home/orin/venv/m20_nav/bin/python
ros2 launch dog_patrol_perception_bringup perception_stack.launch.py
```

当日身份库仍使用仓库根目录的 `data/reid`，不随模型资源复制或发布。当前运行中的服务不会
因文件整理自动切换参数；由用户下次启动感知时加载新配置。控制台感知按钮已使用上述
完整 launch，会一并启动 ReID，无需另开服务终端。整体 READY 等待 ReID 实际模型加载/
预热成功，模型不可用时停留 STARTUP；暂停/继续不重做握手。请先停止已有手工启动的
ReID 服务，避免重复启动。模型路径按 launch 的 assets_root 自动覆盖。
