# dog_patrol_reid_provider — ReID 行人去重节点（端侧）

`/perception/reid` 端侧去重节点：人体检测侧问询"这个人今天是否见过/是否
已处置"，端侧提特征 + 当日身份库比对。
检测节点对接文档：本包 `docs/reid_dedup_interface_for_detection.md`
（服务字段、调用示例、降级约定、编译对接一应俱全）。

## 功能

- service `/perception/reid/dedup`（`ReIDDedup.srv`）：问询"这个人今天
  是否见过"。响应 = status + track_id 回显 + `is_seen` + `handled` +
  error_message；命中细节（相似度/路径）只留在节点日志，不暴露给调用方；
- 端侧：ONNX（CLIP ViT-B/16 after，1280 维）提特征，端侧身份库余弦比对
  （阈值 0.70），运行均值模板合并，track 映射兜底防同一 track 裂化；
- 端侧库每日本地日期切换清零（序号重计）——"当天去重"语义；
- **当日库持久化**：`data_dir/library.bin` 周期防抖落盘（默认 10 s）+
  正常退出兜底 + 临时文件原子替换。重启/开关机自动恢复当日库（应对巡逻
  机器人每天多次关机充电）；跨天文件自动作废，模型/版本不符或损坏不加载；
  异常断电最多丢一个防抖窗口内的更新；
- **handled（已处置）结论通道**（R06-A）：任务侧 supervisor 的处置结论经
  `/perception/reid/handled` 下发落库（track 绑定优先、crop 特征兜底，
  须过身份阈值不误标），ReID 把该结论按外观延续到同人换 track/重启后的
  新问询；**免处置判定必须 `is_seen && handled` 同时成立**，仅见过未处置
  不豁免。任务 reset 下发 clear_all 清除；
- **功能边界（项目组 2026-09-30 确认）**：只做"见过/没见过 + 是否已处置"
  的去重判断，不做白/黑名单判定；内部身份记录保留 match_type 存储
  （将来人脸结论的扩展位），启用时 srv 增加一个字段即可；
- 模型随节点**启动预加载**（TRT 约 1 s，一次性）并默认常驻，问询不含
  加载耗时；`model_idle_unload_sec > 0` 可改回空闲卸载；
- `/perception/reid/status` 1 Hz JSON 状态（库大小/模型状态），供现场观测。

## 文件结构

```
dog_patrol_reid_provider/
├── CMakeLists.txt / package.xml
├── srv/ReIDDedup.srv                    # 检测侧问询接口
├── msg/                                 # handled 结论下发等消息定义
├── include/dog_patrol_reid_provider/
│   ├── reid_gallery.hpp                 # 端侧身份库（纯 C++）
│   ├── feature_extractor.hpp            # 提特征抽象接口 + 工厂
│   └── feature_extractor_trt.hpp        # TensorRT 后端（REID_HAS_TENSORRT）
├── src/
│   ├── reid_node.cpp                    # 端侧节点 + main
│   ├── reid_query.cpp                   # C++ 联调客户端（query/handled/clear）
│   ├── reid_gallery.cpp
│   └── feature_extractor*.cpp           # OpenCV DNN / TensorRT 后端
├── config/reid.yaml                     # 节点参数
├── launch/reid.launch.py
├── docs/reid_dedup_interface_for_detection.md
└── scripts/                             # TRT 构建/精度验证/测速、联调客户端
```

## 构建（Orin / ROS 2 Humble）

```bash
# 放入工作空间 src/ 下
colcon build --packages-select dog_patrol_reid_provider
source install/setup.bash
```

依赖：`rclcpp`、`std_msgs`、`sensor_msgs`、OpenCV（core/dnn/imgproc/imgcodecs）。
不依赖 dog_patrol 任何旧包。

## 模型准备与速度

C++ 侧只认一个 ONNX（图内焙入 /255、mean/std、bottleneck、L2；输入为
RGB 0~255，通道翻转由 `blobFromImage(swapRB=true)` 在 C++ 完成——翻转
**不能**焙进图：torch.flip 导出为负步长 Slice，OpenCV DNN 加载失败）。
已验证的模型文件：`reid_clipreid_vitb16_after.onnx`（CLIP ViT-B/16 after，
1280 维），随部署交接，放 `assets/models/reid/`。

速度基准（本包 `scripts/benchmark_onnx.py`，桌面 Windows CPU 实测）：

| 后端 | 单帧 p50 |
|---|---|
| onnxruntime CPU | ~60 ms |
| OpenCV DNN CPU（本节点 CPU 兜底后端） | ~670 ms |
| 预处理 resize+CHW | ~0.7 ms |

Orin 生产路径为 TensorRT FP16（见下节）。注意：8 人小样本标定的阈值余量
只有 0.007（跨人 max 0.693 vs 阈值 0.70），换精度后必须先跑精度验证。

## TensorRT（Orin 推荐后端）

模型已确认 TRT 10 全算子兼容（opset 17：LayerNormalization/MatMul/Softmax/
Sigmoid/Conv/ReduceL2，无自定义算子）。engine 必须在目标机上构建（与设备
和 TRT 版本绑定，不能跨机拷贝）：

```bash
# 1) 构建 FP16 engine + 测速（首次 1~5 分钟）
bash scripts/build_trt_engine.sh
# 2) 精度与阈值余量验证（需 Jetson Zoo 的 onnxruntime-gpu，脚本头有链接）
python3 scripts/validate_trt_fp16.py
# 3) 节点启用
ros2 run dog_patrol_reid_provider reid_node --ros-args \
    -p device:=tensorrt -p model_path:=/abs/reid_clipreid_vitb16_after.fp16.engine \
    -p device_id:=m20-001
```

CMake 侧自动探测（选项 `REID_BUILD_TENSORRT`，默认 ON）：找到
`NvInfer.h`/`libnvinfer`/CUDA 才编入 TRT 后端（宏 `REID_HAS_TENSORRT`），
否则只编 OpenCV DNN 后端，`device:=tensorrt` 会创建失败并在启动日志说明。
TRT 后端与 OpenCV 后端接口/预处理/输出完全一致（IO 保持 fp32，engine
内部 FP16），blobFromImage(swapRB=true) 后 memcpy + enqueueV3。
预期单帧 2~6ms（AGX Orin FP16 口径），模型 IO 名 `image_rgb_f32`/`feature`。

## 运行

```bash
ros2 launch dog_patrol_reid_provider reid.launch.py
# 或手工传参
ros2 run dog_patrol_reid_provider reid_node --ros-args \
    -p model_path:=/abs/path/reid_clipreid_vitb16_after.onnx \
    -p device_id:=m20-001
```

联调要点：当前阶段为纯端侧（全部问询立即应答，延迟 ≈ 单帧推理）；
`device:=cpu` 为 CUDA/TRT 不可用时的回退路径。

观测与联调客户端：

```bash
ros2 topic echo /perception/reid/status
# C++ 联调客户端（读图问询 / 下发 handled / 清除，不依赖 Python 绑定）
ros2 run dog_patrol_reid_provider reid_query --ros-args -p image:=p1.jpg -p track_id:=t1
ros2 run dog_patrol_reid_provider reid_query --ros-args -p mode:=handled -p track_id:=t1
ros2 run dog_patrol_reid_provider reid_query --ros-args -p mode:=clear
ros2 param get /perception_reid threshold
```

## 检测节点对接

新 track 首次稳定出现时调用一次；可选周期刷新（重复调用安全：命中走
均值合并，不会产生重复身份）。字段语义、判定口径（is_seen/handled）、
编译对接见 `docs/reid_dedup_interface_for_detection.md`。

## 已知边界

- **多狗云端同步为后续阶段**：包内保留的云端代理/同步相关代码与话题
  （`reid_cloud_proxy`、`/reid/cloud/*`）本阶段未启用（`cloud.enable`
  默认 false），不在本轮验证范围；
- `ReIDDedup.srv` 暂定义在本包；负责人建立共享接口包后迁移是机械改动；
- ~~rosidl Python 绑定不生成~~ 已修复根因：CMakeLists 此前缺少
  `find_package(rosidl_default_generators REQUIRED)`（package.xml 声明
  buildtool_depend 不等于 CMake 加载了生成器组），现已补上并导出
  `rosidl_default_runtime`；`reid_query` 仍保留作无 Python 环境的联调手段；
- TRT FP16 engine 启用前必须 `validate_trt_fp16.py` PASS（阈值余量 0.007）。
