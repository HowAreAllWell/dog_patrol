# 本机 fake 联调脚本

该目录提供本机联调入口。最终 `normal` 场景只 fake 导航；supervisor、tracking、face、voice、
readiness 和授权 orchestrator 都运行安装产物中的真实节点：

- `fake_nodes.py --role navigation`：只发布导航 `READY`、`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED`；不连接底盘、雷达或控制器。
- `fake_nodes.py --role capabilities`：仅供 tracking-only 回归场景补齐 capability，不参与最终
  `normal` 验收。
- `run_fake_integration.py`：启动上述真实主流程与感知节点，加上 fake navigation，记录状态、事件、
  分阶段 evidence、流程耗时、进程资源和 `tegrastats`。

## 测试 A：只验证总控与感知的状态交互

这类测试不验证真实距离和路径，使用已有 fake integration：

```bash
python3 tools/fake_integration/run_fake_integration.py \
  --scenario startup_visible \
  --tracking-params /absolute/path/to/perception_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --camera-input-mode ros_image \
  --image-topic /left_camera/image_raw
```

`normal`、`dual_pass` 和 `dual_reject` 还会启动真实 face/voice provider；它们适合验证
感知内部认证和 supervisor 状态转移，不适合作为导航协调器距离计算的验收。

### 测试 B：接入真实 tracking

真实 tracking 在 `runtime.mode:=mission` 下不会无条件发布 bbox：它需要订阅有效的
`/mission/state`，在 `PATROL` 先发布 `TARGET_CONFIRMED`，进入 `CONFIRM_TARGET` 后才发布
当前目标 bbox。因此只启动 tracking 进程而没有 supervisor，通常看不到
`/perception/selected_target_bbox`，这是状态门禁，不是 tracking 一定失效：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node \
  --ros-args \
  --params-file /mnt/nvme/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813/runtime/perception_tracking.yaml \
  -p runtime.mode:=mission \
  -p camera.input_mode:=ros_image \
  -p camera.image_topic:=/left_camera/image_raw \
  -p visualization.enable:=true \
  -p recording.enable:=false
```

运行前还必须有：

- `/left_camera/image_raw` 正在发布 1280x1024 图像；
- 唯一的 `mission_supervisor` 正在发布 `/mission/state`；
- 导航协调器已经发布 navigation `READY`；
- 感知 readiness 满足后，supervisor 已进入 `PATROL`；
- 画面中有可跟踪人员。

验证 tracking 是否发布候选和 bbox：

```bash
ros2 topic echo /mission/event
ros2 topic echo /perception/selected_target_bbox
ros2 topic hz /perception/selected_target_bbox
```

真实 tracking 和导航协调器联调时，推荐先用测试 A 确认坐标、外参和 planner，再切换
到测试 D。这样没有 bbox 时可以明确判断是感知门禁问题，还是导航融合问题。

## 只启用人像检测的完整导航闭环

如果目标是测试“人像检测 -> 雷达测距 -> 3 m 接近 -> 持续跟踪 -> 回到巡检”，不要启动
face 或 voice provider。因为当前感知 readiness 聚合仍要求 face/voice capability，使用
测试工具只补发它们的 READY，不加载任何人脸模型、语音模型或相机输入：

```bash
# 终端 1：唯一的全局状态机
ros2 launch dog_patrol_manager mission_supervisor.launch.py use_sim_time:=false

# 终端 2：真实导航链，包含协调器、Nav2、Pure Pursuit、RL 和 DWB adapter
export DOG_PATROL_NAV_ROOT=/home/orin/workspace/dog_patrol/src/navigation/fast_livo_dog
ros2 launch move navigation.launch.py \
  use_sim_time:=false \
  params_file:=$DOG_PATROL_NAV_ROOT/config/nav_parameters.yaml

# 终端 3：只补 face/voice READY，不运行 face/voice 算法
python3 tools/fake_integration/fake_nodes.py --role capabilities

# 终端 4：感知 READY 聚合，只会等待 detection_tracking、face、voice 三个 READY
ros2 run dog_patrol_perception_orchestrator perception_readiness

# 终端 5：真实人像检测和跟踪，使用导航相机的原始 1280x1024 图像
ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node \
  --ros-args \
  --params-file /home/orin/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813/runtime/perception_tracking.yaml \
  -p runtime.mode:=mission \
  -p camera.input_mode:=ros_image \
  -p camera.image_topic:=/left_camera/image_raw \
  -p visualization.enable:=true \
  -p recording.enable:=false

# 终端 6：只模拟认证结果，不发布 bbox
python3 tools/navigation_coordinator/mock_mission_events.py \
  --result unauthorized
```

终端 6 的行为是：真实导航到达 3 m 并使 supervisor 进入 `VERIFY_IDENTITY` 后，发布
`UNAUTHORIZED`，触发 `TRACK_INTRUDER`。之后需要让真实目标离开画面并持续超过
`target.lost_event_timeout_sec`（默认 10 秒），由 tracking 发布 `TARGET_LOST`，导航协调器再恢复巡检。

验证完整过程：

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/event
ros2 topic echo /perception/selected_target_bbox
ros2 topic echo /navigation/target_point
ros2 topic hz /global_path
```

期望状态序列：

```text
STARTUP
  -> PATROL
  -> CONFIRM_TARGET
  -> APPROACH_TARGET
  -> VERIFY_IDENTITY
  -> TRACK_INTRUDER
  -> PATROL
```

其中 `TARGET_CONFIRMED` 和 bbox 必须来自真实 tracking，`TARGET_POSITION_READY`、
`ARRIVED_AND_STOPPED`、目标点和全局路径必须来自真实导航协调器，只有
`UNAUTHORIZED` 由测试工具模拟，`TARGET_LOST` 必须来自真实 tracking。

先观察 `TRACK_INTRUDER` 是否能够持续收到真实 bbox 和更新目标路径；确认跟踪正常后，
让测试目标离开画面，测试 10 秒最终丢失后的巡检恢复。
如果测试人员一直留在画面中，回到 `PATROL` 后真实 tracking 可能再次发布
`TARGET_CONFIRMED`，这是当前“巡检状态继续检测”的真实行为，不是状态机回退失败。
返回巡检后若测试目标再次出现，tracking 可在新的 `PATROL state_seq` 下开启下一次目标流程。

如果外部相机驱动已经发布原始 ROS 图像，可以将 tracking 切换到 `ros_image` 模式。该模式不会
打开 Hik MVS，只订阅指定的 `sensor_msgs/msg/Image`，适合 `fast_livo_dog` 的双 topic 驱动：

```bash
python3 tools/fake_integration/run_fake_integration.py \
  --scenario normal \
  --camera-input-mode ros_image \
  --image-topic /left_camera/image_raw \
  --tracking-params /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --face-config /absolute/path/to/face.yaml \
  --voice-model-dir /absolute/path/to/vosk-model \
  --voice-config /absolute/path/to/voice.yaml \
  --voice-helper /absolute/path/to/r818_pcm_base64_aarch64 \
  --acceptance-case initial_face_pass \
  --preview
```

运行前需要先启动外部相机驱动，并确认 `/left_camera/image_raw` 正在发布；不要同时启动 tracking
的内部 MVS 输入。

## 运行

先 source ROS 和本仓安装产物，再在当前感知机器狗上执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/fake_integration/run_fake_integration.py \
  --tracking-params /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --face-config /absolute/path/to/face.yaml \
  --voice-model-dir /absolute/path/to/vosk-model \
  --voice-config /absolute/path/to/voice.yaml \
  --voice-helper /absolute/path/to/r818_pcm_base64_aarch64 \
  --acceptance-case initial_face_pass \
  --preview
```

`--preview` 只打开 tracking 已有的 `VisualizerRecorder` 窗口；人脸 bbox 叠加在同一 canvas，不会
启动第二个预览或录制。运行环境必须有本地图形会话和 `$DISPLAY`。

## 最终感知验收

用同一组部署参数依次运行三次，只修改 `--acceptance-case`：

- `initial_face_pass`：白名单人员进入画面并保持可见；必须由初始人脸直接授权，语音不能启动或发布
  cancellation evidence。
- `dual_pass`：非白名单人员保持可见；初始人脸失败后，在第一或第二个语音提示窗说正确口令。必须
  观察到初始人脸失败及并行阶段中的任一真实 provider 通过。
- `dual_reject`：非白名单人员保持可见，两个语音窗均不说正确口令。必须观察到初始人脸失败、
  两轮人脸/语音均未通过，最终进入 `TRACK_INTRUDER`。

例如第二项：

```bash
python3 tools/fake_integration/run_fake_integration.py \
  --scenario normal \
  --tracking-params /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --face-config /absolute/path/to/face.yaml \
  --voice-model-dir /absolute/path/to/vosk-model \
  --voice-config /absolute/path/to/voice.yaml \
  --voice-helper /absolute/path/to/r818_pcm_base64_aarch64 \
  --acceptance-case dual_pass \
  --preview
```

每次启动前让本次测试人员站入相机视野并保持可见。三次均以 `report.json` 的
`functional.status=PASS` 为功能结论；bbox 是否稳定跟随目标还需在同一 tracking 预览窗口目检。

已授权目标短时离开后再次出现的豁免场景使用
`--scenario authorized_reencounter`。第一次授权完成后，脚本提示目标离开画面；真实 tracking 确认
画面内已无目标后再提示返回，确认目标重新出现后观察 10 秒。期间不得再次进入可疑目标流程，也不得
产生新的语音 evidence。生产配置按目标连续
离开画面的时长计算豁免失效，当前 `target.handled_ignore_absence_sec=30` 秒；目标持续可见时不会失效。

诊断开机时目标已在相机画面内的状态切换时，使用 `--scenario startup_visible`。该 tracking-only
场景不启动真实 voice 或 face 算法，fake capability 只补齐启动条件；启动命令前先站到镜头前并保持可见。脚本在真实
tracking 的 `TARGET_CONFIRMED` 被 supervisor 接受、状态推进到 `CONFIRM_TARGET` 后立即结束，
因此不会受后续导航、语音或人脸流程影响。

每次运行在 `data/diagnostics/fake_integration/<timestamp>/` 生成各节点日志、事件/evidence 和
`report.json`。`functional.status` 是唯一影响脚本退出码的结果；`performance` 固定为观测结果，
不设置通过阈值。若本机有 `tegrastats`，原始资源采样保存为 `tegrastats.log`，并在报告中汇总
RAM、每核 CPU、GPU `GR3D_FREQ` 和 Tj 温度范围；报告还保存脚本启动进程的 RSS 与累计 CPU ticks
采样，以及初始人脸、两轮并行阶段和 VERIFY_IDENTITY 总耗时。tracking 日志保留 FPS/drop/延迟，
face 日志保留输入、推理、latest-only 丢弃和窗口指标；采集缺失不会改变功能判定。
