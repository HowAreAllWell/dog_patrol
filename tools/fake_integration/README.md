# 本机 fake 联调脚本

该目录提供本机联调入口。最终 `normal` 场景只 fake 导航；supervisor、tracking、face、voice、
readiness 和授权 orchestrator 都运行安装产物中的真实节点：

- `fake_nodes.py --role navigation`：只发布导航 `READY`、`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED`；不连接底盘、雷达或控制器。
- `fake_nodes.py --role capabilities`：仅供 tracking-only 回归场景补齐 capability，不参与最终
  `normal` 验收。
- `run_fake_integration.py`：启动上述真实主流程与感知节点，加上 fake navigation，记录状态、事件、
  分阶段 evidence、流程耗时、进程资源和 `tegrastats`。

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
