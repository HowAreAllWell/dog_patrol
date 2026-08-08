# 本机 fake 联调脚本

该目录只提供本机联调用的最小 fake 节点，不属于导航或感知生产链路：

- `fake_nodes.py --role navigation`：只发布导航 `READY`、`TARGET_POSITION_READY`、`ARRIVED_AND_STOPPED`；不连接底盘、雷达或控制器。
- `fake_nodes.py --role face`：只针对当前 `STARTUP state_seq` 发布 `face=READY`，并观察真实 tracking 发布的目标 crop；不发布人脸授权 evidence。
- `run_fake_integration.py`：启动主仓真实 supervisor、tracking、orchestrator、voice provider，加上上述 fake 节点，记录事件和 `tegrastats` 原始数据。

## 运行

先 source ROS 和本仓安装产物，再在当前感知机器狗上执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/fake_integration/run_fake_integration.py \
  --tracking-params /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --voice-model-dir /absolute/path/to/vosk-model \
  --voice-config /absolute/path/to/voice.yaml \
  --voice-helper /absolute/path/to/r818_pcm_base64_aarch64
```

需要同时查看 tracking 的实时诊断叠图时，在任一场景命令末尾增加 `--preview`。该开关只覆盖
`visualization.enable=true`，不会自动开启录制；运行环境必须有可用的本地图形会话和 `$DISPLAY`：

```bash
python3 tools/fake_integration/run_fake_integration.py \
  --scenario normal \
  --tracking-params /absolute/path/to/orin_tracking.yaml \
  --tracker-config /absolute/path/to/bot_sort.yaml \
  --voice-model-dir /absolute/path/to/vosk-model \
  --voice-config /absolute/path/to/voice.yaml \
  --voice-helper /absolute/path/to/r818_pcm_base64_aarch64 \
  --preview
```

已授权目标短时离开后再次出现的豁免场景使用
`--scenario authorized_reencounter`。第一次授权完成后，脚本提示目标离开画面；真实 tracking 确认
画面内已无目标后再提示返回，确认目标重新出现后观察 10 秒。期间不得再次进入可疑目标流程，也不得
产生新的语音 evidence。生产配置按目标连续
离开画面的时长计算豁免失效，当前 `target.handled_ignore_absence_sec=30` 秒；目标持续可见时不会失效。

诊断开机时目标已在相机画面内的状态切换时，使用 `--scenario startup_visible`。该场景不启动真实
voice 或 face 算法，fake 仅提供二者的 READY；启动命令前先站到镜头前并保持可见。脚本在真实
tracking 的 `TARGET_CONFIRMED` 被 supervisor 接受、状态推进到 `CONFIRM_TARGET` 后立即结束，
因此不会受后续导航、语音或人脸流程影响。

每次运行在 `data/diagnostics/fake_integration/<timestamp>/` 生成节点日志、事件/evidence 和
`report.json`。`functional.status` 是唯一影响脚本退出码的结果；`performance` 固定为观测结果，
不设置通过阈值。若本机有 `tegrastats`，原始资源采样保存为 `tegrastats.log`，并在报告中汇总
RAM、每核 CPU、GPU `GR3D_FREQ` 和 Tj 温度范围；报告还保存脚本启动进程的 RSS 与累计 CPU ticks
采样。采集缺失不会改变功能判定。

当前入口是第一版 happy path：真实 tracking 和 voice 必须产生真实结果；fake face 仅补齐 readiness，
fake navigation 仅推进完成 VERIFY_IDENTITY 所需的三个导航事件。目标丢失、错误口令、voice 取消和
硬件恢复等场景后续再增加为独立选项。
