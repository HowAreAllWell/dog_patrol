# Issue #37：无人参与的 Orin 语音部署验收

## 入口

验收必须从主仓 clean install 的 console script 启动，不从源码目录、旧
`vision_demo_ws` 或旧仓虚拟环境导入运行时：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/dog_patrol/install/setup.bash
ros2 run dog_patrol_perception_voice perception_voice_acceptance \
  --mode hardware \
  --fixture /srv/dog-patrol/issue37_voice_tasks.json \
  --cycles 33 \
  --model-dir /srv/dog-patrol/vosk-model \
  --config-file /absolute/path/to/dog_patrol/install/dog_patrol_perception_voice/share/dog_patrol_perception_voice/config/voice.yaml \
  --report /srv/dog-patrol/issue37_voice_acceptance.json
```

`voice.yaml` 中的 ADB serial、AC107 播放设备和 mixer 必须是部署机实际值。
`--fixture` 是部署机上的任务结果清单，不包含 PCM、录音、模型或口令；命令不等待
用户输入，也不会把原始音频写入主机。`--cycles` 默认是 33，fixture 的任务数必须
严格匹配该值。

### Fixture 格式

fixture 只允许记录每个任务的一到两个响应窗最终结果：

```json
{
  "schema_version": 1,
  "tasks": [
    {"windows": [{"accepted": true, "decision_time_seconds": 0.4}]},
    {"windows": [
      {"accepted": false, "decision_time_seconds": 20.0},
      {"accepted": true, "decision_time_seconds": 0.7}
    ]}
  ]
}
```

fixture 只注入识别结果；硬件模式仍会加载 Vosk model、播放 Prompt、接管真实 AC107
R818 stream，并在每个任务结束后恢复厂商音频服务。因此它验证的是安装、硬件所有权、
生命周期和 ROS 会话，不替代现场说话效果或 FAR/FRR。

## 自动覆盖范围

脚本先执行已安装 voice package 的真实 read-only preflight，然后启动真实
`perception_voice_readiness`、`perception_voice_provider` 和
`perception_authorization` ROS adapter。33 次任务逐次检查：

- transient-local readiness 的 startup sequence 与 `READY` 状态；
- 每个 `state_seq + target_id` 的 evidence、最终 `MissionEvent` 和目标关联；
- 每个任务清理后的 `/tmp/dog-patrol-r818-stream.{pcm,pid,err}`、远端 helper 和
  `arecord` 进程均不存在；
- host 默认不生成 PCM，报告只保存 JSON 摘要。

每次运行还执行不需要真实设备的故障矩阵：Prompt 取消、第一/第二响应窗取消、
`state_seq`/`target_id` 替换、stream/ADB/播放/恢复故障。矩阵要求旧任务完成清理、
最多一个硬件 session、迟到 evidence 不通过 ROS authorization adapter 形成事件。

## 运行边界

`--mode fixture` 使用同一 ROS adapter 和 fixture/fake hardware seam，只适合 CI 或
部署前验证编排合同；它不证明 Orin、ADB、AC107 或 Vosk model 可用。

`--mode hardware` 返回 0 且报告中 `passed` 为 `true`，才表示本票的自动部署验收
命令通过。该结果仍不声明最终口令、现场 FAR/FRR、安全准入或真实说话场景已验收。
