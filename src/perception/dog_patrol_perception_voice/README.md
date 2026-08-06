# dog_patrol_perception_voice

`dog_patrol_perception_voice` 是按需调用的语音验证核心和启动 readiness provider，不提供常驻监听或产品 CLI。
一个 `R818TaskSession` 只建立一次八通道 R818 流；Prompt 播放期间 reader 持续消费但丢弃完整
音频帧，Prompt 返回后再打开响应窗。每个响应窗独立返回 `VoiceWindowResult`，最多可在同一任务中
连续执行两窗，任务结束时统一清理远端临时节点并恢复厂商音频服务。

生产核心入口是 `R818VoiceAdapter.from_model_dir()` 和 `R818VoiceAdapter.task()`。ROS 生产
evidence 入口是 `perception_voice_provider`：它订阅 `/mission/state`，只对未阻塞的
`VERIFY_IDENTITY` 和正数 `target_id` 启动任务，并发布 `/perception/authorization_evidence`。
provider 的 worker 与 ROS executor 分离；同一 `state_seq + target_id` 不重复建 task，状态替换、
阻塞或离开 VERIFY 时请求取消并在旧 session 清理后才允许新 session。默认不会写入 PCM；回放与
硬件验收使用测试注入的 `TaskStream` seam，不作为安装后的通用入口。

部署时提供 Vosk 模型目录并启动：

```bash
ros2 run dog_patrol_perception_voice perception_voice_provider \
  --ros-args -p model_dir:=/path/to/vosk-model \
  -p config_file:=/path/to/voice.yaml
```

`helper_path` 为空时使用安装包内的 ARM64 helper；`provider`、`mission_state_topic` 和
`authorization_evidence_topic` 可按联调命名空间覆盖。授权 evidence 与 capability readiness 是两条
独立接口：`perception_voice_provider` 只发布前者，`perception_voice_readiness` 只发布后者。

### 启动和 readiness preflight

安装后推荐使用统一 launch：

```bash
ros2 launch dog_patrol_perception_voice voice.launch.py \
  model_dir:=/path/to/vosk-model \
  config_file:=/path/to/voice.yaml
```

`perception_voice_readiness` 订阅 transient-local `/mission/state`。每个有效 STARTUP
`state_seq` 只运行一次真实 preflight，并以同一 `observed_startup_state_seq` 发布 transient-local
`CapabilityStatus`：缺失部署输入或设备不可用为 `NOT_READY`，配置、模型或 helper 校验错误为 `ERROR`，
全部通过才为 `READY`；旧 STARTUP 的 retained status 不会匹配新的 sequence。

preflight 只读取安装后的配置/helper、加载 Vosk 模型、执行 `adb get-state`、查询 FFmpeg 的 flite
filter 和 ALSA 的 `aplay -L`/`amixer scontrols`。它不创建 voice task，不执行 ADB push/shell，不接管
R818，不停止 `vtn_init`/demo，不改变 mixer，不播放 Prompt；readiness 不等价于真实语音效果或现场
FAR/FRR 验收。

## 配置和安装资产

- 默认配置：`config/voice.yaml`，只包含当前 `blue star` 口令、两条 Prompt、响应窗和 R818/Prompt
  运行参数；设备 serial、模型目录由部署机传入。
- ARM64 helper：`assets/r818_pcm_base64_aarch64`，安装在 Python package 的 `assets/` 下；对应
  C 源码安装在 `share/dog_patrol_perception_voice/tools/r818_pcm_base64.c`。
- helper 来源固定为源仓冻结提交
  `b979a7fd33aac5c9ced9591bb507e483faf4aef5`；其内容校验值为
  `c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52`。
- ROS `rosdep` 没有 `python3-vosk` 规则，因此 package.xml 不声明这个不可解析的 key。部署机在
  `rosdep install` 完成后，应使用安装产物中的 `share/dog_patrol_perception_voice/requirements.txt`
  安装固定的 Python runtime wheels；开发工作区可直接对本文件执行同一条 pip 命令：

  ```bash
  python3 -m pip install --requirement requirements.txt
  ```

Vosk 运行时和模型是部署依赖，不随 package 发布；PCM、模型、录音和评测数据不进入仓库。ROS 节点
从安装产物解析默认 config/helper 路径，不回退到源仓源码或源仓虚拟环境。
本 package 的代码和 helper 按 BSD-3-Clause 分发，许可全文见同目录 `LICENSE`。
