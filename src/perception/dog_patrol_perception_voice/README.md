# dog_patrol_perception_voice

`dog_patrol_perception_voice` 是按需调用的语音验证核心和启动 readiness provider，不提供常驻监听；
安装产物另提供无人参与的部署验收 CLI。
每个 `R818TaskSession` 建立一次八通道 R818 流并运行一个响应窗；Prompt 播放期间 reader 持续消费
但丢弃完整音频帧，Prompt 返回后再打开响应窗。窗口结束后清理远端临时节点并恢复厂商音频服务。

当前每轮响应窗 `response_timeout_seconds` 为 10 秒；包默认值、缺省 YAML 字段的回退值与
资产包的两份 voice 配置保持一致。该参数只限制 Prompt 播放完成后的单轮收音窗口，不修改
ADB、启动或恢复厂商服务的超时时间。

生产核心入口是 `R818VoiceAdapter.from_model_dir()` 和 `R818VoiceAdapter.task()`。ROS 生产
evidence 入口是 `perception_voice_provider`：它订阅 `/mission/state` 建立会话门禁，并订阅
`/perception/authorization_command`。`INITIAL_FACE` 不启动硬件；每条 `DUAL_FIRST` 或
`DUAL_SECOND` 命令各运行一个响应窗，并将 stage 原样绑定到 `/perception/authorization_evidence`。
命令和证据的 `round_number` 标识具体轮次：首轮使用 `DUAL_FIRST`，后续轮次复用
`DUAL_SECOND`。重复或倒序轮次不会重新启动收音；旧消息的 `round_number=0` 仅兼容首两轮。
provider 的 worker 与 ROS executor 分离；重复命令不重复建 task，状态替换、`CANCEL`、阻塞或离开
VERIFY 时请求取消并抑制旧 session 的迟到输出。默认不会写入 PCM；回放与
硬件验收使用测试注入的 `TaskStream` seam，不作为安装后的通用入口。

只有 `TASK_RUNNING + VERIFY_IDENTITY + target_id>0` 才能建立语音会话。READY（包括 PAUSE、RETURN、
TERMINATE 后的 READY）或其他非 VERIFY 状态会取消会话；取消只产生 `CANCELLED` 技术证据，
不会产生 `UNAUTHORIZED`。旧 `state_seq`、target、stage 和 provider evidence 均被丢弃。

### 口令策略与照片事件

编排节点 `/perception_authorization` 的 ROS 参数 `voice_prompt_count` 控制播报次数，
默认 2，必须为正整数。每轮播报完成后等待回答；失败才进入下一轮。次数耗尽后不再播报或监听，
按已有任务流程进入入侵跟踪。这里不新增总核验时限，也不修改 face 的核验窗口。
第三轮起沿用第二阶段已经完成的人脸结果，不要求 face 识别新的轮次协议。

`/perception_voice_provider` 支持以下 ROS 参数和同名 launch 参数；launch 留空时使用
`config_file` 的值。直接设置 ROS 参数的优先级高于 YAML：

| 参数 | 默认值/作用 |
| --- | --- |
| `response_timeout_seconds` | 10.0；每轮播报结束后的收音时长，必须大于 0 |
| `first_prompt` | 首次英文播报文本 |
| `retry_prompt` | 后续英文播报文本；需要同一句口令时与 `first_prompt` 设置相同 |
| `passphrase` | `blue star`；正确回答，统一转小写、合并空白后精确匹配 |

这些参数和 `voice_prompt_count` 支持 `ros2 param set`，更新在下一次核验会话生效，
当前人物的各轮沿用开始时的设置。示例：

```bash
ros2 param set /perception_authorization voice_prompt_count 3
ros2 param set /perception_voice_provider response_timeout_seconds 10.0
ros2 param set /perception_voice_provider first_prompt 'Please say the passphrase.'
ros2 param set /perception_voice_provider retry_prompt 'Please say the passphrase.'
ros2 param set /perception_voice_provider passphrase 'blue star'
```

`/perception/voice_verification_event` 发布 `VoiceVerificationEvent`，可通过
`voice_verification_event_topic` 参数改名。每条包含会话、目标、`round_number` 和事件时刻：

- `WAIT_STARTED`：播报结束、真实收音窗口开启；照片模块保留这一时刻关联的整帧。
- `PASSED`：识别通过时立即发出，用于选择通过时刻的整帧。
- `NOT_PASSED`：本轮等待结束且未通过，照片模块使用该轮 `WAIT_STARTED` 的整帧。

上述事件早于设备清理/恢复，不能用后续授权证据的到达时间代替。事件 `header.stamp`
是 ROS 时钟中的事件时间；照片时间戳与文件名应使用关联相机帧的源时间戳。
取消或换目标不补发失败事件，由照片模块按任务会话清理未完成窗口。语音包不直接保存图片。
tracking 将每轮照片按结果保存到照片根目录下的 `voice/passed/`（通过）或
`voice/failed/`（未通过），文件名为
`YYYYMMDD_HHMMSS_t<ID>_rN.jpg`，结果由目录区分；
`ID` 为目标语义 ID，`N` 从 1 开始，表示第几次播报后的口令核验。同名文件追加短序号，避免覆盖。

部署时提供 Vosk 模型目录并启动：

```bash
ros2 run dog_patrol_perception_voice perception_voice_provider \
  --ros-args -p model_dir:=/path/to/vosk-model \
  -p config_file:=/path/to/voice_verification.yaml
```

`helper_path` 为空时使用安装包内的 ARM64 helper；`provider`、`mission_state_topic`、
`authorization_command_topic` 和 `authorization_evidence_topic` 可按联调命名空间覆盖。授权 evidence 与 capability readiness 是两条
独立接口：`perception_voice_provider` 只发布前者，`perception_voice_readiness` 只发布后者。

### 启动和 readiness preflight

安装后推荐使用统一 launch：

```bash
ros2 launch dog_patrol_perception_voice voice.launch.py \
  model_dir:=/path/to/vosk-model \
  config_file:=/path/to/voice_verification.yaml
```

`perception_voice_readiness` 订阅 transient-local `/mission/state`。每个有效 STARTUP
`state_seq` 只运行一次真实 preflight，并以同一 `observed_startup_state_seq` 发布 transient-local
`CapabilityStatus`：缺失部署输入或设备不可用为 `NOT_READY`，配置、模型或 helper 校验错误为 `ERROR`，
全部通过才为 `READY`；旧 STARTUP 的 retained status 不会匹配新的 sequence。

离开 STARTUP 后不会重新运行 preflight；迟到旧 STARTUP 快照和旧 generation 结果都不能发布新的
readiness 状态。
provider 和 readiness 默认使用 `state_timeout=0.75 s`；MissionState heartbeat 超时会取消当前
语音会话并使在途 preflight 结果失效，取消仍只产生技术记录。

preflight 只读取安装后的配置/helper、加载 Vosk 模型、执行 `adb get-state`、查询 FFmpeg 的 flite
filter 和 ALSA 的 `aplay -L`/`amixer scontrols`。它不创建 voice task，不执行 ADB push/shell，不接管
R818，不停止 `vtn_init`/demo，不改变 mixer，不播放 Prompt；readiness 不等价于真实语音效果或现场
FAR/FRR 验收。

### 无人参与的 Orin 验收

安装产物还提供 `perception_voice_acceptance`。它从部署机 fixture 注入识别结果，自动
验证 readiness、voice provider、authorization adapter、R818 清理和取消/替换/故障矩阵，
不保存 PCM，也不等待用户说话。`--mode fixture` 适合 CI；`--mode hardware` 才接管真实
ADB/AC107/R818，并需要部署机提供 3 条代表性任务结果、Vosk model、实际配置和通过统一感知环境
门禁的 `--environment-check-command`。硬件模式会对真实资源执行故障矩阵，并记录主机 PCM、
远端残留、厂商 `demo` owner 和 fixture provenance 证据。完整命令、fixture 格式和验收边界见
[`docs/perception/voice/issue37_voice_hardware_acceptance.md`](../../../docs/perception/voice/issue37_voice_hardware_acceptance.md)。

### 最终用户现场验收

`perception_voice_acceptance --mode field` 使用真实 R818/Vosk 结果执行固定的三项最小用户矩阵：
首窗通过、首窗无应答后次窗通过、两窗无应答。它不接受 fixture，且必须传入同一 model、config
和安装 `acceptance.py` 指纹已通过的 #37 硬件报告；报告只保存 PASSED/NOT_PASSED、ROS 任务关联和清理证据，
不保存 PCM、识别文本或口令。取消、替换和故障的无迟到输出由该 #37 报告覆盖。2026-08-07 的
`field` 报告已通过，固定来源映射归档为
`archive/dog-patrol-deployment-b979a7f-issue38`；这不构成最终口令、错误口令拒绝、FRR/FAR
或安全准入声明。完整命令、报告摘要和归档边界见
[`docs/perception/voice/issue38_voice_field_acceptance.md`](../../../docs/perception/voice/issue38_voice_field_acceptance.md)。

## 配置和安装资产

- 默认配置：`config/voice_verification.yaml`，只包含当前 `blue star` 口令、两条 Prompt、响应窗和 R818/Prompt
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
