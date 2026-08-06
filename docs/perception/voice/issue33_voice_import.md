# Issue #33：voice 核心迁入说明

## 范围和来源

本 package 从本地 `moonshine_voice_commands` 的冻结提交
`b979a7fd33aac5c9ced9591bb507e483faf4aef5` 选择性提炼。#32 已固定该提交及其
`deploy/dog-patrol-integration` 等值分支；本 package 不在运行时 import、读取或调用源仓。

迁入后的公共入口是 `dog_patrol_perception_voice.R818VoiceAdapter`：

- `task()` 建立一个有限生命周期 `R818TaskSession`；任务中只调用一次 stream `start()` 和一次
  `close()`，最多执行两个 `respond()`；两个 `VoiceWindowResult` 不在模块内聚合。
- `SubprocessAdbEncodedPcmStream` 通过最小 ADB transport 推送 helper，在线解码八通道 Base64，
  保持 16 字节帧相位，Prompt 期间持续消费并丢弃帧，响应窗拥有独立墙钟 deadline。
- `R818StreamingVoskSession` 每窗新建六个 recognizer，grammar 只包含配置口令和 `[unk]`；只使用
  完整 Vosk result/final result，目标口令命中按当前窗返回通过，partial 不放行。
- `FfmpegAlsaPromptPlayer` 只承担同步 Prompt 播放；没有常驻 listener、通用命令匹配或产品 CLI。

## 允许清单映射

| 冻结候选职责 | 主仓实现 |
| --- | --- |
| task-level R818 stream | `dog_patrol_perception_voice/r818_stream.py` |
| 最小 ADB argument transport | `dog_patrol_perception_voice/adb.py` |
| 六麦受限 Vosk loader/recognition | `dog_patrol_perception_voice/vosk.py` 与 `r818_stream.py` |
| Prompt、结果和必要配置 | `prompt.py`、`result.py`、`config.py`、`config/voice.yaml` |
| ARM64 helper 与可审计 C 源码 | `dog_patrol_perception_voice/assets/r818_pcm_base64_aarch64`、`tools/r818_pcm_base64.c` |

未迁入模型、PCM/录音、构建目录、源仓 CLI、历史评测工具或任何额外识别路线。生产 Adapter
默认不接受落盘路径；测试直接通过 fake stream 注入有限 PCM，回放 seam 不安装为产品入口。

## 许可、依赖和资产验证

迁入代码和 helper 按 BSD-3-Clause 处理，package 内 `LICENSE` 和根目录
[`LICENSE`](../../../LICENSE) 均可复核。固定的 Python runtime wheels 由 package 安装产物中的
`requirements.txt` 记录；它们是独立部署依赖，不随仓库源码发布：

| 依赖 | 版本 | 许可证 | 上游许可证/来源 |
| --- | --- | --- | --- |
| Vosk | `0.3.45` | Apache-2.0 | [vosk-api](https://github.com/alphacephei/vosk-api) |
| NumPy | `2.2.6` | BSD-3-Clause（wheel 另含其依赖的 notices） | [numpy.org](https://numpy.org/doc/2.2/license.html) |
| PyYAML | `6.0.3` | MIT | [PyYAML](https://github.com/yaml/pyyaml/blob/6.0.3/LICENSE) |

Vosk 模型由部署机提供，不随仓库发布；部署时仍须按各 wheel 自带的许可证和 notices 分发。

helper 是 ELF64 little-endian AArch64 静态可执行文件，SHA-256：
`c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52`。安装后应核对：

```bash
sha256sum install/dog_patrol_perception_voice/lib/python3.10/site-packages/\
dog_patrol_perception_voice/assets/r818_pcm_base64_aarch64
file install/dog_patrol_perception_voice/lib/python3.10/site-packages/\
dog_patrol_perception_voice/assets/r818_pcm_base64_aarch64
```

对应 C 源码安装在 `share/dog_patrol_perception_voice/tools/r818_pcm_base64.c`；默认配置安装在
`share/dog_patrol_perception_voice/config/voice.yaml`。

## 验收边界

便携测试覆盖 Base64/帧对齐、Prompt 前帧丢弃、响应窗 deadline、六麦判定、技术错误和一次性
恢复，不需要 R818、ADB 或 Vosk 模型。真实设备恢复、模型效果、FAR/FRR、ROS evidence producer
和感知整体 READY 仍是后续部署/联调工作。
