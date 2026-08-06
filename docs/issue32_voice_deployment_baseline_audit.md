# dog_patrol #32：语音部署候选基线审计

- 审计日期：2026-08-06（Asia/Shanghai）
- 源仓：本地 `moonshine_voice_commands` 工作区
- 目标仓：`HowAreAllWell/dog_patrol`
- 对应问题：[dog_patrol #32](https://github.com/HowAreAllWell/dog_patrol/issues/32)
- 信息处理：数据、模型、设备序列号和本机配置不写入 Git；本文只记录路径类别、校验值、版本和可复现命令。

## 结论

语音部署候选冻结为源仓提交
`b979a7fd33aac5c9ced9591bb507e483faf4aef5`，提交 tree 为
`2cc4e3fc0030ec8b3f618f6f3c624956889eec4d`。源仓本地已经从该提交创建
`deploy/dog-patrol-integration`；源仓 `main` 和该部署分支当前指向同一提交，分支间没有差异。
本票没有必要的代码或配置准备改动，因此不修改 `main`，部署分支也保持在原提交。

本票冻结的是后续选择性迁入的候选和证据，不表示语音模块已经进入
`dog_patrol`，也不表示当前目标口令已经通过生产 FAR/FRR 或整机验收。后续迁入必须使用本文
允许清单重新组织 package，不能把源仓当前的通用入口和历史路线整体复制进主仓。

## 1. 冻结起点与工作区状态

冻结时源仓事实如下：

| 项目 | 结果 |
| --- | --- |
| `main` | `b979a7fd33aac5c9ced9591bb507e483faf4aef5` |
| `deploy/dog-patrol-integration` | `b979a7fd33aac5c9ced9591bb507e483faf4aef5` |
| commit tree | `2cc4e3fc0030ec8b3f618f6f3c624956889eec4d` |
| 提交时间 | `2026-07-30T12:11:07+08:00` |
| 提交主题 | `docs: confirm one-mic passphrase threshold` |
| tracked 工作树 | 干净；`git status --porcelain` 无输出 |
| 源仓远端 | 当前本地 clone 未配置 Git remote，部署分支尚未推送到远端 |

复核命令：

```bash
cd /home/user/workspace/moonshine_voice_commands
git status --porcelain=v1 --branch
git show -s --format='%H%n%T%n%aI%n%s' main
git show-ref --verify refs/heads/main
git show-ref --verify refs/heads/deploy/dog-patrol-integration
git diff --exit-code main...deploy/dog-patrol-integration
```

源仓无 remote 是当前环境事实，不改变本地冻结结论；若后续需要公开部署分支，必须先为源仓配置经过确认的远端，再推送同一 SHA，并重新核对分支未发生漂移。

## 2. 授权结论与来源

本票按当前任务要求记录以下迁入结论：

- 后续允许迁入的源仓自有代码、配置和对应测试，纳入 `dog_patrol` 时按主仓
  BSD-3-Clause 处理；主仓根 `LICENSE` 继续作为该范围的发行许可。
- 允许清单不包含 Vosk 模型、PCM/WAV、第三方源码或其他需要独立授权的运行资产。
  `vosk` Python 包和模型属于部署依赖，不能因代码迁入而当作主仓自有内容。
- 源仓当前没有独立 `LICENSE`、`NOTICE` 或 SPDX header。实际迁入票必须在目标 package
  中显式保留 BSD-3-Clause 许可边界，并在引入新的第三方依赖时单独记录其版本和许可证；本票
  不把源仓缺失的许可文件解释为第三方内容的授权。
- 来源固定为上述 commit 和本地源仓；不从源仓 `main` 后续漂移内容或旧历史路线补文件。

这是候选冻结和代码归属边界记录，不替代实际公开发布前的权利人确认或第三方依赖审计。

## 3. 可复现资产、配置和依赖

数据和模型都位于源仓 Git 忽略目录，未提交进 Git。下面的 manifest SHA-256 对目录内按相对路径
排序的 `sha256sum` 行计算，复核时使用同一命令即可重建：

```bash
manifest_sha256() {
  manifest_root="${1%/}"
  find "$manifest_root" -type f -print0 | LC_ALL=C sort -z |
    while IFS= read -r -d '' file; do
      relative_path="${file#"$manifest_root"/}"
      file_hash="$(sha256sum "$file" | cut -d' ' -f1)"
      printf '%s  %s\n' "$file_hash" "$relative_path"
    done | sha256sum
}
```

| 类别 | 源仓路径 | 文件数 | 总字节数 | 校验值 |
| --- | --- | ---: | ---: | --- |
| R818 流式历史 PCM | `captures/r818-stream-acceptance-20260730/` | 57 | 218,239,040 | `18248932edb35672ab3b223d23d45cac9fddbe6ec9e259cf7a80fc7335d36dc6` |
| Vosk 模型 | `models/vosk-model-small-en-us-0.15/` | 14 | 70,898,967 | `5c9c563ec18e9a7d176eacdb18788b1c7dde7decf40c92ee2c8374668ade9656` |

逐文件路径和 SHA-256 见 [`issue32_voice_deployment_manifest.sha256`](issue32_voice_deployment_manifest.sha256)。
该文件不包含数据或模型本体，只用于固定清单；上表聚合值由同一逐文件行集合计算。

候选代码和配置的校验值如下：

| 项目 | SHA-256 |
| --- | --- |
| `config/commands.yaml` | `f940637b570fe0a3d3d89bc650854525887e5c0f31f9deec56b1a49d9e2445cb` |
| `pyproject.toml`（冻结时源仓元数据） | `87d6c514030333ef6323227110685c43a7783c2ae642202e80a36a55f8a09253` |
| `src/moonshine_voice_commands/assets/r818_pcm_base64_aarch64` | `c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52` |
| `tools/r818_pcm_base64.c` | `b6c696fff8a237aca45a64a9264696a872e20e14e3e1f5e71dc664c13a81ffe6` |
| `tools/check.sh` | `96e8a9293ad5d38d86df206b72e9a64703eda02e088785370481e56367a071ff` |
| `docs/issue32_voice_deployment_dependency_freeze.txt` | `f0df5ed316045baba3021ae8a0391c59dd8d8eff601a0970bf16b7525bbea054` |

冻结环境中与候选验证直接相关的版本为：Python `3.10.12`、`vosk==0.3.45`、
`PyYAML==6.0.3`、`numpy==2.2.6`、`pytest==8.4.2`、`ruff==0.15.22`。
源仓环境同时安装了 `moonshine-voice==0.0.52` 和 `sherpa-onnx==1.13.4`，但二者属于本文
明确排除的路线，不是后续语音部署 package 的依赖结论。

完整候选 Python 环境见 [`issue32_voice_deployment_dependency_freeze.txt`](issue32_voice_deployment_dependency_freeze.txt)。
它来自冻结源仓的 `env -u PYTHONPATH .venv/bin/python -m pip freeze --exclude-editable`，包含
运行时、测试和传递依赖的精确版本；清除 `PYTHONPATH` 是为了不把外部 ROS overlay 包混入语音
候选环境。可编辑的源仓项目本身不写入清单，因为其来源由本节的 commit、tree 和代码校验值固定。
`pyproject.toml` 中仍保留开发期的版本范围，安装验证必须使用该冻结清单或重新生成并审查等价的
完整锁定环境。

## 4. 冻结的运行参数和验证命令

候选主链路的稳定参数来自 `b979a7f`：

| 参数 | 值 |
| --- | --- |
| R818 PCM | 16 kHz、S16_LE、8 通道交错 PCM |
| 麦克风通道 | 0–5；通道 6–7 为参考通道，不送入 Vosk |
| 口令 | `blue star`；默认 grammar 另加 Vosk `[unk]` 拒识占位 |
| Prompt 1 | `Please face my camera directly or say the passphrase loudly.` |
| Prompt 2 | `Once again, please face my camera directly or say the passphrase loudly.` |
| 响应窗 | 每次 Prompt 完成后 20 秒，最多两窗，共用一次 R818 会话 |
| 目标判定 | 完整结果精确等于目标；当前默认无对比短语，任一麦目标票可形成唯一胜者；partial 不放行 |
| vote guard | `0.75` 秒 |
| stream start timeout | `5` 秒 |
| ADB command timeout | `15` 秒 |
| vendor restore timeout | `20` 秒 |
| live buffer 上限 | 4 秒八通道帧 |
| 正式巡检落盘 | 不传 `--capture-dir`；PCM 只保留在有界内存中 |

源仓已有检查入口：

```bash
cd /home/user/workspace/moonshine_voice_commands
./tools/check.sh
env PYTHONPATH= .venv/bin/moonshine-commands --config config/commands.yaml check-config
```

候选实机入口固定为以下命令模板；设备 serial、ADB 路径和模型目录只在部署机提供，不进入 Git：

```bash
env PYTHONPATH= .venv/bin/moonshine-commands \
  --config config/commands.yaml \
  verify-r818-direct \
  --model-dir models/vosk-model-small-en-us-0.15 \
  --adb-device-serial '<R818_DEVICE_SERIAL>'
```

本票没有修改部署分支的代码或配置，因此“不变更前后使用同一批 PCM、模型和配置比较输出”的条件不触发；
后续若候选分支发生任何代码或配置修改，必须仍使用本文两组 manifest、`config/commands.yaml`、
同一组关键参数和上述入口，比较 `b979a7f` 与新候选的正例、负例及汇总结果，且不得以换数据或换模型
掩盖回退。

## 5. 允许迁入范围

以下是后续选择性迁入的边界，不是要求把这些路径原样复制：

- 任务级 R818 Base64 stream 的核心逻辑：`r818_stream.py`，包含单次 AC107 接管、Prompt
  期间丢弃、完整 16 字节帧边界、在线解码、两窗复用、清理和 vendor restore。
- 最小 ADB 文件/命令 transport：从 `r818_raw.py` 提炼 `SubprocessAdbFileTransfer`、错误类型、
  R818 格式常量和安全文件写入 helper；不迁入 raw `origin.pcm` 识别器。
- 六麦 Vosk 受限 grammar：从 `vosk_grammar.py` 和 streaming session 提炼在线 recognizer、
  `[unk]` 处理、完整结果判定和六通道交错 PCM 解复用。
- Prompt 和业务 seam：`verification.py` 的两窗 `PassphraseVerifier`、`speaker.py` 的 Prompt
  播放，以及其最小的音频/配置/响应类型依赖。
- `config/commands.yaml` 中当前产品口令的最小配置；`commands.py`、`config.py`、`audio.py`、
  `responses.py` 只保留被上述 seam 实际需要的部分。
- ARM64 静态 helper 的对应 C 源码和经过校验的部署二进制：
  `tools/r818_pcm_base64.c` 与 `src/moonshine_voice_commands/assets/r818_pcm_base64_aarch64`。
- 对应行为测试：优先保留 `test_r818_stream.py`、stream 所需的 ADB/Vosk/verification/config
  测试，以及 production Adapter 的 fake；测试回放可以使用本文 PCM，但数据仍不安装到 package。

## 6. 明确排除范围

- Moonshine、`moonshine_evaluation.py`、Moonshine streaming 评测及其模型/依赖。
- sherpa-onnx、`sherpa_kws.py`、`config/sherpa_*` 及其模型/关键词生成工具。
- 通用命令 CLI、standby listener、capture/evaluation/benchmark 工具和对应的非产品测试；后续
  package 只提供上层需要的生产 Adapter seam。
- `origin.pcm`/`iat.pcm` 文件旁路、tmpfs 逐窗录音、`R818RawVoskResponseRecognizer`、串口
  `manual_wakeup` 及 `r818-manual-wakeup` 入口。这些只作为历史取证或回退资料，不属于当前候选主链路。
- `captures/`、`models/`、`build/`、`install/`、`.venv/`、缓存、录音、模型、评测结果和本机日志。
- 源仓过程文档、个人 agent 状态、`worklog.md` 以及不直接支持上述生产 seam 的历史设计资料。

## 7. 历史对象和验证边界

源仓完整 refs 的对象检查结果为 231 个 blob，最大 tracked blob 为静态 ARM64 helper、547,952
字节；只有该 helper 达到 512 KiB 以上。PCM、WAV、模型和构建产物均未进入 Git 历史；它们只存在
于本机忽略目录，并由第 3 节的目录 manifest 固定。源仓当前没有配置 gitleaks/trufflehog，本文不
把规则扫描描述为完整高熵秘密扫描；实际迁入前仍需在过滤后的目标历史上重跑路径、依赖和敏感信息门禁。

`./tools/check.sh` 在冻结源仓运行通过：143 个 pytest、Ruff、formatter 和默认配置检查均成功。
该检查只证明 portable 代码和现有测试闭环；本机当前没有 R818、M260C 或现场设备，未在本票中新增
硬件接管、真实 PCM 回放或生产性能结论。

## 8. 后续迁入门禁

后续 issue 只能从 `deploy/dog-patrol-integration` 的冻结 SHA 开始，完成以下动作后才能在
`dog_patrol` 建立 voice package：

1. 在临时工作树中按第 5 节提炼 allowlist，重建 package metadata，删除第 6 节路线和依赖；
2. 保留 BSD-3-Clause 许可边界，记录 Vosk/模型等外部依赖的独立许可证和版本；
3. 不复制第 3 节 PCM/model；测试以本机受控资产或 fake seam 运行；
4. 用同一 manifest、配置和参数对比冻结候选与迁入候选，保存结果并证明没有正例回退或负例误放行增加；
5. 在 clean install 中验证运行时不 import、source 或访问源仓，之后再由对应 owner 评审。
