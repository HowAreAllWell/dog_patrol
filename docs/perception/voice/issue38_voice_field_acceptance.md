# Issue #38：最终用户现场语音矩阵

## 当前状态

已从主仓 clean install 完成 `perception_voice_acceptance --mode field` 自动门禁和真人
说话矩阵，命令返回 0 且报告 `passed=true`；固定来源映射已经 annotated archive tag
归档。该结论仍不覆盖错误口令、最终口令、FRR/FAR 或安全准入。

## 现场验收结果（2026-08-07）

- 来源提交：`moonshine_voice_commands@b979a7fd33aac5c9ced9591bb507e483faf4aef5`
- 执行提交：`dog_patrol@cf78d028aed921ea4a3e107fb9b3574cea5e98b6`
- #37 报告 SHA-256：`8c5f571da6f65f1e0eb01502190918d1c39d23c122a8725f67d2abd412b7fad3`
- #38 报告 SHA-256：`f8ab6fcd7387a062a12abd1993abbd14ac979ee6347047dc9e08e0d25ffc8a5f`
- 结果：首窗 `PASSED/AUTHORIZED`；次窗 `NOT_PASSED,PASSED/AUTHORIZED`；双窗无应答
  `NOT_PASSED,NOT_PASSED/UNAUTHORIZED`。三项均完成清理且无迟到 evidence。
- 归档 tag：`archive/dog-patrol-deployment-b979a7f-issue38`

验收使用 496 项零失败的非 symlink clean install；统一环境检查、#37 的 3 个正常生命周期与
9 个真实硬件故障场景全部通过。报告不进入 Git；上方摘要只记录允许归档的结果和指纹。

## 前置条件

现场命令必须从主仓安装产物启动，并传入同一部署输入已通过的 #37 硬件验收报告。`field`
模式拒绝 fixture，并检查报告的 `issue=37`、`mode=hardware`、`passed=true`，以及 Vosk
model、voice config、安装 helper 和安装后的 `acceptance.py` 的 SHA-256 指纹与当前运行输入
一致。这样不能以不同设备、配置、验收代码或旧报告替代现场前的自动门禁。

报告只包含结果、任务关联、清理状态和资产指纹；不包含 PCM、录音、识别文本或口令。

## 许可证与来源范围

本票新增的 field runner、测试和文档均为主仓原生内容，继续按根目录 `LICENSE` 的 BSD-3-Clause
分发。它不迁入新的源仓代码、helper、模型、PCM、录音或依赖，因此不改变 #33 已记录的
`moonshine_voice_commands` 来源范围、voice package 的 BSD-3-Clause 范围或第三方 Vosk/NumPy/PyYAML
依赖许可证。成功后的来源 tag 只固定既有部署 SHA 和验收映射，不扩大这些许可证范围。

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/dog_patrol/install/setup.bash
ros2 run dog_patrol_perception_voice perception_voice_acceptance \
  --mode field \
  --model-dir /srv/dog-patrol/vosk-model \
  --config-file /absolute/path/to/dog_patrol/install/dog_patrol_perception_voice/share/dog_patrol_perception_voice/config/voice.yaml \
  --automated-report /srv/dog-patrol/issue37_voice_acceptance.json \
  --environment-check-command "python3 /srv/dog-patrol/check_perception_environment.py --target perception-orin --params-file /srv/dog-patrol/orin_tracking.yaml --tracker-config /srv/dog-patrol/bot_sort.yaml --voice-model-dir /srv/dog-patrol/vosk-model --voice-config-file /absolute/path/to/dog_patrol/install/dog_patrol_perception_voice/share/dog_patrol_perception_voice/config/voice.yaml --install-prefix /absolute/path/to/dog_patrol/install --build-base /absolute/path/to/dog_patrol/build" \
  --report /srv/dog-patrol/issue38_voice_field_acceptance.json
```

`--environment-check-command` 仍必须输出 `PERCEPTION ENVIRONMENT: PASS`。开始前先确认
R818 已连接且由厂商 `demo` 正常持有；缺少 ADB、模型、播放链路或 #37 报告时，命令必须
非零退出而不启动现场任务。

## 用户矩阵

命令按顺序运行以下固定的三个任务；操作员只提示参与者在当前部署受控口令的窗口中回应，
不在报告或文档中记录口令本身。

| 任务 | 参与者动作 | 期望 evidence | 期望任务结果 |
| --- | --- | --- | --- |
| 1 | 第一响应窗正确回应 | `PASSED` | `AUTHORIZED` |
| 2 | 第一响应窗保持无应答，第二响应窗正确回应 | `NOT_PASSED`、`PASSED` | `AUTHORIZED` |
| 3 | 两个响应窗均保持无应答 | `NOT_PASSED`、`NOT_PASSED` | `UNAUTHORIZED` |

每项都额外检查一项任务只占用一个 R818 session、任务结束后的远端临时节点清理、厂商
`demo` owner 恢复以及无迟到 ROS 输出。中途取消、替换、stream/ADB/播放/恢复故障及其
“无迟到 evidence”断言由与当前输入指纹一致的 #37 硬件报告覆盖；因此 `field` 模式强制
要求该报告，而不让用户现场重复故障注入。

## 通过后的归档

通过后才可以为固定来源提交
`moonshine_voice_commands@b979a7fd33aac5c9ced9591bb507e483faf4aef5` 创建 annotated tag
`archive/dog-patrol-deployment-b979a7f-issue38`。tag message 必须记录：来源 SHA、执行
field 命令的 dog_patrol commit、现场报告的 SHA-256 和三个任务的聚合结果；不得纳入
口令、PCM、录音、模型或部署配置。

随后将同一映射和报告摘要回填本文件、根目录 README 与 worklog，并关闭 #38。现场失败时
不创建 tag，不更新迁移映射；先修复相应自动门禁，再从 #37 重新开始验收。

## 非目标

本票不测试错误口令拒绝、最终口令选择、FRR/FAR，也不构成安全准入声明。
