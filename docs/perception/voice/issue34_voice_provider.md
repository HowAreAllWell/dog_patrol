# Issue #34：MissionState 驱动的异步 voice evidence provider

## 入口和合同

生产 ROS 入口是 `dog_patrol_perception_voice` package 的
`perception_voice_provider` executable。它订阅 reliable + transient-local 的
`/mission/state`（`MissionState`），并向 reliable + volatile 的
`/perception/authorization_evidence` 发布现有 `AuthorizationEvidence`；本 issue 不新增或修改
公共 ROS message、manager 状态机、授权聚合规则或 `required_not_passed=2`。

provider 只接受未阻塞的 `VERIFY_IDENTITY` 和正数 `target_id`。`state_seq + target_id` 相同的周期
状态只保留一个任务；首次进入会在独立 worker 中建立一个 `R818VoiceAdapter` task session，最多
执行两个 response window。第一窗失败立即发布 `NOT_PASSED` 并继续第二窗；第二窗失败再发布
`NOT_PASSED`，任一窗通过发布 `PASSED`。模型、ADB、stream、helper、Prompt 或恢复故障发布
`ERROR`。

## 取消和并发边界

状态阻塞、离开 `VERIFY_IDENTITY`、`state_seq` 变化或 `target_id` 变化会递增内部 generation，
请求当前 task 的 cooperative stop，并发布旧 session 的 `CANCELLED`。迟到 response 在发布前同时
通过 generation 和 session 检查；旧 task 的 context cleanup 完成后，worker 才会启动新 task，因而
任意时刻最多存在一个 R818 hardware session。ROS MissionState callback 只更新 desired session 和
stop event，不执行 Prompt、模型加载、音频响应窗或硬件恢复。

任务核心的 `R818TaskSession.cancel()` 只发非阻塞的 `request_stop()`；生产 Prompt player 通过
可终止的 `Popen` 子进程响应 stop，真正的 ADB 清理和厂商音频恢复仍由 task context 的 `close()`
负责。恢复失败会发布 `ERROR` 并锁存 provider 的硬件故障，不启动下一代 hardware session。provider
的默认生产构造从 `model_dir` 和可选
`config_file` 创建并缓存 `R818VoiceAdapter`，模型目录、ADB serial、音频设备和 Vosk model 仍由
部署机提供。

## 启动和验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dog_patrol_perception_voice perception_voice_provider \
  --ros-args -p model_dir:=/path/to/vosk-model
```

provider 测试使用 fake hardware，但让真实 `perception_authorization` ROS adapter 消费 evidence，
覆盖首窗通过、次窗通过、双失败、硬件/恢复错误、重复 MissionState、阻塞/离开 VERIFY、会话替换、
迟到结果和单硬件 session 门禁。真实 R818/ADB、Vosk 模型效果、FAR/FRR 和 capability readiness
仍需要现场部署验收。
