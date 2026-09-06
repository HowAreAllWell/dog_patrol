# Tracking 公共 mission 合同集成

> 本文保留公共 mission 合同的历史验收记录和证据入口。当前实现已经移除旧的
> blocked/reacquire 公共协议；现行状态、事件和恢复行为以
> [`../../contracts/perception_navigation_system_implementation.md`](../../contracts/perception_navigation_system_implementation.md)
> 为准。原始感知记录和协议明细保留在旧文档中，仅作为历史参考。工作区当前的自动化验证以 core/transport/mission transaction 单测和
> `tools/fake_integration`、`tools/navigation_coordinator` 为主。

历史上的 `test_mission_pipeline_integration` 曾是 tracking 在普通 CI 中的最高公共 seam；其测试只依赖主仓构建产物，
由 shell fixture 启动安装后的 `dog_patrol_manager mission_supervisor`，C++ driver 则使用 production
`PrimaryTargetManager`、`MissionFrameTransaction` 和 `MissionRosAdapter`；测试不复制状态机，也不依赖
旧视觉工作区、模型、录制或 Orin SDK。

默认无资产路径覆盖以下合同：

- STARTUP readiness 使真实 supervisor 进入 PATROL；可信 semantic 主目标只产生一次
  `TARGET_CONFIRMED`，PATROL 不发布 bbox。
- 只有任务状态允许的 `CONFIRM_TARGET`、`APPROACH_TARGET` 和 `VERIFY_IDENTITY` 发布当前帧 bbox；消息保留
  semantic `target_id`、源时间戳、光学 frame、原图尺寸和 clamped half-open 像素坐标。
- 持续缺失达到配置的丢失确认时限（当前默认 10 秒）时只产生一次 `TARGET_LOST`，缺失帧不复用缓存框；时限内的
  短暂漏检由 detector/tracker 内部处理，恢复后继续当前任务；`TARGET_LOST` 发出后不再恢复旧任务 bbox。
- 旧 `state_seq`、等序列冲突状态、错误 `target_id` 和只含错误目标的当前帧 evidence 不改变权威 snapshot，
  也不产生公共 event 或 bbox。
- tracking 自身不产生 `AUTHORIZED` 或 `UNAUTHORIZED`；集成 driver 以独立授权结果 owner 的身份推进
  supervisor，以继续核验 VERIFY_IDENTITY 后的 lifecycle。

完整普通环境验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_manager dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=OFF
source install/setup.bash
colcon test --packages-select dog_patrol_perception_tracking \
  --event-handlers console_direct+
colcon test-result --verbose
```

在有受控资产和 Orin runtime 的部署环境中，driver 仍可通过 `--visual-video`、`--detector-engine` 和
`--tracker-config` 把实际 detector/tracker/identity observations 送入同一组 mission 断言。该显式模式
不是普通 CI 的前置条件。
