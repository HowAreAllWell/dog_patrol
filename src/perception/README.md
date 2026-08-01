# 感知模块

感知团队的实现入口。现有 `vision_demo_ws` 仍是独立仓库，人脸实现尚未建立，当前不接入任何具体算法。

`dog_patrol_perception` 当前提供：

- `fake_perception`：跨团队 ROS 2 合同联调替身；
- `AuthorizationCoordinator`：不依赖具体算法的两轮授权编排；
- Fake 授权结果到公共任务事件的映射；
- Topic 级跨包联调测试。

Fake 感知不是生产实现，不得加入实车生产 launch。后续迁入时，检测、跟踪和现有视觉链路保持在感知实现内部；与主状态机交互只使用 `dog_patrol_interfaces`，感知内部的人脸、语音和授权流程不直接暴露给主状态机。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_manager dog_patrol_perception
source install/setup.bash

# 终端 1：主状态机
ros2 run dog_patrol_manager mission_supervisor

# 终端 2：Fake 感知
ros2 launch dog_patrol_perception fake_perception.launch.py
```

主状态机仍会等待导航团队发布导航 `READY`；本仓库不会启动 Fake 导航。

## 联调控制

所有控制服务都是 Fake 节点的私有测试 Interface，不属于跨团队公共合同：

| 服务 | 行为 |
|---|---|
| `/fake_perception/confirm_target` | 在未阻塞 `PATROL` 中发布 `TARGET_CONFIRMED` |
| `/fake_perception/target_lost` | 为当前活动目标发布 `TARGET_LOST` 并停止 bbox |
| `/fake_perception/target_reacquired` | 在目标丢失阻塞中发布 `TARGET_REACQUIRED` |
| `/fake_perception/execution_error` | 发布感知 `EXECUTION_ERROR` |
| `/fake_perception/authorization_passed` | 发布 `AUTHORIZED` |
| `/fake_perception/authorization_not_passed` | 记录一次有效未通过；第二次才发布 `UNAUTHORIZED` |
| `/fake_perception/authorization_error` | 发布 `EXECUTION_ERROR`，不标记未授权 |
| `/fake_perception/authorization_cancelled` | 取消内部授权，不发布公共业务结论 |

调用示例：

```bash
ros2 service call /fake_perception/confirm_target std_srvs/srv/Trigger '{}'
```

Fake 节点在 `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY` 和
`TRACK_INTRUDER` 的未阻塞状态持续发布配置好的 bbox。状态改变、目标改变或阻塞会取消
当前授权会话；重复的同一 `state_seq` 不会重新启动授权。

## 授权编排 Interface

`AuthorizationCoordinator` 是纯 Python Module，不依赖 ROS 2 或具体算法。调用方通过
`start(AuthorizationSession)` 启动当前 `state_seq + target_id` 会话，再通过
`record(session, result)` 提交 `PASSED`、`NOT_PASSED`、`ERROR` 或 `CANCELLED`。

- `PASSED`、`ERROR` 和 `CANCELLED` 立即结束当前会话；
- 第一次有效 `NOT_PASSED` 返回 `None`，第二次返回最终未通过结果；
- 非当前 `state_seq + target_id` 的旧结果返回 `None`，不能影响新会话；
- ROS Adapter 只将最终 `PASSED`、两轮 `NOT_PASSED` 和 `ERROR` 映射为公共事件，
  `CANCELLED` 不发布业务结论。
