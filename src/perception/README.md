# 感知模块

感知团队的实现入口。现有 `vision_demo_ws` 仍是独立仓库，人脸和语音实现尚未建立，当前不接入任何具体算法。

`dog_patrol_perception_orchestrator` 当前只提供 ROS-independent 的感知业务编排：

- `AuthorizationCoordinator`：维护与 `state_seq + target_id` 绑定的授权会话；
- 两轮未通过、立即通过、技术错误、取消和旧会话结果拒绝规则；
- 不依赖 ROS 2、具体人脸算法或语音算法的纯 Python 测试面。

当前 package 不安装 ROS 节点、console executable 或 launch。后续接入 readiness、真实人脸和语音结果时，由独立 ROS adapter 调用编排核心；检测、跟踪和现有视觉链路保持在感知实现内部。与主状态机交互只使用 `dog_patrol_interfaces`，感知内部的人脸、语音和授权流程不直接暴露给主状态机。

## 构建和测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dog_patrol_interfaces dog_patrol_manager \
  dog_patrol_perception_orchestrator
source install/setup.bash
colcon test --packages-select \
  dog_patrol_interfaces dog_patrol_manager \
  dog_patrol_perception_orchestrator \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 授权编排 Interface

`AuthorizationCoordinator` 是纯 Python Module，不依赖 ROS 2 或具体算法。调用方通过
`start(AuthorizationSession)` 启动当前 `state_seq + target_id` 会话，再通过
`record(session, result)` 提交 `PASSED`、`NOT_PASSED`、`ERROR` 或 `CANCELLED`。

- `PASSED`、`ERROR` 和 `CANCELLED` 立即结束当前会话；
- 第一次有效 `NOT_PASSED` 返回 `None`，第二次返回最终未通过结果；
- 非当前 `state_seq + target_id` 的旧结果返回 `None`，不能影响新会话；
- 后续 ROS adapter 负责将最终结果映射为公共任务事件；该职责不下沉到人脸或语音算法。
