# dog_patrol

机器狗巡逻项目的主仓库。当前仓库负责主任务状态机、跨模块 ROS 2 接口和联调约定；导航与感知实现按团队目录独立演进。

## 当前状态

- `dog_patrol_interfaces`：已实现，保存主状态机、任务事件、目标框和导航状态消息。
- `dog_patrol_manager`：已实现，包含 ROS-independent 状态机和 `mission_supervisor` ROS 2 节点。
- `navigation/`：只保留模块入口说明，导航实现尚未迁入。
- `dog_patrol_perception_orchestrator`：已实现纯 Python 授权编排；readiness、真实人脸和语音结果尚未接入。
- 现有 `vision_demo_ws` 尚未迁入；tracking 迁移候选已冻结在远端分支
  `deploy/dog_patrol-integration` 的提交
  `380b44582c0c55e5e46d2eb862da7700f05349b3`，迁入/排除、历史敏感信息和许可证审计见
  [`docs/issue4_tracking_baseline_audit.md`](docs/issue4_tracking_baseline_audit.md)。
- 人脸实现尚未建立。
- 目标公开远程：`https://github.com/HowAreAllWell/dog_patrol`

## 目录

```text
src/contracts/dog_patrol_interfaces/       # 两团队共同维护的 ROS 2 合同
src/orchestration/dog_patrol_manager/      # 主状态机和 supervisor
src/navigation/                            # 导航模块预留位置
src/perception/                            # 感知业务编排入口
docs/contracts/                             # 可评审的接口协议
docs/workflows/                             # 业务流程参考文档
```

ROS 2 package 名称保持 `dog_patrol_` 前缀；上层目录是所有权和代码组织目录，不是额外的 ROS 2 package。

## 运行和验证

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

模型、录制视频、相机日志、人脸白名单、特征向量和现场配置不进入公开仓库；它们必须通过本机部署或受控资产目录提供。

## 核心协作约定

- `dog_patrol_interfaces` 是跨团队共享合同，不属于导航或感知任一实现目录。
- 主状态机只编排业务状态，不实现检测、跟踪、人脸或语音算法。
- 导航和感知可以通过 ROS 2 直接交换数据，但不得依赖对方的私有代码 Module。
- `TARGET_LOST` 和 `TARGET_REACQUIRED` 由感知发布；导航发现 bbox 过期时先本地停车并发布导航 `BLOCKED` 状态。
- 感知内部的授权流程只向主状态机映射最终授权、未授权或技术错误结果。

详细合同见 [`docs/contracts/perception_navigation_interface.md`](docs/contracts/perception_navigation_interface.md)，业务流程见 [`docs/workflows/机器狗巡逻与可疑目标处置流程（更新后）.docx`](docs/workflows/机器狗巡逻与可疑目标处置流程（更新后）.docx)。

感知编排模块说明见 [`src/perception/README.md`](src/perception/README.md)。

## 协作方式

日常修改通过短分支和 Pull Request 合并到 `main`。模块所有权由 `.github/CODEOWNERS` 管理，合并前必须通过 CI 和对应 owner 审查。
