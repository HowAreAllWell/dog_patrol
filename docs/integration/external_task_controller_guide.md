# 外部任务系统控制指南

本文说明如何由 Robot Console 以外的系统控制巡检父任务。外部系统可以是同一台机器上的业务服务、调度程序、测试脚本或另一台通过 ROS 2 网络连接的上位机。

外部系统不需要调用 Robot Console，也不需要复制父任务状态机。机器人端的 `mission_supervisor` 是父任务状态和命令的唯一维护者。外部系统只通过服务发送命令，通过状态 Topic 判断结果。

## 1. 控制边界

生产控制入口只有：

```text
Service: /mission/task_command
Type:    dog_patrol_interfaces/srv/PatrolTaskCommand
```

外部系统必须遵守以下边界：

- 只调用 `/mission/task_command` 发送 START、PAUSE、RESUME、RETURN、TERMINATE。
- 只读取 `/mission/state`、`/patrol_task/state`、`/waypoint_sequence/state` 判断状态和路线。
- 不发布 `/mission/state`、`/patrol_task/state`、`/mission/event`。
- 不发布 `/global_path`、waypoint path 或底盘速度命令来代替父任务命令。
- 不通过 waypoint clicked、pause/resume、`/mission/reset` 等调试接口控制生产任务。
- 不把服务响应当成物理动作完成确认；服务响应只表示命令是否在线性化接受。

Robot Console 和外部系统是同一个服务的两个客户端，不能同时作为两个独立控制器发送相互冲突的命令。生产环境应明确一个当前控制方，并由该控制方串行发送命令。

## 2. ROS 2 环境

外部系统进程必须使用与机器人节点相同的 ROS 2 domain、网络配置和接口安装环境。例如同机运行：

```bash
cd /mnt/nvme/workspace/dog_patrol
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42                 # 使用现场实际值
```

确认服务和状态 Topic 可见：

```bash
ros2 service list | grep '^/mission/task_command$'
ros2 topic list | grep -E '^/(mission/state|patrol_task/state|waypoint_sequence/state)$'
ros2 interface show dog_patrol_interfaces/srv/PatrolTaskCommand
```

如果外部系统在另一台机器运行，必须保证两台机器使用同一个 `ROS_DOMAIN_ID`，并且 DDS 网卡、发现和防火墙配置允许通信。不要为了“控制方便”启动第二个 `mission_supervisor`。

## 3. 命令接口

`PatrolTaskCommand.srv` 的完整定义为：

```text
uint8 START=0
uint8 PAUSE=1
uint8 RESUME=2
uint8 RETURN=3
uint8 TERMINATE=4

uint8 command
---
bool accepted
bool changed
uint32 state_seq
uint8 task_state
uint8 mission_state
uint8 last_task_outcome
string error_code
string message
```

Python ROS 2 代码中，命令常量必须从 request 类读取：

```python
from dog_patrol_interfaces.srv import PatrolTaskCommand

PatrolTaskCommand.Request.START
PatrolTaskCommand.Request.PAUSE
PatrolTaskCommand.Request.RESUME
PatrolTaskCommand.Request.RETURN
PatrolTaskCommand.Request.TERMINATE
```

不要写 `PatrolTaskCommand.START`。ROS 2 Python service 常量属于 `PatrolTaskCommand.Request`。

### 3.1 命令行调用

启动巡检：

```bash
ros2 service call /mission/task_command \
  dog_patrol_interfaces/srv/PatrolTaskCommand \
  "{command: 0}"
```

暂停：

```bash
ros2 service call /mission/task_command \
  dog_patrol_interfaces/srv/PatrolTaskCommand \
  "{command: 1}"
```

继续：

```bash
ros2 service call /mission/task_command \
  dog_patrol_interfaces/srv/PatrolTaskCommand \
  "{command: 2}"
```

返航：

```bash
ros2 service call /mission/task_command \
  dog_patrol_interfaces/srv/PatrolTaskCommand \
  "{command: 3}"
```

终止：

```bash
ros2 service call /mission/task_command \
  dog_patrol_interfaces/srv/PatrolTaskCommand \
  "{command: 4}"
```

服务返回示例：

```yaml
accepted: true
changed: true
state_seq: 18
task_state: 1
mission_state: 1
last_task_outcome: 0
error_code: ''
message: 'task started'
```

`accepted=true` 表示命令已经被 supervisor 接受并发布了新的权威状态。`changed=true` 表示状态确实发生变化。`accepted=false` 时不得根据 `message` 猜测下一状态，也不得继续发送同一命令直到状态被重新读取。

## 4. 外部系统必须订阅的状态

外部系统可以在任务运行期间随时读取当前父任务阶段和 waypoint 进度，不需要通过 Robot Console 转发。
建议同时订阅 `/mission/state`、`/patrol_task/state` 和 `/waypoint_sequence/state`，并以相同的
`state_seq` 组合判断当前业务状态是否一致。

实时查看三个权威状态：

```bash
ros2 topic echo /mission/state \
  --qos-reliability reliable \
  --qos-durability transient_local

ros2 topic echo /patrol_task/state \
  --qos-reliability reliable \
  --qos-durability transient_local

ros2 topic echo /waypoint_sequence/state \
  --qos-reliability reliable \
  --qos-durability transient_local
```

只查看当前最新快照：

```bash
ros2 topic echo /mission/state --qos-reliability reliable \
  --qos-durability transient_local --once
ros2 topic echo /patrol_task/state --qos-reliability reliable \
  --qos-durability transient_local --once
ros2 topic echo /waypoint_sequence/state --qos-reliability reliable \
  --qos-durability transient_local --once
```

`WaypointSequenceState` 中的 `current_index` 从 0 开始。外部界面通常按下面方式显示：

```text
显示总点数       = waypoint_count
显示当前点       = current_index + 1
显示已完成点数   = completed_count
```

例如：

```text
waypoint_count = 5
current_index = 2
completed_count = 2
sequence_done = false
```

表示路线共 5 个点，前 2 个点已经完成，当前正在执行第 3 个点。

当 `sequence_done=true` 时，表示整条非空路线已经完成，不要再把 `current_index + 1` 显示为正在执行的点。例如：

```text
waypoint_count = 5
current_index = 4
completed_count = 5
sequence_done = true
```

表示 5 个点全部完成。`waypoint_count=0` 表示没有路线或路线已清空，即使 `sequence_done` 为 true，也不能当作正常完成。

进度推进时 `route_revision` 可以保持不变；外部系统必须处理每一条状态消息，不能只在 `route_revision` 变化时刷新进度。

### 4.1 `/mission/state`

这个 Topic 表达感知、目标和认证阶段：

```text
STARTUP=0
PATROL=1
CONFIRM_TARGET=2
APPROACH_TARGET=3
VERIFY_IDENTITY=4
TRACK_INTRUDER=5
RECOVER_PATROL=6
READY=7
```

主要字段：

```text
state_seq
state
target_id
handled_target_id
```

### 4.2 `/patrol_task/state`

这个 Topic 是父任务的权威状态：

```text
TASK_NO_TASK=0
TASK_RUNNING=1
TASK_PAUSED=2
TASK_RETURNING=3

TASK_OUTCOME_NONE=0
TASK_OUTCOME_COMPLETED=1
TASK_OUTCOME_RETURNED=2
TASK_OUTCOME_TERMINATED=3
TASK_OUTCOME_FAILED=4

NAVIGATION_STOP=0
NAVIGATION_WAYPOINT=1
NAVIGATION_MISSION=2
```

外部系统重点读取：

```text
state_seq
task_state
last_task_outcome
navigation_mode
home_pose_valid
```

`/mission/state.state_seq` 和 `/patrol_task/state.state_seq` 必须相同。外部系统只接受两条消息的 `state_seq` 相同且都是最新状态的组合；不同序号时视为状态不同步，不能继续发送依赖状态的下一条命令。

### 4.3 `/waypoint_sequence/state`

路线状态用于决定 START 或 RESUME 是否可以发送：

```text
route_revision
waypoint_count
current_index
completed_count
sequence_done
paused
```

发布路线后，外部系统必须等待新的 `route_revision` 状态，并确认：

```text
waypoint_count > 0
sequence_done == false
current_index < waypoint_count
completed_count <= waypoint_count
```

路线编辑和父任务命令必须由同一个控制方串行完成。START 成功后，直到任务结束，不能继续发送路线编辑、删除路线、替换路线或清空路线。

## 5. 各命令的前置条件和完成确认

### 5.1 START：开始巡检

发送前必须确认：

```text
task_state == TASK_NO_TASK
mission_state == READY
waypoint_count > 0
sequence_done == false
current_index < waypoint_count
```

还必须已经完成感知和导航的 READY 握手。START 被接受后，预期权威状态为：

```text
task_state     = TASK_RUNNING
mission_state  = PATROL
navigation_mode = NAVIGATION_WAYPOINT
last_task_outcome = TASK_OUTCOME_NONE
```

START 成功不表示机器人已经开始运动，只表示父任务运行许可已经建立。外部系统应继续观察 waypoint path、运动反馈和 `/patrol_task/state`。

没有显式 START 时，即使路线非空，机器人也不能执行路线。

### 5.2 PAUSE：暂停任务

PAUSE 只从 `TASK_RUNNING` 接受。发送后成功响应表示：

```text
task_state      = TASK_PAUSED
mission_state   = READY
navigation_mode = NAVIGATION_STOP
```

PAUSE 会保留：

- home pose；
- waypoint 列表；
- route revision；
- current index 和完成进度；
- 已处理目标排除上下文。

PAUSE 不重新执行 readiness，不执行返航，也不清除路线。服务成功不表示底盘已经物理停稳；外部系统需要根据底盘/运动反馈确认停止。

外部系统应等待 `/patrol_task/state` 确认 `TASK_PAUSED + state_seq` 更新后，再允许用户发送 RESUME。

### 5.3 RESUME：继续任务

RESUME 只从以下组合接受：

```text
task_state    = TASK_PAUSED
mission_state = READY
```

并且路线必须仍然非空、未完成、进度合法。成功后有两种结果：

没有有效巡检中断点时：

```text
task_state      = TASK_RUNNING
mission_state   = PATROL
navigation_mode = NAVIGATION_WAYPOINT
target_id       = 0
```

暂停发生在目标接近、认证、跟踪或恢复阶段并且有有效中断点时：

```text
task_state      = TASK_RUNNING
mission_state   = RECOVER_PATROL
navigation_mode = NAVIGATION_MISSION
target_id       = 0
```

`target_id=0` 的 `RECOVER_PATROL` 表示暂停恢复上下文，不表示重新创建人员目标或认证会话。外部系统应依据新的 `/mission/state` 和 `/patrol_task/state` 判断恢复方向，不能强行把它改成 PATROL。

如果路线已经完成、路线为空或进度非法，RESUME 会被拒绝。此时外部系统应选择 RETURN 或 TERMINATE，不能用 RESUME 重新打开旧路线。

### 5.4 RETURN：返航

RETURN 可以从：

```text
TASK_RUNNING
TASK_PAUSED
```

接受。成功响应只表示返航上下文建立，预期状态为：

```text
task_state      = TASK_RETURNING
mission_state   = READY
navigation_mode = NAVIGATION_MISSION
home_pose_valid = true
```

返航目标是 START 时保存的 home pose，不是当前 pose，也不是最新路线起点。返航过程中机器人端会根据当前位姿周期性重新规划到同一个 home pose。

外部系统必须等待最终 `/patrol_task/state`：

成功到家：

```text
task_state       = TASK_NO_TASK
mission_state    = READY
last_task_outcome = TASK_OUTCOME_RETURNED
```

返航失败或 watchdog 超时：

```text
task_state       = TASK_NO_TASK
mission_state    = READY
last_task_outcome = TASK_OUTCOME_FAILED
```

RETURN 服务成功不等于已经到家。只有最终 `last_task_outcome` 才是父任务返航结果。

### 5.5 TERMINATE：终止任务

TERMINATE 可以终止运行中、暂停中或返航中的父任务。它不会返航。

成功后预期状态为：

```text
task_state       = TASK_NO_TASK
mission_state    = READY
navigation_mode  = NAVIGATION_STOP
last_task_outcome = TASK_OUTCOME_TERMINATED
```

TERMINATE 会立即关闭父任务运动许可、清除全局路径、取消目标和认证会话、清理巡检路线。旧的路径、planner 回调、bbox 或认证结果不能重新生效。

服务成功表示父任务状态已经转为终止态，不表示所有底层异步线程已经完成退出；外部系统仍应观察运动反馈确认机器人停稳。

## 6. 推荐的外部控制流程

### 6.1 启动一个新任务

```text
1. 订阅 /mission/state、/patrol_task/state、/waypoint_sequence/state
2. 等待 state_seq 对齐
3. 等待 TASK_NO_TASK + READY
4. 准备路线
5. 等待新的 route_revision
6. 检查路线非空且 sequence_done=false
7. 串行调用 START
8. 检查服务 accepted=true
9. 等待 TASK_RUNNING + PATROL
10. 观察运动和路线进度
```

### 6.2 暂停后继续

```text
1. 调用 PAUSE
2. 等待 TASK_PAUSED + READY
3. 等待运动反馈确认停稳
4. 调用 RESUME
5. 根据新的 MissionState 判断 PATROL 或 RECOVER_PATROL
6. 观察 waypoint 或恢复路径重新建立
```

### 6.3 结束并返航

```text
1. 调用 RETURN
2. 等待 TASK_RETURNING + READY
3. 观察 navigation_mode=MISSION 和返航路径
4. 等待 TASK_NO_TASK + READY
5. 读取 last_task_outcome
6. RETURNED 表示到家，FAILED 表示返航失败
```

### 6.4 立即终止

```text
1. 调用 TERMINATE
2. 等待 TASK_NO_TASK + READY
3. 确认 last_task_outcome=TERMINATED
4. 额外观察底盘反馈确认停稳
```

## 7. Python 外部客户端示例

以下示例只负责调用服务，不发布任何机器人状态：

```python
#!/usr/bin/env python3
import sys

import rclpy
from dog_patrol_interfaces.srv import PatrolTaskCommand
from rclpy.node import Node


class ExternalTaskClient(Node):
    def __init__(self):
        super().__init__("external_task_client")
        self.client = self.create_client(
            PatrolTaskCommand, "/mission/task_command"
        )

    def send(self, command: int):
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/mission/task_command is not available")
        request = PatrolTaskCommand.Request()
        request.command = int(command)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise TimeoutError("task command service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("task command returned no response")
        return response


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: external_task_client.py START|PAUSE|RESUME|RETURN|TERMINATE")
    command = {
        "START": PatrolTaskCommand.Request.START,
        "PAUSE": PatrolTaskCommand.Request.PAUSE,
        "RESUME": PatrolTaskCommand.Request.RESUME,
        "RETURN": PatrolTaskCommand.Request.RETURN,
        "TERMINATE": PatrolTaskCommand.Request.TERMINATE,
    }[sys.argv[1].upper()]
    rclpy.init()
    node = ExternalTaskClient()
    try:
        response = node.send(command)
        print({
            "accepted": response.accepted,
            "changed": response.changed,
            "state_seq": response.state_seq,
            "task_state": response.task_state,
            "mission_state": response.mission_state,
            "last_task_outcome": response.last_task_outcome,
            "error_code": response.error_code,
            "message": response.message,
        })
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

调用示例：

```bash
source /opt/ros/humble/setup.bash
source /mnt/nvme/workspace/dog_patrol/install/setup.bash
export ROS_DOMAIN_ID=42
python3 external_task_client.py START
python3 external_task_client.py PAUSE
python3 external_task_client.py RESUME
python3 external_task_client.py RETURN
python3 external_task_client.py TERMINATE
```

生产客户端应把服务调用和状态订阅放在同一个外部任务服务中，并保存自己的业务任务 ID、调用日志和状态历史。业务 ID 不写入 `PatrolTaskCommand`，也不能自行扩展 ROS 接口字段。

## 8. 超时、拒绝和重试

服务调用超时时，不能直接重发命令。先读取：

```bash
ros2 topic echo /mission/state --once
ros2 topic echo /patrol_task/state --once
ros2 topic echo /waypoint_sequence/state --once
```

然后按当前权威状态决定是否重试：

- 已经进入目标状态：把命令视为已生效，不再重发；
- 仍在原状态：确认是否满足前置条件后再重试；
- 收到状态冲突、state_seq 不一致或 heartbeat 超时：停止发送命令，等待状态恢复；
- `accepted=false`：读取 `error_code` 记录原因，修正前置条件后再发送；
- 不要解析 `message` 作为状态机逻辑，程序只使用枚举字段和布尔字段。

服务不提供 `command_id` 或命令历史查询。外部系统如需幂等记录，必须在自己的数据库中保存请求时间、命令、服务响应和随后观察到的 `state_seq`，不能把业务 ID 添加到机器人接口。

## 9. 常用状态查看命令

```bash
ros2 topic echo /mission/state \
  --qos-reliability reliable \
  --qos-durability transient_local

ros2 topic echo /patrol_task/state \
  --qos-reliability reliable \
  --qos-durability transient_local

ros2 topic echo /waypoint_sequence/state \
  --qos-reliability reliable \
  --qos-durability transient_local
```

检查服务连接：

```bash
ros2 service type /mission/task_command
ros2 service info /mission/task_command
```

外部系统以 `/patrol_task/state` 的 `task_state + last_task_outcome` 作为父任务结果，以 `/mission/state` 的 `state` 作为当前业务阶段，以两者相同的 `state_seq` 作为一致性条件。
