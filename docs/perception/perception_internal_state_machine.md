# 感知内部状态机与任务事件说明

本文描述感知侧当前代码的行为、它与总状态机的边界，以及目标丢失和核验结果的最终闭环。
感知内部只负责检测、跟踪、主目标选择和认证结果输出；任务恢复由总状态机和导航协调器完成。

> **当前实现基线（2026-09-05）**：本文以当前工作区代码和消息定义为准。文中
> `TARGET_REACQUIRED`、`HANDLING_COMPLETE`、`blocked`、`block_cause` 和人工处置
> 分支均不属于当前公共协议；如果历史验收文档仍出现这些名称，只能按历史记录阅读。
> 感知内部的 `OCCLUDED`、`PENDING_RECOVERY` 和 `LOST` 仍然保留，但它们不是公共
> `MissionState`，也不会直接写入总状态机。

## 1. 感知侧的职责边界

感知模块负责四类工作：

1. 从相机获得图像，运行人员/车辆检测。
2. 对检测结果进行多目标跟踪，并维护原始跟踪 ID。
3. 从当前人员中选择一个业务主目标，分配稳定的语义目标 ID。
4. 在任务状态允许时，把当前业务主目标的最新 bounding box 发布给导航，并上报目标确认和目标丢失事件。

感知模块不负责以下工作：

- 根据 bounding box 和雷达点云计算地图坐标；这属于导航协调器。
- 生成到目标的路径；这属于导航协调器和 Nav2/PCT 链路。
- 判断是否到达目标 3 m 停止距离；这属于导航协调器。
- 决定发生故障后是否恢复巡检；这是总状态机的职责。
- 直接把 `AUTHORIZED`、`UNAUTHORIZED` 以外的业务状态写入总状态机。

感知侧因此是“证据提供者”，总状态机是“任务状态唯一所有者”。感知可以发布事件，但不能自行修改总任务状态。

## 2. 当前数据处理链路

当前链路如下：

```text
相机输入
  -> detector
  -> MOT tracker
  -> identity/semantic target
  -> PrimaryTargetManager
  -> MissionFrameTransaction
  -> MissionCoordinator
  -> MissionRosAdapter
  -> /perception/selected_target_bbox
  -> /mission/event
```

### 2.1 Detector

Detector 对每一帧图像产生检测框。检测框仍然属于当前图像帧，不代表已经被总任务选为目标。

检测框必须包含：

- 原始图像坐标中的 `x/y/width/height`。
- 类别和置信度。
- 当前图像的时间戳。
- 与图像一致的图像尺寸和相机 frame。

单纯出现一个 `person` 检测框，不会直接触发导航。它还必须通过 tracker、identity 和主目标选择的校验。

### 2.2 MOT tracker

跟踪器维护每个原始 track 的短期生命周期。它主要解决同一个人在相邻图像帧中的关联，不直接决定这个人是不是业务目标。

当前跟踪器参数中，`lost_threshold_frames` 控制原始目标连续缺失多少帧后进入内部
`LOST`。当前默认值是 100 帧；在当前 ROS 图像输入约 10 FPS 时约为 10 秒，直接
使用 30 FPS MVS 输入时约为 3.3 秒。它只控制 tracker/identity 的生命周期，不能
替代公共任务丢失计时。

任务级目标丢失由 `MissionCoordinator` 使用单调的 source-time 确认，当前统一为
`target.lost_event_timeout_sec=10.0` 秒。该计时在任务状态下已有目标并且后续帧仍在
被处理时进行；如果整个相机/推理线程完全停止，协调器没有新的 `Update` 调用，不能
凭空产生 `TARGET_LOST`，此时应由运行期 readiness/技术故障监控发现输入故障。

### 2.3 Identity 和 PrimaryTargetManager

Identity 层把原始 track 和业务语义目标关联起来。业务语义目标使用 semantic ID，导航任务使用的是 semantic ID，而不是容易变化的 raw track ID。

`PrimaryTargetManager` 在巡检状态中从符合条件的人员中选择业务主目标。当前选择逻辑的核心是：

- 当前帧存在合格的人员 identity。
- identity 可见、有效，并且有对应的确认 raw track。
- 目标满足面积、跳变、类别和 track 权威性检查。
- 满足条件后进入 `LOCKED`，并保留当前 semantic ID。

当前主目标生命周期包括：

```text
IDLE
  -> LOCKED
  -> OCCLUDED / PENDING_RECOVERY
  -> LOCKED             （短暂遮挡或跟踪恢复）
  -> LOST               （超过内部帧阈值）
```

进入 `LOST` 后，当前主目标会被清空，后续巡检帧可以重新选择新的业务目标。这个
层级的 `LOST` 是感知内部状态，不是直接等价于总状态机的 `TARGET_LOST`；公共事件
只有在任务目标已经建立且连续没有可信当前目标达到 10 秒时才由任务协调器发出。

## 3. MissionFrameTransaction 的作用

`MissionFrameTransaction` 把同一帧中的主目标选择和任务输出串起来，保证不会出现“主目标来自这一帧、bounding box 却来自旧帧”的组合。

它每帧按以下顺序工作：

1. 使用当前 identity 列表更新 `PrimaryTargetManager`。
2. 读取总状态机当前的 `MissionState`。
3. 如果当前是 `PATROL` 且选择出了可信主目标，发布一次 `TARGET_CONFIRMED`。
4. 如果当前处于目标任务状态，只允许发布当前任务 target ID 对应的可信 bounding box。
5. 对 bounding box 做图像边界、尺寸、时间戳和 frame 校验。
6. 如果当前目标持续没有可信 bounding box，在后续仍被处理的 source frame 中交给
   `MissionCoordinator` 按 10 秒时间阈值产生目标丢失事件。

当前允许发布目标框的总状态是：

- `CONFIRM_TARGET`
- `APPROACH_TARGET`
- `VERIFY_IDENTITY`
- `TRACK_INTRUDER`

在 `STARTUP` 和 `PATROL` 中不会发布选中目标的导航 bounding box。`PATROL` 阶段只负责发现目标并发布 `TARGET_CONFIRMED`。

## 4. 当前 MissionCoordinator 的任务级逻辑

### 4.1 正常目标流程

当前正常流程为：

```text
总状态 PATROL
  感知发现可信人员
  -> 感知发布 TARGET_CONFIRMED

总状态 CONFIRM_TARGET
  感知持续发布当前目标 bbox
  导航计算目标位置并发布 TARGET_POSITION_READY

总状态 APPROACH_TARGET
  感知持续发布当前目标 bbox
  导航跟踪移动目标并在停止距离发布 ARRIVED_AND_STOPPED

总状态 VERIFY_IDENTITY
  感知持续发布当前目标 bbox
  授权协调器开始内部核验
```

### 4.2 当前目标丢失判断

当前任务级丢失判断基于“最后一次可信、可投影 bounding box 的 source time”：

- `target.lost_event_timeout_sec` 默认是 10.0 秒。
- 目标在任务状态中有效，但超过该时间没有新的可信 bounding box 时，感知产生一次 `TARGET_LOST`。
- 感知不会用其他人的 bounding box 替代当前任务目标。
- 目标丢失期间不会继续发布旧的缓存框。

该逻辑对 `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY` 和 `TRACK_INTRUDER` 共用，因此理论上覆盖：

1. 前往目标过程中丢失。
2. 到达后核验过程中丢失。
3. 核验失败进入持续跟踪后丢失。

### 4.3 目标丢失后的任务收口

当前不存在单独的目标重新获取事件或 retention 参数。短时漏检由 detector/tracker 内部处理；
任务级协调器只关心最后一次可信目标框：

- 在 `lost_event_timeout_sec` 内重新出现时，继续发布当前任务的可信 bbox；
- 连续丢失达到超时后，只发布一次 `TARGET_LOST`；
- `TARGET_LOST` 发出后，同一个 `state_seq` 内禁止重新发布该任务的 bbox；
- 总状态机收到事件后进入 `RECOVER_PATROL`，导航完成恢复并发布
  `PATROL_RECOVERY_COMPLETE` 后才回到新的 `PATROL` 状态序号；
- 只有新的 `PATROL` 状态序号允许重新选择目标。

## 5. 当前授权内部状态机

授权协调器只在总状态为 `VERIFY_IDENTITY` 且目标有效时激活。

当前内部阶段为：

```text
INITIAL_FACE
  初始只做人脸识别，不播放提示
  通过 -> PASSED
  未通过 -> DUAL_FIRST

DUAL_FIRST
  同时等待人脸和口令结果
  任一通过 -> PASSED
  两者都未通过 -> DUAL_SECOND

DUAL_SECOND
  再次同时等待人脸和口令结果
  任一通过 -> PASSED
  两者都未通过 -> NOT_PASSED
```

当前授权结果到总状态机事件的映射是：

| 授权内部结果 | 对外事件 | 总状态机行为 |
|---|---|---|
| `PASSED` | `AUTHORIZED` | `VERIFY_IDENTITY -> RECOVER_PATROL -> PATROL` |
| `NOT_PASSED` | `UNAUTHORIZED` | `VERIFY_IDENTITY -> TRACK_INTRUDER` |
| `ERROR` | `EXECUTION_ERROR` | 目标任务中进入巡检恢复；无活动目标时只记录诊断 |

按目前确定的业务语义，最终两轮人脸和口令均失败时，应视为“核验超时”，但外部接口仍保留 `UNAUTHORIZED` 这个名称。也就是说：

- 不新增必须由导航识别的新业务状态。
- `UNAUTHORIZED` 在这个流程中表示“核验未通过，按核验超时处理”。
- 进入 `TRACK_INTRUDER` 后，只要目标仍可跟踪，就持续发布目标框给导航。
- `TRACK_INTRUDER` 仍持续跟踪目标，不要求操作员参与才能继续。
- 不需要操作员事件才能结束该流程；目标持续存在时继续跟踪，目标最终丢失时由
  `TARGET_LOST` 进入统一的 `RECOVER_PATROL`。

## 6. 事件和状态的责任归属

### 6.1 感知发布的事件

当前感知可以发布：

- `READY`
- `TARGET_CONFIRMED`
- `TARGET_LOST`
- `AUTHORIZED`
- `UNAUTHORIZED`
- 感知自身技术故障对应的 `EXECUTION_ERROR`

### 6.2 导航发布的事件

导航负责发布：

- `READY`
- `TARGET_POSITION_READY`
- `ARRIVED_AND_STOPPED`
- 导航规划、TF、点云融合等技术错误对应的 `EXECUTION_ERROR`

当前接口中 `TARGET_LOST` 的发布者是感知。导航虽然也可能发现目标位置无法由点云计算出来，但这不是图像目标丢失，不能直接伪装成感知的 `TARGET_LOST`。如果需要由导航报告“目标仍在相机中、但已经离开有效点云/地图区域”，应单独定义导航侧语义，或者明确允许导航发布一个不同的目标位置失效事件。

### 6.3 总状态机的责任

总状态机负责：

- 保存唯一的当前业务状态和 `state_seq`。
- 校验事件来源、目标 ID 和状态序号。
- 根据事件推进业务状态。
    - 根据正式事件决定目标任务是否进入恢复，并由导航执行停车、清理和巡检恢复。
- 清空旧目标，防止旧目标数据进入下一次巡检任务。

总状态机不应该根据某个感知内部阶段自行推测核验结果，也不应该根据某一帧图像直接修改导航状态。

## 7. 已确认的闭环行为

以下是当前采用的任务级逻辑。感知内部只提供数据和事件，总状态机负责统一收口，导航负责执行恢复。

### 7.1 核验失败

```text
VERIFY_IDENTITY
  -> 初始人脸失败
  -> 第一轮人脸+口令均失败
  -> 第二轮人脸+口令均失败
  -> 对外仍发布 UNAUTHORIZED
  -> 总状态进入 TRACK_INTRUDER
```

这里的 `UNAUTHORIZED` 需要在 `detail` 或日志中标明“核验超时”，避免后续人员误以为它是一个需要人工处置完成的终态。

### 7.2 进入持续跟踪后

```text
TRACK_INTRUDER
  目标仍有可信 bbox
  -> 感知持续发布 bbox
  -> 导航持续更新目标地图位置和跟踪路径

TRACK_INTRUDER
  目标丢失
  -> 感知上报目标丢失
  -> 导航停车
  -> 总状态上报异常并恢复巡检
```

这里“目标仍有可信 bbox”是继续跟踪的唯一必要条件，不需要等待操作员参与。

### 7.3 10 秒目标丢失确认窗口

当前实现把两个层次明确区分：

1. **短时漏检处理**：用于防止单帧漏检或短暂遮挡立即改变业务状态。期间由 detector/tracker
   保留内部跟踪结果，恢复后继续提供当前帧证据；这不是公共 mission event。
2. **最终丢失确认**：同一任务目标从最后一次可信 observation 开始，在后续处理帧的
   source-time 上持续丢失达到 10 秒后，产生一个明确事件，让总状态机结束目标任务并
   进入 `RECOVER_PATROL`，再由导航恢复并回到 `PATROL`。

最终采用现有 `TARGET_LOST` 事件承载任务级丢失，不新增“最终丢失”事件类型。事件
只在最终丢失确认后发送，短暂丢失完全由感知内部处理。事件流程是：

```text
可信目标最后一次出现
  -> 短时丢失：感知内部保持，不发布任务事件
   -> 10 秒内同目标重新出现：继续当前任务，不发布额外事件
   -> 超过 10 秒仍未出现：发送一次 TARGET_LOST
  -> 总状态结束目标任务并进入 RECOVER_PATROL
  -> 导航恢复巡检后发送 PATROL_RECOVERY_COMPLETE，总状态回到 PATROL
```

`TARGET_LOST` 的 `detail` 应携带具体原因，例如 `target_lost_timeout_10s`，便于日志和
报警模块区分“接近时丢失”“核验时丢失”和“持续跟踪时丢失”。不能在短暂漏检时就
发送该事件，否则总控会提前结束任务。也不应通过重复发送 `TARGET_LOST` 表示不同
阶段，因为总控收到一次后就会递增 `state_seq`。

## 8. 当前实现的测试场景

当前实现应持续用单元、传输和现场联调覆盖以下场景。这里的“恢复”指总控进入
`RECOVER_PATROL`，不是在感知内部增加新的公共状态：

1. `PATROL` 发现人，产生一次 `TARGET_CONFIRMED`，不重复确认同一状态序号。
2. `CONFIRM_TARGET` 中目标短暂消失后重新出现，不切换到其他人。
3. `APPROACH_TARGET` 中目标持续丢失，产生目标丢失事件。
4. `VERIFY_IDENTITY` 中目标持续丢失，授权流程被取消，产生目标丢失事件。
5. `TRACK_INTRUDER` 中目标仍存在时持续发布 bbox，不需要人工事件才能跟踪。
6. `TRACK_INTRUDER` 中目标持续丢失，产生目标丢失事件并能恢复巡检。
7. 目标丢失超过 10 秒后，验证只发布一次 `TARGET_LOST`，并进入恢复巡检。
8. 恢复阶段不接受新的目标确认；恢复超时直接回到 `PATROL`。
9. 两轮人脸和口令全部失败，外部只收到一次 `UNAUTHORIZED`，并且 detail 表示核验超时。
10. 授权过程中目标丢失，后续迟到的授权结果不能改变新的巡检状态。
11. 导航 `EXECUTION_ERROR` 与感知 `TARGET_LOST` 不混用，分别由总控按当前目标任务
    或无目标诊断规则处理。
12. 所有事件都验证 `observed_state_seq`、`target_id`、来源和重复事件。

## 9. 当前结论

感知内部的分层结构保持不变，不需要把 detector、tracker、identity、授权和任务协调器揉成一个大状态机。任务级闭环规则如下：

- 授权最终失败的业务语义需要固定为“核验超时后进入持续跟踪”。
- `TRACK_INTRUDER` 持续依赖可信 bbox；目标丢失后由 `TARGET_LOST` 完成任务收口。
- 目标丢失需要覆盖接近、核验、持续跟踪三个阶段。
- 10 秒后发送一次最终 `TARGET_LOST` 事件。
- 总状态机收到最终丢失或技术故障后进入 `RECOVER_PATROL`，导航取消目标任务并恢复之前巡检任务的断点。
- 恢复阶段错误不再形成新的业务分支；恢复超时由 supervisor 直接进入 `PATROL`。

“恢复巡检断点”的执行位置已经确定：总状态机发布 `RECOVER_PATROL`，导航在进入目标接近前保存当前 `map -> base_footprint` 位姿，同时保留 waypoint 当前索引；恢复时先返回这个位姿，再恢复 waypoint。路径恢复成功时发布 `PATROL_RECOVERY_COMPLETE`，恢复超时时由 supervisor 直接进入 `PATROL`。

## 10. 已确认设计：目标位置确认超时后的重新尝试

以下行为已经确认，后续设计和实现不得与其冲突。

### 10.1 保留 `CONFIRM_TARGET`

`CONFIRM_TARGET` 表示感知已经选出业务目标，导航正在使用 bbox、雷达点云、标定和 TF 计算稳定的目标地图位置。它不是第二次视觉确认。

如果导航成功计算出稳定位置，则发布 `TARGET_POSITION_READY`，总状态机进入 `APPROACH_TARGET`。

如果在规定时间内一直无法计算出位置，总状态机结束本次确认并进入
`RECOVER_PATROL`。导航完成恢复或恢复 watchdog 超时后，状态机才进入新的
`PATROL` 周期；进入 `PATROL` 时必须增加 `state_seq`，并令 `target_id=0`。

### 10.2 返回巡检后允许重新确认

从 `CONFIRM_TARGET` 超时进入 `RECOVER_PATROL`，并在恢复完成或恢复超时后进入新的
`PATROL` 后，感知不能永久禁止原目标。感知应在新的 `PATROL state_seq` 下重新评估当前画面：

```text
PATROL(seq=N)
  -> TARGET_CONFIRMED(target=T, seq=N)
  -> CONFIRM_TARGET(seq=N+1)
  -> 目标位置计算超时
  -> RECOVER_PATROL(seq=N+1, target_id=T)
  -> PATROL(seq=N+2, target_id=0)
  -> 感知重新评估目标 T
  -> 条件满足后允许再次发布 TARGET_CONFIRMED(target=T, seq=N+2)
```

`TARGET_CONFIRMED` 因此是“当前巡检周期选中了目标”的一次性事件，不是某个 semantic ID 整个生命周期只能发布一次的事件。

### 10.3 目标位置确认超时后的重新评估

如果目标始终在相机中，但因为超过雷达有效距离、bbox 内点数不足、时间同步或 TF 问题一直无法定位，
直接从 `CONFIRM_TARGET` 跳回 `PATROL` 会形成：

```text
PATROL -> CONFIRM_TARGET -> timeout -> PATROL -> CONFIRM_TARGET -> ...
```

当前代码没有一个专门的“定位失败冷却”参数。确认超时由总控 watchdog 结束当前
`CONFIRM_TARGET`，先进入 `RECOVER_PATROL`，导航恢复完成或恢复超时后再进入新的
`PATROL state_seq`。新的巡检序号允许感知重新评估画面，因此不能把旧序号的
`TARGET_CONFIRMED` 或 bbox 继续带入新任务。当前感知侧实际保留的 handled-ID 机制
只针对从 `VERIFY_IDENTITY` 或 `TRACK_INTRUDER` 正常结束后回到巡检的目标，参数是
`target.handled_ignore_absence_sec=60.0`；它不是确认超时专用冷却，也不适用于把目标
永久屏蔽。

确认超时后的实际行为是：

- 感知在 `RECOVER_PATROL` 中清理旧任务目标，不发布旧任务 bbox 或确认事件。
- 进入新的 `PATROL state_seq` 后，感知重新执行巡检选目标规则。
- 从核验/持续跟踪任务结束回到巡检时，handled-ID 机制会暂时排除上一目标；该目标
  连续不可见达到 `handled_ignore_absence_sec` 后才解除排除，期间其他合格目标仍可被选中。
- 这不是公共事件，不新增 `TARGET_REACQUIRED`。

确认位置超时的等待时间由总控参数 `confirm_target_timeout=5.0` 秒控制；恢复等待由
`patrol_recovery_timeout=10.0` 秒控制。

### 10.4 与目标真正丢失的区别

目标位置计算超时和目标丢失必须采用不同策略：

| 情况 | 判断方 | 结果 |
|---|---|---|
| 目标框持续存在，但导航算不出地图位置 | 导航/总状态机 | `CONFIRM_TARGET` 继续原巡检并后台重试；5 秒确认超时后进入恢复，再由新巡检序号重新评估 |
| 感知连续超过 10 秒无法找回当前目标 | 感知 | 上报最终目标丢失和报警，结束旧目标任务，再恢复 `PATROL` |

前者表示目标仍然可见，只是当前无法可靠定位；后者表示目标生命周期已经结束。两者不能共用同一个错误原因或相同的目标抑制策略。

## 11. 感知、导航和总状态机的统一逻辑

本节把感知内部状态机、导航协调器和总状态机放在同一个任务流程中说明。三者的
职责必须分开：

```text
感知模块：检测、跟踪、核验、提供 bbox 和业务事件
导航协调器：bbox + 雷达 + TF + 标定 -> 目标地图位置和目标路径
总状态机：保存唯一业务状态，根据事件决定流程转移和恢复
```

导航协调器不维护独立状态机。类似 `PATROLLING`、`ACQUIRING_TARGET` 和
`TRACKING_TARGET` 的名称只能表示当前执行策略，不能成为第二个独立的业务状态机。
业务状态唯一以总状态机发布的 `/mission/state` 为准。这样可以避免感知认为
系统正在核验、导航认为系统正在跟踪、总控却仍处于巡检的并行状态冲突。

### 11.1 三层之间的基本原则

1. 感知不直接让导航运动。感知只发布当前主目标的 bbox 和事件。
2. 导航不直接修改总任务状态。导航通过 `/mission/event` 请求状态变化。
3. 总状态机不计算像素、点云或地图目标位置，只校验事件来源、目标 ID 和
   `state_seq`，再推进业务状态。
4. 目标位置未确认时，导航不能把不可靠的位置当作目标终点，也不能因此让整台狗
   停在原地等待。当前目标确认阶段继续执行原来的巡检路径并在后台重试。
5. 目标位置确认后，只有总状态机进入 `APPROACH_TARGET`，导航才暂停巡检并开始
   目标接近。导航可以提前准备数据，但不能提前向底盘发送目标运动意图。
6. 目标丢失、目标位置计算失败和导航技术故障是三个不同的故障原因，不能都使用
   同一个事件或同一个恢复路径。

## 12. 总状态机

总状态机是业务流程的唯一所有者。推荐的状态和转移如下：

```text
STARTUP
  感知 READY + 导航 READY
  -> PATROL

PATROL
  感知 TARGET_CONFIRMED
  -> CONFIRM_TARGET

CONFIRM_TARGET
  导航 TARGET_POSITION_READY
  -> APPROACH_TARGET
  确认超时 / 目标位置始终无效
  -> RECOVER_PATROL

APPROACH_TARGET
  导航 ARRIVED_AND_STOPPED
  -> VERIFY_IDENTITY
  感知或任务级目标最终丢失
  -> RECOVER_PATROL

VERIFY_IDENTITY
  感知 AUTHORIZED
  -> RECOVER_PATROL
  感知 UNAUTHORIZED（语义为核验超时）
  -> TRACK_INTRUDER
  目标最终丢失
  -> RECOVER_PATROL

TRACK_INTRUDER
  目标仍然有效
  -> 保持 TRACK_INTRUDER
  目标最终丢失 / 跟踪结束
  -> RECOVER_PATROL

RECOVER_PATROL
  导航 PATROL_RECOVERY_COMPLETE 或恢复超时
  -> PATROL
```

每次业务转移都要递增 `state_seq`，并清理不属于新状态的数据。回到 `PATROL` 时必须
满足：

- `target_id=0`；
- 清空旧目标位置、目标 standoff goal 和目标路径；
- 取消旧的接近或跟踪任务；
- 恢复原巡检路径，或者从保存的巡检断点重新请求路径；
- 允许感知重新评估当前画面中的目标，而不是永久屏蔽原 semantic ID。

### 12.1 STARTUP

`STARTUP` 只等待两个模块的 `READY`：

- 感知确认相机输入、检测器、跟踪器和任务输出已经可用；
- 导航确认点云、TF、定位和 planner/control 链已经可用。

单独一个模块 READY 不能让总状态机进入巡检。启动阶段不应接受旧目标事件，也不应
使用上一次任务遗留的 bbox 或目标位置。

### 12.2 PATROL

`PATROL` 是系统的常态：

- 导航执行原来的巡检 waypoint/global path/local path 链；
- 感知持续检测和跟踪，但不向导航发送持续的目标导航框；
- 感知选出可信业务主目标后，只发布一次当前 `state_seq` 下的
  `TARGET_CONFIRMED`；
- 总状态机收到该事件后进入 `CONFIRM_TARGET`。

目标确认事件不是 semantic ID 的永久一次性标记。同一个人在一次确认失败并回到新的
巡检周期后，仍由当前巡检选目标规则重新评估。确认超时本身没有独立的公共冷却事件；
总控先进入 `RECOVER_PATROL`，随后以新的 `PATROL state_seq` 开始下一轮。

### 12.3 CONFIRM_TARGET

`CONFIRM_TARGET` 的含义是“已经发现候选目标，正在确认目标的空间位置”。它不是
只做计算而禁止运动的状态。

该状态分两种情况：

#### 目标位置还没有确认

导航使用最新 bbox、雷达点云、相机-雷达外参、TF 和时间戳尝试融合。如果出现以下
情况，目标位置判定为无效：

- bbox 已过期或时间戳与点云无法匹配；
- bbox 投影区域内没有足够雷达点；
- 点云点数不足、深度簇不稳定或出现明显跳变；
- `livox_frame`、`camera_link`、`base_link` 或 `map` TF 不可用；
- 目标位置超出有效雷达范围，或者不在当前地图有效区域。

此时的处理是：

```text
保留并继续执行原巡检路径
导航后台重试 bbox/点云融合
不暂停底盘、不生成目标路径、不把无效位置发送给 planner
```

因此“目标框存在但导航暂时算不出位置”不会让机器狗卡在确认状态。导航可以在
导航日志中报告 `POSITION_UNAVAILABLE` 和具体原因，供总控和 UI 诊断，但
不能把这个诊断误报为感知侧 `TARGET_LOST`。

#### 目标位置已经确认

当导航得到满足稳定性要求的地图目标位置后：

1. 导航发布 `TARGET_POSITION_READY`，携带目标 ID、位置和当前 `state_seq`。
2. 总状态机校验事件后转入 `APPROACH_TARGET`，生成新的状态序号。
3. 导航收到新的 `APPROACH_TARGET` 状态后，暂停巡检 waypoint，保存巡检断点，
   再开始目标路径规划和运动。

确认阶段可以预计算 standoff goal，但不能在总状态仍为 `CONFIRM_TARGET` 时提前
控制机器人驶向目标。这条边界可以避免“导航已经运动而总状态机仍认为只是确认”的
状态竞争。

#### 确认超时

确认超时由总状态机负责收口。若在规定时间内一直没有稳定目标位置：

```text
CONFIRM_TARGET(seq=N, target=T)
  -> 超时
  -> RECOVER_PATROL(seq=N+1, target_id=T)
  -> PATROL(seq=N+2, target_id=0)
```

导航侧可以在此之前发布技术诊断或执行错误，但总状态机不应永久停在
`CONFIRM_TARGET`。总控先进入 `RECOVER_PATROL`；导航返回保存的巡检中断位姿或在
恢复超时后释放等待，再进入新的 `PATROL state_seq`。感知在恢复阶段清理旧任务上下文，
进入新的巡检序号后重新执行选目标规则。

## 13. 导航执行策略

导航协调器不再被理解为一个和总状态机并行的业务 FSM，而是一个状态驱动的执行
策略函数：

```text
收到 /mission/state
  -> 校验 state_seq 和 target_id
  -> 选择当前执行策略
  -> 执行巡检、目标融合、目标规划、停止或恢复
  -> 通过 /mission/event 回报结果
```

### 13.1 状态到导航策略的映射

| 总状态 | 导航策略 | 是否继续巡检 | 是否融合 bbox | 是否生成目标路径 |
|---|---|---:|---:|---:|
| `STARTUP` | 等待导航依赖就绪 | 否 | 否 | 否 |
| `PATROL` | 执行/恢复巡检路径 | 是 | 否 | 否 |
| `CONFIRM_TARGET` | 后台确认目标空间位置 | 是 | 是 | 否 |
| `APPROACH_TARGET` | 生成 3 m standoff 路径并接近 | 否 | 是 | 是 |
| `VERIFY_IDENTITY` | 停车保持 | 否 | 可继续 | 否 |
| `TRACK_INTRUDER` | 按最新目标位置持续跟踪 | 否 | 是 | 是 |

“继续巡检”只适用于 `CONFIRM_TARGET` 中目标位置尚未确认的阶段。一旦
`TARGET_POSITION_READY` 被总状态机接受并转入 `APPROACH_TARGET`，导航才暂停巡检。

### 13.2 APPROACH_TARGET

接近目标时，目标不是一次性固定终点。导航每次使用最新稳定的目标地图位置，计算
距离目标约 3 m 的 standoff goal，并请求原有 planner 生成路径。

目标移动时：

- 按固定周期重新融合和重规划；
- 目标位置相对上次规划变化超过阈值时允许提前重规划；
- 同一时刻只保留一个有效 planner 请求；
- 只有 action 成功、目标 ID 和状态序号仍然匹配时才替换全局路径；
- 失败时不能发布半成品路径，也不能让旧目标路径无限期继续使用；
- 目标 bbox 短暂抖动时使用融合和稳定性判断，避免每一帧将目标路径改成新的方向。

到达判断不能只看几何距离。至少要同时满足目标距离阈值、线速度、角速度和持续
稳定时间，然后发布 `ARRIVED_AND_STOPPED`。收到该事件后，总状态机进入
`VERIFY_IDENTITY`，导航清空移动目标路径并持续保持停止。

### 13.3 VERIFY_IDENTITY

导航在此状态只负责保持安全：

- 持续输出停止/空路径，避免控制器继续执行旧的接近路径；
- 可以继续接收 bbox 并更新 UI 距离，但不因 bbox 更新而驱动机器人移动；
- 不解释人脸识别、口令识别的内部阶段；
- 等待感知发布 `AUTHORIZED` 或 `UNAUTHORIZED`。

`AUTHORIZED` 使总状态机进入 `RECOVER_PATROL`，由导航恢复巡检后再回到 `PATROL`。
`UNAUTHORIZED` 不是等待人工处理的终态，
它表示核验超时，随后进入 `TRACK_INTRUDER`。

### 13.4 TRACK_INTRUDER

该状态下感知和导航都必须持续工作：

```text
最新可信 bbox
  -> 感知持续发布 bbox
  -> 导航重新计算目标地图位置
  -> 更新 3 m standoff goal
  -> 更新跟踪路径
```

目标还在，就不能因为核验失败而停止跟踪；目标丢失达到最终超时，才结束本次目标
任务并恢复巡检。跟踪状态同样需要目标 ID 和 `state_seq` 校验，迟到的旧 bbox 或
旧授权结果不能改变新的巡检任务。

## 14. 目标丢失、定位失败和技术故障

三类情况的边界如下：

| 情况 | 主要判断者 | 处理 | 是否结束目标任务 |
|---|---|---|---:|
| bbox 短暂漏检 | 感知内部 tracker | 保持内部跟踪，等待同一目标重新出现 | 否 |
| bbox 存在但点云无法融合 | 导航协调器 | `CONFIRM_TARGET` 继续原巡检并后台重试；5 秒确认超时后进入 `RECOVER_PATROL`，再由新巡检序号重新评估 | 由确认超时或任务策略决定 |
| 目标持续丢失超过 10 秒 | 感知任务级逻辑 | 发布最终目标丢失/告警事件 | 是 |
| TF、planner、控制链故障 | 导航协调器 | 停车并发布 `EXECUTION_ERROR` | 由总状态机恢复策略决定 |
| 感知检测器、跟踪器或核验服务故障 | 感知 | 发布 `EXECUTION_ERROR` | 由总状态机恢复策略决定 |

### 14.1 目标丢失的统一覆盖范围

最终目标丢失必须覆盖三个阶段：

1. `APPROACH_TARGET`：正在前往目标时丢失；
2. `VERIFY_IDENTITY`：已经到达、正在核验时丢失；
3. `TRACK_INTRUDER`：核验超时后持续跟踪时丢失。

建议的时序是：

```text
最后一次可信 bbox
  -> 短时丢失窗口：感知内部保留 track
  -> 同一目标重新出现：继续当前任务
  -> 连续丢失达到 10 秒：发布最终目标丢失事件
  -> 导航停车并清空目标路径
  -> 总状态清空目标、递增 state_seq、恢复 PATROL
```

这里不新增 `TARGET_LOST_TIMEOUT` 或其他目标丢失事件。感知只有在内部累计丢失超过
10 秒后才发布现有 `TARGET_LOST`，总状态机将它作为本次目标任务的结束事件处理，而
不是进入永久等待。总状态机收到后进入 `RECOVER_PATROL`，不能阻止其自动恢复
`PATROL`。

导航遇到 bbox 位置计算失败时，不得直接伪装成最终目标丢失，因为这表示“图像目标
仍然存在，但当前空间定位不可靠”。如果导航在接近或跟踪阶段无法继续保证安全，
应发布导航技术错误并停车，由总状态机执行统一恢复。

## 15. 事件时序和来源

### 15.1 正常流程

```text
感知                  总状态机                    导航
 |                       |                         |
 |-- READY ------------->|                         |
 |                       |<------------- READY ---|
 |                       |                         |
 |-- TARGET_CONFIRMED -->|                         |
 |                       |-- CONFIRM_TARGET ------>|
 |<-- bbox --------------|------------------------>|
 |                       |                         |
 |                       |<-- TARGET_POSITION_READY|
 |                       |-- APPROACH_TARGET ----->|
 |<-- bbox --------------|------------------------>|
 |                       |<-- ARRIVED_AND_STOPPED -|
 |                       |-- VERIFY_IDENTITY ----->|
 |<-- 核验请求/结果 ------|                         |
 |-- AUTHORIZED/         |                         |
 |   UNAUTHORIZED ------>|                         |
```

### 15.2 核验超时后的流程

```text
VERIFY_IDENTITY
  -> 感知完成规定的内部核验次数
  -> 发布 UNAUTHORIZED（detail=verification_timeout）
  -> 总状态进入 TRACK_INTRUDER
  -> 感知继续发布目标 bbox
  -> 导航持续更新 3 m 跟踪路径
```

目标仍存在时持续跟踪，目标最终丢失时由 `TARGET_LOST` 闭环回到巡检。

### 15.3 确认目标失败后的流程

```text
PATROL
  -> TARGET_CONFIRMED
  -> CONFIRM_TARGET
  -> bbox 存在，但雷达/TF/时间同步无法得到稳定位置
  -> 导航只记录 POSITION_UNAVAILABLE 等诊断并继续巡检路径
  -> CONFIRM_TARGET 超时
  -> 总状态进入 RECOVER_PATROL
  -> 导航恢复或恢复超时后回到 PATROL，清空目标
  -> 新 PATROL state_seq 下重新评估目标
```

这条流程解决“确认目标阶段因为距离限制或点云不足而永久卡住”的问题。

## 16. 数据和事件的共同校验

所有目标相关数据都必须同时校验：

- `target_id` 是否是当前业务目标；
- `observed_state_seq` 是否对应当前总状态；
- 消息时间戳是否在允许范围内；
- bbox 的图像尺寸和 frame 是否正确；
- 点云、bbox、TF 是否属于同一时间窗口；
- 事件发布者是否有权发布该事件；
- 状态已经变化后，旧目标的异步 action、bbox、核验结果是否被丢弃。

状态切换的安全顺序是：

```text
总状态机发布新 state_seq
  -> 导航/感知确认新的 state_seq
  -> 清理不属于新状态的缓存和 action
  -> 再处理新状态下的数据
```

不能仅依靠 topic 到达顺序判断新旧关系，因为 bbox、点云、action result 和 mission
event 来自不同的异步执行链。

## 17. 当前实现与后续改造清单

本文件是三方统一逻辑说明。当前代码已经具备感知检测、跟踪、授权、bbox 发布、导航
空间融合和基础目标路径能力。以下是当前实现的验收矩阵，后续改动应逐项回归：

1. 导航确认位置失败时，`CONFIRM_TARGET` 是否确实保持巡检路径并后台重试，而不是
   直接停车等待；当前总控 5 秒超时后进入 `RECOVER_PATROL`。
2. 只有总状态切换到 `APPROACH_TARGET` 后，导航才暂停巡检并发送目标路径。
3. `CONFIRM_TARGET` 是否有明确的总控 5 秒超时，并经 `RECOVER_PATROL` 回到新的
   `PATROL state_seq`。
4. 目标任务进入 `APPROACH_TARGET` 时是否保存巡检断点，恢复阶段是否返回该断点；
   回到新的巡检序号后是否重新执行选目标规则。
5. `APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER` 三个阶段都是否覆盖
   10 秒最终目标丢失和恢复巡检。
6. 核验失败是否固定映射为 `UNAUTHORIZED -> TRACK_INTRUDER`，不再依赖人工完成事件。
7. `TARGET_LOST` 是否只在感知内部累计丢失超过 10 秒后发送，并且总控收到后自动
   清理目标并进入 `RECOVER_PATROL`，由导航完成巡检恢复后再回到 `PATROL`。
8. 导航技术错误是否停车、清理旧路径并通过总控统一恢复，而不是只改变导航内部
   模式。
9. 所有事件、bbox 和异步 planner 结果是否进行 `state_seq`、`target_id` 和时间
   校验。

当前代码的传输和核心单元测试已覆盖协议关键路径；现场验收仍不能把“导航节点没有
崩溃”当成“整个任务流程已经闭环”。最终验收
应至少覆盖：正常巡检、确认位置失败、接近时目标丢失、核验时目标丢失、核验超时后
持续跟踪、跟踪时目标丢失、导航 planner 错误和回到巡检断点。

## 18. 事件类型最终约定

按当前简化方案，任务流程不新增业务 `MissionEvent` 类型。现有事件的使用方式固定
如下：

| 场景 | 使用的现有事件 | 说明 |
|---|---|---|
| 感知和导航就绪 | `READY` | 两边都就绪后，`STARTUP -> PATROL` |
| 感知选中候选目标 | `TARGET_CONFIRMED` | 当前巡检状态序号内发送一次 |
| 导航得到稳定地图位置 | `TARGET_POSITION_READY` | 总控据此进入 `APPROACH_TARGET` |
| 到达目标前约 3 m 并稳定停车 | `ARRIVED_AND_STOPPED` | 总控进入 `VERIFY_IDENTITY` |
| 核验成功 | `AUTHORIZED` | 进入 `RECOVER_PATROL`，恢复完成后返回 `PATROL` |
| 两轮核验均失败 | `UNAUTHORIZED` | `detail` 标明 `verification_timeout`，进入跟踪 |
| 目标连续丢失超过 10 秒 | `TARGET_LOST` | `detail` 标明阶段和 `target_lost_timeout_10s`，总控进入 `RECOVER_PATROL`，再恢复巡检 |
| 导航或感知技术故障 | `EXECUTION_ERROR` | `detail` 携带具体技术原因，由总控执行恢复 |

以下规则同时生效：

- 目标位置暂时算不出来时不新增事件；导航只更新日志诊断，总控继续让
  `CONFIRM_TARGET` 计时，5 秒超时后先进入 `RECOVER_PATROL`，再回到新的
  `PATROL`。
- 短暂目标丢失不发布 `TARGET_LOST`，感知内部 tracker 负责恢复。
- `TARGET_LOST` 被总控处理后进入 `RECOVER_PATROL`。停车和清理路径是导航的安全动作，
  业务状态必须递增 `state_seq` 并在恢复完成后回到 `PATROL`。

### 18.1 报警信息与业务事件分离

报警是给 UI、记录系统或上层处置系统看的告警，不负责驱动业务状态转移。报警可以
复用事件中的原因，但不应为了报警再增加一组“丢失超时”“定位失败”等业务事件。

建议单独提供报警接口，例如 `/mission/alarm`，内容至少包括：

- 报警代码和严重等级；
- `target_id`、当前任务状态和 `state_seq`；
- 发生时间；
- 简短原因和可读详情；
- 是否需要清除或已经恢复。

例如目标在接近、核验或跟踪阶段连续丢失超过 10 秒时：

```text
感知内部累计丢失超过 10 秒
  -> 发布一次 TARGET_LOST(detail=target_lost_timeout_10s)
  -> 同时发布一次目标丢失报警
  -> 导航停车并清空目标路径
  -> 总控清空目标并恢复 PATROL
```

如果暂时不新增报警 topic，也可以由 UI/记录系统根据 `TARGET_LOST`、
`EXECUTION_ERROR` 的 `detail` 生成报警；这不影响任务事件数量和状态转移逻辑。
