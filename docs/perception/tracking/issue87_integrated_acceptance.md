# Issue #87 Hik MVS 无损链路与 mission-aware patrol 集成验收

本页独立核验 umbrella spec #78 的完整链路。子票据的关闭状态只作为待复核证据；最终结论来自
当前 `dev` 代码、真实 `dog_patrol_manager`、Orin/Hik 硬件和现存 artifact 的组合验证。

## 验收结论

#79–#94 中除本验收票 #87 外的全部依赖均已关闭，#85 标记为 required 的 #93 也已完成；父
spec #78 保持打开直到本票验收完成。当前代码通过从 `STARTUP` 到第二个 `CONFIRM_TARGET` 的
真实 ROS 2 跨进程集成测试，
并保持 capture、offline replay、live preview/record、retired surface 和 shared patrol protocol 的
既有证据成立。

验收结论为通过。当前实现满足以下关键行为：

- `PATROL` 首帧从可用 person 中选择最大框并只发一次 `TARGET_CONFIRMED`，不发 bbox；
- authoritative target 在 `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY` 中只发当前帧
  新鲜 bbox，保留 semantic ID、源时间戳、光学 frame 和原图 half-open 坐标；
- 缺失 0.5 秒后只发一次 `TARGET_LOST`，不复用缓存框；真实 mission supervisor 进入结构化
  `BLOCK_TARGET_LOST`；
- 6 秒保留窗内 raw track 从 7 变为 8 时，只有同一 semantic target 42 可发一次
  `TARGET_REACQUIRED`，真实 supervisor 保持业务 state/target 并解除 block；
- 完成 verification 回到新 `PATROL` 后，target 42 仍留在 identity observations，但默认 30 秒
  连续离场条件未满足，因此不具 mission eligibility；同一首帧选择下一个最大可用 target 99；
- authorization/navigation 仍由外部 event owner 提供；vision 侧 readiness 通过 required contributor
  聚合，authorization placeholder 默认 not-ready，只有显式认可或同名 provider 替换后才能 READY。

## 依赖与既有证据复核

| 范围 | 复核结果 | 主要证据 |
| --- | --- | --- |
| #79 / #92 retired downstream surface | 通过 | 活动 UDP、upstream bearing、`Detection2DArray` 已删除；当前搜索只命中拒绝 guard、退役说明或历史记录。 |
| #80 / #85 / #93 Hik Bayer contract | 通过 | 实机仍读回 `BayerGB8`，active 默认为 `balanced`、smoothing=false；四种 interpolation override 仍有 parser/config tests。 |
| #81 standalone capture | 通过 | canonical 901-frame take 的 metadata/timestamps/hash 与既有基线一致；本次当前 HEAD 另采 151 帧零 drop/gap。 |
| #82 offline replay | 通过 | 当前 HEAD 完整解码 canonical FFV1 901 帧和受限 historical H.264 539 帧；result/source 边界测试通过。 |
| #83 live mode matrix | 通过 | 既有 Orin/X11 四模式、真人 detector/tracker/identity/primary overlay 和非阻塞 bounded queue 证据仍在；本次另复跑 preview+record。 |
| #86 retired media surface | 通过 | `record_test_set`、RTSP/GStreamer/H.264 active CLI/build/install 面保持删除；MP4 只允许 `orin_hik_h264_MOT/.../video.mp4` migration 输入。 |
| #88 / #94 shared patrol protocol | 通过 | `dog_patrol_manager` 12/12 tests；真实 supervisor 接受 READY、loss/reacquire、VERIFY fresh bbox contract。 |
| #89 / #90 / #91 / #84 mission route | 通过 | unit/ROS seam tests加本次真实 supervisor 跨进程 lifecycle test。 |

## 端到端测试

新增 CTest `test_mission_pipeline_integration`。shell fixture 在独立 process group 中启动安装后的
真实 `dog_patrol_manager mission_supervisor`；C++ driver 使用 production
`PrimaryTargetManager`、`MissionCoordinator` 和 `MissionRosAdapter`，不复制 supervisor 状态机或
ROS message mapping。fixture 退出时按 process group 清理 supervisor，成功和失败路径都不会遗留
ROS 进程。

测试顺序如下：

1. real detector/tracker readiness contribution 为 READY，并用显式 integration authorization
   provider 替换 required placeholder；外部 navigation READY 使 supervisor 从 STARTUP 进入 PATROL。
2. 第一帧同时输入 semantic 42/raw 7 和 semantic 99/raw 9；较大的 42 立即成为 primary，
   `TARGET_CONFIRMED` 经 DDS 到达 supervisor，且 PATROL bbox 数保持 0。
3. authoritative `CONFIRM_TARGET(42)` 到达后发布新鲜 bbox；断言 source stamp
   `1710000000.100000000`、frame `issue87_hik_camera_optical_frame` 和 clamped half-open 坐标
   `[100,200,401,601)`。
4. navigation 进入 APPROACH；目标缺失 499 ms 时无 loss、无 bbox，达到默认 500 ms 时只发一次
   `TARGET_LOST`，supervisor 发布同 target 的 blocked APPROACH。
5. semantic 42 以 raw 8 返回，证明 raw track 变化不改变业务 target；blocked frame 不提前发 bbox，
   一次 `TARGET_REACQUIRED` 使 supervisor 解除 `BLOCK_TARGET_LOST`，之后新鲜 bbox 恢复。
6. navigation 进入 VERIFY_IDENTITY；verification frame 继续输出 target 42 的新鲜 bbox。外部
   authorization owner 发布 `AUTHORIZED` 后 supervisor 返回 PATROL。
7. 新 PATROL 第一帧仍含 semantic 42 和 99；42 保留在 observations 但被 handled policy 排除，
   primary 立即选择 99 并发一次确认，PATROL 仍不发 bbox。

本测试曾先以不存在的 `PrimaryTargetManager::UpdateForMission` 编译失败作为 RED；production
frame loop 中原本私有且不可独立验证的 mission-primary lifecycle 随后迁入该 public seam，测试和
runtime 现在调用同一实现。

## 时间配置

mission-facing duration 全部使用注入的 `steady_clock` time point，不按相机或推理帧数计时。

| 配置 | 默认值证据 | 非默认证据 |
| --- | --- | --- |
| `target.lost_event_timeout_sec` | integration 在 499/500 ms 边界断言；coordinator default timeout test | coordinator 使用 100 ms loss timeout |
| `target.reacquire_retention_sec` | integration 在默认 6 秒窗内同 semantic/raw-change 重获；unit tests 覆盖过期后拒绝 | coordinator 使用 2 秒 retention 并覆盖重复 cycle |
| `target.handled_ignore_absence_sec` | primary test 覆盖连续离场 30 秒边界和可见时重置 | primary test 使用 2 秒连续离场 |

参数声明仍为 `0.5`、`6.0`、`30.0`；coordinator 验证 loss timeout 为正且短于 retention，primary
验证 handled absence 为正。

## 软件回归

执行环境均先 source ROS 2 Humble 和 `/path/to/workspace/dog_patrol/install/setup.bash`：

```bash
colcon build --packages-select vision_demo_host --event-handlers console_direct+
colcon test --packages-select vision_demo_host --event-handlers console_direct+
colcon test-result --verbose --all
```

结果为 48/48 CTest targets、373 tests，0 error、0 failure、0 skipped。聚焦重跑
`test_primary_target_manager`、`test_mission_coordinator`、`test_mission_ros_adapter` 和
`test_mission_pipeline_integration` 为 4/4 通过。`git diff --check` 和 integration shell `bash -n`
通过；本机没有安装 `clang-format`，未把该工具列为验收结果。

共享工作区另执行：

```bash
cd /path/to/workspace/dog_patrol
colcon build --packages-up-to dog_patrol_manager --event-handlers console_direct+
colcon test --packages-select dog_patrol_manager --event-handlers console_direct+
colcon test-result --verbose --all
```

结果为 `dog_patrol_manager` 12/12 tests 通过。

## Orin / Hik 硬件验收

平台为 AGX Orin/aarch64、L4T R36.4.7，MVS SDK 已安装；相机为 USB3 Vision
`MV-CU013-A0UC`。本次所有大体积 artifact 均位于 `.gitignore` 覆盖的 `data/`，不是源码提交。

### 当前 HEAD standalone capture

`capture_ffv1 --headless --auto-record --max-seconds 5` 生成：

`data/captures/issue87_current_head_capture_20260731_201900/take_001`

- state=`complete`，captured/written/dropped/write-error/camera-gap=`151/151/0/0/0`；
- measured capture/write FPS 均为 `30.073385`；
- source=`BayerGB8`，1280x1024，payload=1,310,720，conversion=`balanced`、smoothing=false；
- writer=`native_ffmpeg`，FFV1/MKV，12 slice threads，`bgr0`；
- video/metadata/timestamps SHA-256 分别为
  `d3815df5382e61c58c4560bd513d6fcbe1feee0b43497c8750350403b420f822`、
  `efe3af5477dc63bc9096bac98671f9c17199d06999c0aa6561fe7e8a54587f94`、
  `d941915cbb78c5b6a2829b669d215d0241efd400465feca59c831a24d86afd34`。

### 当前 HEAD live / preview / record / mission input

真实 Hik + TensorRT + tracker/identity 的 inference-only run 成功初始化并持续处理真实帧；当前视场为
无人的暗场，因此得到 `det=0/tracks=0/IDLE`，不能替代 #83 已接受的真人 overlay 证据。

在本地 `DISPLAY=:1` 同时开启 preview+record，并运行真实 mission supervisor。最终 worker metrics：

- submitted/enqueued/queue-drop=`257/250/7`；
- rendered/previewed/written=`250/250/250`；render/write drop 和 error 全为 0；
- `ffprobe -count_frames` 为 FFV1、bgr0、1280x1024、`30000/1001`、250 帧；
- artifact SHA-256 为
  `7d3706d95805d1670a9c287f0952edf85e84fa31d6f89c7cbc396c69c6e113f0`。

同一 run 中 supervisor 先接受 `NAVIGATION/READY(seq=8700)`，再接收 vision aggregate
`PERCEPTION/READY(seq=8700)`，明确记录 `STARTUP -> PATROL`。这验证真实 camera processing 与
mission state input/ROS event output 可并行工作。由于当前硬件画面没有 person，target event/bbox 的
完整状态序列由同一台 Orin 上的 deterministic identity fixture 经真实 DDS 和真实 supervisor 验证，
不宣称本次暗场产生了真实 target bbox；真人 detector/tracker/identity/primary 可视化继续引用 #83
已接受的现场证据。

### 当前环境吞吐边界

#85/#93 已接受的 headless、无 override、balanced full-live 证据为稳定 29.97–30.04 FPS，只有启动期
两个 frame discontinuity。本次复验时 GNOME Shell、远程桌面、Xorg 和 terminal 持续占用 CPU，且
无法以当前用户执行 `jetson_clocks` 锁频：

- 未绑核 inference-only 约 22 FPS，最终 296 frames、107 estimated drops；
- `taskset -c 0-3` 后约 27–28 FPS，最终观测 275 frames、35 estimated drops；
- preview+record 约 24–25 FPS，diagnostic writer 保持 bounded queue 且无 render/write error。

因此本次不把当前桌面负载下的实时 run 写成 30 FPS 通过，也没有推翻同硬件、干净 headless 条件下
已接受的 #85/#93 30 FPS baseline。部署验收应在关闭远程桌面负载并锁定性能模式后复跑 35 秒
inference-only；功能链路、无损 capture 和非阻塞输出本次均已通过。

### 当前 HEAD offline replay

- canonical FFV1：`metadata/timestamps/decoded=901/901/901`，source kind=`ffv1_capture`，
  `ok=true`，47.469 FPS；结果位于
  `data/eval_results/issue87_current_head_canonical_20260731_203207/`。
- historical migration：显式 `orin_hik_h264_MOT/03/video.mp4` 完整解码 539 帧，
  source kind=`historical_h264`，35.475 FPS，det/track-positive=539/539，
  LOCKED/OCCLUDED=`358/177`；结果位于
  `data/eval_results/issue87_current_head_historical_h264_20260731_203245/`。

## 边界

- authorization、navigation、bearing/range/calibration 和动作控制仍不属于 vision repository；
  integration driver 只以外部 owner 身份发送对应 shared event。
- current-head hardware scene 无 person；本次没有把暗场 IDLE 当作 target-visible 验收。真人 overlay
  使用 #83 artifact，mission target lifecycle 使用同 Orin 上的 production coordinator/adapter、
  真实 DDS 和真实 supervisor。
- 历史 H.264 只保留为受限 migration regression，不能成为默认 dataset 或 active media surface。
