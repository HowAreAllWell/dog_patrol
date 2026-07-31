# Issue #87 Hik MVS 无损链路与 mission-aware patrol 集成验收

本页独立核验 umbrella spec #78 的完整链路。子票据的关闭状态只作为待复核证据；最终结论来自
当前 `dev` 代码、真实 `dog_patrol_manager`、Orin/Hik 硬件和现存 artifact 的组合验证。

## 验收结论

#79–#94 中除本验收票 #87 外的全部依赖均已关闭，#85 标记为 required 的 #93 也已完成；父
spec #78 保持打开直到本票验收完成。当前代码通过无资产默认 CTest 和显式 Orin 视觉验收两种模式，
均从 `STARTUP` 运行到第二个 `CONFIRM_TARGET`。显式模式把受限 historical Hik 录制送入当前 HEAD
的 TensorRT detector、filter、tracker 和 identity，再将其真实 observations 接到 production
mission stack 与真实 ROS 2 supervisor；capture、offline replay、live preview/record、retired
surface 和 shared patrol protocol 的既有证据也保持成立。

验收结论为通过。当前实现满足以下关键行为：

- `PATROL` 首帧从可用 person 中选择最大框并只发一次 `TARGET_CONFIRMED`，不发 bbox；
- authoritative target 在 `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY` 中只发当前帧
  新鲜 bbox，保留 semantic ID、源时间戳、光学 frame 和原图 half-open 坐标；
- 缺失 0.5 秒后只发一次 `TARGET_LOST`，不复用缓存框；真实 mission supervisor 进入结构化
  `BLOCK_TARGET_LOST`；
- 6 秒保留窗内 raw track 改变时，只有同一 semantic target 可发一次 `TARGET_REACQUIRED`，
  真实 supervisor 保持业务 state/target 并解除 block；显式视觉证据为 semantic 2、raw 2→3；
- 完成 verification 回到新 `PATROL` 后，已处置 target 仍留在 identity observations，但默认
  30 秒连续离场条件未满足，因此不具 mission eligibility；显式视觉证据在首帧选择 semantic 1；
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

不带参数时，CTest 使用确定性 identity observations，保持测试无模型/视频资产依赖并精确验证时间和
坐标边界。显式提供以下三个参数时，driver 先执行实际视觉链路，并自动寻找包含两个 person、最大目标
离场、同 semantic/raw-change 重获和下一可用目标的片段，再复用同一 mission 断言：

```bash
bash src/vision_demo_host/test/test_mission_pipeline_integration.sh \
  build/vision_demo_host/test_mission_pipeline_integration_driver \
  --visual-video data/datasets/orin_hik_h264_MOT/03/video.mp4 \
  --detector-engine assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --tracker-config src/vision_demo_host/config/bot_sort.yaml
```

默认无资产模式的精确断言顺序如下：

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

### 真实视觉与 DDS 证据

显式模式在本机 AGX Orin 上完整解码 539 帧，raw detection-positive 和 track-positive 均为
539/539。driver 从实际 `IdentityManager` 输出中找到：frame 89 的最大可用 person 为 semantic 2 / raw 2；
frame 335–350 中该 semantic target 不可用；frame 351 以 semantic 2 / raw 3 重获；新 patrol 首帧
仍可见的 semantic 2 被 handled policy 排除，并选择 semantic 1。源视频、engine 和 tracker config
SHA-256 分别为：

- `939e85d6c80ac17cd1cc81fee17401c14b8efb7aa3ccdd6100426026109fe480`；
- `92477cc6dceb1d8c737646469e07bebd2f387ae7ea050f93cd012781dcccb8a8`；
- `2ee7339e04f00a1066c3071e630d83cdb84c67b2ae8079a2811b2474fe7b2064`。

独立 `ros2 topic echo --no-daemon` 订阅记录位于
`log/issue87_visual_mission_20260731/`：

- `mission_event_evidence.log` 共 9 条，依次覆盖 perception/navigation READY、target 2 confirmed、
  position ready、lost、reacquired、arrived、authorized 和 target 1 confirmed；
- `mission_state_evidence.log` 的周期发布消息去重后为
  `8700 STARTUP → 8701 PATROL → 8702 CONFIRM(2) → 8703 APPROACH(2) →`
  `8704 APPROACH(2, BLOCK_TARGET_LOST) → 8705 APPROACH(2, unblocked) →`
  `8706 VERIFY(2) → 8707 PATROL → 8708 CONFIRM(1)`；
- `target_bbox_evidence.log` 共 4 条，全部是 semantic 2 的实际视觉框，分别对应 confirm、approach、
  unblock 后恢复和 verify；PATROL 与 blocked frame 均无 bbox。

三份 echo 的 SHA-256 分别为
`9ecc8600d4591ebdf1222626f2a766dd78f4538e4bd7c2e9528e67cfd7c412ea`、
`5c0c6c0f00f3ac573c05482309147376bc360f24e53c316e211ea9930c935de1` 和
`ca3c2fb666829fc6947cd37243137207d28752f3932eae828098fb14d5ab7a82`。
同目录三份 `*_info_evidence.log` 由 `ros2 topic info -v --no-daemon` 留存，确认 state 为
reliable/transient-local、event 为 reliable/volatile、bbox 为 best-effort/volatile；独立真实
supervisor 的 `ros2 topic hz` 记录为 20.001–20.005 Hz。driver 结果日志 SHA-256 为
`a2570668ebeaf6ce590c2a8d459045e4e88167bbcc3f63734db1a631a3c8844d`。

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
mission state input/ROS event output 可并行工作。由于当前物理画面没有 person，这一 live run 只验证
readiness 和 mission input，不宣称暗场产生了 target bbox。target event/bbox 的完整状态序列由同一台
Orin 上的显式 visual mode 验证：它使用实际 detector/tracker/identity observations、真实 DDS 和真实
supervisor；真人 live overlay 继续引用 #83 已接受的现场证据。

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
- current-head 物理相机现场无 person；本次没有把暗场 IDLE 当作 target-visible 验收，也没有声称完成
  当前现场真人的 live loss/reacquire。真人 live overlay 使用 #83 artifact；mission target lifecycle
  使用同 Orin 上对 historical Hik 录制的实际 current-head detector/tracker/identity 输出、production
  coordinator/adapter、真实 DDS 和真实 supervisor。
- historical H.264 只保留为受限 migration regression 和本测试显式指定的视觉验收输入；默认 CTest
  仍无资产，production/live/capture CLI 没有重新引入 H.264 active surface。
