# dog_patrol_perception_tracking

Dog patrol 的正式 perception tracking 模块，包含相机接入、检测、短期跟踪、语义身份、主目标选择、mission ROS 2 输出、录制和离线评估工具。

## 当前链路状态

已接入：
- `camera_ingest`：仅 Hikrobot MVS；向下游返回自持有的 BGR8 图像和源帧元数据
- `preprocess_infer`：YOLO26n TensorRT engine 推理（本机导出的 `.engine`）
- `det_filter`：`person + car` 阈值过滤（配置化）
- `mot_tracker`：BoT-SORT 风格实现（Kalman + 两阶段关联 + 可选 GMC + appearance）
- `perception_config_materializer`：ROS 参数 / 离线 request 到 tracker、identity、visualizer
  运行态配置的统一 materialization 入口
- `primary_target_manager`：`person`-only 主目标规则（首锁最大框 + continuity-first）
- `mission_coordinator`：ROS-independent 任务输出协调 seam；按任务状态 / semantic target / state sequence 只产生当前帧可信 bbox，并负责配置化同目标 loss/reacquire event 时序
- `mission_frame_transaction`：ROS-independent 一帧任务事务；在 identity 输出后统一执行 primary 更新、PATROL 目标确认、fresh bbox、loss/reacquire，并返回本帧 primary 诊断
- `perception_readiness`：ROS-independent READY 聚合 seam；以 required capability contribution 和 `STARTUP state_seq` 产生至多一次 aggregate READY action
- `mission_ros_adapter`：`dog_patrol_interfaces` 的唯一 ROS transport seam；可靠/transient state 输入、aggregate READY/event 输出和 best-effort 新鲜 bbox 输出均在此处映射

未接入或未完成：
- identity 层尚未迁移为完整的新状态机；`IdentityManager` 是公开接口并持有当前 identity runtime state。内部 `IdentityAssignmentEngineAdapter` 只承担 assignment / recovery / update engine 适配，调用已抽取的 policy、cost、solver、birth、binding、store、mutation 和 projection helper，保持既有 debug row、raw-to-semantic mapping 与 public behavior。Phase 5 BirthManager 已固定为唯一运行时 birth / hidden candidate 路径，但 primary/output 仍保持当前公开语义。
- 距离/2D 相对位置
- 控制逻辑

当前工作流分为三个独立入口：`capture_ffv1` 只采集 Hik MVS clean BGR8 并写 FFV1/MKV；
`dog_patrol_perception_tracking_node` 执行 live inference，并可独立开启 diagnostic overlay preview/record；
`offline_eval_recordings` 只回放显式选择的 capture take 或视频并把结果写入 eval 目录。三者不共享
录制开关，也不会把 overlay 写回 clean source dataset。

## 关键配置

默认参数文件：`config/perception_tracking_params.yaml`

live ROS 参数和 `OfflineReplayRun` request 会先进入
`PerceptionConfigMaterializer`，再生成 tracker、identity 和 visualizer 的运行态 config。该入口统一
GMC/ReID 默认、tracker/SID ReID 强制开启、ReID 输入尺寸下限、SID 参数 clamp，以及 visualizer 对
SID 生效配置的镜像；`MotTracker` 的 `config/bot_sort.yaml` 解析仍由 tracker 内部负责。

核心项：
- `camera.mvs_model` / `camera.mvs_serial`
- `camera.width` / `camera.height` / `camera.fps` / `camera.timeout_ms`
- `camera.bayer_interpolation`（默认 `balanced`；`fast|balanced|optimal|optimal_plus` 均可显式覆盖）
- `camera.bayer_smoothing`（默认 `false`）
- `detector.runtime_path`
- `detector.raw_conf_threshold`
- `detector.person_conf_threshold` / `detector.car_conf_threshold`
- `tracker.*`
- `target.lost_threshold_frames`
- `target.lost_event_timeout_sec` / `target.reacquire_retention_sec` / `target.handled_ignore_absence_sec`
- `mission.state_topic` / `mission.event_topic` / `mission.selected_target_bbox_topic`
- `perception.camera_optical_frame_id`
- `visualization.enable`（overlay preview；默认 `false`）
- `visualization.queue_capacity`（异步 overlay render/write queue；默认 `4`）
- `recording.enable` / `recording.output_root` / `recording.path` / `recording.fps`（当前 live result 录制强制为 FFV1/MKV；`path` 必须位于可信 diagnostic output root，且 result root 不得与受保护的 clean `data/captures/` 重叠或经符号链接指向它）
- `runtime.inference_timing_metrics`（默认 `true`，输出 inference p50/p95/p99）

`MissionCoordinator::Config` 当前提供 `lost_event_timeout=0.5s` 和
`reacquire_retention=6s`，两者必须为正且前者更短。它只使用注入的单调
source-time，不按固定帧数计时；输出当前 target 的新鲜 bbox 只允许在
`CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY`、`TRACK_INTRUDER`。
live node 将它绑定到上述 ROS 参数：`TARGET_LOST` / `TARGET_REACQUIRED` 只由
coordinator 的 one-shot action 转发，绝不缓存 bbox。`PATROL` 只会在当前帧
锁定语义 ID 后发送一次 `TARGET_CONFIRMED`，不会发送 bbox；收到相同目标的
authoritative `CONFIRM_TARGET`、`APPROACH_TARGET`、`VERIFY_IDENTITY` 或
`TRACK_INTRUDER` state 后才发布新的 bbox。

`PerceptionReadinessAggregator` 只接受 `MissionSnapshot` 和 required contributor，复用
`MissionCoordinator` 的 mission phase / `state_seq` 术语：只有当前 `STARTUP state_seq`
中所有 required contributor 都为 ready 时，才产生一次 `PerceptionReadyAction`；非
`STARTUP`、旧 sequence、同 sequence duplicate update 都不产生 READY。视觉侧应使用
`DetectionTrackingReadinessContributor` 报告 detector/tracker 的 ready、not-ready 或
failure；既有 `dog_patrol_perception_tracking_node` 的 detector/tracker 初始化、camera source-frame failure 和
detector/tracker frame-processing exception 会更新它。尚未接入的 authorization 和任意未来能力必须使用
`PlaceholderReadinessContributor(capability, owner, replacement_seam, readiness)`，显式
写明 owner 与替换入口；placeholder 默认 not-ready，只有部署明确认可临时能力时才传
ready。接入实际 capability 时以同名 `ReplaceRequiredContributor` 替换，不能改变
mission-supervisor 或 vision-node policy。`MissionRosAdapter` 已把 real
detector/tracker contributor 和 owner 为 `dog_patrol authorization module` 的
`authorization` required placeholder 注册到同一聚合中；后者默认 not-ready，替换入口为
`MissionRosAdapter::ReplaceRequiredReadinessContributor("authorization", provider)`。
只有部署显式设置 `perception.authorization_placeholder_ready:=true`（临时认可）或通过该
replacement seam 接入真实 provider，当前 `STARTUP` sequence 才能汇总后发布一次
`SOURCE_PERCEPTION/READY`。外部模块可通过
`AddRequiredReadinessContributor` 在同一 aggregate seam 注册明确的 placeholder 或
实际 contributor；未 ready 的 required contributor 不会被假装成 ready。

## Patrol ROS 2 contract

构建和运行本包前，先 source 已构建的 `dog_patrol_interfaces` overlay。该 package 是
`MissionState`、`MissionEvent` 和 `TargetBoundingBox` 的唯一消息来源。

```bash
source /opt/ros/humble/setup.bash
source /path/to/workspace/dog_patrol/install/setup.bash
```

- `/mission/state`：reliable、transient-local、keep-last 1。adapter 拒绝未知 enum、无效
  phase/target/block 组合、同 sequence 冲突消息以及旧的 wraparound-safe `state_seq`。
- `/mission/event`：reliable、volatile、keep-last 10。发布 aggregate `READY`、
  `TARGET_CONFIRMED` 和 coordinator 的 `TARGET_LOST` / `TARGET_REACQUIRED`，source 为
  `SOURCE_PERCEPTION`。
- `/perception/selected_target_bbox`：best-effort、volatile、keep-last 5。只传当前帧、当前
  semantic `target_id` 的可信框；`Header.stamp` 为相机 source timestamp，`frame_id` 为
  `perception.camera_optical_frame_id`，且 x/y 以原图像素的 clamped half-open 区间表示。

共享 `TargetBoundingBox` 当前没有 camera-frame-number 字段，因此 MVS frame number 保留为
runtime 诊断元数据；公开消息不编码未在 shared contract 中定义的字段。

ROS subscription callback 只验证并加锁保存 snapshot；camera、detector、tracker 和 identity
由 live tick mutex 串行执行。identity 输出后的 primary、target confirmation、bbox/loss/reacquire
frame decision 由 `MissionFrameTransaction` 在 adapter 的 mission mutex 下执行。即使用
`MultiThreadedExecutor` 也不会并发进入 detector/tracker/identity；mission-state subscription
置于独立的 mutually-exclusive callback group，使它在 camera acquisition 或 inference 较慢时
仍能及时写入最新 snapshot。READY、event 和 bbox 都在发布前于同一 mission mutex 下复核
完整 snapshot；frame transaction 的 one-shot action 在该短临界区内计算并发布，因此不会把旧
sequence、旧 target 或已 blocked 的 frame action 发到 ROS，亦不会因 state 交错而吞掉
新的 target-confirmed/loss/reacquisition action。

无图形会话可运行独立进程 smoke：启动 `mission_ros_adapter_smoke`，使用另一个 ROS 2
进程向默认 `/issue84/smoke/mission/state` 发布 `STARTUP(100)`、`PATROL(101)`、
`CONFIRM_TARGET(102,target_id=42)`，再发布同目标的 blocked
`CONFIRM_TARGET(103, BLOCK_TARGET_LOST)`；通过外部 `ros2 topic echo` 观察 READY、
TARGET_CONFIRMED、当前帧 bbox、TARGET_LOST 与 TARGET_REACQUIRED。该 fixture 使用固定的
可信 synthetic frame，只验收 DDS/adapter transport，不能替代真 Hik 相机中的真实检测验收。
默认 `smoke.reacquire_retention_sec=30` 只为给 CLI 操作留出时间；production node 仍使用
`target.reacquire_retention_sec=6`。

完整 lifecycle 回归由 CTest `test_mission_pipeline_integration` 覆盖。它启动安装后的真实
`dog_patrol_manager mission_supervisor`，再通过 production `MissionFrameTransaction`
和 `MissionRosAdapter` 验证 STARTUP readiness、PATROL 首帧确认、fresh bbox、
loss/block/reacquire/unblock、VERIFY、handled suppression 和 next-target selection。运行测试前必须
source `/path/to/workspace/dog_patrol/install/setup.bash`；fixture 使用独立 ROS domain，并在退出时清理
整个 supervisor process group。默认 CTest 使用无资产确定性 observations；在 Orin 上可显式追加
`--visual-video`、`--detector-engine`、`--tracker-config`，使 historical Hik migration 录制先经过实际
`PreprocessInfer → DetFilter → MotTracker → IdentityManager`，再用其 observations 执行同一 mission
lifecycle。该模式不是 active H.264 runtime 入口。命令、资产哈希、帧号和 DDS echo/info/hz 证据见
`docs/issue87_integrated_acceptance.md`。

建议 engine 路径：
- `/path/to/my_workplace/vision_demo_ws/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine`

## 下游输出

下游 patrol 集成只使用 `dog_patrol_interfaces`：`/mission/event` 上的
`SOURCE_PERCEPTION` READY、TARGET_CONFIRMED、TARGET_LOST、TARGET_REACQUIRED，以及
`/perception/selected_target_bbox` 的当前帧 `TargetBoundingBox`。UDP JSON、upstream bearing
和 `vision_msgs/Detection2DArray` 不属于支持的接口。

## 主目标绑定策略（当前）

- 初次锁定：选最大有效 `person`。
- 锁定后：优先保持同一业务主目标（`primary_target_id`），不因 raw tracker id 变化立刻切换。
- 短时丢失：进入 `OCCLUDED` 时间窗，优先等待主语义目标恢复，不做“候选人重绑定”。
- 长时丢失（超过 `target.lost_threshold_frames`）：才允许重选新的最大 `person` 并分配新的业务主目标 ID。
- `car` 仍不允许成为主目标。

## 构建

构建目标按依赖边界拆分为：`dog_patrol_perception_tracking_core`（tracking、identity、primary、mission
transaction、配置及可移植 offline schema/metrics）、`dog_patrol_perception_tracking_ros_adapter`、
`dog_patrol_perception_tracking_orin_runtime`（CUDA/TensorRT/Hik MVS）、`dog_patrol_perception_tracking_recording`
（FFmpeg）和 `dog_patrol_perception_tracking_offline_runtime`。core 与完整 runtime 使用同一套算法实现，不存在
CI 专用算法分支。

普通 ROS 2 Ubuntu 环境使用 portable 构建；它不查找 CUDA、TensorRT、Hik MVS 或 FFmpeg，也不
构建真实相机/推理 executable、录制工具、真实离线 replay 和对应测试：

```bash
cd /path/to/vision_demo_ws
source /opt/ros/humble/setup.bash
source /path/to/dog_patrol/install/setup.bash
colcon build --packages-select dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=OFF
colcon test --packages-select dog_patrol_perception_tracking --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose --all
```

Orin 完整构建显式开启 runtime（该选项当前默认也是 `ON`，命令中仍显式写出以保证部署可复现）：

```bash
cd /path/to/vision_demo_ws
source /opt/ros/humble/setup.bash
source /path/to/dog_patrol/install/setup.bash
colcon build --packages-select dog_patrol_perception_tracking \
  --cmake-args -DTRACKING_ENABLE_ORIN_RUNTIME=ON
colcon test --packages-select dog_patrol_perception_tracking --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose --all
```

完整 runtime 需要目标 JetPack 对应的 CUDA toolkit、TensorRT development files、安装在
`/opt/MVS` 的 Hikrobot MVS SDK，以及可由 `pkg-config` 找到的 `libavcodec`、`libavformat`、
`libavutil` development packages。显式开启后缺少任一项会在 CMake 配置阶段给出安装/路径提示，
并指出可用 `-DTRACKING_ENABLE_ORIN_RUNTIME=OFF` 执行 portable 构建。

## 运行（示例）

Hikrobot MVS / USB3 Vision 工业相机：

```bash
cd /path/to/my_workplace/vision_demo_ws
source /opt/ros/humble/setup.bash
source /path/to/workspace/dog_patrol/install/setup.bash
source install/setup.bash
ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node --ros-args \
  -p camera.mvs_model:='MV-CU013-A0UC' \
  -p camera.mvs_serial:='CAMERA_SERIAL' \
  -p camera.width:=1280 \
  -p camera.height:=1024 \
  -p camera.fps:=30.0 \
  -p camera.bayer_interpolation:='balanced' \
  -p camera.bayer_smoothing:=false \
  -p detector.runtime_path:='/path/to/my_workplace/vision_demo_ws/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine' \
  -p detector.enable_fake_detection:=false
```

说明：
- 在线相机采集固定通过 `/opt/MVS` SDK 完成，接口不再提供 GStreamer backend 或 pipeline 参数。
- `camera.mvs_serial` 可留空；若同机只接一台目标型号相机，可仅用 `camera.mvs_model` 过滤。
- 当前海康 USB3 Vision 相机通常不会暴露为 `/dev/video*`，因此不走 `v4l2src` 直连路径。
- 启动日志记录实际 PixelType、分辨率、源 payload、帧号/时间戳，以及显式 Bayer 插值和平滑设置。
- `camera_metrics` 日志独立于录制开关，报告 acquisition、conversion、copy 的 p50/p95/p99，
  并报告相机帧号不连续、估计 drop 和 MVS 丢包计数。

## 一键现场运行

直接启动 `dog_patrol_perception_tracking_node`；mission ROS 2 输出由 `MissionRosAdapter` 发布。

四种 live mode 独立配置（追加到 `ros2 run ... dog_patrol_perception_tracking_node`）：

- inference-only：`-p visualization.enable:=false -p recording.enable:=false`（干净性能 baseline）
- preview：`-p visualization.enable:=true -p recording.enable:=false`（需要本地图形会话）
- record：`-p visualization.enable:=false -p recording.enable:=true -p recording.path:=/path/to/diagnostics/live.mkv`
- preview+record：`-p visualization.enable:=true -p recording.enable:=true -p recording.path:=/path/to/diagnostics/live.mkv`

预览和录制从同一 worker 产生同一 tracking/identity/primary overlay canvas。worker 队列有界、队满丢弃最新诊断帧而不等待编码或显示；每秒日志会输出 capture、inference、render/write 的 FPS、queue/render/write drop 和 p50/p95/p99。请将 live overlay 放在 `data/diagnostics/live_overlays/` 等结果目录，不能当作 source dataset。

## 基本验证重点

- build 成功
- 节点可加载真实 TensorRT engine
- Hik MVS ingest 输出 BGR8 acquired-frame 并携带源帧元数据
- detector 输出可驱动后续链路
- BoT-SORT tracker 初始化并输出轨迹（画面有目标时）
- mission ROS 2 event/bbox contract 可被外部节点观测

## Tracker-Core 配置

默认配置入口：`config/bot_sort.yaml`（`core_mode=new_core`）。默认入口只保留当前 runtime 生效字段；历史对照字段隔离在 `config/legacy/` 与 `config/diagnostics/`。
- `core_mode`：默认 `new_core`；`old_minimal` 仅保留在 legacy 对照配置中
- `track_high_thresh`
- `track_low_thresh`
- `new_track_thresh`
- `match_thresh`
- `track_buffer`
- `gmc_method`（当前默认：`sparseOptFlow`，仅在 `tracker.gmc_enabled=true` 时生效）
- `gmc_downscale`（当前默认：`4`，表示 GMC 估计在 `1/4` 分辨率上进行）
- `with_reid`（默认：`true`；运行时强制开启）
- `confirm_hits`
- `stage1_iou_min` / `stage2_iou_min` / `unconfirmed_iou_min`
- `motion_gate_thresh`
- `assoc_iou_weight` / `assoc_motion_weight` / `assoc_app_weight`
- `appearance_gate` / `appearance_alpha` / `appearance_h_bins` / `appearance_s_bins`
- `tracker.gmc_enabled` 由 ROS 参数控制，当前默认 `false`；移动平台或明显相机运动场景可显式设为 `true`。`tracker.reid_enabled` / `with_reid` 运行时会强制为 `true` 并打印告警。

主目标生命周期配置：
- `target.lost_threshold_frames`（默认 `180`；主目标/语义 identity missing 超过该帧数才进入 LOST 并允许重选，约 7.2s@25fps）

语义 ID / 外观库恢复配置（`sid.*`）：
- `sid.feat_bank_size`（每个语义 ID 的特征库长度）
- `sid.recover_sim_thresh_strict`（默认 `0.85`；INACTIVE 语义恢复的严格外观相似度阈值）
- `sid.missing_assign_min_area_ratio`（默认 `0.40`；missing 但仍 ACTIVE 的语义重新绑定时允许的最小面积比例；若外观足够强且几何成本不过高，会内部放宽到 `0.20` 以覆盖边缘小面积恢复）
- `sid.missing_assign_max_app_cost`（默认 `0.50`；missing 但仍 ACTIVE 的语义重新绑定时允许的最大外观成本；短 missing 且几何很强时会内部放宽 appearance gate，以覆盖 raw id 短断但位置连续的小目标）
- `sid.overlap_iou_freeze`（重叠冻结阈值；IoU 超过阈值即冻结更新）
- `sid.split_stable_frames`（分离恢复后稳定多少帧再恢复更新）
- `sid.merge_hold_frames`（进入 merged 后最少保持帧数）
- `sid.app_w` / `sid.geo_w` / `sid.time_w`（语义恢复一对一分配权重）
- `sid.reid_enable`（兼容输入；运行时强制开启语义层外观特征）
- `sid.reid_backend`（`light` 或 `osnet_onnx`）
- `sid.reid_model_path`（`osnet_onnx` 的 ONNX 模型路径）
- `sid.reid_input_width` / `sid.reid_input_height`（默认 `128x256`）

Tracker ReID 配置（`tracker.*`）：
- `tracker.reid_backend`（`light` 或 `osnet_onnx`）
- `tracker.reid_model_path`（`osnet_onnx` 的 ONNX 模型路径）
- `tracker.reid_input_width` / `tracker.reid_input_height`（默认 `128x256`）

语义恢复 / birth 口径（当前）：
- 先匹配 `ACTIVE`（`missing_frames <= target.lost_threshold_frames`）语义。
- short missing 且几何强时优先解释为已有语义 ID，即使 appearance 成本高于普通 missing appearance gate。
- 仅当当前 track 无法匹配任何 `ACTIVE` 语义时，才尝试 `INACTIVE` 恢复。
- `INACTIVE` 恢复使用分层阈值 + 同帧一对一约束。
- 新 `raw_id` 不等于新 `semantic_id`：只有无法解释为已有语义、且不像 active identity 的重复/分裂框、身体局部、虚影或宽矮碎片时，才分配新语义 ID。
- 非 primary 新语义 ID 由内部 `SemanticIdAllocator` 分配：从 `2` 开始，跳过 `1` 和当前 identity storage 已占用的 semantic id；primary bootstrap 仍使用 semantic id `1`。
- 小目标新人稳定确认的 per-raw-track pending hit count / last-seen frame 由内部 `BirthCandidateStore` 维护；它不分配 semantic id，只向 `BirthCandidateDecision` 提供稳定帧 evidence。
- hidden candidate 完全不显示、不进入正式 identity 输出、不参与 primary、不占用 semantic id；小远目标可在快速连续确认后晋升并分配语义 ID。

历史对照配置（仅用于离线诊断记录；当前 runtime ReID 主链为强制开启）：
- `config/legacy/tracker_old_minimal.yaml`
- `config/legacy/tracker_new_core_no_app.yaml`
- `config/legacy/tracker_new_core_with_app.yaml`

`offline_eval_recordings` 默认关闭 GMC；需要做显式对照或移动平台复现时使用
`--tracker-gmc-enabled true|false`。该参数只覆盖本次离线回放的 tracker 配置，默认值为
`false`，不会改变 live ROS 参数或 `config/bot_sort.yaml`。

离线评估对照脚本必须显式传入 capture take 的 FFV1/MKV：

```bash
/path/to/my_workplace/vision_demo_ws/src/dog_patrol_perception_tracking/scripts/eval_tracker_core_round1.sh \
  /absolute/path/to/capture_session/take_001/video.mkv
```

## 问题文档

- 当前 tracking identity 实现状态：`/path/to/my_workplace/vision_demo_ws/docs/current_tracking_identity_state.md`
- `orin_hik_h264_MOT` 01/02 遮挡与 ID 问题复盘：`/path/to/my_workplace/vision_demo_ws/docs/orin_hik_h264_MOT_01_02_issue_resolution.md`

## Hik MVS 相机性能基线

一键运行无录制 live inference，并收集实际 PixelType、吞吐、阶段 p50/p95/p99 和 frame drop：

```bash
/path/to/my_workplace/vision_demo_ws/src/dog_patrol_perception_tracking/scripts/bench_hik_mvs_camera.sh
```

默认目标为 `1280x1024@30 FPS`、`fast` Bayer interpolation，运行 60 秒，输出到
`log/bench_hik_mvs_camera/`。这是固定的性能对照，非 production runtime 默认；硬件验收应保存
`summary.txt` 和完整日志。
本轮 `fast` / `optimal` 对照证据见
`docs/issue80_hik_mvs_frame_contract_baseline.md`。

`#85` 已补充同源 Bayer、含人受控 detector/ReID/tracking 和完整 `balanced` live 30 FPS 证据，
见 `docs/issue85_bayer_preprocess_audit.md`。当前 config file、参数声明、live helper 和 Hik MVS
capture 工具均默认 `balanced`，且 smoothing 默认关闭；`optimal` rollback 可用
`-p camera.bayer_interpolation:=optimal` 或 capture 的 `--bayer-interpolation optimal`。
`bench_hik_mvs_camera.sh` 的 `fast` 默认只作为性能对照基准，不代表 runtime 默认。

`benchmark_bayer_input` 是仅 MVS-enabled 构建中的 headless 审计工具。它对一批自持有 Bayer
buffers 比较全部 SDK quality、写出 FFV1 variants、检测 CSV 和分段 p50/p95/p99；可用
`--input-video` 生成明确标记的 synthetic Bayer 控制输入。它启用 detector timing metrics，普通
`PreprocessInfer` 默认不启用，避免因审计改变 production inference 行为。

建议默认配置（当前）：
- `gmc_enabled=false`
- `gmc_method=sparseOptFlow`
- `gmc_downscale=4`
- `with_reid=true`（强制）

## 无损 BGR8 测试集采集（capture_ffv1）

`capture_ffv1` 是当前独立采集入口：只初始化一台 Hik MVS 相机，不加载 detector、tracker
或 identity。它始终把 `CameraIngest` 的 clean BGR8 帧写为 FFV1/MKV；FFV1 不可用或写入
失败会显式报错，绝不会回退到有损编码。writer 直接链接 native FFmpeg，并在大于等于一百万
像素的帧上使用最多 12 个 FFV1 slice 线程；`metadata.json.encoder` 会记录实际线程数、slice
数和容器像素格式。构建机需要 `pkg-config`、`libavcodec-dev`、`libavformat-dev` 与
`libavutil-dev`。

构建后在预览优先模式运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dog_patrol_perception_tracking capture_ffv1 \
  --mvs-model MV-CU013-A0UC \
  --width 1280 --height 1024 --fps 30.0 \
  --session-name orin_hik_lossless_01
```

不传 `--bayer-interpolation` 时 capture 默认 `balanced`，且 Bayer smoothing 保持关闭。需要与旧
`optimal` 默认作可回滚对照时，显式加 `--bayer-interpolation optimal`；四种 SDK 支持 mode 都可覆盖。

预览默认处于 `STANDBY`，录制的视频不含任何 preview 文字。热键为：

- `R`：开始一个新 take
- `S`：停止并完成当前 take；预览继续，可再次 `R`
- `M`：在当前 take 添加 marker
- `I`：切换预览信息
- `Q`：完成当前 take（如有）并退出

写入通过一个有界 queue 与采集解耦。queue 满时会丢弃最新采集帧并在 take metadata 中
记录 `captured_frames`、`written_frames`、`dropped_frames`、`write_errors` 与
`camera_frame_gaps`；不得把目标 30 FPS 误报为实际写入速率。

每个 session 的 take 目录形如 `data/captures/<session>_<timestamp>/take_001/`，其中包含：

- `video.mkv`（FFV1）
- `frame_timestamps.csv`（source timestamp、camera frame number、PixelType、payload 与丢包）
- `markers.csv`
- `metadata.json`（#80 BGR8 frame contract、相机请求、native FFmpeg encoder 配置、codec、计数与 `complete`/`incomplete` 状态）

MKV 的 stream FPS 是 configured nominal rate；数据集真实采集/写入速率以 `metadata.json.timing`
的 `captured_fps` / `written_fps` 为准，`stream_fps_is_nominal=true` 明确标注这一区别。

自动化模式必须显式无预览、自动开始和限时：

```bash
source install/setup.bash
ros2 run dog_patrol_perception_tracking capture_ffv1 \
  --headless --auto-record --max-seconds 30 \
  --session-name headless_capture_01
```

## 离线回放评估工具（offline_eval_recordings）

入口：
- 可执行：`offline_eval_recordings`
- 默认数据集根目录：`data/captures`
- 默认结果目录：`data/eval_results`
- 必须显式传 `--video` 或 `--datasets`；没有隐含的历史默认数据集

用途：
- 对固定录制集逐帧回放，复用当前主链模块做可重复评估：
  - `preprocess_infer`
  - `det_filter`
  - `mot_tracker`
  - `primary_target_manager`

canonical 回放入口是 #81 格式完整 take 的显式 `video.mkv` 路径：

```bash
ros2 run dog_patrol_perception_tracking offline_eval_recordings \
  --video /absolute/path/to/capture_session/take_001/video.mkv \
  --results-root /absolute/path/to/eval_results \
  --run-name ffv1_baseline
```

当 `video.mkv` 同目录有 `metadata.json` 时，工具只接受 `complete` 的 FFV1/MKV capture metadata，
并校验 `counts.written_frames`、`frame_timestamps.csv` 行数、capture index/source timestamp 单调性，
以及实际从首帧到末帧解码出的帧数。任何不一致都会在 `summary.json`/`summary.md` 和进程退出码中
显式失败。若没有 capture metadata，目录发现优先 `video.mkv`。MP4 只允许
`orin_hik_h264_MOT/.../video.mp4` historical migration 路径，并标记为 `historical_h264`；
其他 MP4 和非 MKV 扩展明确拒绝。该路径只保留为
`docs/issue82_ffv1_offline_eval_baseline.md` 记录的 migration regression 输入，不是默认或 canonical 数据集。

overlay 是结果工件，不是 source dataset。四种独立模式如下；所有 CSV 和可选 overlay 都只能写入
`--results-root` 生成的 run 目录，工具拒绝把它们放进 source dataset。

- headless（默认）：`--overlay-preview false --overlay-record false`
- preview-only：`--overlay-preview true --overlay-record false`
- record-only：`--overlay-preview false --overlay-record true`
- preview + record：`--overlay-preview true --overlay-record true`

record-only/preview+record 将当前 tracking/identity/primary overlay 写成 FFV1 `eval_overlay.mkv`；
overlay 的渲染画布从 source BGR8 frame clone，绝不回写或标注 clean source video。带 preview 的模式
需要本地可用的图形显示；无显示环境使用默认 headless 或 record-only。

常用参数：
- `--detector-engine <path>`
- `--det-raw-conf <f>`（默认 `0.10`）
- `--det-person-conf <f>`（默认 `0.10`）
- `--det-car-conf <f>`（默认 `0.10`）
- `--tracker-config <path>`
- `--tracker-gmc-enabled <true|false>`（默认 `false`）
- `--tracker-reid-backend <light|osnet_onnx>`
- `--tracker-reid-model-path <path>`
- `--save-frame-csv <true|false>`
- `--video <path>`（显式单视频输入，会覆盖 `--datasets`）
- `--overlay-preview <true|false>`（默认关闭）
- `--overlay-record <true|false>`（默认关闭；record 时只写 FFV1 MKV）
- `--overlay-video-name <name>`（默认 `eval_overlay.mkv`，仅接受文件名）
- `--save-eval-video <true|false>` / `--eval-video-name <name>`（旧参数别名，分别等同 overlay record/name）
- `--short-dataset-dir-names <true|false>`（默认开启，子目录用 `s01/s02/...`）
- `--target-lost-threshold-frames <n>`（默认 `180`）
- `--results-root <path>`
- `--run-name <name>`
- `--datasets <a,b,c>`
- `--sid-feat-bank-size <n>`
- `--sid-recover-sim-thresh-strict <f>`
- `--sid-recover-sim-thresh-relaxed <f>`
- `--sid-recover-relaxed-max-missing-frames <n>`
- `--sid-missing-assign-min-area-ratio <f>`（默认 `0.40`）
- `--sid-missing-assign-max-app-cost <f>`（默认 `0.50`）
- `--sid-overlap-iou-freeze <f>`
- `--sid-split-stable-frames <n>`
- `--sid-merge-hold-frames <n>`
- `--sid-app-w <f>` / `--sid-geo-w <f>` / `--sid-time-w <f>`
- `--sid-reid-backend <light|osnet_onnx>`
- `--sid-reid-model-path <path>`
- `--sid-reid-enable <true|false>`
- `--sid-reid-backend <light|mature_stub>`
- `--sid-reid-model-path <path>`

输出结构：
- `/path/to/my_workplace/vision_demo_ws/data/eval_results/<run_name>_<timestamp>/`
- 每个测试集子目录（默认短名）：
  - `s01/`, `s02/`, ...（可通过参数关闭短名）
  - `eval_overlay.mkv`（仅 `--overlay-record=true`；FFV1 的诊断 overlay，不是 source dataset）
  - `summary.json`
  - `summary.md`
  - `per_frame.csv`（可关闭）
  - `tracklet_hypotheses.csv`（随 `--save-tracks-csv=true` 输出，用于 shadow candidate 验收）
  - `phase3_shadow_state.csv`（随 `--save-tracks-csv=true` 输出，用于 Phase 3 identity shadow state 验收）
  - `sid_scores.csv`（默认输出，用于当前 identity assignment、birth / hidden candidate 与 update-policy 证据）
  - `identity_metrics.json` / `identity_metrics.md`（每个数据集固定输出的 additive identity acceptance metrics）
- 映射表：
  - `dataset_dir_map.csv`（`sXX`、原始数据集目录、实际 source video 与 source kind 的对应关系）
- 总表：
  - `global_summary.md`

指标定义（当前最小集）：
- 平均 FPS（离线处理吞吐）
- 总帧数
- `det>0` 帧数/占比
- `tracks>0` 帧数/占比
- `state=LOCKED` 帧数/占比
- `state=OCCLUDED` 帧数/占比
- `state=LOST` 帧数/占比
- 主目标切换次数（连续有效主目标 ID 变化计数）
- `per_frame.csv` 中 `primary_semantic_id` 为视频叠字使用的语义主目标 ID
- `per_frame.csv` 中新增：
  - `sid_mode`（`NORMAL` / `MERGED` / `SPLIT_RECOVERY`）
  - `sid_freeze`（1=语义特征更新冻结，0=可更新）
- `sid_scores.csv` 中 `feature_update_allowed`、`geometry_update_allowed` 和 `sid_freeze` 保持既有 boolean 语义；`feature_update_reason` / `geometry_update_reason` 是 Phase 6 决策证据字段，用于解释 feature-bank / reliable-geometry update 为什么允许、等待或阻断。当前 reason 包括 `allowed_update`、`global_merge_split_freeze`、`overlapping_track_freeze`、`unreliable_low_quality_observation`、`insufficient_stable_frames`、`update_blocked_by_rejected_assignment`。
- `identity_metrics.json` / `identity_metrics.md` 聚合现有 debug CSV，不改变 tracker、identity、primary、overlay 或既有 CSV schema；缺失的可选输入会标记为 `unavailable`，对应分布保持空/零计数。当前聚合项包括 primary state（含 `PENDING_RECOVERY`）、primary decision/reject/recovery reason、identity state、assignment stage/reject reason、feature/geometry update reason、Phase 3 shadow `event_type`、NewBirthCandidate hidden/pending/allocated reason、Phase 4 handoff event、tracklet hypothesis status/reason。
- `primary_raw_track_id_debug` 仅用于离线排障，不参与画面叠字语义
- `tracklet_hypotheses.csv` 固定字段：
  - `frame_idx,hypothesis_idx,status,raw_track_id,class_id,score,x,y,w,h,reason,related_raw_track_id,assoc_stage,assoc_cost,assoc_iou,assoc_motion_dist,assoc_app_dist,assoc_appearance_used,assoc_final_gate,assoc_reject_reason`
  - `frame_idx` 是离线视频帧号，按 0-based 计数；它也是 `phase3_shadow_state.csv` 回链本表的帧号键。
  - `status/reason` 说明 candidate 是最终 `tracked` 输出、被 new-track duplicate suppression 压制，还是 duplicate output hidden。
  - `related_raw_track_id` 用于把 suppressed/hidden candidate 关联到触发压制或隐藏关系的 raw track；无关联时为 `-1`。
  - 复盘 `orin_hik_h264_MOT/01` 时，优先筛 `frame_idx` 在 `760`、`795`、`1030` 附近的行，结合 `reason`、`related_raw_track_id` 和 association 摘要判断 candidate 为何保留、压制或隐藏。
- `phase3_shadow_state.csv` 固定字段：
  - `frame_idx,event_idx,event_type,group_id,semantic_ids,carrier_semantic_id,carrier_raw_track_id,candidate_raw_track_id,candidate_semantic_id,candidate_score,candidate_x,candidate_y,candidate_w,candidate_h,reason,related_raw_track_id,hypothesis_status,candidate_stable_frames,group_age_frames,group_last_update_frame,decision_app_cost,decision_geo_cost,decision_time_cost,decision_final_score,decision_margin,decision_selected,decision_accepted,pairwise_selected_pairs,pairwise_alternate_pairs,pairwise_selected_final_cost,pairwise_alternate_final_cost,pairwise_selected_app_cost,pairwise_alternate_app_cost,pairwise_margin,pairwise_appearance_override`
  - `frame_idx` 与 `tracklet_hypotheses.csv` 使用同一个 0-based 离线视频帧号；`group_last_update_frame` 也使用同一规则。不要把它与 `sid_scores.csv` 中 assignment engine adapter 内部 1-based debug frame 混用。
  - 当前 `#7` 只输出 `event_type=hypothesis_input`，用于证明 identity 层已接收 `tracklet_hypotheses.csv` 对齐的 tracked/suppressed/hidden shadow evidence。
  - `#8` 起输出 `MergedGroup` shadow lifecycle：`merged_group_enter`、`merged_group_update`、`merged_group_end`；`group_age_frames` 和 `group_last_update_frame` 用于观察组持续时间和最后更新时间。
  - `#9` 起输出 `SplitCandidate` shadow lifecycle：`split_candidate_enter`、`split_candidate_update`、`split_candidate_end`；候选行包含 group id、candidate raw id、bbox、score、shadow-only `candidate_semantic_id` guess、`reason`、`related_raw_track_id` 和 `candidate_stable_frames`，用于回链 `tracklet_hypotheses.csv` 的 candidate evidence。
  - `#14` 起输出 single-blob handoff decision shadow rows：`event_type=single_blob_handoff_decision`，`reason` 可为 `single_blob_continuity_kept`、`single_blob_handoff_eligible`、`single_blob_rejected_by_missing_age`、`single_blob_rejected_by_appearance_or_geometry_margin`、`single_blob_handoff_accepted`；`decision_*` 字段镜像 `merged_candidate` score row，用于解释 single visible blob 为何保持 continuity、可 handoff、被拒绝或已被当前 runtime 接受。该行只补观测，不迁移 merged single-blob handoff 行为。
  - `#15` 起输出 2x2 pairwise assignment matrix shadow rows：`event_type=pairwise_assignment_matrix`；`pairwise_*` 字段记录 selected / alternate pairings、final cost sum、appearance cost sum、margin 和 appearance override 是否触发。该行只补观测，不迁移 2x2 pairwise assignment 行为。
  - `#23` 起输出 Phase 5 `NewBirthCandidate` shadow lifecycle rows：`event_type=new_birth_candidate_pending` 表示小目标新人等待稳定确认，`event_type=new_birth_candidate_hidden` 表示 birth decision 隐藏或延迟分配，`event_type=new_birth_candidate_allocated` 表示正式分配；`reason` 可为 `small_new_person_pending`、`small_stable_new_person_promoted`、`new_semantic_allocated`、`phase5_birth_manager_allocated`、`ambiguous_recovery_pending`、`duplicate_split_hidden`、`skinny_partial_hidden`、`wide_fragment_hidden`。
  - Phase 5 BirthManager 路径固定启用，hidden / pending / accepted birth decision surface 由 `IdentityManager` Phase 5 path 表达，small pending 在 `sid_scores.csv` 中使用 `stage=phase5_birth_candidate`、`reject_reason=small_new_person_pending`，accepted allocation 使用 `stage=phase5_new_semantic`，对应 `new_birth_candidate_allocated` row 使用 `reason=phase5_birth_manager_allocated` 或 `small_stable_new_person_promoted`。hidden / pending birth decisions 仍不分配 semantic id。
  - Phase 4 四条 handoff 路径已固定为当前运行时行为：`phase4_merged_split_handoff`、`phase4_merged_side_recovery`、`phase4_merged_single_blob_handoff` 和 `phase4_pairwise_assignment`。对应行仍写入 `phase3_shadow_state.csv`，并保留原有 reason / pairwise 字段语义。
  - `#13` 起输出 side reappearance shadow evidence：Phase 4 side recovery 路径会输出可 join 的 `phase4_merged_side_recovery` 行，回链到 preceding `MergedGroup`、carrier raw id 和 missing semantic guess。
  - 回链方法：先按相同 `frame_idx` 筛两张表，再用 `candidate_raw_track_id` 对 `raw_track_id`，并核对 `reason`、`related_raw_track_id`、bbox/score；`duplicate_output_hidden` 行必须保留 hidden candidate 事实，不表示该候选参与显示或 semantic id 分配。
  - 离线 smoke 验收应先看 `phase3_shadow_state.csv` 的 `event_type` 分布，确认至少覆盖 `hypothesis_input`、`merged_group_enter/update/end` 和 `split_candidate_enter/update/end`；再看 `tracklet_hypotheses.csv` 的 #6 reason 分布仍只使用既有 reason 字符串。
  - 人工抽查窗口：`orin_hik_h264_MOT/01` 的 `746-771` 看 group lifecycle，`793-795` 看 hidden split candidate，`1015-1031` 看 split recovery evidence；`760`、`795`、`1030` 附近同时回查 `tracklet_hypotheses.csv`；`orin_hik_h264_MOT/02` 的 `790-850` 用于第二段 handoff/恢复场景抽查。
  - 该 CSV 是 shadow-only debug 输出，不参与 semantic id 分配、primary、overlay 或 `IdentityAssignmentEngineAdapter` 决策。
- Phase 5 birth / hidden candidate readiness：
  - 证据基线见 `docs/phase5_birth_hidden_candidate_readiness.md`。
  - `sid_scores.csv` 中 `stage=phase5_birth_candidate` 表示 Phase 5 birth proposal / hidden / pending decision，`stage=phase5_new_semantic`、`accepted=1` 表示 Phase 5 path 已正式分配 semantic id。
  - `phase3_shadow_state.csv` 中 `new_birth_candidate_pending|new_birth_candidate_hidden|new_birth_candidate_allocated` 为 Phase 5 lifecycle evidence；pending / allocation decision 由 `IdentityManager` Phase 5 path 产生。
  - `sid_scores.csv` 的 `frame_idx` 来自 assignment engine adapter 内部 debug frame；`tracklet_hypotheses.csv` 和 `phase3_shadow_state.csv` 使用 0-based 离线视频帧号。跨文件复盘时优先按 raw id、reason、bbox/score 和相邻窗口核对，不要只靠帧号直接等值 join。
  - 当前 `orin_hik_h264_MOT/01,02` 可提供 tracker hidden / split candidate 样本，但不覆盖全部 birth hidden reason；`ambiguous_recovery_pending`、`duplicate_split_hidden`、`skinny_partial_hidden`、`wide_fragment_hidden` 仍以 `test_identity_assignment_engine_adapter` 作为主要自动化证据。
- `LOCKED -> LOST` 转换次数
叠字 ID 语义（runtime 可视化与 offline `eval_overlay.mkv` 一致）：
- 画面中显示的 `id=` 为语义 ID（不是 tracker raw id）。
- 第一位成功锁定的主目标 `person` 语义 ID 固定为 `1`（红框）。
- 主目标后续切换时，沿用该目标既有语义 ID，仅变更为红框，不重写编号。
- 仅当出现新目标时，才分配新的语义 ID。

说明：
- 当 `--save-eval-video=true` 时，`avg_fps` 包含视频编码写盘开销，数值会低于“仅评估不写视频”的吞吐。
