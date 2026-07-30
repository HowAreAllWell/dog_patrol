# vision_demo_host

Orin 宿主机侧视觉验收 demo，包含检测、短期跟踪、语义身份、主目标选择、方位估计、录制和离线评估工具。

## 当前链路状态

已接入：
- `camera_ingest`：仅 Hikrobot MVS；向下游返回自持有的 BGR8 图像和源帧元数据
- `preprocess_infer`：YOLO26n TensorRT engine 推理（本机导出的 `.engine`）
- `det_filter`：`person + car` 阈值过滤（配置化）
- `mot_tracker`：BoT-SORT 风格实现（Kalman + 两阶段关联 + GMC + appearance）
- `primary_target_manager`：`person`-only 主目标规则（首锁最大框 + continuity-first）
- `bearing_estimator`：demo 近似 bearing（非标定真值）
- `udp_json_adapter`：localhost UDP JSON

未接入或未完成：
- identity 层尚未迁移为完整的新状态机；`IdentityManager` 是公开接口并持有当前 identity runtime state。内部 `IdentityAssignmentEngineAdapter` 只承担 assignment / recovery / update engine 适配，调用已抽取的 policy、cost、solver、birth、binding、store、mutation 和 projection helper，保持既有 debug row、raw-to-semantic mapping 与 public behavior。Phase 5 BirthManager 已固定为唯一运行时 birth / hidden candidate 路径，但 primary/output 仍保持当前公开语义。
- 距离/2D 相对位置
- 控制逻辑

## 关键配置

默认参数文件：`config/demo_params.yaml`

核心项：
- `camera.mvs_model` / `camera.mvs_serial`
- `camera.width` / `camera.height` / `camera.fps` / `camera.timeout_ms`
- `camera.bayer_interpolation`（`fast|balanced|optimal|optimal_plus`）
- `camera.bayer_smoothing`
- `detector.runtime_path`
- `detector.raw_conf_threshold`
- `detector.person_conf_threshold` / `detector.car_conf_threshold`
- `tracker.*`
- `target.lost_threshold_frames`
- `bearing.camera_horizontal_fov_rad`
- `bearing.camera_mount_x/y/z`
- `bearing.camera_mount_roll/pitch/yaw`
- `udp.ip` / `udp.port`
- `visualization.enable` / `recording.enable`

建议 engine 路径：
- `/path/to/my_workplace/vision_demo_ws/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine`

## 输出 JSON 字段

当前 UDP JSON 字段：
- `target_id`
- `track_state`
- `target_class`
- `bbox_xywh`
- `u_norm`
- `v_norm`
- `bearing_base_rad`
- `elevation_base_rad`
- `bearing_confidence`

`target_id` 当前语义（重要）：
- `target_id` 是业务主目标 ID（`primary_target_id`），用于跨 raw tracker id 变化保持稳定。
- `target_id` 初次通常为 `1`；当主目标长时丢失（超过 `target.lost_threshold_frames`）并重选后，`target_id` 可切到 `2/3/...`。
- tracker 原生 ID（`raw_track_id`）仅作为内部调试语义，不作为对外稳定身份字段。

`bearing_base_rad` 当前语义（重要）：
- 当前实现会读取配置中的 `base_link -> camera_link` 静态外参初值（roll/pitch/yaw），
  将相机视线方向旋转到 `base_link` 后计算 `bearing_base_rad = -atan2(y, x)`。
- 相比纯框中心/FOV 近似更接近机身方位语义，但仍不是正式外参标定后的严格几何真值。
- 当前不消费运行中的 `/tf_static` 或 `tf2 lookupTransform()`。
- 若 bearing 外参配置无效，代码会回退到旧近似：
  `bearing_base_rad ~= -(u_norm - 0.5) * camera_horizontal_fov_rad`，并在日志给出告警。
- `camera_mount_x/y/z` 当前作为静态外参初值记录保留，当前方位角计算主要使用 roll/pitch/yaw 旋转链。
- 当前采用“图像坐标近似光学视线 + 静态外参初值旋转”的假设；未来可在补齐
  `camera_link -> camera_optical_frame` 与正式内外参后继续升级。
- 符号约定：
  - `bearing_base_rad > 0` 表示目标在机身左侧（left-positive）；
  - `bearing_base_rad < 0` 表示目标在机身右侧（right-negative）。

`elevation_base_rad` 当前语义（重要）：
- 由同一条 `ray_base=[x,y,z]` 计算：
  `elevation_base_rad = atan2(z, sqrt(x*x + y*y))`。
- 语义：相对机身水平面的俯仰角/仰角。
- 符号约定：
  - `elevation_base_rad > 0` 表示目标方向高于水平面；
  - `elevation_base_rad < 0` 表示目标方向低于水平面。

## 主目标绑定策略（当前）

- 初次锁定：选最大有效 `person`。
- 锁定后：优先保持同一业务主目标（`primary_target_id`），不因 raw tracker id 变化立刻切换。
- 短时丢失：进入 `OCCLUDED` 时间窗，优先等待主语义目标恢复，不做“候选人重绑定”。
- 长时丢失（超过 `target.lost_threshold_frames`）：才允许重选新的最大 `person` 并分配新的业务主目标 ID。
- `car` 仍不允许成为主目标。

## 构建

```bash
cd /path/to/my_workplace/vision_demo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_demo_host
```

## 运行（示例）

Hikrobot MVS / USB3 Vision 工业相机：

```bash
cd /path/to/my_workplace/vision_demo_ws
source install/setup.bash
ros2 run vision_demo_host vision_demo_node --ros-args \
  -p camera.mvs_model:='MV-CU013-A0UC' \
  -p camera.mvs_serial:='CAMERA_SERIAL' \
  -p camera.width:=1280 \
  -p camera.height:=1024 \
  -p camera.fps:=30.0 \
  -p camera.bayer_interpolation:='optimal' \
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

## UDP 接收

```bash
nc -u -l -p 5005
```

## 一键现场方位测试脚本

脚本入口：
- `/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/scripts/live_bearing_test.sh`

功能：
- 自动启动 UDP 监听并实时打印：`track_state`、`target_id`、`u/v`、`bearing_base_rad`、`elevation_base_rad`（含角度制）
- 前台启动 `vision_demo_node`
- 支持可视化/录制开关参数

示例（实时预览 + 实时方位打印，不录制）：

```bash
cd /path/to/my_workplace/vision_demo_ws
src/vision_demo_host/scripts/live_bearing_test.sh \
  --mvs-model MV-CU013-A0UC \
  --viz true \
  --rec false
```

示例（实时预览 + 录制）：

```bash
cd /path/to/my_workplace/vision_demo_ws
src/vision_demo_host/scripts/live_bearing_test.sh \
  --mvs-model MV-CU013-A0UC \
  --viz true \
  --rec true \
  --record-path '/path/to/my_workplace/vision_demo_ws/data/recordings/live_$(date +%Y%m%d_%H%M%S).mp4' \
  --record-fps 25.0
```

## 基本验证重点

- build 成功
- 节点可加载真实 TensorRT engine
- Hik MVS ingest 输出 BGR8 acquired-frame 并携带源帧元数据
- detector 输出可驱动后续链路
- BoT-SORT tracker 初始化并输出轨迹（画面有目标时）
- UDP JSON 可接收并包含完整字段

## Tracker-Core 配置

默认配置入口：`config/bot_sort.yaml`（`core_mode=new_core`）。默认入口只保留当前 runtime 生效字段；历史对照字段隔离在 `config/legacy/` 与 `config/diagnostics/`。
- `core_mode`：默认 `new_core`；`old_minimal` 仅保留在 legacy 对照配置中
- `track_high_thresh`
- `track_low_thresh`
- `new_track_thresh`
- `match_thresh`
- `track_buffer`
- `gmc_method`（当前默认：`sparseOptFlow`）
- `gmc_downscale`（当前默认：`4`，表示 GMC 估计在 `1/4` 分辨率上进行）
- `with_reid`（默认：`true`；运行时强制开启）
- `confirm_hits`
- `stage1_iou_min` / `stage2_iou_min` / `unconfirmed_iou_min`
- `motion_gate_thresh`
- `assoc_iou_weight` / `assoc_motion_weight` / `assoc_app_weight`
- `appearance_gate` / `appearance_alpha` / `appearance_h_bins` / `appearance_s_bins`
- `tracker.gmc_enabled` 由 ROS 参数控制；`tracker.reid_enabled` / `with_reid` 运行时会强制为 `true` 并打印告警。

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

一键离线评估对照脚本：

```bash
/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/scripts/eval_tracker_core_round1.sh
```

## 问题文档

- 当前 tracking identity 实现状态：`/path/to/my_workplace/vision_demo_ws/docs/current_tracking_identity_state.md`
- `orin_hik_h264_MOT` 01/02 遮挡与 ID 问题复盘：`/path/to/my_workplace/vision_demo_ws/docs/orin_hik_h264_MOT_01_02_issue_resolution.md`

## Hik MVS 相机性能基线

一键运行无录制 live inference，并收集实际 PixelType、吞吐、阶段 p50/p95/p99 和 frame drop：

```bash
/path/to/my_workplace/vision_demo_ws/src/vision_demo_host/scripts/bench_hik_mvs_camera.sh
```

默认目标为 `1280x1024@30 FPS`、`fast` Bayer interpolation，运行 60 秒，输出到
`log/bench_hik_mvs_camera/`。硬件验收应保存 `summary.txt` 和完整日志。
本轮 `fast` / `optimal` 对照证据见
`docs/issue80_hik_mvs_frame_contract_baseline.md`。

历史 RTSP 基线（仅作迁移前对照，不是当前 live input）：
- `A(gmc_off)` 平均 FPS 约 `24.07`
- `B(sparseOptFlow, downscale=2)` 平均 FPS 约 `15.87`
- `C(sparseOptFlow, downscale=4)` 平均 FPS 约 `21.49`（init 日志显示 `gmc_downscale=4` 已生效）

旧 `scripts/bench_gmc_rtsp.sh` 保留给 #86 统一清理；当前 live inference
节点已不再接受 RTSP 参数，不应把该历史脚本用于新的 #80 基线。

建议默认配置（当前）：
- `gmc_enabled=true`
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
ros2 run vision_demo_host capture_ffv1 \
  --mvs-model MV-CU013-A0UC \
  --width 1280 --height 1024 --fps 30.0 \
  --bayer-interpolation optimal \
  --session-name orin_hik_lossless_01
```

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
ros2 run vision_demo_host capture_ffv1 \
  --headless --auto-record --max-seconds 30 \
  --session-name headless_capture_01
```

`record_test_set` 仍保留旧 RTSP/GStreamer/H.264 路径，等待 `#86` 统一清理；不要将其用于
新的无损 Hik MVS 数据集采集。

## 离线回放评估工具（offline_eval_recordings）

入口：
- 可执行：`offline_eval_recordings`
- 默认数据集根目录：`/path/to/my_workplace/vision_demo_ws/data/datasets`
- 默认结果目录：`/path/to/my_workplace/vision_demo_ws/data/eval_results`

用途：
- 对固定录制集逐帧回放，复用当前主链模块做可重复评估：
  - `preprocess_infer`
  - `det_filter`
  - `mot_tracker`
  - `primary_target_manager`
  - `bearing_estimator`

运行（默认处理历史 `orin_hik_h264_MOT/01,02,03`，仅作为迁移回归）：

```bash
cd /path/to/my_workplace/vision_demo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_demo_host offline_eval_recordings
```

新的 canonical 回放入口是 #81 完整 take 的显式 `video.mkv` 路径；它不依赖数据集目录中曾经
硬编码的 `video.mp4`：

```bash
ros2 run vision_demo_host offline_eval_recordings \
  --video /absolute/path/to/capture_session/take_001/video.mkv \
  --results-root /absolute/path/to/eval_results \
  --run-name ffv1_baseline
```

当 `video.mkv` 同目录有 `metadata.json` 时，工具只接受 `complete` 的 FFV1/MKV capture metadata，
并校验 `counts.written_frames`、`frame_timestamps.csv` 行数、capture index/source timestamp 单调性，
以及实际从首帧到末帧解码出的帧数。任何不一致都会在 `summary.json`/`summary.md` 和进程退出码中
显式失败。若没有 capture metadata，目录发现优先 `video.mkv`，再保留历史 `video.mp4` 作为迁移
回归输入。

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
- `--tracker-reid-backend <light|osnet_onnx>`
- `--tracker-reid-model-path <path>`
- `--enable-udp`（默认关闭）
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
- `identity_metrics.json` / `identity_metrics.md` 聚合现有 debug CSV，不改变 tracker、identity、primary、UDP、overlay 或既有 CSV schema；缺失的可选输入会标记为 `unavailable`，对应分布保持空/零计数。当前聚合项包括 primary state（含 `PENDING_RECOVERY`）、primary decision/reject/recovery reason、identity state、assignment stage/reject reason、feature/geometry update reason、Phase 3 shadow `event_type`、NewBirthCandidate hidden/pending/allocated reason、Phase 4 handoff event、tracklet hypothesis status/reason。
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
  - 该 CSV 是 shadow-only debug 输出，不参与 semantic id 分配、primary、overlay、UDP 或 `IdentityAssignmentEngineAdapter` 决策。
- Phase 5 birth / hidden candidate readiness：
  - 证据基线见 `docs/phase5_birth_hidden_candidate_readiness.md`。
  - `sid_scores.csv` 中 `stage=phase5_birth_candidate` 表示 Phase 5 birth proposal / hidden / pending decision，`stage=phase5_new_semantic`、`accepted=1` 表示 Phase 5 path 已正式分配 semantic id。
  - `phase3_shadow_state.csv` 中 `new_birth_candidate_pending|new_birth_candidate_hidden|new_birth_candidate_allocated` 为 Phase 5 lifecycle evidence；pending / allocation decision 由 `IdentityManager` Phase 5 path 产生。
  - `sid_scores.csv` 的 `frame_idx` 来自 assignment engine adapter 内部 debug frame；`tracklet_hypotheses.csv` 和 `phase3_shadow_state.csv` 使用 0-based 离线视频帧号。跨文件复盘时优先按 raw id、reason、bbox/score 和相邻窗口核对，不要只靠帧号直接等值 join。
  - 当前 `orin_hik_h264_MOT/01,02` 可提供 tracker hidden / split candidate 样本，但不覆盖全部 birth hidden reason；`ambiguous_recovery_pending`、`duplicate_split_hidden`、`skinny_partial_hidden`、`wide_fragment_hidden` 仍以 `test_identity_assignment_engine_adapter` 作为主要自动化证据。
- `LOCKED -> LOST` 转换次数
- `bearing_base_rad` 抖动指标：
  - `bearing_diff_abs_mean`
  - `bearing_diff_stddev`

叠字 ID 语义（runtime 可视化与 offline `eval_overlay.mp4` 一致）：
- 画面中显示的 `id=` 为语义 ID（不是 tracker raw id）。
- 第一位成功锁定的主目标 `person` 语义 ID 固定为 `1`（红框）。
- 主目标后续切换时，沿用该目标既有语义 ID，仅变更为红框，不重写编号。
- 仅当出现新目标时，才分配新的语义 ID。

说明：
- 当 `--save-eval-video=true` 时，`avg_fps` 包含视频编码写盘开销，数值会低于“仅评估不写视频”的吞吐。
