# Issue #86 活动媒体路径退役审计

## 结论

当前运行面已收敛为 Hik MVS live input 和 FFV1/MKV capture、replay、diagnostic recording。
仓库不再构建或安装 RTSP/GStreamer camera path，也不再提供 H.264 recorder 或有损 fallback。
历史 MP4 仅由 offline migration regression 读取，不是默认或 canonical 数据集。

## 删除和收敛

- 删除 `record_test_set` 源码、CMake target 和 install target；它原有的 RTSP/GStreamer input、
  H.264 mode 与 `mp4v` fallback 一并删除。
- 删除 `scripts/bench_gmc_rtsp.sh`；当前性能基线入口是 `bench_hik_mvs_camera.sh`。
- `offline_eval_recordings` 的输入根默认改为 `data/captures`，结果根默认改为
  `data/eval_results`；启动时必须显式传 `--video` 或 `--datasets`，不再默认选择历史数据集。
- `eval_tracker_core_round1.sh` 必须接收显式视频路径，并使用当前 `--overlay-record` 选项。
- live node 在 mission、camera 和 detector 初始化前检查 parameter overrides；
  `camera.backend`、`camera.rtsp_url`、`camera.gstreamer_pipeline` 均明确报错并以非零状态退出，
  不会被 ROS 2 静默保留。

## 当前三个入口

- standalone capture：`capture_ffv1` 只初始化 Hik MVS，写 clean BGR8 FFV1/MKV take。
- live inference：`dog_patrol_perception_tracking_node` 运行 detector/tracker/identity/primary 和
  `dog_patrol_interfaces` mission output；preview 与 FFV1 diagnostic recording 独立可选。
- offline evaluation：`offline_eval_recordings` 回放显式选择的 take/video；preview 与 FFV1
  result recording 独立可选，结果不写入 clean source dataset。

## 历史兼容边界

`OfflineEvalInput` 只为路径组件属于 `orin_hik_h264_MOT` 且文件名为 `video.mp4` 的输入保留
discovery，并统一标记为 `historical_h264`。其他 MP4 和非 MKV 显式视频会失败，不能作为未标记
的 `explicit_video` 穿透。该边界仅用于迁移回归。
对应自动化在 `test_offline_eval_input`，实际历史证据在
`docs/issue82_ffv1_offline_eval_baseline.md` 和 `docs/issue85_bayer_preprocess_audit.md`。
`orin_hik_h264_MOT` 问题复盘、架构记录、旧 worklog 与上述 evidence docs 保持历史原文；这些内容
不构成当前运行说明。源码中旧 camera 参数名只允许出现在 explicit rejection guard 和其 CLI test。

## 验证

定向实现搜索未发现 `rtspsrc`、H.264 parser/encoder、`CAP_GSTREAMER`、`mp4v` 或旧 pipeline
builder。源码树与 CMake 不再含 `record_test_set` / `bench_gmc_rtsp`，构建 target 和
`ros2 pkg executables dog_patrol_perception_tracking` 均只列当前入口。

`test_retired_media_cli` 通过：三个 retired ROS 参数各自以 exit 1 明确失败；
`offline_eval_recordings` 无输入和 `--rtsp-url` 分别以 exit 2 明确失败。

端到端 headless 回放：

- #81 canonical FFV1 take：901/901 帧解码，source metadata 对账通过。
- `orin_hik_h264_MOT/03` historical migration：显式指定 `data/datasets` 根，539/539 帧解码通过。

最终 `colcon build`、全量 `colcon test` 和 `colcon test-result --verbose --all` 通过：
47 个 CTest targets、371 tests、0 errors/failures/skipped。`git diff --check` 与 shell syntax
检查通过。
