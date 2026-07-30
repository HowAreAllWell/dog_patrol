# Issue #93 `balanced` Bayer 默认验证

本页记录 #93 对 #85 决策的最小可回滚配置实施与验证。变更只统一 active Hik MVS
live/capture/record entry 的默认 Bayer interpolation；不改变 detector、tracker、ReID、identity、
`CameraIngest::AcquiredFrame` 的自持有 BGR8 合同，或 gamma/CCM 控制，也不加入 GPU preprocessing
或 demosaic。

## 默认收敛与回滚

| 入口 | 无 override 的 Bayer interpolation | smoothing |
| --- | --- | --- |
| `CameraIngest::Config`、ROS node parameter declaration、`config/demo_params.yaml` | `balanced` | `false` |
| `live_bearing_test.sh` | `balanced` | `false` |
| `capture_ffv1` 与 `Ffv1CaptureWorkflow::Config` | `balanced` | `false` |
| `record_test_set` 的 Hik MVS path | `balanced` | `false` |

`bench_hik_mvs_camera.sh` 仍以显式 `fast` 作为固定性能比较，不是 production runtime 默认。
`fast`、`optimal`、`optimal_plus` 都继续由既有 parser 接受；旧 `optimal` 比较/回滚可使用：

```bash
ros2 run vision_demo_host vision_demo_node --ros-args \
  -p camera.bayer_interpolation:=optimal

ros2 run vision_demo_host capture_ffv1 --bayer-interpolation optimal ...
```

## 默认配置与 override 测试

在 `CameraIngest` 和 capture workflow 两个已有 public configuration seam 上先新增 RED
断言：无 override 的 config/metadata 必须是 `balanced` 与 `false`，而 `fast`、`balanced`、`optimal`、
`optimal_plus` 都必须被 parser 接受。改动前两条 default 断言分别得到 `optimal`，确认测试可检测
旧行为；改动后运行：

```bash
cmake --build build/vision_demo_host \
  --target test_camera_ingest_contract test_ffv1_capture_workflow -j2
source /opt/ros/humble/setup.bash
colcon test --packages-select vision_demo_host --event-handlers console_direct+ \
  --ctest-args -R 'test_camera_ingest_contract|test_ffv1_capture_workflow'
```

结果：2/2 CTest targets、16 tests 通过。`Ffv1CaptureWorkflow` 的 no-override take metadata 记录
`bayer_interpolation=balanced` 与 `bayer_smoothing=false`。

## Headless 1280x1024@30 committed-default 复核

在 Orin 的 MVS SDK / TensorRT / light ReID 正常配置下运行 35 秒 full live inference。命令没有传入
`camera.bayer_interpolation` 或 `camera.bayer_smoothing`，且禁用 preview/recording：

```bash
export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:/opt/MVS/lib/64:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout --signal=INT 35s ros2 run vision_demo_host vision_demo_node --ros-args \
  -p camera.mvs_model:=MV-CU013-A0UC \
  -p camera.width:=1280 -p camera.height:=1024 -p camera.fps:=30.0 \
  -p detector.runtime_path:="$PWD/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine" \
  -p detector.enable_fake_detection:=false \
  -p tracker.config_path:="$PWD/src/vision_demo_host/config/bot_sort.yaml" \
  -p visualization.enable:=false -p recording.enable:=false
```

启动日志明确记录 `bayer_interpolation=balanced bayer_smoothing=false`、BayerGB8
(`0x0108000a`) 和 BGR8 conversion target。最终 metrics 为 983 frames、0 acquisition failure、0 MVS
lost packets；两个 frame discontinuity 都在启动阶段，之后未再增加。稳态 FPS 为 29.97–30.04，最终
conversion p50/p95/p99 为 5.806/6.371/6.529 ms，copy 为 0.512/0.574/0.599 ms。完整忽略的 runtime
日志位于 `log/issue93_balanced_committed_default_20260731/hik_mvs_1280x1024_30fps.log`。

这复核了 #85 的 `balanced` full-live 结论应用到 no-override committed default 后仍保持 30 FPS；
它不替代需要图形会话的 #82 preview 验收。
