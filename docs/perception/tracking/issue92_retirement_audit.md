# Issue #92 Retired Downstream Surface Audit

日期：2026-07-31

## Supported contract

视觉 runtime 的唯一下游 patrol 集成是 `dog_patrol_interfaces`：

- `/mission/state` 输入；
- `/mission/event` 上 `SOURCE_PERCEPTION` 的 READY、TARGET_CONFIRMED、TARGET_LOST、TARGET_REACQUIRED；
- `/perception/selected_target_bbox` 上当前帧可信 `TargetBoundingBox`。

`MissionRosAdapter` 是该 ROS transport 的唯一边界。没有 UDP compatibility mode、bearing output
或 `vision_msgs/Detection2DArray` 输出。

## Removed surfaces

- 删除 `BearingEstimator`、`UdpJsonAdapter`、`BearingOutput` 和 UDP adapter unit test；CMake 不再编译或注册它们。
- live node 不声明或读取 `bearing.*`、`udp.*` 参数，也不创建 socket、估算方位或发送 JSON。
- `offline_eval_recordings` 不再接受 `--enable-udp`、`--udp-ip`、`--udp-port`，也不生成 bearing
  overlay、per-frame CSV 字段或 summary 指标。已移除的选项会按未知命令行参数失败。
- 删除 `config/demo_params.yaml` 的 bearing/UDP 配置、`live_bearing_test.sh` 和
  `docs/downstream_bearing_udp.md`。

## Search Boundary

实现完成后，以以下搜索复核 active surface：

```bash
rg -n -i "udp|bearing|Detection2DArray|vision_msgs" \
  README.md docs src/vision_demo_host --glob '!worklog.md'
```

允许的剩余命中只用于明确退役结论或历史追溯：

- `README.md` 和 `src/vision_demo_host/README.md`：明确声明旧面不受支持；
- 本文档：审计结论；
- `docs/tracking_identity_refactor_plan.md`：2026-06-30 标记为草案的重构历史；
- `docs/orin_hik_h264_MOT_01_02_issue_resolution.md`：2026-06-24 的历史问题复盘；
- `worklog.md`：不可改写的历史工作记录。

仓库当前没有任何 `vision_msgs` include、package dependency、CMake dependency 或
`Detection2DArray` proposal；仅在 supported-contract 声明中明确其不受支持。
