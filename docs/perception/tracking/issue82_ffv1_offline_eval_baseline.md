# Issue #82 FFV1 离线回放基线

本页记录 `offline_eval_recordings` 对 #81 canonical clean FFV1 capture 的首个可重复回放基线，
并保留历史 H.264 迁移 smoke 结果。评估工件都位于本机忽略的 `data/eval_results/`；capture source
不属于评估输出，未被评估过程修改。

## Canonical source 与合同

- source take：`data/captures/issue81_native_ffmpeg_q2_30s_20260730_200049/take_001/`
- source video：`video.mkv`，metadata 为 `complete`、`FFV1`、`MKV`、`bgr0`、1280x1024。
- `metadata.json.counts.written_frames=901`；`frame_timestamps.csv` 有 header 加 901 条记录。
- 回放前读取 sidecar，校验 capture index / source timestamp 严格递增、timestamps 行数等于
  `written_frames`；回放后再把实际从第一帧至最后一帧的 decoded count 与 901 对照。

该 source 的验收后 SHA-256（按 video、metadata、timestamps 顺序）为：

```text
55ea2996edbf17e7239cee6a00b0239ca2bb6a971a849f7ee736cbe2ef3eb271
b1ebb5b91383cf5d0551fb3f078f264a76bc4adbacdc041300103d266c5c9b9c
8242bec109c3c58a67d5a5fc5f4977ac67b7cfd1c3b63db0ebf3d1a76d6d637c
```

它们与回放前值一致，证明 source dataset 没有被 CSV、overlay 或其他评估结果覆盖或标注。
结果目录边界在比较前会解析符号链接；指向 source take 的 `--results-root` 也会在创建任何
run 目录前被拒绝。

## 回放命令与结果

在本工作区（使用本机 detector engine 和 tracker config）执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dog_patrol_perception_tracking offline_eval_recordings \
  --video /path/to/dog_patrol/data/captures/issue81_native_ffmpeg_q2_30s_20260730_200049/take_001/video.mkv \
  --results-root /path/to/dog_patrol/data/eval_results \
  --run-name issue82_ffv1_lossless_headless \
  --detector-engine /path/to/dog_patrol/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --tracker-config /path/to/dog_patrol/src/perception/dog_patrol_perception_tracking/config/bot_sort.yaml \
  --save-frame-csv false --save-sid-scores false --save-tracks-csv false
```

headless run `issue82_ffv1_lossless_headless_20260730_232542/s01` 成功：metadata/timestamps/decoded
分别为 `901/901/901`，离线吞吐 `44.613 FPS`。这个 `--video` 入口和 result summary 是 #85 做
固定 source sequence Bayer / detector 比较的 lossless baseline；不要将结果 overlay 当作 source。

同一 source 的 record-only 运行 `issue82_ffv1_lossless_record_retry_20260730_232640/s01` 也成功：
decoded `901`，吞吐 `11.122 FPS`。它的 `eval_overlay.mkv` 由 `ffprobe -count_frames` 验证为
`codec_name=ffv1`、`1280x1024`、`avg_frame_rate=2997/100`、`nb_read_frames=901`，大小
`524734159` bytes。逐帧抽取的 frame 450 显示 source 场景未被改写，而结果 overlay 有
`frame=450 det=0 tracks=0`、`IDLE` 和历史第三行诊断文字；这也是本 take 的无检测 review window。

## 本地图形会话验收

2026-07-31 在本机 seat0 的 X11 会话执行：`DISPLAY=:1`、
`XDG_SESSION_TYPE=x11`，并使用 `/run/user/1000/gdm/Xauthority`。`xdpyinfo` 确认该会话为
1920x1080 本地 X server；不使用 Xvfb、远程 X11 或无头窗口替代。

- preview-only run `issue82_preview_only_local_x11_20260731_152654/s01` 以
  `--overlay-preview true --overlay-record false` 正常退出。summary 为 metadata/timestamps/decoded
  `901/901/901`、`avg_fps=32.743`；运行中 X11 window tree 和 active-window 都确认
  `offline_eval_overlay` 由 `offline_eval_recordings` 创建。
- preview+record run `issue82_preview_record_local_x11_20260731_152756/s01` 以
  `--overlay-preview true --overlay-record true` 正常退出。窗口在运行中仍为 active，连续两次本地
  GNOME screen capture 分别显示 `frame=185` 和 `frame=513` 的实际 canvas，含 `IDLE` 诊断文字，
  证明窗口持续刷新。summary 为 metadata/timestamps/decoded `901/901/901`、`avg_fps=10.168`。
- 同一 preview+record run 的结果 `eval_overlay.mkv` 为 `520107210` bytes；`ffprobe` 确认为
  `ffv1`、`bgra`、1280x1024、2997/100 FPS，`ffprobe -count_frames` 实解码
  `nb_read_frames=901`。这只是结果目录内的 overlay；随后复算 source video、metadata、timestamps
  三个 SHA-256，仍与本页所列值相同。

这补齐四种独立 overlay mode 的本地图形验收。无图形环境仍应使用 headless 或 record-only。

## 历史 H.264 migration smoke

旧数据集仍可读，且旧 `metadata.json` 不被误作 #81 capture sidecar：

```bash
ros2 run dog_patrol_perception_tracking offline_eval_recordings \
  --recordings-root /path/to/dog_patrol/data/datasets \
  --datasets orin_hik_h264_MOT/03 \
  --results-root /path/to/dog_patrol/data/eval_results \
  --run-name issue82_historical_h264_smoke_retry \
  --detector-engine /path/to/dog_patrol/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --tracker-config /path/to/dog_patrol/src/perception/dog_patrol_perception_tracking/config/bot_sort.yaml \
  --save-frame-csv false --save-sid-scores false --save-tracks-csv false
```

run `issue82_historical_h264_smoke_retry_20260730_233151/s01` 成功回放 539 帧（39.432 FPS），
`det>0=539`、`tracks>0=539`、`LOCKED=358`、`OCCLUDED=177`。这是 migration regression，历史 H.264
不是 canonical source，也不在本票中删除。
