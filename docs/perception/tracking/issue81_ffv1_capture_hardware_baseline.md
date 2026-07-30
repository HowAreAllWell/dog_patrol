# Issue #81 FFV1 采集硬件基线

本页记录 `capture_ffv1` 在 Orin + `MV-CU013-A0UC` 上的首次硬件验证。采集工具只初始化
Hik MVS 和 FFV1 写入路径，不加载 detector、tracker 或 identity。

## 命令与产物

两次运行均为 headless 自动 take（当前环境没有 `DISPLAY`）：

```bash
build/vision_demo_host/capture_ffv1 \
  --headless --auto-record --max-seconds <seconds> \
  --output-root data/captures --width 1280 --height 1024 --fps 30.0 \
  --bayer-interpolation <optimal|balanced>
```

| 运行 | Bayer | 结果目录 | 结果 |
| --- | --- | --- | --- |
| 2 秒 smoke | `optimal` | `data/captures/issue81_hardware_smoke_20260730_131042/` | complete；captured/written/dropped=`54/54/0`，camera frame gaps=`6` |
| 10 秒压力验证（queue=120） | `balanced` | `data/captures/issue81_balanced_10s_20260730_131121/` | complete；captured/written/dropped=`301/254/47`，write errors=`0`，camera frame gaps=`0` |
| 3 秒严格背压验证（queue=1） | `balanced` | `data/captures/issue81_balanced_queue1_3s_20260730_132054/` | complete；captured/written/dropped=`91/41/50`，write errors=`0`，camera frame gaps=`0` |

数据目录被 `.gitignore` 忽略，仅作为本机验收证据保留。

## 容器、合同与连续性

`ffprobe` 对 post-fix 的 3 秒 take 的结果为 `codec_name=ffv1`、`1280x1024`、
`avg_frame_rate=30/1` 且 `nb_read_frames=41`，与 `metadata.json.counts.written_frames=41`
一致。FFV1/OpenCV stream 是 configured CFR，所以 `avg_frame_rate` 只代表 nominal stream rate；
metadata 明确写入 `stream_fps_is_nominal=true`、take elapsed、`captured_fps` 与
`written_fps`，供数据集消费者使用真实吞吐口径。

3 秒 take 的 metadata 记录 `output_pixel_format=BGR8`、source PixelType=`BayerGB8`
(`0x0108000a`)、payload=`1310720` bytes、`bayer_interpolation=balanced`。逐帧 CSV
有 41 行；其 source timestamp span 为 `3.000503` 秒。camera frame number 连续；capture
index 的缺口与有界 writer queue 的 50 个显式丢帧一致，而非相机侧丢帧。

从 `video.mkv` 抽取的首帧为 `1280x1024` PNG，人工查看未发现 preview 状态文字或其他
overlay；场景本身光线较暗。`test_ffv1_capture_workflow` 还以已知 BGR8 像素验证了 FFV1
round-trip 的逐像素一致性。

## 结论与遗留验收

`balanced` 下相机采集达到目标：post-fix 3 秒 run 的
`captured_fps=30.188`，但 `written_fps=13.601`；queue=1 时 50 帧按约定显式丢弃。较大的
queue=120 在 10 秒 run 中缓冲后写出 254 帧、仍丢弃 47 帧。30 FPS capture target 已被观测到，
但当前 CPU FFV1 写入端是明确吞吐瓶颈；工具不会把 container 的 nominal 30 FPS 误报为实际
写入速率。

当前环境没有 `DISPLAY`，所以尚未在真实预览窗口中手动完成两个 take 和 marker；状态机
的 `R → S → R → M → Q` 等价路径已由 `test_ffv1_capture_workflow` 覆盖。下一次带显示器的
Orin 运行应完成该交互验收，并将两 take/marker 目录加入本机证据。
