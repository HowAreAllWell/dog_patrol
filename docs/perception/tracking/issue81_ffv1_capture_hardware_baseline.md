# Issue #81 FFV1 采集硬件基线

本页记录 `capture_ffv1` 在 Orin + `MV-CU013-A0UC` 上的首次硬件验证。采集工具只初始化
Hik MVS 和 FFV1 写入路径，不加载 detector、tracker 或 identity。

## 命令与产物

早期三次运行均为 headless 自动 take：

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
| 3 秒本地显示预览对照（queue=1） | `balanced` | `data/captures/issue81_local_display_preview_control_20260730_193300/` | complete；captured/written/dropped=`90/46/44`，camera frame gaps=`0` |
| 本地显示人工交互（queue=120） | `balanced` | `data/captures/issue81_local_interactive_acceptance_20260730_193338/` | `take_001` complete，captured/written/dropped=`234/234/0`，含 1 个 Marker；`take_002` complete，`65/65/0`；两段 camera frame gaps 均为 `0` |

数据目录被 `.gitignore` 忽略，仅作为本机验收证据保留。

## 容器、合同与连续性

`ffprobe` 对 post-fix 的 3 秒 take 的结果为 `codec_name=ffv1`、`1280x1024`、
`avg_frame_rate=30/1` 且 `nb_read_frames=41`，与 `metadata.json.counts.written_frames=41`
一致。FFV1/OpenCV stream 是 configured CFR，所以 `avg_frame_rate` 只代表 nominal stream rate；
metadata 明确写入 `stream_fps_is_nominal=true`、take elapsed、`captured_fps` 与
`written_fps`，供数据集消费者使用真实吞吐口径。

本地显示人工交互产生的两个 `video.mkv` 也经 `ffprobe` 确认为 `codec_name=ffv1`、
`1280x1024`（OpenCV writer 在容器中报告 `pix_fmt=bgra`；采集和 metadata 的帧合同仍为
`BGR8`）。

3 秒 take 的 metadata 记录 `output_pixel_format=BGR8`、source PixelType=`BayerGB8`
(`0x0108000a`)、payload=`1310720` bytes、`bayer_interpolation=balanced`。逐帧 CSV
有 41 行；其 source timestamp span 为 `3.000503` 秒。camera frame number 连续；capture
index 的缺口与有界 writer queue 的 50 个显式丢帧一致，而非相机侧丢帧。

从 `video.mkv` 抽取的首帧为 `1280x1024` PNG，人工查看未发现 preview 状态文字或其他
overlay；场景本身光线较暗。`test_ffv1_capture_workflow` 还以已知 BGR8 像素验证了 FFV1
round-trip 的逐像素一致性。

## 本地预览与人工交互验收

接入本地显示器（`DISPLAY=:1`）后，3 秒 preview-first 自动 take 在 queue=1 下仍采集 90
帧，即相机采集保持 30 FPS。随后在真实预览窗口中手动执行
`R → M → S → R → S → Q`：首段耗时 `7.794971` 秒、captured/written FPS 均为
`30.019354`，Marker `manual_marker` 位于 `3.795776` 秒；第二段耗时 `2.171389` 秒、
captured/written FPS 均为 `29.934761`。两个 take 均为 complete，且没有 queue 丢帧、写入
错误或 camera frame gap。至此两个 take、Marker 和正常退出的真实预览交互均完成验收。

同一设备经 MobaXterm X11 转发时，3 秒 preview-first 对照仅采集 1 帧；相同采集配置切换
headless 后采集 90 帧。因此远程 X11 预览会阻塞采集循环，不应用于实时录制；录制时应使用
`--headless`，MobaXterm preview 仅用于构图检查。本次及后续最小复现均未再次触发先前观察到
的退出后 segmentation fault，故其仍是未稳定复现的风险，而非已归因的程序缺陷。

## 结论与遗留风险

`balanced` 下相机采集达到目标：post-fix 3 秒 run 的
`captured_fps=30.188`，但 `written_fps=13.601`；queue=1 时 50 帧按约定显式丢弃。较大的
queue=120 在 10 秒 run 中缓冲后写出 254 帧、仍丢弃 47 帧。30 FPS capture target 已被观测到，
但当前 CPU FFV1 写入端是明确吞吐瓶颈；工具不会把 container 的 nominal 30 FPS 误报为实际
写入速率。

本地显示下的真实预览交互已完成，覆盖两个 take、一个 Marker 和正常退出。当前 CPU FFV1
写入吞吐仍需按实测判断：早期 queue=1/queue=120 压力基线有显式 drop，而本次较短的本地
交互运行没有 drop；不能只按 nominal 30 FPS 或单次无 drop 结果推断持续写入能力。先前一次
preview/headless 运行曾在退出后报告 segmentation fault，但后续 headless（有/无 `DISPLAY`）、
本地 preview 对照和人工交互均以 exit code 0 完成；若再次出现，应保留 `coredumpctl` 记录后
再归因。
