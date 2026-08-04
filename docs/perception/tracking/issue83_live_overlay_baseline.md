# #83 Live overlay 非阻塞基线

本页记录 live Hik MVS inference 的 diagnostic overlay 输出合同、自动化验证和首个 Orin headless 基线。
它只描述 result overlay；clean BGR8 source dataset 继续由 `capture_ffv1`（#81）负责。

## 合同与配置

`dog_patrol_perception_tracking_node` 的 live mode 由两个独立开关组成：

| mode | `visualization.enable` | `recording.enable` | 行为 |
| --- | --- | --- | --- |
| `inference_only` | false | false | 不启动 overlay worker，是性能 baseline。 |
| `preview` | true | false | worker 只显示 overlay；需要真实交互式 `DISPLAY`。 |
| `record` | false | true | worker 把 overlay 结果写到 FFV1/MKV。 |
| `preview_record` | true | true | 同一个 worker canvas 同时显示并写入。 |

`recording.path` 在 active live path 必须以 `.mkv` 结尾，且必须位于 `recording.output_root`（默认 `data/diagnostics/live_overlays/`）之下；初始化会在创建目录后、创建 artifact parent 前再次 canonicalize，拒绝 lexical 越界和 diagnostic root 内的 symlink escape。live camera 没有 source dataset path，因此 `VisualizerRecorder::Config::protected_source_dataset_roots` 默认声明项目 clean capture root `data/captures/`：result root 与它的任一 ancestor/descendant 重叠或 symlink 到它都会被拒绝。native FFmpeg FFV1 slice writer 是唯一实现，容器像素格式为 `bgr0`。

`visualization.queue_capacity` 默认 `4`。inference thread 只进行一次 O(1) job move；队列锁在 worker 开始 canvas render、HighGUI 和 FFmpeg/filesystem write 前释放。队满时丢弃最新 diagnostic frame；不会等待这些输出阶段。每秒日志分别报告 `queue_drops`、`render_drops`、`write_drops`、错误计数、submitted/rendered/previewed/written FPS，以及 queue wait/render/write p50/p95/p99。

Overlay canvas 在 worker 内从 self-owned BGR8 job clone 生成，保留 primary、identity 和 track label 语义；preview 和 recorder 消费同一个 canvas。因而 source frame 不被 overlay 改写，recording artifact 也不是 source capture。

## 自动化验证

- `test_visualizer_recorder`：四 mode 配置、FFV1/MKV 与 diagnostic output root 规则、root 外路径、root 内 escape、受保护 capture root 和 root symlink 拒绝（并确认拒绝前不创建外部子目录）、无 `DISPLAY` 时 preview 的明确失败/安全 `Shutdown`、blocked writer 时 non-blocking submit 与满队列丢最新帧、native FFV1/MKV decode。
- deterministic artifact test 以固定 BGR8 frame、raw track `3`、ACTIVE semantic identity `7`、LOCKED primary `7` 录制一帧，并逐像素比较 decoded FFV1 artifact 与独立构造的 box、`id=7 ACTIVE raw=3` label 和 `LOCKED id=7 raw=3` primary line；source frame 保持不变。
- `test_ffv1_capture_workflow`：#81 artifact writer 继续通过共享 `Ffv1MkvWriter` 的 native FFmpeg path 回归；capture metadata 保持 native FFmpeg/slice contract。
- 未启动 Xvfb/Weston 或把无头 GUI 冒充人工视觉验收。

## Orin headless 基线（2026-07-31）

环境为 AGX Orin（aarch64）、MV-CU013-A0UC、MVS SDK、1280x1024@30、`balanced` / smoothing=false、真实 TensorRT engine、真实 detector/tracker/identity。inference-only 采样 38 秒；record 复跑至 SIGINT 前约 22.85 秒。相机画面没有 person，故现场 inspection 只验证 IDLE overlay 文本与 artifact；含 tracking/identity/primary box 的渲染语义由上述确定性 FFV1 artifact test 覆盖。

| mode | processed FPS | capture p50/p95/p99 (ms) | inference p50/p95/p99 (ms) | overlay p50/p95/p99 (ms) | drops / errors |
| --- | ---: | --- | --- | --- | --- |
| inference_only | 30.04 | acquisition 4.479/5.265/5.975; conversion 5.635/6.251/6.461; copy 0.512/0.569/0.589 | 18.435/18.787/19.033 | N/A | camera 2 startup discontinuities; output all 0 |
| record | 29.85 | acquisition 4.773/5.992/6.739; conversion 5.528/6.076/6.326; copy 0.521/0.583/0.633 | 18.339/21.029/21.614 | queue wait 0.043/0.295/2.346; render 0.683/0.761/0.996; write 24.007/31.027/34.378 | camera 2 startup discontinuities; queue/render/write drops and errors all 0 |

退出阶段会在 `Shutdown()` drain/close 后输出 `overlay_metrics phase=final`，所以 final worker counter 可与 artifact 精确对账。此次 record-only 复跑为 submitted/enqueued/rendered/written `682/682/682/682`，29.85 FPS；`ffprobe -count_frames` 同为 682 帧、FFV1、1280x1024、`bgr0`、30/1；file size 121,085,262 bytes，SHA-256 `ec028be76661b70d503f7be1b2caa62732ab9b382efda00908aeab82a6265d96`。先前 `1128/1133` 差异是 SIGINT 后 drain 发生在最后一个周期日志之后的可观测性缺口，不是丢帧；新的 final lifecycle log 消除了该歧义。

The inspected first decoded image is the true result artifact frame: a black camera scene with the white `IDLE id=-1 raw=-1` primary overlay at top-left. It confirms diagnostic overlay output and source/result separation in record-only mode; it is not a preview visual acceptance.

Raw local logs (ignored runtime outputs):

```text
log/issue83_live_modes_20260731/inference_only.log
log/issue83_live_modes_20260731/record_only_final.log
log/issue83_live_modes_20260731/record_only_final_ffprobe.txt
log/issue83_live_modes_20260731/record_only_final_sha256.txt
data/diagnostics/live_overlays/issue83_record_final_20260731_010442.mkv
```

## Orin X11 mode matrix（2026-07-31）

`DISPLAY=:1` 的本地 X11 会话已可用。对同一台 AGX Orin / `MV-CU013-A0UC`、1280x1024@30、`balanced` /
`smoothing=false`、真实 TensorRT engine 和真实 detector/tracker/identity 执行了四模式现场采样。所有运行均在
SIGINT 后输出 `overlay_metrics phase=final`。

| mode | 运行时长 | capture p50/p95/p99 (ms) | inference p50/p95/p99 (ms) | overlay p50/p95/p99 (ms) | final 结果 |
| --- | ---: | --- | --- | --- | --- |
| `inference_only` | 35 s | acquisition 0.035/0.160/1.697; conversion 10.124/10.309/10.486; copy 0.623/0.753/0.901 | 28.305/35.305/38.544 | N/A | 22.98 FPS；不启动 worker，output counters 全为 0。 |
| `preview` | 65 s | acquisition 0.035/2.118/7.680; conversion 9.905/10.422/10.830; copy 0.599/0.722/0.937 | 25.683/32.430/34.314 | queue wait 0.052/0.084/0.311; render 0.718/1.044/1.419 | `submitted/enqueued/rendered/previewed=1530/1530/1530/1530` (24.24 FPS), all queue/render/write drops and errors 0. |
| `record` | 35 s | acquisition 0.031/7.756/11.016; conversion 4.161/6.928/8.681; copy 0.498/1.449/2.244 | 20.118/28.808/34.839 | queue wait 11.070/54.370/73.652; render 0.672/0.901/1.188; write 30.690/43.824/48.656 | `submitted/enqueued/rendered/written=977/977/977/977` (29.36 FPS), all queue/render/write drops and errors 0. |
| `preview_record` | 45 s | acquisition 0.030/5.975/9.763; conversion 4.565/7.799/9.876; copy 0.518/1.921/2.904 | 22.140/31.573/35.660 | queue wait 149.243/175.680/186.616; render 0.681/0.905/1.161; write 32.903/46.837/52.067 | `submitted/enqueued/rendered/previewed/written=1202/1011/1011/1011/1011`; 191 bounded queue drops, no render/write drop or error. |

`preview` 的 X11 窗口 `dog_patrol_perception_tracking` 已实际映射并用屏幕截图目检；画布持续刷新，当前现场画面的
primary line 为 `IDLE id=-1 raw=-1`。`preview_record` 的第一个 decoded FFV1 frame 有相同的当前画布和
primary line，因此未把 preview 与 recording 当作两套 overlay 实现。

两个结果 artifact 均在可信 diagnostic root 下，`ffprobe -count_frames` 与 final worker `written`
完全一致：

| mode | artifact | `ffprobe` | SHA-256 |
| --- | --- | --- | --- |
| `record` | `issue83_record_only_20260731_160500.mkv` | FFV1, 1280x1024, `bgr0`, 30/1, 977 decoded frames | `2d29c7c279a7dab52631c66238817f256c7d37383e5414465dfdb6d51a7261c6` |
| `preview_record` | `issue83_preview_record_20260731_155700.mkv` | FFV1, 1280x1024, `bgr0`, 30/1, 1011 decoded frames | `509139cb8af61f6ac7e5ad190d776d6e0e4912a136718eaa9dbbd6a23b3f2b43` |

Raw local evidence (ignored runtime output):

```text
log/issue83_graphical_acceptance_20260731/inference_only_current.log
log/issue83_graphical_acceptance_20260731/preview_window_inspection.log
log/issue83_graphical_acceptance_20260731/preview_active_window.png
log/issue83_graphical_acceptance_20260731/record_only_current.log
log/issue83_graphical_acceptance_20260731/preview_record.log
log/issue83_graphical_acceptance_20260731/*_ffprobe.txt
log/issue83_graphical_acceptance_20260731/*_sha256.txt
data/diagnostics/live_overlays/issue83_{record_only,preview_record}_20260731_*.mkv
```

## 现场人像验收（2026-07-31）

在同一 Orin/X11/Hik MVS 环境把真实 person 置入视场后，重新执行 `preview` 和
`preview_record`。两个模式的 `runtime_monitor` 都连续出现
`det=1 filtered=1 tracks=1 state=LOCKED primary_id=1 raw_track_id=1`。

- `preview` 的实际 X11 `dog_patrol_perception_tracking` 窗口显示真人红色 box、`id=1 ACTIVE raw=1` 标签和
  `LOCKED id=1 raw=1` primary line。60 秒运行 final 为 submitted/enqueued `1519/1519`，
  queue/render/write drops 和 errors 全为 0，previewed FPS 26.17；queue wait p50/p95/p99 为
  0.054/0.305/2.526 ms，render 为 0.972/1.301/1.957 ms。相机最后采样的
  acquisition/conversion/copy p50/p95/p99 分别为 0.034/4.146/7.387、
  6.823/10.339/11.686、0.575/0.765/1.307 ms；inference 为
  20.518/29.516/33.292 ms。
- `preview_record` 的同一 canvas decoded FFV1 frame 显示相同真人 box、identity 与 primary
  语义，运行日志在对应时段连续报告上述 `LOCKED` 状态。50 秒运行 final submitted/enqueued 为
  `1392/1143`，249 个 bounded queue drop，
  render/write drops 和 errors 全为 0；rendered/previewed/written FPS 都为 23.70。queue wait、
  render、write p50/p95/p99 分别为 148.244/174.195/182.022、0.763/0.993/1.317、
  31.999/45.364/50.324 ms。相机最后采样的 acquisition/conversion/copy 为
  0.030/7.685/10.871、4.382/7.457/10.865、0.517/0.756/1.295 ms；inference 为
  19.079/28.148/35.358 ms。
- `issue83_person_preview_record_20260731_1721.mkv` 为 FFV1、`bgr0`、1280x1024、30/1；
  `ffprobe -count_frames` 实解码 1143 帧，与 final enqueued 且无 render/write drop/error 的
  drain 结果精确一致。SHA-256 为
  `1ce0667b50d336a435cee4b7de46b346dc803d4780f7b35c06d77dbd4bb5a0e4`。

本次证据补齐了先前 IDLE-only mode matrix 缺少的真人 tracking/identity/primary 目检；四种
live mode 和同 canvas 的 preview/record 合同均已验收。

Raw local evidence (ignored runtime output):

```text
log/issue83_person_acceptance_20260731/preview.log
log/issue83_person_acceptance_20260731/preview_person_window.png
log/issue83_person_acceptance_20260731/preview_record_person.log
log/issue83_person_acceptance_20260731/preview_record_person_frame_1s.png
data/diagnostics/live_overlays/issue83_person_preview_record_20260731_1721.mkv
```

可复现命令必须加载参数文件；本机资产路径显式覆盖参数文件中部署相关的绝对路径：

```bash
source /opt/ros/humble/setup.bash
source /path/to/workspace/dog_patrol/install/setup.bash
source install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node --ros-args \
  --params-file src/dog_patrol_perception_tracking/config/perception_tracking_params.yaml \
  -p detector.runtime_path:="$PWD/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine" \
  -p tracker.config_path:="$PWD/src/dog_patrol_perception_tracking/config/bot_sort.yaml" \
  -p visualization.enable:=true -p recording.enable:=false
ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node --ros-args \
  --params-file src/dog_patrol_perception_tracking/config/perception_tracking_params.yaml \
  -p detector.runtime_path:="$PWD/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine" \
  -p tracker.config_path:="$PWD/src/dog_patrol_perception_tracking/config/bot_sort.yaml" \
  -p visualization.enable:=true -p recording.enable:=true \
  -p recording.path:="data/diagnostics/live_overlays/issue83_preview_record_$(date +%Y%m%d_%H%M%S).mkv"
```
