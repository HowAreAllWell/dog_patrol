# Issue #85 Bayer 转换与 detector 输入预处理审计

本页记录 #85 的可复现 headless 审计。它不更改 production 默认行为；`optimal` 仍是
当前已提交配置默认值，直到 required follow-up `#93` 被独立实现、验证并合入。

## 结论

`balanced` 是当前 `MV-CU013-A0UC` / `BayerGB8` / 1280x1024@30 的最小风险在线默认候选：

- 真实 headless live inference（detector、tracker、light ReID、identity 均启用，preview/record
  均关闭）稳定达到 30 FPS；
- 它比 `optimal` 的 Bayer conversion 快约一个数量级；
- 在含人的受控 Bayer 输入比较中，它相对于 `optimal` 的 detector box/confidence 差异远小于
  `fast` 和 `optimal_plus`，同时 tracker、ReID 和 semantic identity 结果保持相同的连续性；
- `fast` 是可用的性能 fallback，但不是画质/检测差异最小的实时候选；`optimal_plus` 虽然快，
  但实际输出与 detector box 偏差最大，不能把 SDK 的名称当作质量保证。

因此创建了 required implementation ticket [#93](https://github.com/HowAreAllWell/vision_demo_ws/issues/93)：
只把 active Hik MVS 入口的显式默认插值设为 `balanced`，保留全部 mode 的 override 与
`optimal` rollback。#93 阻塞 #87；#85 本票不直接修改默认值。

## 控制事实

平台为 Jetson Orin / aarch64，MVS SDK `0x04080003`，相机为 `MV-CU013-A0UC`
(`CAMERA_SERIAL`)。目标格式为实际从相机读回的 `BayerGB8` (`0x0108000a`)，payload
`1,310,720` bytes，1280x1024@30。

每次 Bayer compare 都在取得 source 后先复制 bytes，再把**同一批** immutable Bayer buffers
依次交给 `MV_CC_ConvertPixelTypeEx`。不是在不同时间拍四次场景。SDK 接受并成功转换以下全部
quality code：`fast=0`、`balanced=1`、`optimal=2`、`optimal_plus=3`。每种输出均以独立
FFV1/MKV take 保存并经 `ffprobe -count_frames` 核对帧数。

审计显式调用 `MV_CC_SetBayerFilterEnable(false)`；四个 variant 均为 smoothing=false。它只读
相机 image-control nodes，不擅自开启增强：

| 控制项 | 读回事实 | 审计解释 |
| --- | --- | --- |
| `GammaEnable` | supported，`false` | gamma 没有生效；`Gamma=0.7` 只是 disabled node 的当前值，不当作 SDK default。 |
| `GammaSelector` | supported，`1` | 记录为设备状态，不改写。 |
| `ColorTransformationEnable/Selector/Value` | unsupported，`0x80000106` | 此设备没有该 GenICam CCM surface。 |
| `CCMEnable` | unsupported，`0x80000106` | 不存在可开启的 CCM。 |
| `ColorCorrectionEnable` | unsupported，`0x80000109` | 不假定任何隐式 color correction。 |

这说明 gamma/CCM 是受控事实而非“SDK 默认必然如此”：同源 raw compare 固定了它们的输入，
未来若设备、固件或 node 状态改变，需要重新读回，不应把本页数值复制成默认设定。

## 真实相机同源 Bayer 比较

运行 `benchmark_bayer_input` 捕获 30 warmup 后的 60 个真实相机 Bayer frame；source acquisition
p50/p95/p99 为 `32.019/32.236/33.338 ms`，source ownership copy 为
`1.304/1.358/1.399 ms`。以下 conversion 和 detector stage 只针对该批相同 raw bytes：

| quality | conversion p50/p95/p99 ms | detector total p50/p95/p99 ms | raw pixel abs diff vs optimal p50/p95/p99, max | detector boxes |
| --- | ---: | ---: | ---: | --- |
| optimal | 27.272 / 27.358 / 27.738 | 17.194 / 17.798 / 17.924 | reference | 60/60 帧无检测 |
| fast | 1.741 / 1.780 / 1.806 | 15.933 / 16.175 / 16.721 | 0 / 0 / 1, 18 | 60/60 帧无检测 |
| balanced | 3.720 / 3.750 / 3.816 | 15.939 / 16.252 / 16.400 | 0 / 0 / 1, 7 | 60/60 帧无检测 |
| optimal_plus | 0.436 / 0.501 / 0.532 | 15.958 / 16.822 / 17.303 | 0 / 1 / 1, 18 | 60/60 帧无检测 |

当时真实相机取景没有 person/car，所有 mode 的 detector CSV 都为空。因此该 run 是
source-format、SDK-mode、像素和时延证据，不能把“空检测相同”误写成 ReID/tracking 精度结论。
四个 60-frame FFV1 take 仍通过完整 headless replay；均为 0 detection/track/identity，符合输入而非
失败。

## 含人 detector / ReID / tracking 受控比较

为取得有意义的 downstream evidence，另做了**受控但非传感器画质真值**的第二个实验：读取历史
`data/datasets/orin_hik_h264_MOT/03/video.mp4`（SHA-256
`939e85d6c80ac17cd1cc81fee17401c14b8efb7aa3ccdd6100426026109fe480`），跳过 30 帧后把同一
90 张 1280x1024 BGR8 输入按 top-left G 的 GB/RG pattern 重镶为 synthetic `BayerGB8` buffers，
再用连接的 MVS SDK 按四 mode 转换。历史 source 未被修改。

这个实验回答“同一有人的 Bayer-like 输入对 detector/tracker/identity 有何影响”，不回答真实镜头
颜色、噪声或低照度效果；真实 camera raw test 保留其专门的角色。

| quality | conversion p50 ms | 相对 optimal pixel mean / p95 / p99 / max abs | raw detections | 相对 optimal mean conf Δ / bbox L1 Δ | count-different frames |
| --- | ---: | ---: | ---: | ---: | ---: |
| optimal | 30.586 | reference | 182 | reference | 0 |
| fast | 1.790 | 1.449 / 6 / 18 / 190 | 181 | 0.002885 / 6.736 | 3 |
| balanced | 3.684 | 0.474 / 2 / 4 / 190 | 183 | 0.002376 / 0.686 | 1 |
| optimal_plus | 0.534 | 2.349 / 9 / 31 / 230 | 183 | 0.006216 / 42.597 | 5 |

每个 variant 再经同一 `offline_eval_recordings`、同一 TensorRT engine、同一 BoT-SORT config、
`with_reid=true` / light ReID 与同一 identity 参数从第一帧回放至最后一帧：

| quality | decoded / detection-positive / track-positive | tracks / identities | primary | semantic IDs | switches / LOST |
| --- | ---: | ---: | ---: | --- | ---: |
| optimal | 90 / 90 / 90 | 180 / 180 | 89 LOCKED + 1 PENDING_RECOVERY | 1 | 0 / 0 |
| fast | 90 / 90 / 90 | 180 / 180 | 89 LOCKED + 1 PENDING_RECOVERY | 1 | 0 / 0 |
| balanced | 90 / 90 / 90 | 180 / 180 | 89 LOCKED + 1 PENDING_RECOVERY | 1 | 0 / 0 |
| optimal_plus | 90 / 90 / 90 | 180 / 180 | 90 LOCKED | 1 | 0 / 0 |

所以四者在这个短、稳定的人物片段上都没有 identity continuity regression；但只有 `balanced`
同时接近 `optimal` 的 detection geometry 并保有实时 conversion 余量。不能据此推断遮挡、低照度、
多人或真实 Bayer 的所有质量场景已完全覆盖。

这不是只看 track/identity 最终计数：四个 full replay 的 `tracks.csv` 与 `sid_scores.csv` 还按
quality 汇总了实际 appearance evidence。下表 p 值均为 nearest-rank；`assoc_app_dist` 是 tracker
输出且 `assoc_appearance_used=1` 的所有行，`sid_app_cost` 是 semantic identity score 行。四种
mode 都是 180/180 tracker rows 使用 appearance，且没有 tracker appearance reject；semantic 的仅有
reject 均为 `weak_mot_association`，并非 appearance gate：

| quality | assoc_app_dist p50 / p95 / p99 / max（n=180） | sid_app_cost p50 / p95 / p99 / max | SID accepted / feature-update-allowed / weak-MOT reject |
| --- | ---: | ---: | ---: |
| optimal | 0.062078 / 0.108439 / 0.129170 / 0.133058 | 0.000047 / 0.000208 / 0.000693 / 0.001416 | 179 / 177 / 2 |
| fast | 0.058051 / 0.099008 / 0.109232 / 0.113548 | 0.000036 / 0.000199 / 0.000274 / 0.000497 | 179 / 178 / 1 |
| balanced | 0.060826 / 0.106211 / 0.119405 / 0.121710 | 0.000038 / 0.000223 / 0.001168 / 0.001468 | 179 / 176 / 3 |
| optimal_plus | 0.056928 / 0.101255 / 0.111820 / 0.113689 | 0.000032 / 0.000216 / 0.000333 / 0.000367 | 179 / 179 / 0 |

这些值在短而稳定的片段上没有显示 ReID/appearance continuity regression，也不能替代遮挡、多目标
或低照度的 appearance robustness 验证；它们与前表的 detector geometry evidence 共同支撑选择
`balanced`，而不是把相同的最终 semantic ID 误当作充分证据。

## detector preprocessing 分段和吞吐

`PreprocessInfer` 新增 opt-in metrics seam。CPU stage 用 wall clock，H2D / TensorRT / D2H 用同一 CUDA
stream 上的 CUDA events；因此它们不是 host enqueue time。以下是上述含人、`balanced` variant 的
90 sample p50/p95/p99（ms）：

| stage | p50 / p95 / p99 ms |
| --- | ---: |
| resize | 0.862 / 1.201 / 1.480 |
| border | 0.115 / 0.130 / 0.148 |
| BGR→RGB | 0.111 / 0.147 / 0.297 |
| uint8→float normalize | 0.452 / 0.469 / 0.562 |
| HWC→CHW layout（含 allocation/split） | 4.170 / 4.293 / 5.050 |
| H2D | 0.974 / 1.002 / 1.017 |
| TensorRT | 9.140 / 9.164 / 9.169 |
| D2H | 0.048 / 0.063 / 0.223 |
| parser | 0.008 / 0.008 / 0.011 |
| total | 16.033 / 16.623 / 17.055 |

该 benchmark 的 detector-only throughput 为 62.06 FPS；它不是 full live FPS。真实 full live
balanced run（35 秒、982 acquired frames）稳态 `29.97–30.04 FPS`，最终 camera metrics：
acquisition `4.456/4.972/5.433 ms`、conversion `5.742/6.259/6.463 ms`、copy
`0.527/0.585/0.606 ms`，`acquisition_failures=0`、`camera_lost_packets=0`。仅有两个启动/预热期的
frame discontinuities，之后不增长。此运行不需要 `DISPLAY`；启动时的 XOpenDisplay warning 未影响
headless pipeline 或退出。

`#80` 的 fast full-live 对照仍有价值：它为 30 FPS、conversion
`2.125/2.341/2.414 ms`、copy `0.515/0.581/0.630 ms`。这两份 live evidence 表明在 BGR8 contract
和现有 detector/tracker/ReID/identity 路径中，balanced 的约 3–4 ms conversion 成本仍有足够余量，
而 optimal 的约 30 ms conversion 不满足实时目标。

## 机会排序和实施决策

| 排名 | 机会 | 已测 bottleneck / 预期收益 | seam / 最小验证 | 决策 |
| ---: | --- | --- | --- | --- |
| 1 | 显式默认改为 balanced | 相比 optimal 移除约 24 ms Bayer conversion；真实 full live 30 FPS；受控 detector geometry 接近 optimal。 | camera config/defaults；#93 的 focused config tests、full regression、30 FPS no-overlay rerun；`optimal` parameter rollback。 | **创建 required #93，阻塞 #87。** |
| 2 | fused GPU detector preprocess | CPU layout + H2D p50 约 5.14 ms，是 TensorRT 之外最大的已测 detector cost；理论上可降低 host churn，但须保持 letterbox rounding、RGB/NCHW/float 与 output boxes 等价。 | 保持 `PreprocessInfer` 的 BGR8 input seam；固定 FFV1 + per-frame box/identity compare。 | 不创建 ticket：balanced 已达实时目标，收益尚不足以抵消 CUDA kernel/engine compatibility 与比较成本。 |
| 3 | clone / buffer reuse | BGR ownership clone p50 约 0.53 ms，远低于 optimal→balanced 的收益；它是 SDK buffer lifetime 和下游 BGR8 stable ownership 的安全边界。 | `CameraIngest::AcquiredFrame` ownership contract 与 camera contract tests；release build live drop comparison。 | 不创建 ticket：低杠杆，错误实现会破坏 GMC/ReID/preview/record consumer safety。 |
| 4 | GPU Bayer demosaic | balanced full-live conversion p50 5.74 ms 且已经 30 FPS。 | BGR8 interface 后的 isolated implementation；所有 quality/detector/identity/live-drop comparisons。 | 不创建 ticket：最高复杂度，当前没有 material bottleneck。 |

## 可复现入口与工件

`benchmark_bayer_input` 仅在 MVS SDK 可用时构建，默认不启用 `PreprocessInfer` timing；该工具显式
启用 metrics，因此不为完成审计改变 production inference 行为。示例：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vision_demo_host benchmark_bayer_input \
  --output-dir data/audit/issue85_bayer_identical_20260730_161800 \
  --detector-engine assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --mvs-model MV-CU013-A0UC --mvs-serial CAMERA_SERIAL \
  --warmup-frames 30 --frames 60 --width 1280 --height 1024 --fps 30

ros2 run vision_demo_host benchmark_bayer_input \
  --output-dir data/audit/issue85_synthetic_people_20260731_002000 \
  --input-video data/datasets/orin_hik_h264_MOT/03/video.mp4 \
  --detector-engine assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --mvs-model MV-CU013-A0UC --mvs-serial CAMERA_SERIAL \
  --warmup-frames 30 --frames 90 --width 1280 --height 1024 --fps 30
```

`benchmark_bayer_input` 默认只匹配 `MV-CU013-A0UC`；若同型号设备不止一台，必须给
`--mvs-serial`，否则工具显式失败而不会审计枚举顺序中的任意相机。synthetic run 仍会打开这台 MVS
设备以使用同一 SDK converter。上述 run 的 historical source、detector engine 与 tracker config
SHA-256 分别为
`939e85d6c80ac17cd1cc81fee17401c14b8efb7aa3ccdd6100426026109fe480`、
`92477cc6dceb1d8c737646469e07bebd2f387ae7ea050f93cd012781dcccb8a8`、
`2ee7339e04f00a1066c3071e630d83cdb84c67b2ae8079a2811b2474fe7b2064`。

四个 downstream replay 均由相同 headless command 运行；将 `mode` 依次替换为
`optimal`、`fast`、`balanced`、`optimal_plus`，并给每个 run 单独的 `--run-name`（本次工件分别为
`issue85_synthetic_optimal_20260731_001655`、`issue85_synthetic_fast_20260731_001659`、
`issue85_synthetic_balanced_20260731_001703`、`issue85_synthetic_optimal_plus_20260731_001707`）：

```bash
ros2 run vision_demo_host offline_eval_recordings \
  --video data/audit/issue85_synthetic_people_20260731_002000/bayer_${mode}/video.mkv \
  --results-root data/eval_results --run-name issue85_synthetic_${mode}_rerun \
  --detector-engine assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine \
  --det-raw-conf 0.10 --det-person-conf 0.10 --det-car-conf 0.10 \
  --tracker-config src/vision_demo_host/config/bot_sort.yaml \
  --tracker-reid-backend light --tracker-reid-input-width 128 --tracker-reid-input-height 256 \
  --save-frame-csv true --save-sid-scores true --save-tracks-csv true \
  --overlay-preview false --overlay-record false
```

这条入口使用本次审计 revision 的 identity 默认值（feature bank=30，strict/relaxed recovery
similarity=0.85/0.75，max missing=180，`sid_missing_assign_max_app_cost=0.50`，
`sid_app_w/sid_geo_w/sid_time_w=0.70/0.20/0.10`）；每次 replay 应核对 FFV1 metadata/timestamps 为
90、`summary.json` 的 `total_frames=90`、detection/tracks-positive=90、switch/lost=0，并从
`tracks.csv` / `sid_scores.csv` 重新计算前表 appearance 汇总。

full-live balanced run 的命令、输出位置和验收条件也固定如下；它只关闭 visualization/recording，
不要求图形会话：

```bash
WS_DIR="$PWD" MVS_MODEL=MV-CU013-A0UC MVS_SERIAL=CAMERA_SERIAL \
CAMERA_WIDTH=1280 CAMERA_HEIGHT=1024 CAMERA_FPS=30.0 \
BAYER_INTERPOLATION=balanced BAYER_SMOOTHING=false RUN_SECONDS=35 \
OUT_DIR="$PWD/log/issue85_balanced_live_20260731_002300" \
"$PWD/src/vision_demo_host/scripts/bench_hik_mvs_camera.sh"
```

核对 `summary.txt` 的 `BayerGB8` / `0x0108000a`、982 frames、0 acquisition failure / 0 MVS lost
packets、conversion `5.742/6.259/6.463 ms`，以及日志末段稳态约 30 FPS；实际帧数可随启动相位略变，
不把 982 当作硬性功能门槛。

原始 artifacts 位于忽略的 `data/audit/` 和 `data/eval_results/`：每个 `bayer_*` take 都有
`metadata.json`、timestamp CSV 与 FFV1 MKV；其 session 根目录有汇总 `report.txt` 和每个 mode 的
`<mode>_detections.csv`，full replay 输出另在 `data/eval_results/`。真实 balanced live summary/log
位于忽略的 `log/issue85_balanced_live_20260731_002300/`。

## 未覆盖项和边界

- 当前无本地可授权 `DISPLAY`；这不阻塞本票的 raw/source、conversion、detector 或 offline tracking
  evidence，也不代替 #82 preview-only / preview+record 的现场窗口验收。
- 真实 camera 原始取景无目标；含人比较使用明确标记的 synthetic Bayer，而非声称真实 camera
  colorimetric benchmark。后续现场若采到含人原始 Bayer，可用同一工具补充低照度、多人和遮挡窗口，
  但它不是 #93 最小 default-change implementation 的前置条件。
- 没有 production 默认行为在 #85 中被改变；没有把 GPU fused preprocess、buffer reuse 或 GPU demosaic
  当作“顺手优化”实施。
