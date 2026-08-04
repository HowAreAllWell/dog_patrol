# Issue #80 Hik MVS 帧合同与性能基线

## 硬件与输入事实

- 平台：Jetson aarch64，JetPack/L4T `R36.4.7`
- 相机：Hikrobot `MV-CU013-A0UC`，USB ID `2bdf:0001`
- 请求模式：`1280x1024@30 FPS`
- 实际源格式：`BayerGB8`（`0x0108000a`）
- 实际 payload：`1310720` bytes/frame
- 转换输出：自持有的 OpenCV `CV_8UC3` BGR8
- 测试时 preview、inference recording 均关闭；真实 TensorRT detector、tracker、
  ReID、GMC 和 identity 主链保持启用。

## 显式 Bayer 配置对照

2026-07-28 使用 `scripts/bench_hik_mvs_camera.sh` 在同一硬件和 live inference
路径运行。指标为 CameraIngest 最近窗口的 nearest-rank p50/p95/p99。

| Bayer quality | 稳态处理 FPS | acquisition ms p50/p95/p99 | conversion ms p50/p95/p99 | copy ms p50/p95/p99 | frames | estimated drops | MVS lost packets |
| --- | ---: | --- | --- | --- | ---: | ---: | ---: |
| `optimal` | 20.00 | 0.095 / 1.283 / 13.355 | 29.979 / 30.110 / 30.214 | 0.504 / 0.524 / 0.550 | 1128 | 584 | 0 |
| `fast` | 30.00 | 5.769 / 6.276 / 6.781 | 2.125 / 2.341 / 2.414 | 0.515 / 0.581 / 0.630 | 825 | 2 | 0 |

`fast` 的 2 个不连续帧发生在启动/预热阶段；进入稳态后计数未继续增加。
`optimal` 的 Bayer 转换本身约占 30 ms，因此串行 live inference 只能处理约
20 FPS，并持续跳过相机帧。

## 结论与边界

- 显式、受 SDK 支持的 `fast` 插值使当前纯 Hik MVS live inference 达到目标
  30 FPS；没有引入 GPU demosaic 或 fused detector preprocessing。
- runtime 默认仍使用 `optimal`，避免在没有图像质量/任务精度对照的情况下擅自改变
  Bayer 质量。选择长期默认质量属于 `#85` 的证据门控范围。
- `source_timestamp_ns` 使用取到对应 MVS frame buffer 时的系统时钟纳秒值，适合后续
  ROS header；SDK 的 `nHostTimeStamp` 因本机头文件未声明单位，另以
  `sdk_host_timestamp` 原值保留，不冒充纳秒。
- 原始完整日志保存在本机忽略目录 `log/bench_hik_mvs_camera*/`；本文件保存可追溯的
  关键硬件验收结果。
