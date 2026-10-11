#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TensorRT FP16 精度与阈值余量验证（在 Orin 上运行）。

用 onnxruntime 的 TensorRT EP（FP16）与 CPU EP（FP32 基准）分别提特征，
在真实图（data/persons，目录名即身份）上对比：
  1. 同一张图 TRT-fp16 vs CPU-fp32 的余弦漂移（数值一致性）
  2. 同人/跨人相似度分布（阈值是否仍然安全：跨人最大值须低于阈值）
PASS 条件：漂移 < 3e-3，且 (阈值 - 跨人最大值) > 0.002，且 (同人最小值 - 阈值) > 0.002。

前置：安装 Jetson Zoo 的 onnxruntime-gpu（含 TRT EP），
  https://elinux.org/Jetson_Zoo#ONNX_Runtime  （JetPack 6 / Python 3.10 版）
  pip3 install <onnxruntime_gpu-...-cp310-cp310-linux_aarch64.whl>

用法（仓库根目录）：
  python3 scripts/validate_trt_fp16.py \
      [--onnx weights/reid_clipreid_vitb16_after.onnx] \
      [--images data/persons] [--threshold 0.70] [--per-person 6] \
      [--engine-cache /tmp/reid_trt_cache]
首次运行会由 TRT EP 构建 engine（数分钟），之后走缓存。
"""
import argparse
import glob
import os

import cv2
import numpy as np


def preprocess(path):
    im = cv2.imread(path)
    if im is None:
        raise SystemExit(f"无法读取图片: {path}")
    im = cv2.resize(im, (128, 256))
    chw = im.transpose(2, 0, 1)[::-1]  # BGR -> RGB CHW float32 0..255
    return np.ascontiguousarray(chw[None].astype(np.float32))


def cosine(a, b):
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default="weights/reid_clipreid_vitb16_after.onnx")
    ap.add_argument("--images", default="data/persons")
    ap.add_argument("--threshold", type=float, default=0.70)
    ap.add_argument("--per-person", type=int, default=6)
    ap.add_argument("--engine-cache", default="/tmp/reid_trt_cache")
    args = ap.parse_args()

    import onnxruntime as ort
    have_trt = "TensorrtExecutionProvider" in ort.get_available_providers()
    if not have_trt:
        raise SystemExit(
            "onnxruntime 无 TensorRT EP（应装 Jetson Zoo 的 onnxruntime-gpu），中止")

    os.makedirs(args.engine_cache, exist_ok=True)
    trt_opts = ort.SessionOptions()
    trt_provs = [
        ("TensorrtExecutionProvider", {
            "device_id": 0,
            "trt_fp16_enable": True,
            "trt_max_workspace_size": 2 << 30,
            "trt_engine_cache_enable": True,
            "trt_engine_cache_path": args.engine_cache,
        }),
        "CUDAExecutionProvider", "CPUExecutionProvider",
    ]
    print(f"TRT session 构建中（首次含 engine 编译，需数分钟）...")
    sess_trt = ort.InferenceSession(args.onnx, trt_opts, providers=trt_provs)
    sess_ref = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    name = sess_trt.get_inputs()[0].name

    persons = {}
    for d in sorted(glob.glob(os.path.join(args.images, "*"))):
        files = sorted(glob.glob(os.path.join(d, "*.jpg")))[: args.per_person]
        if files:
            persons[os.path.basename(d)] = files
    if len(persons) < 2:
        raise SystemExit("至少需要两个身份目录做跨人对比")

    emb_trt, emb_ref, labels = [], [], []
    for pid, files in persons.items():
        for f in files:
            blob = preprocess(f)
            emb_trt.append(sess_trt.run(None, {name: blob})[0].ravel())
            emb_ref.append(sess_ref.run(None, {name: blob})[0].ravel())
            labels.append(pid)
    emb_trt = np.stack(emb_trt)
    emb_ref = np.stack(emb_ref)
    n = len(labels)
    print(f"图 {n} 张 / 身份 {len(persons)} 个\n")

    # 1) fp16 漂移
    drift = [1.0 - cosine(emb_trt[i], emb_ref[i]) for i in range(n)]
    # 2) 同/跨人分布（TRT fp16 输出）
    same_t, cross_t = [], []
    for i in range(n):
        for j in range(i + 1, n):
            s = cosine(emb_trt[i], emb_trt[j])
            (same_t if labels[i] == labels[j] else cross_t).append(s)
    drift_max = max(drift)
    cross_max = max(cross_t)
    same_min = min(same_t)

    print(f"fp16 漂移   : max {drift_max:.5f}  mean {np.mean(drift):.5f}")
    print(f"同人相似度  : min {same_min:.4f}  mean {np.mean(same_t):.4f}")
    print(f"跨人相似度  : max {cross_max:.4f}  mean {np.mean(cross_t):.4f}")
    print(f"阈值 {args.threshold} 的余量  : 跨人侧 {args.threshold - cross_max:+.4f}"
          f"（同人侧 {same_min - args.threshold:+.4f}）")

    ok = drift_max < 3e-3 and (args.threshold - cross_max) > 0.002 \
        and (same_min - args.threshold) > 0.002
    print("\nPASS：FP16 engine 可直接沿用现阈值" if ok else
          "\nFAIL：余量不足或漂移过大——按跨人最大值上调/下调阈值并重新标定"
          "（scripts/calibrate_threshold.py），不要沿用旧阈值")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
