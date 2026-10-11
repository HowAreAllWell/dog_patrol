# -*- coding: utf-8 -*-
"""ONNX 特征提取速度基准（ViT-B/16 after）。

测量单帧延迟，预处理与推理分开计时：
  - onnxruntime CPU / CUDA（如安装了 GPU 版）
  - OpenCV DNN（C++ 节点当前使用的后端，顺便验证其能否加载该图）
用法：
  python scripts/benchmark_onnx.py [--onnx weights/reid_clipreid_vitb16_after.onnx]
      [--image data/persons/xxx.jpg] [--iters 30]
同一脚本可直接在 Orin 上跑，得到目标机的真实数字。
"""
import os

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # Windows numpy/torch OpenMP 冲突

import argparse
import glob
import statistics
import time

import cv2
import numpy as np


def bench(fn, warmup, iters):
    for _ in range(warmup):
        fn()
    ts = []
    for _ in range(iters):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    return statistics.mean(ts), statistics.median(ts)


def report(name, mean_ms, p50_ms):
    print(f"  {name:<34s} mean {mean_ms:8.2f} ms   p50 {p50_ms:8.2f} ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default="weights/reid_clipreid_vitb16_after.onnx")
    ap.add_argument("--image", default=None)
    ap.add_argument("--iters", type=int, default=30)
    args = ap.parse_args()

    img_path = args.image
    if img_path is None:
        cands = sorted(glob.glob("data/persons/*/*.jpg")) or sorted(
            glob.glob("data/persons/*/*.png"))
        assert cands, "data/persons 下找不到测试图，用 --image 指定"
        img_path = cands[0]
    print(f"模型: {args.onnx}")
    print(f"图片: {img_path}   iters={args.iters}\n")

    # 预处理与 C++ blobFromImage(swapRB=true) 等价：BGR 读图 -> RGB CHW float32
    def preprocess():
        im = cv2.imread(img_path)
        im = cv2.resize(im, (128, 256))
        chw = im.transpose(2, 0, 1)[::-1]     # BGR -> RGB（模型输入为 RGB）
        return np.ascontiguousarray(chw[None].astype(np.float32))

    m, p = bench(preprocess, 5, 20)
    report("预处理 imread+resize+CHW", m, p)
    blob = preprocess()
    print()

    ref = None  # 取一个输出做各后端一致性对照

    # ---- onnxruntime ----
    try:
        import onnxruntime as ort
        provs = ort.get_available_providers()
        print(f"onnxruntime 可用 Provider: {provs}")
        for want in ("CUDAExecutionProvider", "CPUExecutionProvider"):
            if want not in provs:
                continue
            sess = ort.InferenceSession(args.onnx, providers=[want])
            iname = sess.get_inputs()[0].name
            out = sess.run(None, {iname: blob})[0].ravel()
            if ref is None:
                ref = out
            cos = float(np.dot(ref, out) /
                        (np.linalg.norm(ref) * np.linalg.norm(out) + 1e-12))
            m, p = bench(lambda: sess.run(None, {iname: blob}), 10, args.iters)
            report(f"ORT {want} (dim={out.size})", m, p)
            print(f"    L2={np.linalg.norm(out):.4f}  与参考输出余弦={cos:.6f}")
    except ImportError:
        print("onnxruntime 未安装，跳过")
    print()

    # ---- OpenCV DNN（C++ 节点当前后端）----
    try:
        net = cv2.dnn.readNetFromONNX(args.onnx)

        def run_cv():
            net.setInput(blob)
            return net.forward()

        out = run_cv().ravel()
        if ref is None:
            ref = out
        cos = float(np.dot(ref, out) /
                    (np.linalg.norm(ref) * np.linalg.norm(out) + 1e-12))
        m, p = bench(run_cv, 3, min(args.iters, 15))
        report(f"OpenCV DNN CPU (dim={out.size})", m, p)
        print(f"    与参考输出余弦={cos:.6f}")
    except cv2.error as e:
        print(f"OpenCV DNN 加载/推理失败：{e}")


if __name__ == "__main__":
    main()
