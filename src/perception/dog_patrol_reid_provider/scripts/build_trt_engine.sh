#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# 在 Orin（JetPack 6 / TensorRT 10）上构建 ReID ViT FP16 engine 并测速。
# engine 与设备/TRT 版本绑定，每台机器各自构建；不能跨机拷贝。
#
# 用法：
#   bash scripts/build_trt_engine.sh [onnx] [engine]
#   默认: weights/reid_clipreid_vitb16_after.onnx -> weights/reid_clipreid_vitb16_after.fp16.engine
set -euo pipefail

ONNX="${1:-weights/reid_clipreid_vitb16_after.onnx}"
ENGINE="${2:-weights/reid_clipreid_vitb16_after.fp16.engine}"

TRTEXEC="$(command -v trtexec || true)"
if [ -z "$TRTEXEC" ] && [ -x /usr/src/tensorrt/bin/trtexec ]; then
  TRTEXEC=/usr/src/tensorrt/bin/trtexec
fi
if [ -z "$TRTEXEC" ]; then
  echo "错误：找不到 trtexec（JetPack 应预装于 /usr/src/tensorrt/bin）" >&2
  exit 1
fi
[ -f "$ONNX" ] || { echo "错误：ONNX 不存在: $ONNX" >&2; exit 1; }

echo "== trtexec: $TRTEXEC"
echo "== 构建 FP16 engine（首次约 1~5 分钟），输出: $ENGINE"
"$TRTEXEC" --onnx="$ONNX" --fp16 \
  --memPoolSize=workspace:2048 \
  --saveEngine="$ENGINE"

echo
echo "== 测速（engine 直接推理，看 GPU Compute Time 与 Latency）=="
"$TRTEXEC" --loadEngine="$ENGINE" --iterations=100 --avgRuns=100

echo
echo "完成：$ENGINE"
echo "节点启用：ros2 run dog_patrol_reid_provider reid_node --ros-args \\"
echo "  -p device:=tensorrt -p model_path:=$(pwd)/$ENGINE -p device_id:=m20-001"
echo
echo "若构建报 unsupported node：先做图简化再重试："
echo "  pip install onnxsim && python -m onnxsim $ONNX ${ONNX%.onnx}.sim.onnx"
echo "  bash scripts/build_trt_engine.sh ${ONNX%.onnx}.sim.onnx $ENGINE"
echo "构建后务必跑精度/阈值余量验证：python3 scripts/validate_trt_fp16.py"
