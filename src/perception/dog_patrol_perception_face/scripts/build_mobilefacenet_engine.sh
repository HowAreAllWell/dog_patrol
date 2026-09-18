#!/usr/bin/env bash
set -euo pipefail

ZIP_PATH="${1:-/home/orin/迁移orin_MobileFaceNet_原始版.zip}"
ASSETS_ROOT="${2:-/mnt/nvme/workspace/dog_patrol/src/perception/dog_patrol_perception_assets_20260813}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
MODEL_PATH="$ASSETS_ROOT/face/w600k_mbf.onnx"
YUNET_PATH="$ASSETS_ROOT/face/face_detection_yunet_2023mar.onnx"
ENGINE_PATH="$ASSETS_ROOT/face/mobilefacenet_fp16.engine"
EXPECTED_SHA256="9cc6e4a75f0e2bf0b1aed94578f144d15175f357bdc05e815e5c4a02b319eb4f"
EXPECTED_YUNET_SHA256="8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4"

test -f "$ZIP_PATH" || { echo "model package is missing: $ZIP_PATH" >&2; exit 1; }
test -x "$TRTEXEC" || { echo "trtexec is missing: $TRTEXEC" >&2; exit 1; }
mkdir -p "$(dirname "$MODEL_PATH")"
TEMP_MODEL="$(mktemp "$(dirname "$MODEL_PATH")/.w600k_mbf.onnx.XXXXXX")"
TEMP_YUNET="$(mktemp "$(dirname "$YUNET_PATH")/.face_detection_yunet.onnx.XXXXXX")"
TEMP_ENGINE="$(mktemp "$(dirname "$ENGINE_PATH")/.mobilefacenet.engine.XXXXXX")"
trap 'rm -f "$TEMP_MODEL" "$TEMP_YUNET" "$TEMP_ENGINE"' EXIT

unzip -p "$ZIP_PATH" \
  '迁移orin_MobileFaceNet_原始版/models/w600k_mbf.onnx' > "$TEMP_MODEL"
ACTUAL_SHA256="$(sha256sum "$TEMP_MODEL" | cut -d' ' -f1)"
if [[ "$ACTUAL_SHA256" != "$EXPECTED_SHA256" ]]; then
  echo "MobileFaceNet checksum mismatch: $ACTUAL_SHA256" >&2
  exit 1
fi
unzip -p "$ZIP_PATH" \
  '迁移orin_MobileFaceNet_原始版/models/face_detection_yunet_2023mar.onnx' \
  > "$TEMP_YUNET"
ACTUAL_YUNET_SHA256="$(sha256sum "$TEMP_YUNET" | cut -d' ' -f1)"
if [[ "$ACTUAL_YUNET_SHA256" != "$EXPECTED_YUNET_SHA256" ]]; then
  echo "YuNet checksum mismatch: $ACTUAL_YUNET_SHA256" >&2
  exit 1
fi
mv "$TEMP_MODEL" "$MODEL_PATH"
mv "$TEMP_YUNET" "$YUNET_PATH"

"$TRTEXEC" \
  --onnx="$MODEL_PATH" \
  --saveEngine="$TEMP_ENGINE" \
  --fp16 \
  --builderOptimizationLevel=5 \
  --memPoolSize=workspace:1024 \
  --minShapes=input.1:1x3x112x112 \
  --optShapes=input.1:1x3x112x112 \
  --maxShapes=input.1:1x3x112x112 \
  --skipInference

test -s "$TEMP_ENGINE"
mv "$TEMP_ENGINE" "$ENGINE_PATH"
echo "MobileFaceNet engine ready: $ENGINE_PATH"
echo "YuNet enrollment model ready: $YUNET_PATH"
