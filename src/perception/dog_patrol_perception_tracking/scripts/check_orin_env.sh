#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
ENGINE_PATH="${ENGINE_PATH:-${WS_DIR}/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine}"
REID_DIR="${REID_DIR:-${WS_DIR}/assets/models/reid}"

FAILS=0
WARNS=0

pass() {
  echo "[PASS] $*"
}

warn() {
  echo "[WARN] $*"
  WARNS=$((WARNS + 1))
}

fail() {
  echo "[FAIL] $*"
  FAILS=$((FAILS + 1))
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

check_cmd() {
  local name="$1"
  if have_cmd "$name"; then
    pass "command found: $name ($(command -v "$name"))"
  else
    fail "command missing: $name"
  fi
}

echo "[env] workspace: $WS_DIR"
echo "[env] engine:    $ENGINE_PATH"
echo

ARCH="$(uname -m || true)"
if [[ "$ARCH" == "aarch64" ]]; then
  pass "architecture is aarch64"
else
  warn "architecture is $ARCH, expected aarch64 on Jetson Orin"
fi

if [[ -f /etc/nv_tegra_release ]]; then
  pass "Jetson marker found: /etc/nv_tegra_release"
  sed -n '1p' /etc/nv_tegra_release || true
else
  warn "Jetson marker not found: /etc/nv_tegra_release"
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  pass "ROS 2 Humble setup found"
else
  fail "ROS 2 Humble setup missing: /opt/ros/humble/setup.bash"
fi

check_cmd cmake
check_cmd g++
check_cmd colcon
check_cmd python3

if have_cmd nvcc; then
  pass "nvcc found"
  nvcc --version | sed -n '1,4p' || true
else
  warn "nvcc not found in PATH; CUDAToolkit may still be discoverable by CMake"
fi

if ldconfig -p 2>/dev/null | grep -q 'libnvinfer'; then
  pass "TensorRT runtime library found by ldconfig"
else
  fail "TensorRT runtime library not found by ldconfig"
fi

if find /usr /usr/local -name NvInfer.h -print -quit 2>/dev/null | grep -q .; then
  pass "TensorRT header NvInfer.h found"
else
  fail "TensorRT header NvInfer.h not found"
fi

if pkg-config --exists opencv4 2>/dev/null; then
  pass "OpenCV pkg-config found: $(pkg-config --modversion opencv4)"
else
  warn "opencv4 pkg-config not found; CMake may still find OpenCV via OpenCVConfig.cmake"
fi

if [[ -f /opt/MVS/include/MvCameraControl.h ]]; then
  pass "Hikrobot MVS header found"
else
  fail "Hikrobot MVS header missing: /opt/MVS/include/MvCameraControl.h"
fi

if [[ -f /opt/MVS/lib/64/libMvCameraControl.so ]]; then
  pass "Hikrobot MVS library found"
else
  fail "Hikrobot MVS library missing: /opt/MVS/lib/64/libMvCameraControl.so"
fi

if [[ -d /opt/MVS ]]; then
  pass "MVS SDK directory found: /opt/MVS"
else
  fail "MVS SDK directory missing: /opt/MVS"
fi

if have_cmd lsusb; then
  echo
  echo "[env] USB devices visible to system:"
  lsusb || true
else
  warn "lsusb not found; cannot print USB camera visibility"
fi

if [[ -f "$ENGINE_PATH" ]]; then
  pass "Orin TensorRT engine exists: $ENGINE_PATH"
else
  warn "Orin TensorRT engine missing; run src/vision_demo_host/scripts/export_yolo26n_engine_orin_jp621.sh on Orin"
fi

if [[ -f "${REID_DIR}/osnet_x1_0_market1501_256x128.onnx" ]]; then
  pass "ReID ONNX found"
else
  warn "ReID ONNX missing: ${REID_DIR}/osnet_x1_0_market1501_256x128.onnx"
fi

if [[ -f "${REID_DIR}/osnet_x1_0_market1501_256x128.onnx.data" ]]; then
  pass "ReID ONNX external data found"
else
  warn "ReID ONNX external data missing: ${REID_DIR}/osnet_x1_0_market1501_256x128.onnx.data"
fi

echo
echo "[env] summary: fails=$FAILS warns=$WARNS"
if [[ "$FAILS" -gt 0 ]]; then
  exit 1
fi
