#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
MVS_MODEL="${MVS_MODEL:-MV-CU013-A0UC}"
MVS_SERIAL="${MVS_SERIAL:-}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1024}"
CAMERA_FPS="${CAMERA_FPS:-30.0}"
BAYER_INTERPOLATION="${BAYER_INTERPOLATION:-fast}"
BAYER_SMOOTHING="${BAYER_SMOOTHING:-false}"
ENGINE_PATH="${ENGINE_PATH:-${WS_DIR}/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine}"
TRACKER_CONFIG="${TRACKER_CONFIG:-${WS_DIR}/src/dog_patrol_perception_tracking/config/bot_sort.yaml}"
RUN_SECONDS="${RUN_SECONDS:-60}"
OUT_DIR="${OUT_DIR:-${WS_DIR}/log/bench_hik_mvs_camera}"

if [[ ! -f "$ENGINE_PATH" ]]; then
  echo "[bench] detector engine not found: $ENGINE_PATH" >&2
  exit 2
fi
if [[ ! -f "$TRACKER_CONFIG" ]]; then
  echo "[bench] tracker config not found: $TRACKER_CONFIG" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
LOG_FILE="${OUT_DIR}/hik_mvs_${CAMERA_WIDTH}x${CAMERA_HEIGHT}_${CAMERA_FPS}fps.log"

set +u
export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:/opt/MVS/lib/64:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

ROS_ARGS=(
  -p "camera.mvs_model:=$MVS_MODEL"
  -p "camera.width:=$CAMERA_WIDTH"
  -p "camera.height:=$CAMERA_HEIGHT"
  -p "camera.fps:=$CAMERA_FPS"
  -p "camera.bayer_interpolation:=$BAYER_INTERPOLATION"
  -p "camera.bayer_smoothing:=$BAYER_SMOOTHING"
  -p "detector.runtime_path:=$ENGINE_PATH"
  -p "detector.enable_fake_detection:=false"
  -p "tracker.config_path:=$TRACKER_CONFIG"
  -p "visualization.enable:=false"
  -p "recording.enable:=false"
)
if [[ -n "$MVS_SERIAL" ]]; then
  ROS_ARGS+=(-p "camera.mvs_serial:=$MVS_SERIAL")
fi

echo "[bench] running Hik MVS live inference for ${RUN_SECONDS}s; recording disabled"
set +e
timeout --signal=INT "${RUN_SECONDS}s" \
  ros2 run dog_patrol_perception_tracking dog_patrol_perception_tracking_node --ros-args "${ROS_ARGS[@]}" \
  2>&1 | tee "$LOG_FILE"
status=${PIPESTATUS[0]}
set -e
if [[ $status -ne 0 && $status -ne 124 && $status -ne 130 ]]; then
  exit "$status"
fi

SUMMARY_FILE="${OUT_DIR}/summary.txt"
{
  echo "Hik MVS camera baseline"
  echo "requested=${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS}"
  echo "bayer_interpolation=${BAYER_INTERPOLATION}"
  echo "bayer_smoothing=${BAYER_SMOOTHING}"
  grep "startup_camera_source" "$LOG_FILE" | tail -n 1
  grep "runtime_monitor fps=" "$LOG_FILE" | tail -n 1
  grep "camera_metrics " "$LOG_FILE" | tail -n 1
} | tee "$SUMMARY_FILE"

echo "[bench] evidence: $SUMMARY_FILE"
