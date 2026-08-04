#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
BIN="$WS_DIR/build/dog_patrol_perception_tracking/offline_eval_recordings"
INPUT_VIDEO="${1:-}"

if [[ ! -x "$BIN" ]]; then
  echo "[eval] missing binary: $BIN"
  exit 1
fi
if [[ -z "$INPUT_VIDEO" ]]; then
  echo "Usage: $0 <ffv1-take-video.mkv>" >&2
  exit 2
fi

run_case() {
  local name="$1"
  local cfg="$2"
  echo "[eval] running $name with $cfg"
  "$BIN" \
    --video "$INPUT_VIDEO" \
    --run-name "$name" \
    --tracker-config "$cfg" \
    --overlay-record true \
    --short-dataset-dir-names true
}

run_case "t1_old_final" "$WS_DIR/src/dog_patrol_perception_tracking/config/legacy/tracker_old_minimal.yaml"
run_case "t2_core_final" "$WS_DIR/src/dog_patrol_perception_tracking/config/legacy/tracker_new_core_no_app.yaml"
run_case "t3_app_final" "$WS_DIR/src/dog_patrol_perception_tracking/config/legacy/tracker_new_core_with_app.yaml"

echo "[eval] done"
