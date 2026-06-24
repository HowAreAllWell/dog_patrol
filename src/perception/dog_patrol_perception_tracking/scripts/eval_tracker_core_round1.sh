#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/path/to/vision_demo_ws}"
BIN="$WS_DIR/build/vision_demo_host/offline_eval_recordings"

if [[ ! -x "$BIN" ]]; then
  echo "[eval] missing binary: $BIN"
  exit 1
fi

run_case() {
  local name="$1"
  local cfg="$2"
  echo "[eval] running $name with $cfg"
  "$BIN" \
    --run-name "$name" \
    --tracker-config "$cfg" \
    --save-eval-video true \
    --short-dataset-dir-names true
}

run_case "t1_old_final" "$WS_DIR/src/vision_demo_host/config/legacy/tracker_old_minimal.yaml"
run_case "t2_core_final" "$WS_DIR/src/vision_demo_host/config/legacy/tracker_new_core_no_app.yaml"
run_case "t3_app_final" "$WS_DIR/src/vision_demo_host/config/legacy/tracker_new_core_with_app.yaml"

echo "[eval] done"
