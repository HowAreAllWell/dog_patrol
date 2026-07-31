#!/usr/bin/env bash
set -euo pipefail

VISION_NODE="$1"
OFFLINE_EVAL="$2"

expect_failure() {
  local expected_code="$1"
  local expected_message="$2"
  shift 2

  set +e
  local output
  output="$("$@" 2>&1)"
  local actual_code=$?
  set -e

  if [[ $actual_code -ne $expected_code || "$output" != *"$expected_message"* ]]; then
    printf 'expected exit %s and message %q, got exit %s:\n%s\n' \
      "$expected_code" "$expected_message" "$actual_code" "$output" >&2
    exit 1
  fi
}

for retired_name in camera.backend camera.rtsp_url camera.gstreamer_pipeline; do
  expect_failure 1 "Retired camera parameter override '${retired_name}'" \
    "$VISION_NODE" --ros-args -p "${retired_name}:=retired"
done

expect_failure 2 "Choose an FFV1 capture with --video or --datasets" "$OFFLINE_EVAL"
expect_failure 2 "Unknown argument: --rtsp-url" "$OFFLINE_EVAL" --rtsp-url retired
