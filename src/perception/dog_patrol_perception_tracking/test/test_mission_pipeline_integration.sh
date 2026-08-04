#!/usr/bin/env bash
set -euo pipefail

driver=${1:?mission integration driver path is required}
shift
test_tmp=$(mktemp -d)
supervisor_pid=""
driver_timeout=35s
if [[ $# -gt 0 ]]; then
  driver_timeout=90s
fi

cleanup() {
  if [[ -n "${supervisor_pid}" ]]; then
    kill -INT -- "-${supervisor_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${supervisor_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -TERM -- "-${supervisor_pid}" 2>/dev/null || true
    wait "${supervisor_pid}" 2>/dev/null || true
  fi
  rm -rf "${test_tmp}"
}
trap cleanup EXIT

ros2 pkg prefix dog_patrol_manager >/dev/null

setsid ros2 run dog_patrol_manager mission_supervisor --ros-args \
  -p state_topic:=/dog_patrol/integration/mission/state \
  -p event_topic:=/dog_patrol/integration/mission/event \
  -p state_publish_rate:=20.0 \
  -p initial_state_seq:=900 \
  >"${test_tmp}/mission_supervisor.log" 2>&1 &
supervisor_pid=$!

set +e
timeout --signal=INT "${driver_timeout}" "${driver}" "$@"
driver_status=$?
set -e

if [[ ${driver_status} -ne 0 ]]; then
  sed -n '1,240p' "${test_tmp}/mission_supervisor.log" >&2
fi
exit "${driver_status}"
