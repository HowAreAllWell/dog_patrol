#!/usr/bin/env bash
set -euo pipefail

smoke_executable="$1"
subscriber_log="$(mktemp)"
trap 'rm -f "$subscriber_log"' EXIT

"$smoke_executable" subscribe >"$subscriber_log" 2>&1 &
subscriber_pid=$!
sleep 1
"$smoke_executable" publish
if ! wait "$subscriber_pid"; then
  sed -n '1,160p' "$subscriber_log" >&2
  exit 1
fi
