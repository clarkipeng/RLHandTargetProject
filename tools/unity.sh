#!/bin/bash
# Run Unity 2022.3 in batchmode under the machine-wide one-Unity-at-a-time rule.
# Waits until no Unity editor is running (any project), takes /tmp/unity.lock, runs at nice 10.
# Usage: tools/unity.sh [unity args...]   (-batchmode -nographics -projectPath are added for you;
#        pass --graphics to drop -nographics, e.g. for captures that need a render target)
set -euo pipefail
UNITY="${UNITY:-/Applications/Unity/Hub/Editor/2022.3.62f3/Unity.app/Contents/MacOS/Unity}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOCK=/tmp/unity.lock
EDITOR_PATTERN='Unity\.app/Contents/MacOS/Unity( |$)'

gfx=(-nographics)
args=()
for a in "$@"; do
  if [[ "$a" == --graphics ]]; then gfx=(); else args+=("$a"); fi
done

waited=0
while true; do
  if ! pgrep -f "$EDITOR_PATTERN" >/dev/null && shlock -f "$LOCK" -p $$; then
    if ! pgrep -f "$EDITOR_PATTERN" >/dev/null; then break; fi
    rm -f "$LOCK"
  fi
  if (( waited % 60 == 0 )); then
    echo "unity.sh: waiting for the Unity slot ($(pgrep -f "$EDITOR_PATTERN" | head -1 || cat "$LOCK" 2>/dev/null))" >&2
  fi
  sleep 10; waited=$((waited + 10))
done
trap 'rm -f "$LOCK"' EXIT

nice -n 10 "$UNITY" -batchmode ${gfx[@]+"${gfx[@]}"} -disable-audio -projectPath "$ROOT" ${args[@]+"${args[@]}"}
