#!/bin/bash
# Run the Windows Unity 2022.3.62f3 in batchmode on the Windows mirror of this project (see sync.sh),
# under the one-Unity-editor-at-a-time rule, at below-normal priority, with no window and no audio.
#   tools/win/unity.sh [--graphics] [unity args...]
# Paths after -logFile/-take/-out/... may be WSL paths; arguments that start with / are converted to Windows paths.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
UNITY="${UNITY:-/mnt/c/Program Files/Unity/Hub/Editor/2022.3.62f3/Editor/Unity.exe}"
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
LOCK=/tmp/unity.lock
WIN_PROJECT="$("$ROOT/tools/win/sync.sh")"

gfx=(-nographics)
args=()
for a in "$@"; do
  if [[ "$a" == --graphics ]]; then gfx=()
  elif [[ "$a" == /* ]]; then mkdir -p "$(dirname "$a")"; args+=("$(wslpath -w "$a")")
  else args+=("$a"); fi
done

editor_running() { "$PS" -NoProfile -Command 'if (Get-Process Unity -ErrorAction SilentlyContinue) { exit 0 } else { exit 1 }' >/dev/null 2>&1; }
exec 9>"$LOCK"
waited=0
until flock -n 9 && ! editor_running; do
  flock -u 9 2>/dev/null || true
  (( waited % 60 == 0 )) && echo "unity.sh: waiting for the Unity slot" >&2
  sleep 10; waited=$((waited + 10))
done

WSL_FFMPEG="$(ls "$ROOT"/ml/.venv/lib/python3.10/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-* 2>/dev/null | head -1)"
[[ -n "$WSL_FFMPEG" ]] && args+=(-wslFfmpeg "$WSL_FFMPEG")
"$UNITY" -batchmode ${gfx[@]+"${gfx[@]}"} -disable-audio -projectPath "$WIN_PROJECT" ${args[@]+"${args[@]}"} &
pid=$!
sleep 3
"$PS" -NoProfile -Command 'Get-Process Unity -ErrorAction SilentlyContinue | % { $_.PriorityClass = "BelowNormal" }' >/dev/null 2>&1 || true
wait $pid
