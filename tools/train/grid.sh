#!/bin/bash
# The same far throw by several policies, each in its own locked-camera panel, slow through the wind-up, in a grid.
#   tools/train/grid.sh <out.mp4> <target> "<label>|<policy.onnx>|<eval.py --set flags>" ...
# Each policy is recorded on the same held-out target (eval.py --set eval_target=<target>) and rendered with
# Assets/Showcase/Reels/windup2.json, which anchors on the first hit's release, so the panels' releases line up.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(realpath -m "$1")"; TARGET="$2"; shift 2
WORK="${OUT%.mp4}"; mkdir -p "$WORK"
FF="${FFMPEG:-ffmpeg}"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
inputs=(); filters=(); k=0
for spec in "$@"; do
  IFS='|' read -r label onnx sets <<< "$spec"
  take="$WORK/$k.take"
  # shellcheck disable=SC2086
  [[ -f "$take" ]] || (cd "$ROOT/ml" && uv run python eval.py "$(realpath "$ROOT/$onnx")" --take "$take" --seconds 40 \
      --set eval_target="$TARGET" $sets --port 6500 2>/dev/null | tail -1)
  [[ -f "$WORK/$k.mp4" ]] || (cd "$ROOT" && nice -n 10 tools/win/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
      -take "$take" -look "${LOOK:-art}" -reel Assets/Showcase/Reels/windup2.json -out "$WORK/$k.mp4" -stills "$WORK/$k-stills" \
      -logFile "$WORK/$k.render.log" >/dev/null)
  printf '%s' "$label" > "$WORK/$k.txt"   # a text file: labels may hold ':' and quotes
  inputs+=(-i "$WORK/$k.mp4")
  filters+=("[$k:v]scale=960:540,tpad=stop_mode=clone:stop_duration=4,drawtext=fontfile=$FONT:textfile=$WORK/$k.txt:expansion=none:x=24:y=20:fontsize=30:fontcolor=white:box=1:boxcolor=black@0.45:boxborderw=10[p$k]")
  k=$((k + 1))
done
# A grid as close to square as fits: 2 -> 2x1, 3-4 -> 2x2, 5-6 -> 3x2; empty cells are black.
cols=1; while (( cols * cols < k )); do cols=$((cols + 1)); done
rows=$(( (k + cols - 1) / cols )); cells=$((cols * rows))
pads=""; for ((i = k; i < cells; i++)); do pads+="color=black:s=960x540:r=60,format=yuv420p[p$i];"; done
layout=""; for ((i = 0; i < cells; i++)); do layout+="$((i % cols * 960))_$((i / cols * 540))|"; done
stack="$pads$(for ((i = 0; i < cells; i++)); do printf "[p%d]" "$i"; done)xstack=inputs=$cells:layout=${layout%|}:shortest=1:fill=black"
"$FF" -v error -y "${inputs[@]}" -filter_complex "$(IFS=';'; echo "${filters[*]}");$stack[v]" \
  -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart "$OUT"
ls -la "$OUT"
