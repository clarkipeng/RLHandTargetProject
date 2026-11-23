#!/bin/bash
# Old vs new throwing at the same far held-out target, side by side, slow through the wind-up (Reels/windup.json).
#   tools/train/sidebyside.sh <out.mp4> "<left label>" <left.onnx> [old|new] "<right label>" <right.onnx> [old|new] [--target 20]
# old|new picks the arm (eval.py --limits). Target 20 is 52 out at azimuth -10. Both reels start 1 s of take time before
# the release of the first hit, so the two releases line up.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(realpath -m "$1")"; LL="$2"; LO="$(realpath "$3")"; LA="$4"; RL="$5"; RO="$(realpath "$6")"; RA="$7"; shift 7
TARGET=20; [[ "${1:-}" == --target ]] && TARGET="$2"
WORK="${OUT%.mp4}"; mkdir -p "$WORK"
for side in left right; do
  if [[ $side == left ]]; then onnx="$LO"; arm="$LA"; else onnx="$RO"; arm="$RA"; fi
  take="$WORK/$side.take"
  [[ -f "$take" ]] || (cd "$ROOT/ml" && uv run python eval.py "$onnx" --take "$take" --seconds 24 --limits "$arm" \
      --set eval_target="$TARGET" --port 6500 2>/dev/null | tail -1)
  [[ -f "$WORK/$side.mp4" ]] || (cd "$ROOT" && tools/win/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
      -take "$take" -look minimal -reel Assets/Showcase/Reels/windup.json -out "$WORK/$side.mp4" -stills "$WORK/$side-stills" \
      -logFile "$WORK/$side.render.log" >/dev/null)
done
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
label() { printf "drawtext=fontfile=%s:text='%s':x=32:y=28:fontsize=34:fontcolor=white:box=1:boxcolor=black@0.45:boxborderw=12" "$FONT" "$1"; }
ffmpeg -v error -y -i "$WORK/left.mp4" -i "$WORK/right.mp4" -filter_complex \
  "[0:v]tpad=stop_mode=clone:stop_duration=3,$(label "$LL")[a];[1:v]tpad=stop_mode=clone:stop_duration=3,$(label "$RL")[b];[a][b]hstack=shortest=1,drawtext=fontfile=$FONT:text='slow motion through the wind-up and release':x=(w-tw)/2:y=h-56:fontsize=28:fontcolor=white@0.8[v]" \
  -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart "$OUT"
ls -la "$OUT"
