#!/bin/bash
# A short eval clip of one checkpoint: record a rollout on the held-out targets, replay it in the minimal look.
#   tools/train/clip.sh <policy.onnx> <out.mp4> [--look minimal] [--seconds 40]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ONNX="$(realpath "$1")"; OUT="$(realpath -m "$2")"; shift 2
LOOK=minimal; SECONDS_REC=40
while [[ $# -gt 0 ]]; do case "$1" in --look) LOOK="$2"; shift ;; --seconds) SECONDS_REC="$2"; shift ;; esac; shift; done
TAKE="${OUT%.mp4}.take"
mkdir -p "$(dirname "$OUT")"
cd "$ROOT/ml"
[[ -f "$TAKE" ]] || uv run python eval.py "$ONNX" --take "$TAKE" --seconds "$SECONDS_REC" --port 6500 2>/dev/null | tail -1
cd "$ROOT"
# The looks are generated assets kept in git: build them once with the pedestal material, then bring them back
# from the Windows mirror so the next sync does not undo them.
if ! grep -qE "^  prop: \{fileID: [1-9]" Assets/Showcase/Looks/MinimalLook.asset; then
  tools/win/unity.sh --graphics -quit -executeMethod Showcase.Editor.ShowcaseBuilder.BuildFromCommandLine -logFile "$ROOT/renders/logs/showcase-build.log"
  rsync -a --exclude '*.cs' --exclude '*.cs.meta' /mnt/c/agentworld/rlhand/Assets/Showcase/ Assets/Showcase/
fi
tools/win/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
  -take "$TAKE" -look "$LOOK" -reel Assets/Showcase/Reels/eval.json -out "$OUT" -logFile "${OUT%.mp4}.render.log"
ls -la "$OUT"
