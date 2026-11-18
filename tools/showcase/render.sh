#!/bin/bash
# Re-render the website reel and stills from any trained policy.
#   tools/showcase/render.sh <policy.onnx> [out_dir] [--preview]
# 1. records a rollout of the policy in the training scene (headless, inference, no trainer),
# 2. regenerates the showcase assets from code,
# 3. replays the rollout offline: 4K 60 fps H.264 with accumulated motion blur, plus stills.
# --preview renders at quarter resolution with few sub-frames, for checking framing quickly.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
POLICY="${1:?usage: render.sh <policy.onnx> [out_dir] [--preview]}"
NAME="$(basename "${POLICY%.*}")"
OUT="${2:-renders/$NAME}"
PREVIEW=()
[[ " $* " == *" --preview "* ]] && PREVIEW=(-scale 0.25 -subframes 4)
SECONDS_TO_RECORD="${SECONDS_TO_RECORD:-90}"

# Policies outside Assets/ are copied in so Unity imports them.
if [[ "$POLICY" != Assets/* ]]; then
  mkdir -p Assets/Showcase/Policies
  cp "$POLICY" "Assets/Showcase/Policies/$NAME.onnx"
  POLICY="Assets/Showcase/Policies/$NAME.onnx"
fi
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
TAKE="$OUT/$NAME.take"

echo "== record $POLICY -> $TAKE"
tools/unity.sh -executeMethod Showcase.Editor.RecordCommand.Run \
  -showcaseTake "$TAKE" -showcaseSeconds "$SECONDS_TO_RECORD" -showcaseModel "$POLICY" -showcasePolicy "$NAME" \
  -logFile "$OUT/record.log"
grep "\[showcase\] saved" "$OUT/record.log"

echo "== build showcase assets"
tools/unity.sh --graphics -quit -executeMethod Showcase.Editor.ShowcaseBuilder.BuildFromCommandLine -logFile "$OUT/build.log"

echo "== render reel"
tools/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
  -take "$TAKE" -reel Assets/Showcase/Reels/website.json -out "$OUT/$NAME.mp4" -stills "$OUT/stills" \
  "${PREVIEW[@]}" -logFile "$OUT/render.log"

if [[ ${#PREVIEW[@]} -eq 0 ]]; then
  echo "== hero still"
  tools/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderStillFromCommandLine \
    -take "$TAKE" -reel Assets/Showcase/Reels/website.json -shot hero -u 0.55 -width 5120 -height 2880 -subframes 128 \
    -out "$OUT/stills/hero_5k.png" -logFile "$OUT/still.log"
fi
echo "== done: $OUT"
ls -la "$OUT" "$OUT/stills"
