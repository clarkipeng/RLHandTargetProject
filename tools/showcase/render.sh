#!/bin/bash
# Re-render the website reel and stills from any trained policy, in every showcase look.
#   tools/showcase/render.sh <policy.onnx> [out_dir] [--preview] [--looks "minimal art"]
# 1. records a rollout of the policy in the training scene (headless, inference, no trainer),
#    unless out_dir already holds its take (delete the .take to re-record),
# 2. regenerates the showcase assets and scenes from code,
# 3. per look, replays the rollout offline: 4K 60 fps H.264 with accumulated motion blur, stills, a 5K hero still.
# --preview renders at quarter resolution with few sub-frames, for checking framing quickly.
# Publish to the lab wall afterwards with tools/showcase/gallery.sh <out_dir>.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
POLICY="${1:?usage: render.sh <policy.onnx> [out_dir] [--preview] [--looks \"minimal art\"]}"
shift
NAME="$(basename "${POLICY%.*}")"
OUT="renders/$NAME"
if [[ $# -gt 0 && "$1" != --* ]]; then OUT="$1"; shift; fi
PREVIEW=()
LOOKS="minimal art"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --preview) PREVIEW=(-scale 0.25 -subframes 4) ;;
    --looks) LOOKS="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
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

if [[ ! -f "$TAKE" ]]; then
  echo "== record $POLICY -> $TAKE"
  tools/unity.sh -executeMethod Showcase.Editor.RecordCommand.Run \
    -showcaseTake "$TAKE" -showcaseSeconds "$SECONDS_TO_RECORD" -showcaseModel "$POLICY" -showcasePolicy "$NAME" \
    -logFile "$OUT/record.log"
  grep "\[showcase\] saved" "$OUT/record.log"
fi

echo "== build showcase assets"
tools/unity.sh --graphics -quit -executeMethod Showcase.Editor.ShowcaseBuilder.BuildFromCommandLine -logFile "$OUT/build.log"

for LOOK in $LOOKS; do
  DIR="$OUT/$LOOK"
  mkdir -p "$DIR"
  echo "== render $LOOK reel"
  tools/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
    -take "$TAKE" -look "$LOOK" -reel Assets/Showcase/Reels/website.json -out "$DIR/$NAME.mp4" -stills "$DIR/stills" \
    ${PREVIEW[@]+"${PREVIEW[@]}"} -logFile "$DIR/render.log"

  if [[ ${#PREVIEW[@]} -eq 0 ]]; then
    echo "== $LOOK hero still"
    tools/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderStillFromCommandLine \
      -take "$TAKE" -look "$LOOK" -reel Assets/Showcase/Reels/website.json -shot hero -u 0.55 -width 5120 -height 2880 -subframes 128 \
      -out "$DIR/stills/hero_5k.png" -logFile "$DIR/still.log"
  fi
done
echo "== done: $OUT"
ls -la "$OUT"/*/ "$OUT"/*/stills
