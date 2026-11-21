#!/bin/bash
# Re-render the website reel and stills from any trained policy, in every showcase look.
#   tools/showcase/render.sh <policy.onnx> [out_dir] [--preview] [--looks "minimal art"]
# 1. records a rollout of the policy in the training scene (headless, inference, no trainer),
#    unless out_dir already holds its take (delete the .take to re-record),
# 2. regenerates the showcase assets and scenes from code,
# 3. per look, replays the rollout offline: 4K 60 fps H.264 with accumulated motion blur, stills, a 5K hero still.
# REELS="website cinematic montage" picks the reels (default: website).
# --preview renders at quarter resolution with few sub-frames, for checking framing quickly.
# The live view (tools/showcase/serve.sh) switches to this policy as soon as its take is exported.
# Publish stills and clips to the lab wall afterwards with tools/showcase/gallery.sh <out_dir>.
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
REELS="${REELS:-website}" # reels in Assets/Showcase/Reels to render, e.g. REELS="website cinematic montage"
# On the Windows PC (WSL), Unity runs through tools/win/unity.sh (Windows editor, path conversion, lock).
UNITY=tools/unity.sh
[[ -x tools/win/unity.sh && -d /mnt/c ]] && UNITY=tools/win/unity.sh

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
  $UNITY -executeMethod Showcase.Editor.RecordCommand.Run \
    -showcaseTake "$TAKE" -showcaseSeconds "$SECONDS_TO_RECORD" -showcaseModel "$POLICY" -showcasePolicy "$NAME" \
    -logFile "$OUT/record.log"
  grep "\[showcase\] saved" "$OUT/record.log"
fi

echo "== build showcase assets"
grep -qE "^  trail: \{fileID: [1-9]" Assets/Showcase/Looks/MinimalLook.asset ||  # already built (they are kept in git)
{ $UNITY --graphics -quit -executeMethod Showcase.Editor.ShowcaseBuilder.BuildFromCommandLine -logFile "$OUT/build.log"
  # On the PC the build lands in the Windows mirror, which the next sync would overwrite: bring it back.
  if [[ "$UNITY" == tools/win/unity.sh ]]; then
    rsync -a --exclude '*.cs' --exclude '*.cs.meta' /mnt/c/agentworld/rlhand/Assets/Showcase/ Assets/Showcase/
  fi; }

echo "== export for the live view and publish it on the wall"
$UNITY -executeMethod Showcase.Editor.WebExport.ExportFromCommandLine -take "$TAKE" -out "$OUT/$NAME.web.json" -logFile "$OUT/export.log"
tools/showcase/serve.sh "$OUT/$NAME.web.json"

for LOOK in $LOOKS; do
  DIR="$OUT/$LOOK"
  mkdir -p "$DIR"
  for REEL in $REELS; do
    # The website reel keeps its old names; other reels are named after themselves.
    if [[ "$REEL" == website ]]; then CLIP="$DIR/$NAME.mp4"; STILLS="$DIR/stills"; LOG="$DIR/render.log"
    else CLIP="$DIR/$REEL.mp4"; STILLS="$DIR/$REEL"; LOG="$DIR/$REEL.log"; fi
    echo "== render $LOOK $REEL reel"
    $UNITY --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
      -take "$TAKE" -look "$LOOK" -reel "Assets/Showcase/Reels/$REEL.json" -out "$CLIP" -stills "$STILLS" \
      ${PREVIEW[@]+"${PREVIEW[@]}"} -logFile "$LOG"
  done

  if [[ ${#PREVIEW[@]} -eq 0 && " $REELS " == *" website "* ]]; then
    echo "== $LOOK hero still"
    $UNITY --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderStillFromCommandLine \
      -take "$TAKE" -look "$LOOK" -reel Assets/Showcase/Reels/website.json -shot hero -u 0.55 -width 5120 -height 2880 -subframes 128 \
      -out "$DIR/stills/hero_5k.png" -logFile "$DIR/still.log"
  fi
done
echo "== done: $OUT"
ls -la "$OUT"/*/
