#!/bin/bash
# Render the website's RL arm hover clip from scratch, resumably:
#   tools/showcase/site-clips.sh [hit strength, default 0.5] [--preview]
# 1. records each segment's take (seeded rollout of its policy) unless TAKES already holds it,
# 2. replays each segment through its reel (Assets/Showcase/Reels), every one on the final take's stage,
# 3. crossfades the segments of each clip and encodes it for the site (1200x800, H.264, BT.709) into OUT.
# Every step skips work whose output exists, so a rerun after a crash or a new hit strength redoes only what changed.
# --preview renders at quarter resolution with few sub-frames into OUT/preview, to check framing quickly.
# OUT defaults to renders/site and TAKES to OUT/takes (TAKES=renders/l1 reuses takes recorded there).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
HIT=0.5
PREVIEW=()
for a in "$@"; do
  case "$a" in
    --preview) PREVIEW=(-scale 0.25 -subframes 4) ;;
    *) HIT="$a" ;;
  esac
done
OUT="$(mkdir -p "${OUT:-renders/site}" && cd "${OUT:-renders/site}" && pwd)"
TAKES="$(mkdir -p "${TAKES:-$OUT/takes}" && cd "${TAKES:-$OUT/takes}" && pwd)"
[[ ${#PREVIEW[@]} -gt 0 ]] && OUT="$OUT/preview"
mkdir -p "$OUT/logs"
UNITY=tools/unity.sh
[[ -x tools/win/unity.sh && -d /mnt/c ]] && UNITY=tools/win/unity.sh
SEED=1
FADE=0.25

# Segment: reel name, policy (Assets/Trained Models/<policy>.onnx), take length in seconds.
# rl-01..03 come from takes on the gaming PC: copy those takes into TAKES and their reels into Assets/Showcase/Reels,
# then add a segment line for each and a clip line below.
SEGMENTS="
l1-early Drecon-501180 450
l1-mid Test11_throw 900
l1-final Target18_2 1800
"
# Clip: site file name, then its segments in order, crossfaded.
CLIPS="
rl-learn l1-early l1-mid l1-final
"
STAGE="$TAKES/Target18_2-s$SEED.take"   # one stage for every segment, so the cuts match

while read -r reel policy seconds; do
  [[ -z "$reel" ]] && continue
  take="$TAKES/$policy-s$SEED.take"
  if [[ ! -f "$take" ]]; then
    echo "== record $policy ($seconds s, seed $SEED)"
    $UNITY -executeMethod Showcase.Editor.RecordCommand.Run -showcaseTake "$take" -showcaseSeconds "$seconds" \
      -showcaseModel "Assets/Trained Models/$policy.onnx" -showcasePolicy "$policy" -showcaseSeed "$SEED" \
      -logFile "$OUT/logs/record-$policy.log" </dev/null
    grep "\[showcase\] saved" "$OUT/logs/record-$policy.log"
  fi
  clip="$OUT/$reel-hit$HIT.mp4"
  if [[ ! -s "$clip" ]]; then
    echo "== render $reel (hit $HIT)"
    $UNITY --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine \
      -take "$take" -stageTake "$STAGE" -look Art -hit "$HIT" -reel Assets/Showcase/Reels/$reel.json \
      -out "$clip.part.mp4" -stills "$OUT/stills/$reel-hit$HIT" ${PREVIEW[@]+"${PREVIEW[@]}"} -logFile "$OUT/logs/render-$reel-hit$HIT.log" </dev/null
    mv "$clip.part.mp4" "$clip"
  fi
done <<< "$SEGMENTS"

while read -r name segs; do
  [[ -z "$name" ]] && continue
  inputs=(); graph=""; prev="[0:v]"; offset=0; i=0
  for seg in $segs; do
    f="$OUT/$seg-hit$HIT.mp4"; inputs+=(-i "$f")
    if (( i > 0 )); then
      graph+="$prev[$i:v]xfade=transition=fade:duration=$FADE:offset=$offset[x$i];"; prev="[x$i]"
    fi
    offset=$(echo "$offset + $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$f") - $FADE" | bc)
    i=$((i + 1))
  done
  echo "== encode $name"
  ffmpeg -nostdin -v error -y "${inputs[@]}" -filter_complex "${graph}${prev}scale=1200:800:flags=lanczos,fps=60,format=yuv420p,setparams=color_primaries=bt709:color_trc=bt709" \
    -an -c:v libx264 -profile:v high -preset slow -crf 23 -maxrate 2M -bufsize 4M \
    -colorspace bt709 -color_primaries bt709 -color_trc bt709 -color_range tv -movflags +faststart "$OUT/$name-hit$HIT.mp4"
  echo "$OUT/$name-hit$HIT.mp4"
done <<< "$CLIPS"
