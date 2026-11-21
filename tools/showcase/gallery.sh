#!/bin/bash
# Publish a render.sh output to the lab wall: ~/.agentworld/gallery/rl-arm/<look>-*.
#   tools/showcase/gallery.sh renders/<policy>
# Per look and reel: the stills as high-quality JPEGs, a contact sheet of exactly the stills there are
# (no empty tiles), and a 1080p copy of the clip. The website reel (stills/ and <policy>.mp4) publishes as
# <look>-NN_shot.jpg, <look>-sheet.jpg and <look>-reel.mp4; any other reel R (R/ and R.mp4) as <look>-R-*.
# Replaces only that reel's previous files.
set -euo pipefail
OUT="${1:?usage: gallery.sh <render_dir>}"
GALLERY="${GALLERY:-$HOME/.agentworld/gallery/rl-arm}"
mkdir -p "$GALLERY"
publish() { # publish <prefix> <stills_dir> <clip> <wall_clip_name>
  local PREFIX="$1" SDIR="$2" CLIP="$3" OUTCLIP="$GALLERY/$4"
  local STILLS=( "$SDIR"/*.png )
  [[ -e "${STILLS[0]}" ]] || return 0
  rm -f "$GALLERY/$PREFIX"-[0-9]*.jpg "$GALLERY/$PREFIX-sheet.jpg"
  for f in "${STILLS[@]}"; do
    name="$(basename "${f%.png}")"
    ffmpeg -loglevel error -threads 4 -y -i "$f" -vf "scale='min(2560,iw)':-2" -q:v 2 "$GALLERY/$PREFIX-$name.jpg"
  done

  # Contact sheet: 3 columns, as many rows as needed, tiles only for stills that exist.
  local n=${#STILLS[@]}
  local cols=$(( n < 3 ? n : 3 ))
  local rows=$(( (n + cols - 1) / cols ))
  local inputs=() filter="" layout="" labels=""
  for i in "${!STILLS[@]}"; do
    inputs+=( -i "${STILLS[$i]}" )
    filter+="[$i:v]scale=960:540,setsar=1[t$i];"
    layout+="$(( (i % cols) * 960 ))_$(( (i / cols) * 540 ))|"
    labels+="[t$i]"
  done
  ffmpeg -loglevel error -threads 4 -y "${inputs[@]}" \
    -filter_complex "${filter}${labels}xstack=inputs=$n:layout=${layout%|}:fill=0x0d1117" \
    -frames:v 1 -q:v 2 "$GALLERY/$PREFIX-sheet.jpg"
  echo "$PREFIX: $n stills in a ${cols}x${rows} sheet"

  [[ -e "$CLIP" ]] && ffmpeg -loglevel error -threads 4 -y -i "$CLIP" -vf scale=1920:-2 -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p \
    -movflags +faststart -an "$OUTCLIP"
  return 0
}

for DIR in "$OUT"/*/; do
  LOOK="$(basename "$DIR")"
  DIR="${DIR%/}"
  WEB=( "$DIR"/Throw*.mp4 )
  publish "$LOOK" "$DIR/stills" "${WEB[0]}" "$LOOK-reel.mp4"
  for CLIP in "$DIR"/*.mp4; do
    REEL="$(basename "${CLIP%.mp4}")"
    [[ "$REEL" == Throw* || ! -d "$DIR/$REEL" ]] && continue
    publish "$LOOK-$REEL" "$DIR/$REEL" "$CLIP" "$LOOK-$REEL.mp4"
  done
done
INDEX="$HOME/.agentworld/gallery/make-index.py"; [[ -f "$INDEX" ]] && python3 "$INDEX"; exit 0
