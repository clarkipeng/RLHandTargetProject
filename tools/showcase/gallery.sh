#!/bin/bash
# Publish a render.sh output to the lab wall: ~/.agentworld/gallery/rl-arm/<look>-*.
#   tools/showcase/gallery.sh renders/<policy>
# Per look: the stills as high-quality JPEGs, a contact sheet of exactly the stills there are
# (no empty tiles), and a 1080p copy of the reel. Replaces that look's previous files.
set -euo pipefail
OUT="${1:?usage: gallery.sh <render_dir>}"
GALLERY="${GALLERY:-$HOME/.agentworld/gallery/rl-arm}"
mkdir -p "$GALLERY"
for DIR in "$OUT"/*/; do
  LOOK="$(basename "$DIR")"
  STILLS=( "$DIR"stills/*.png )
  [[ -e "${STILLS[0]}" ]] || continue
  rm -f "$GALLERY/$LOOK"-*
  for f in "${STILLS[@]}"; do
    name="$(basename "${f%.png}")"
    ffmpeg -loglevel error -threads 4 -y -i "$f" -vf "scale='min(2560,iw)':-2" -q:v 2 "$GALLERY/$LOOK-$name.jpg"
  done

  # Contact sheet: 3 columns, as many rows as needed, tiles only for stills that exist.
  n=${#STILLS[@]}
  cols=$(( n < 3 ? n : 3 ))
  rows=$(( (n + cols - 1) / cols ))
  inputs=(); filter=""; layout=""
  for i in "${!STILLS[@]}"; do
    inputs+=( -i "${STILLS[$i]}" )
    filter+="[$i:v]scale=960:540,setsar=1[t$i];"
    layout+="$(( (i % cols) * 960 ))_$(( (i / cols) * 540 ))|"
  done
  labels="$(for i in "${!STILLS[@]}"; do printf '[t%d]' "$i"; done)"
  ffmpeg -loglevel error -threads 4 -y "${inputs[@]}" \
    -filter_complex "${filter}${labels}xstack=inputs=$n:layout=${layout%|}:fill=0x0d1117" \
    -frames:v 1 -q:v 2 "$GALLERY/$LOOK-sheet.jpg"
  echo "$LOOK: $n stills in a ${cols}x${rows} sheet"

  for clip in "$DIR"*.mp4; do
    [[ -e "$clip" ]] || continue
    ffmpeg -loglevel error -threads 4 -y -i "$clip" -vf scale=1920:-2 -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p \
      -movflags +faststart -an "$GALLERY/$LOOK-reel.mp4"
  done
done
python3 "$HOME/.agentworld/gallery/make-index.py"
