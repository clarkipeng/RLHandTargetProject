#!/bin/bash
# Evaluate new checkpoints of a run on the held-out targets, redraw the hit-rate chart on the rl-arm wall,
# and at milestones (first checkpoint, every 10M steps, a new best held-out hit rate) render a short eval clip.
#   tools/train/publish.sh <run_id> [--clip-every 10]
# Safe to run repeatedly (e.g. from watch.sh): every checkpoint is evaluated once.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RUN="${1:?usage: publish.sh <run_id>}"; shift
CLIP_EVERY=10
[[ "${1:-}" == --clip-every ]] && CLIP_EVERY="$2"
RES="$ROOT/ml/results/$RUN"
WALL="$HOME/.agentworld/gallery/rl-arm"
CLIPS="$ROOT/renders/train/$RUN"
mkdir -p "$WALL" "$CLIPS"
cd "$ROOT/ml"

best() { [[ -f "$RES/eval.jsonl" ]] && python3 -c "import json,sys;print(max([json.loads(l)['hit_rate'] for l in open(sys.argv[1])] or [-1]))" "$RES/eval.jsonl" || echo -1; }
for onnx in $(ls "$RES"/Throw/Throw-*.onnx 2>/dev/null | sort -t- -k2 -n); do
  name="$(basename "$onnx")"
  grep -q "\"$name\"" "$RES/eval.jsonl" 2>/dev/null && continue
  before="$(best)"
  row="$(uv run python eval.py "$onnx" --episodes 48 --arenas 8 --port 6400 2>/dev/null | tail -1)"
  echo "eval $row"
  step="$(sed -E 's/.*-([0-9]+)\.onnx/\1/' <<<"$name")"
  rate="$(python3 -c "import json,sys;print(json.loads(sys.argv[1])['hit_rate'])" "$row")"
  m=$(( step / 1000000 ))
  first=$([[ -z "$(ls "$CLIPS"/*.mp4 2>/dev/null)" ]] && echo 1 || echo 0)
  if [[ $first == 1 || $(( m % CLIP_EVERY )) == 0 && ! -f "$CLIPS/eval-${m}M.mp4" ]] || python3 -c "import sys;sys.exit(0 if float(sys.argv[1])>float(sys.argv[2]) and float(sys.argv[1])>0 else 1)" "$rate" "$before"; then
    lesson="$(uv run python stats.py "$RES" "Environment/Lesson Number/lesson" 2>/dev/null | awk 'NR==2{printf "%d", $NF}')"
    "$ROOT/tools/train/clip.sh" "$onnx" "$CLIPS/eval-${m}M.mp4" --lesson "${lesson:-5}" && cp "$CLIPS/eval-${m}M.mp4" "$WALL/train-eval-$(printf %04d $m)M.mp4"
  fi
done
uv run python chart.py "$RES" "$WALL/train-hit-rate.png" >/dev/null
[[ -f "$HOME/.agentworld/gallery/make-index.py" ]] && python3 "$HOME/.agentworld/gallery/make-index.py" >/dev/null || true
echo "published $RUN: $(tail -1 "$RES/eval.jsonl" 2>/dev/null)"
