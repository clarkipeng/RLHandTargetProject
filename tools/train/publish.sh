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
# Runs initialised from another run's checkpoint restart their step counter: results/<run>/offset holds the
# millions of steps trained before, so wall clips are named by overall steps.
OFFSET="$(cat "$RES/offset" 2>/dev/null || echo 0)"
# Wall file names: the throw runs of PR #6 keep "train-"; other runs are named by their run family (windup01 -> windup).
PREFIX="$(sed -E 's/[0-9]+r?$//' <<<"$RUN")"; [[ "$PREFIX" == throw ]] && PREFIX=train
cd "$ROOT/ml"

best() { [[ -f "$RES/eval.jsonl" ]] && python3 -c "import json,sys;print(max([json.loads(l)['hit_rate'] for l in open(sys.argv[1])] or [-1]))" "$RES/eval.jsonl" || echo -1; }
for onnx in $(ls "$RES"/Throw/Throw-*.onnx 2>/dev/null | sort -V); do
  name="$(basename "$onnx")"
  grep -q "\"$name\"" "$RES/eval.jsonl" 2>/dev/null && continue
  before="$(best)"
  row="$(uv run python eval.py "$onnx" --episodes 48 --arenas 8 --port 6400 2>/dev/null | tail -1)"
  echo "eval $row"
  step="$(sed -E 's/.*-([0-9]+)\.onnx/\1/' <<<"$name")"
  rate="$(python3 -c "import json,sys;print(json.loads(sys.argv[1])['hit_rate'])" "$row")"
  m=$(( (step + 500000) / 1000000 ))
  first=$([[ -z "$(ls "$CLIPS"/*.mp4 2>/dev/null)" ]] && echo 1 || echo 0)
  # A clip render is a second heavy process (Unity): only start one when Windows has 5 GB free.
  free_gb="$(/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -NoProfile -Command '[math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB)' 2>/dev/null | tr -d '\r')"
  if (( ${free_gb:-0} < 5 )); then echo "clip skipped for ${m}M: only ${free_gb:-?} GB free"; continue; fi
  if [[ $first == 1 || $(( m % CLIP_EVERY )) == 0 && ! -f "$CLIPS/eval-${m}M.mp4" ]] || python3 -c "import sys;sys.exit(0 if float(sys.argv[1])>float(sys.argv[2]) and float(sys.argv[1])>0 else 1)" "$rate" "$before"; then
    lesson="$(uv run python stats.py "$RES" "Environment/Lesson Number/lesson" 2>/dev/null | awk 'NR==2{printf "%d", $NF}')"
    "$ROOT/tools/train/clip.sh" "$onnx" "$CLIPS/eval-${m}M.mp4" --lesson "${lesson:-3}" && cp "$CLIPS/eval-${m}M.mp4" "$WALL/$PREFIX-eval-$(printf %04d $(( m + OFFSET )))M.mp4"
  fi
done
uv run python chart.py "$RES" "$WALL/$PREFIX-hit-rate.png" >/dev/null
[[ -f "$HOME/.agentworld/gallery/make-index.py" ]] && python3 "$HOME/.agentworld/gallery/make-index.py" >/dev/null || true
echo "published $RUN: $(tail -1 "$RES/eval.jsonl" 2>/dev/null)"
