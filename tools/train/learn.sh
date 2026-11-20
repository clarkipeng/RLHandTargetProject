#!/bin/bash
# The watch-it-learn clip: held-out eval clips of checkpoints from early (flailing) to late (clean throws),
# each labelled with its training step and held-out hit rate, with a live hits/attempts counter.
#   tools/train/learn.sh <out.mp4> <label>=<policy.onnx>[@lesson] [...]   (@lesson: show that lesson's training targets; default held-out)
# Hit rates come from the run's eval.jsonl next to each checkpoint (tools/train/publish.sh writes it).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(realpath -m "$1")"; shift
WORK="$ROOT/renders/learn"; mkdir -p "$WORK"
segs=()
for spec in "$@"; do
  label="${spec%%=*}"; rest="${spec#*=}"; lesson=()
  if [[ "$rest" == *@* ]]; then lesson=(--lesson "${rest##*@}"); rest="${rest%@*}"; fi
  onnx="$(realpath "$rest")"
  key="$(basename "$(dirname "$(dirname "$onnx")")")-$(basename "${onnx%.onnx}")${lesson[1]:+-L${lesson[1]}}"
  clip="$WORK/$key.mp4"; web="$WORK/$key.web.json"
  [[ -f "$clip" ]] || "$ROOT/tools/train/clip.sh" "$onnx" "$clip" --seconds 30 ${lesson[@]+"${lesson[@]}"} >/dev/null
  [[ -f "$web" ]] || "$ROOT/tools/win/unity.sh" -executeMethod Showcase.Editor.WebExport.ExportFromCommandLine \
      -take "${clip%.mp4}.take" -out "$web" -logFile "$WORK/$key.export.log" >/dev/null
  rate="$(python3 -c "
import json,sys
rows=[json.loads(l) for l in open(sys.argv[1])] if __import__('os').path.exists(sys.argv[1]) else []
print(next((r['hit_rate'] for r in rows if r['checkpoint']==sys.argv[2]),0.0))" "$(dirname "$(dirname "$onnx")")/eval.jsonl" "$(basename "$onnx")")"
  segs+=(--segment "$clip" "$web" "$label" "$rate")
done
cd "$ROOT/ml" && uv run python learn_clip.py "$OUT" "${segs[@]}"
