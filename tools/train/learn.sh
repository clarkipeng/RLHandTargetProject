#!/bin/bash
# The watch-it-learn clip: held-out eval clips of checkpoints from early (flailing) to late (clean throws),
# each labelled with its training step and held-out hit rate, with a live hits/attempts counter.
#   [LEARN_SECONDS=9] tools/train/learn.sh <out.mp4> <label>=<policy.onnx>[@lesson] [...]   (@lesson: show that lesson's training targets; default held-out)
# LEARN_TARGET=K records only held-out target K (20: 52 out) and labels each clip with its hit rate at that distance.
# Hit rates come from the run's stages.jsonl (240-throw evals) when it has the checkpoint, else its eval.jsonl (tools/train/publish.sh).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(realpath -m "$1")"; shift
WORK="$ROOT/renders/learn"; mkdir -p "$WORK"
segs=(); T="${LEARN_TARGET:-}"; tgt=(); band=""; ratelabel="held-out hit rate"
if [[ -n "$T" ]]; then tgt=(--target "$T"); band=$(python3 -c "print([25,33,42,52][($T%24)//6])"); ratelabel="held-out hit rate at $band out"; fi
for spec in "$@"; do
  label="${spec%%=*}"; rest="${spec#*=}"; lesson=()
  if [[ "$rest" == *@* ]]; then lesson=(--lesson "${rest##*@}"); rest="${rest%@*}"; fi
  onnx="$(realpath "$rest")"
  key="$(basename "$(dirname "$(dirname "$onnx")")")-$(basename "${onnx%.onnx}")${lesson[1]:+-L${lesson[1]}}${T:+-T$T}"
  clip="$WORK/$key.mp4"; web="$WORK/$key.web.json"
  [[ -f "$clip" ]] || "$ROOT/tools/train/clip.sh" "$onnx" "$clip" --seconds 30 ${lesson[@]+"${lesson[@]}"} ${tgt[@]+"${tgt[@]}"} >/dev/null
  [[ -f "$web" ]] || "$ROOT/tools/win/unity.sh" -executeMethod Showcase.Editor.WebExport.ExportFromCommandLine \
      -take "${clip%.mp4}.take" -out "$web" -logFile "$WORK/$key.export.log" >/dev/null
  rate="$(python3 -c "
import json,os,sys
d=sys.argv[1]; rows=[]
for f in ('stages.jsonl','eval.jsonl'):
    if os.path.exists(os.path.join(d,f)): rows+=[json.loads(l) for l in open(os.path.join(d,f))]
r=next((r for r in rows if r['checkpoint']==sys.argv[2] and not r.get('set')),{})
print(r.get('bands',{}).get(sys.argv[3],{}).get('hit_rate',0.0) if sys.argv[3] else r.get('hit_rate',0.0))" "$(dirname "$(dirname "$onnx")")" "$(basename "$onnx")" "$band")"
  segs+=(--segment "$clip" "$web" "$label" "$rate")
done
cd "$ROOT/ml" && uv run python learn_clip.py "$OUT" "${segs[@]}" --seconds "${LEARN_SECONDS:-9}" --rate-label "$ratelabel"
