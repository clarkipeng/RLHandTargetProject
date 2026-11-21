#!/bin/bash
# Keeps the rl-arm wall current while a run trains: every 20 minutes, evaluates new checkpoints and redraws the chart.
#   tools/train/watch.sh <run_id>   (stops when the run's trainer exits)
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RUN="${1:?usage: watch.sh <run_id>}"
while pgrep -f "mlagents-learn .*--run-id $RUN( |$)" >/dev/null; do
  "$ROOT/tools/train/publish.sh" "$RUN" 2>&1 | grep -E "^eval|^published|rror" | sed "s/^/$(date +%H:%M) /"
  sleep 1200
done
"$ROOT/tools/train/publish.sh" "$RUN" 2>&1 | tail -1
