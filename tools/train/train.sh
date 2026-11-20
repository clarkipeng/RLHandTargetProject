#!/bin/bash
# Headless PPO training of the throw curriculum on this PC: Windows training players, Linux mlagents-learn.
#   tools/train/train.sh <run_id> [--build] [--resume] [--envs 6] [--arenas 8]
# Builds the Windows player when it is missing (or with --build), then runs mlagents-learn in the background.
# Watch it: tail -f ml/results/<run_id>/train.log ; TensorBoard: tools/train/board.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
RUN="${1:?usage: train.sh <run_id> [--build] [--resume] [--envs N] [--arenas N]}"; shift
BUILD=0; RESUME=(); ENVS=6; ARENAS=8
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=1 ;;
    --resume) RESUME=(--resume) ;;
    --envs) ENVS="$2"; shift ;;
    --arenas) ARENAS="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac; shift
done
WIN_ROOT=/mnt/c/agentworld/rlhand
PLAYER="$WIN_ROOT/Builds/Throw/Throw.exe"
if [[ $BUILD == 1 || ! -f "$PLAYER" ]]; then
  echo "== build the training player"
  tools/win/unity.sh -executeMethod Training.Editor.TrainingBuilder.BuildFromCommandLine \
    -out "$PLAYER" -logFile "$ROOT/ml/results/build.log" || { grep -E "error|\[train\]" ml/results/build.log | tail -20; exit 1; }
  grep "\[train\]" ml/results/build.log
fi
mkdir -p "ml/results/$RUN"
export THROW_PLAYER="$PLAYER" THROW_ARENAS="$ARENAS"
cd ml
nohup nice -n 5 uv run mlagents-learn config/throw.yaml --env ../tools/train/player --num-envs "$ENVS" \
  --run-id "$RUN" --results-dir results --torch-device cpu ${RESUME[@]+"${RESUME[@]}"} \
  > "results/$RUN/train.log" 2>&1 &
echo $! > "results/$RUN/train.pid"
echo "training $RUN (pid $(cat "results/$RUN/train.pid")): tail -f ml/results/$RUN/train.log"
