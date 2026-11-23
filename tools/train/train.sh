#!/bin/bash
# Headless PPO training of the throw curriculum on this PC: Windows training players, Linux mlagents-learn.
#   tools/train/train.sh <run_id> [--build] [--resume] [--envs 6] [--arenas 8]
# Builds the Windows player when it is missing (or with --build), then runs mlagents-learn in the background.
# Watch it: tail -f ml/results/logs/<run_id>.log ; TensorBoard: tools/train/board.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
RUN="${1:?usage: train.sh <run_id> [--build] [--resume | --init <run>] [--envs N] [--arenas N] [--gpu] [--config throw.yaml] [--base-port 5005]}"; shift
# Memory: each player is about 110 MB; 12 environments keep the project under about 3 GB.
BUILD=0; RESUME=(); ENVS=12; ARENAS=8; DEVICE=cpu; PROJECT=(); CONFIG=throw.yaml; PORT=5005
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=1 ;;
    --resume) RESUME=(--resume) ;;
    --init) RESUME=(--initialize-from "$2"); shift ;;
    --envs) ENVS="$2"; shift ;;
    --arenas) ARENAS="$2"; shift ;;
    --config) CONFIG="$2"; shift ;;          # another file in ml/config (e.g. throw_windup_bonus.yaml)
    --base-port) PORT="$2"; shift ;;         # a second run at the same time needs its own ports
    --gpu) DEVICE=cuda; PROJECT=(--project gpu) ;;   # PyTorch with CUDA from ml/gpu (the RTX 5070 does the policy updates)
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac; shift
done
WIN_ROOT=/mnt/c/agentworld/rlhand
PLAYER="$WIN_ROOT/Builds/Throw/Throw.exe"
if [[ $BUILD == 1 || ! -f "$PLAYER" ]]; then
  echo "== build the training player"
  tools/train/build.sh || exit 1
fi
mkdir -p ml/results/logs
"$ROOT/tools/win/keepawake.sh" # Windows Modern Standby would freeze the players overnight
# A resume that dies while loading saves a fresh step-0 state over the run's own; keep a copy of the real one.
if [[ -f "ml/results/$RUN/Throw/checkpoint.pt" ]] && python3 - "ml/results/$RUN/run_logs/training_status.json" <<'PY'
import json, sys
try: sys.exit(0 if json.load(open(sys.argv[1]))["Throw"]["checkpoints"][-1]["steps"] > 0 else 1)
except Exception: sys.exit(1)
PY
then
  mkdir -p "ml/results/$RUN/backup" && cp "ml/results/$RUN/Throw/checkpoint.pt" "ml/results/$RUN/run_logs/training_status.json" "ml/results/$RUN/backup/"
fi
# The distance ramp lives in the players: a resumed run starts them at the reach it had got to.
THROW_EXTRA=""
if [[ ${RESUME[0]:-} == --resume ]]; then
  reach="$(cd ml && uv run python stats.py "results/$RUN" Task/Reach 2>/dev/null | awk 'NR==2{print $NF}')"
  [[ -n "$reach" ]] && THROW_EXTRA="-reachStart $reach" && echo "resuming at reach $reach"
fi
export THROW_PLAYER="$PLAYER" THROW_ARENAS="$ARENAS" THROW_EXTRA
cd ml
nohup nice -n 5 uv run ${PROJECT[@]+"${PROJECT[@]}"} mlagents-learn "config/$CONFIG" --base-port "$PORT" --env ../tools/train/player --num-envs "$ENVS" \
  --run-id "$RUN" --results-dir results --torch-device "$DEVICE" ${RESUME[@]+"${RESUME[@]}"} \
  > "results/logs/$RUN.log" 2>&1 &
echo $! > "results/logs/$RUN.pid"
# Unity players at below-normal priority, like every other Unity process on this PC.
( sleep 90; /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -NoProfile -Command \
    'Get-Process Throw -ErrorAction SilentlyContinue | % { $_.PriorityClass = "BelowNormal" }' >/dev/null 2>&1 ) &
echo "training $RUN (pid $(cat "results/logs/$RUN.pid")): tail -f ml/results/logs/$RUN.log"
