#!/bin/bash
# TensorBoard for all runs, behind the seat port: http://localhost:$AGENTWORLD_SEAT_PORT/tensorboard/
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TB_PORT="${TENSORBOARD_PORT:-42051}"
if ! (exec 3<>/dev/tcp/127.0.0.1/$TB_PORT) 2>/dev/null; then
  cd "$ROOT/ml"
  nohup uv run tensorboard --logdir results --port "$TB_PORT" --host 127.0.0.1 --path_prefix /tensorboard >results/logs/tensorboard.log 2>&1 &
  sleep 5
fi
"$ROOT/tools/showcase/serve.sh" >/dev/null
echo "http://localhost:$(cat "$ROOT/renders/web/port")/tensorboard/"
