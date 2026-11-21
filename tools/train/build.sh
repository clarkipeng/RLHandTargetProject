#!/bin/bash
# Build the Windows training player (blocking). Prints compiler errors and the builder's summary.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"; mkdir -p ml/results
tools/win/unity.sh -executeMethod Training.Editor.TrainingBuilder.BuildFromCommandLine \
  -out /mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe -logFile "$ROOT/ml/results/build.log" > ml/results/build.out 2>&1
code=$?
grep -E "error CS|\[train\]|No valid|^[A-Za-z]*Exception" ml/results/build.log | sort -u | head -20
echo "build exit $code"; exit $code
