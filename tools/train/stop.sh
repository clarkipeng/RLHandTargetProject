#!/bin/bash
# Stop a training run (the trainer, its env workers and their Windows players). Checkpoints stay; resume with train.sh <run> --resume.
#   tools/train/stop.sh <run_id>
RUN="${1:?usage: stop.sh <run_id>}"
pkill -INT -f "mlagents-learn .*--run-id $RUN( |$)"; sleep 20
pkill -KILL -f "mlagents-learn .*--run-id $RUN( |$)"
/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -NoProfile -Command \
  'Get-Process Throw -ErrorAction SilentlyContinue | Stop-Process -Force' >/dev/null 2>&1
echo "stopped $RUN"
