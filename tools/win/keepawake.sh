#!/bin/bash
# Start the keep-awake helper unless it is already running (see keepawake.ps1).
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
cp "$(dirname "$0")"/keepawake*.ps1 /mnt/c/agentworld/
"$PS" -NoProfile -ExecutionPolicy Bypass -File 'C:\agentworld\keepawake-running.ps1' && exit 0
nohup "$PS" -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File 'C:\agentworld\keepawake.ps1' >/dev/null 2>&1 &
echo "keep-awake on while training"
