#!/bin/bash
# Mirror this worktree onto the Windows drive, where the Windows Unity and the Windows Python run
# (Unity on a \\wsl$ path imports very slowly). Edits happen here; run this before any Windows-side step.
#   tools/win/sync.sh            -> prints the Windows path of the mirror
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WIN_ROOT="${WIN_ROOT:-/mnt/c/agentworld/rlhand}"
mkdir -p "$WIN_ROOT"
rsync -a --delete \
  --exclude '/Library/' --exclude '/Temp/' --exclude '/Logs/' --exclude '/obj/' --exclude '/UserSettings/' \
  --exclude '/Builds/' --exclude '/results/' --exclude '/renders/' --exclude '.venv/' --exclude '/.git/' \
  --exclude '*.csproj' --exclude '*.sln' \
  "$ROOT/" "$WIN_ROOT/"
wslpath -w "$WIN_ROOT"
