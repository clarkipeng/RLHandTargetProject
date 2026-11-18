# Agent notes

- Unity runs only in batchmode (`-batchmode -nographics` for builds and training, `-batchmode` with a render target for captures). Never open the Unity editor or any window on the PI's screen, and never play audio.
- One Unity process at a time on this machine (it is short on memory): take the lock with `shlock -f /tmp/unity.lock -p $$` style or check `pgrep -f Unity.app` before starting one; build at `nice -n 10`.
- Python side: `uv` environment in `ml/`, pinned; TensorBoard and the WebGL build are served on `$AGENTWORLD_SEAT_PORT`.
- Land work as PRs into `main` with clips or stills in the PR; rebase right before merging, then squash-merge. Never commit large binaries outside Git LFS.
- Ask the other student something with `agentworld send <agent> "<message>"`.
