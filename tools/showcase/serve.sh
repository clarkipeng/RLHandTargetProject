#!/bin/bash
# The browser live view of the current best policy, for the lab wall.
#   tools/showcase/serve.sh [take.json]
# With a take (written by render.sh), publishes it as the current policy; open pages pick it up within 30 s.
# Always makes sure the site is set up and served on $AGENTWORLD_SEAT_PORT (default 42050, localhost),
# and points the rl-arm wall at it. TensorBoard (tools/train/board.sh) is proxied at /tensorboard/.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SITE="$ROOT/renders/web"
PORT="${AGENTWORLD_SEAT_PORT:-42050}"
THREE_VERSION=0.160.0
mkdir -p "$SITE/takes"

# Page sources are linked, so edits under tools/showcase/web show up on reload.
ln -sf "$ROOT/tools/showcase/web/index.html" "$SITE/index.html"
ln -sf "$ROOT/tools/showcase/web/viewer.js" "$SITE/viewer.js"

# three.js, vendored once from the npm registry so the wall does not depend on a CDN.
if [[ ! -f "$SITE/vendor/three/build/three.module.js" ]]; then
  tmp="$(mktemp -d)"
  curl -fsSL "https://registry.npmjs.org/three/-/three-$THREE_VERSION.tgz" -o "$tmp/three.tgz"
  mkdir -p "$SITE/vendor/three"
  tar -xzf "$tmp/three.tgz" -C "$tmp"
  cp -R "$tmp/package/build" "$tmp/package/examples" "$SITE/vendor/three/"
  rm -rf "$tmp"
fi

if [[ $# -gt 0 ]]; then
  name="$(basename "$1")"
  [[ "$(cd "$(dirname "$1")" && pwd)/$name" == "$SITE/takes/$name" ]] || cp "$1" "$SITE/takes/$name"
  printf '{"take":"takes/%s","version":"%s-%s"}\n' "$name" "${name%.json}" "$(date +%s)" > "$SITE/current.json.tmp"
  mv "$SITE/current.json.tmp" "$SITE/current.json"
  echo "published $name"
fi

# Our server already up? (recorded port). Otherwise use the seat port, or the next free one when another program holds it
# (on the Windows PC, mirrored networking shares ports with Windows, and OneDrive can sit on the seat port).
free() { python3 -c "import socket,sys;s=socket.socket();s.bind(('127.0.0.1',int(sys.argv[1])));s.close()" "$1" 2>/dev/null; }
ours() { curl -s -m 3 -o /dev/null "http://127.0.0.1:$1/viewer.js"; }
if [[ -f "$SITE/port" ]] && ours "$(cat "$SITE/port")"; then
  PORT="$(cat "$SITE/port")"
else
  while ! free "$PORT"; do PORT=$((PORT + 2)); done
  echo "$PORT" > "$SITE/port"
  nohup python3 "$ROOT/tools/showcase/server.py" "$SITE" "$PORT" "${TENSORBOARD_PORT:-42051}" >"$SITE/server.log" 2>&1 &
  sleep 1
  echo "serving $SITE on http://localhost:$PORT/"
fi

GALLERY="$HOME/.agentworld/gallery/rl-arm"
if [[ -d "$GALLERY" ]]; then
  echo "http://localhost:$PORT/" > "$GALLERY/live.url"
  [[ -f "$HOME/.agentworld/gallery/make-index.py" ]] && python3 "$HOME/.agentworld/gallery/make-index.py" >/dev/null || true
fi
