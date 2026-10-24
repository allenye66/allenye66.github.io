#!/usr/bin/env bash
# Waits until https://allenye.me serves the build currently in dist/.
# GitHub Pages takes ~30-90s to rebuild after `npm run deploy`, and can briefly
# serve a stale copy, so we compare the hashed JS filename instead of trusting
# the first response.
set -euo pipefail

cd "$(dirname "$0")/.."
SITE="https://allenye.me"

expected=$(grep -oE '/assets/index-[^"]+\.js' dist/index.html || true)
if [ -z "$expected" ]; then
  echo "No build found in dist/. Run npm run deploy first." >&2
  exit 1
fi

for i in $(seq 1 30); do
  live=$(curl -s "$SITE/?nocache=$RANDOM" | grep -oE '/assets/index-[^"]+\.js' || true)
  if [ "$live" = "$expected" ]; then
    echo "Live: $SITE is serving $expected"
    exit 0
  fi
  echo "Waiting for Pages ($i/30): live=${live:-none} expected=$expected"
  sleep 10
done

echo "Timed out. Check https://github.com/allenye66/allenye66.github.io/deployments" >&2
exit 1
