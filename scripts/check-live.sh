#!/usr/bin/env bash
# Waits until https://allenye.me serves the build currently in dist/.
# GitHub Pages takes ~30-90s to rebuild after `npm run deploy`, and can briefly
# serve a stale copy, so we compare every HTML page in dist/ against the live
# site by checksum instead of trusting the first response.
set -euo pipefail

cd "$(dirname "$0")/.."
SITE="https://allenye.me"

if [ ! -f dist/index.html ]; then
  echo "No build found in dist/. Run npm run deploy first." >&2
  exit 1
fi

files=$(cd dist && find . -name '*.html' | sed 's|^\./||' | sort)

for i in $(seq 1 30); do
  stale=""
  for file in $files; do
    url_path=${file%index.html}   # projects/index.html -> projects/
    live_sum=$(curl -s "$SITE/$url_path" | md5 -q)
    [ "$(md5 -q "dist/$file")" = "$live_sum" ] || stale="$stale /$url_path"
  done
  if [ -z "$stale" ]; then
    echo "Live: $SITE serves the current dist/ ($(echo "$files" | wc -l | tr -d ' ') pages)"
    exit 0
  fi
  echo "Waiting for Pages ($i/30): stale:$stale"
  sleep 10
done

echo "Timed out. Check https://github.com/allenye66/allenye66.github.io/deployments" >&2
exit 1
