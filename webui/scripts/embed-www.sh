#!/usr/bin/env bash
# embed-www.sh — build the Svelte app and stage the gzipped bundle for ESP-IDF embedding.
# Run this from the repo root OR from webui/.  Must be run before `idf.py build`.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBUI_DIR="$SCRIPT_DIR/.."
CONTROLLER_WWW="$SCRIPT_DIR/../../controller/main/www"

echo "==> Building webui..."
cd "$WEBUI_DIR"
npm run build

echo "==> Staging gzipped asset..."
mkdir -p "$CONTROLLER_WWW"
gzip -9 -c dist/index.html > "$CONTROLLER_WWW/index.html.gz"

SIZE=$(wc -c < "$CONTROLLER_WWW/index.html.gz")
echo "==> Done: $CONTROLLER_WWW/index.html.gz  (${SIZE} bytes)"
