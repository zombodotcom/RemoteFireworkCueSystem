#!/usr/bin/env bash
# Build the fireworkcore rig to a single-file ES module.
# Self-activates emsdk on PATH (Git Bash: sourcing emsdk_env.sh does NOT work;
# emcc.exe lives in upstream/emscripten). Override EMSDK_DIR if installed elsewhere.
set -euo pipefail

EMSDK_DIR="${EMSDK_DIR:-/c/emsdk}"
export PATH="$EMSDK_DIR/upstream/emscripten:$PATH"
command -v emcc >/dev/null || { echo "emcc not found; check EMSDK_DIR ($EMSDK_DIR)"; exit 1; }

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CORE="$ROOT/components/fireworkcore"
OUT="$ROOT/webui/src/core/fireworkcore.js"
mkdir -p "$(dirname "$OUT")"

EXPORTS='["_rig_reset","_rig_set_switch","_rig_heartbeat","_rig_arm","_rig_disarm","_rig_estop","_rig_clear_estop","_rig_fire","_rig_load_sequence","_rig_start_sequence","_rig_stop_sequence","_rig_seq_running","_rig_tick","_rig_box_state","_rig_box_switch","_rig_box_estopped","_rig_box_can_fire","_rig_channel_firing","_rig_channel_msleft","_malloc","_free"]'

emcc \
  "$CORE/src/crc32.cpp" "$CORE/src/arming.cpp" "$CORE/src/sequence.cpp" \
  "$CORE/wasm/sim_bindings.cpp" \
  -I "$CORE/include" -I "$CORE/wasm" \
  -O2 -std=c++11 \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 \
  -sEXPORTED_FUNCTIONS="$EXPORTS" \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","setValue","getValue"]' \
  -o "$OUT"

echo "built $OUT"
