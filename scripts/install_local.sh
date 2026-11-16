#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLUGIN_DIR="$HOME/.config/obs-studio/plugins/obs-emeet-control/bin/64bit"

cmake -B "$REPO_DIR/build" -S "$REPO_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO_DIR/build" -j"$(nproc)"

mkdir -p "$PLUGIN_DIR"
cp "$REPO_DIR/build/obs-emeet-control.so" "$PLUGIN_DIR/obs-emeet-control.so"
echo "Plugin installé dans : $PLUGIN_DIR/obs-emeet-control.so"
echo "Redémarrez OBS Studio, puis ajoutez le filtre : 'EMEET CamControl (V4L2 + Auto-Framing + HDR + LUT)'"
