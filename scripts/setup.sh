#!/usr/bin/env bash
#
# setup.sh — First-time workspace initialisation
#
# Initialises west, fetches ZMK + Zephyr, and installs Python deps.
# Safe to re-run; skips steps that are already done.
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "▸ Initialising west workspace…"
if [ ! -d .west ]; then
    west init -l config
else
    echo "  (already initialised)"
fi

echo "▸ Updating west modules (shallow fetch)…"
west update --narrow --fetch-opt=--depth=1

echo "▸ Exporting Zephyr CMake package…"
west zephyr-export

echo ""
echo "✓ Setup complete. Build with:"
echo "    ./scripts/build.sh          # both halves"
echo "    ./scripts/build.sh left     # left only"
echo "    ./scripts/build.sh right    # right only"
