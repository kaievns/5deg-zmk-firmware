#!/usr/bin/env bash
#
# build.sh — Build 5deg firmware
#
# Usage:
#   ./scripts/build.sh              # build both halves
#   ./scripts/build.sh left         # left half only
#   ./scripts/build.sh right        # right half only
#   ./scripts/build.sh left clean   # clean + build left
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

BOARD="${ZMK_BOARD:-nice_nano_v2}"
CONFIG_DIR="$PROJECT_DIR/config"
EXTRA_MODULES="$PROJECT_DIR/modules/drivers;$PROJECT_DIR/modules/behaviors"

build_side() {
    local side="$1"
    local shield="5deg_${side}"
    local build_dir="build/${side}"
    local clean="${2:-}"

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Building: ${shield} (board: ${BOARD})"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    local pristine_flag=""
    if [ "$clean" = "clean" ]; then
        pristine_flag="-p"
    fi

    west build -s zmk/app \
        -b "$BOARD" \
        -d "$build_dir" \
        $pristine_flag \
        -- \
        -DSHIELD="$shield" \
        -DZMK_CONFIG="$CONFIG_DIR" \
        -DZMK_EXTRA_MODULES="$EXTRA_MODULES"

    # Copy UF2 to output directory for easy access
    mkdir -p "$PROJECT_DIR/output"
    if [ -f "$build_dir/zephyr/zmk.uf2" ]; then
        cp "$build_dir/zephyr/zmk.uf2" "$PROJECT_DIR/output/${shield}.uf2"
        echo "  → output/${shield}.uf2"
    fi

    echo ""
}

SIDE="${1:-both}"
CLEAN="${2:-}"

# Ensure west workspace is initialised
if [ ! -d .west ]; then
    echo "West workspace not initialised. Running setup first…"
    "$SCRIPT_DIR/setup.sh"
fi

case "$SIDE" in
    left)
        build_side left "$CLEAN"
        ;;
    right)
        build_side right "$CLEAN"
        ;;
    both|all)
        build_side left "$CLEAN"
        build_side right "$CLEAN"
        ;;
    *)
        echo "Usage: $0 [left|right|both] [clean]"
        exit 1
        ;;
esac

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Build complete."
if [ -d "$PROJECT_DIR/output" ]; then
    echo "  Firmware files in: output/"
    ls -lh "$PROJECT_DIR/output/"*.uf2 2>/dev/null || true
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
