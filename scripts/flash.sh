#!/usr/bin/env bash
#
# flash.sh — Copy UF2 firmware to a mounted nice!nano bootloader volume
#
# Usage:
#   ./scripts/flash.sh left          # flash left half
#   ./scripts/flash.sh right         # flash right half
#
# The nice!nano enters UF2 bootloader mode when you double-tap reset.
# It mounts as a USB mass-storage device (typically "NICENANO").
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

SIDE="${1:?Usage: $0 <left|right>}"

UF2_FILE="$PROJECT_DIR/output/5deg_${SIDE}.uf2"

if [ ! -f "$UF2_FILE" ]; then
    echo "Error: $UF2_FILE not found. Build first with:"
    echo "  ./scripts/build.sh $SIDE"
    exit 1
fi

# Detect mounted bootloader volume
MOUNT_POINT=""
for candidate in \
    "/Volumes/NICENANO" \
    "/Volumes/NRF52BOOT" \
    "/media/$USER/NICENANO" \
    "/media/$USER/NRF52BOOT" \
    "/run/media/$USER/NICENANO" \
    "/run/media/$USER/NRF52BOOT"; do
    if [ -d "$candidate" ]; then
        MOUNT_POINT="$candidate"
        break
    fi
done

if [ -z "$MOUNT_POINT" ]; then
    echo "No bootloader volume detected."
    echo "Double-tap reset on the $SIDE half, then re-run this script."
    exit 1
fi

echo "Flashing $UF2_FILE → $MOUNT_POINT …"
cp "$UF2_FILE" "$MOUNT_POINT/"

echo "Done. The board will reboot automatically."
