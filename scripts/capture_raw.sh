#!/usr/bin/env bash
set -euo pipefail

# Capture one raw frame from a specific camera node and write provenance metadata.
#
# Usage:
#   ./capture_raw.sh --sensor gc5035 --video /dev/video8 --out capture/gc5035-raw.bin
#   ./capture_raw.sh --sensor ov5678 --video /dev/video24 --out capture/ov5678-raw.bin

SENSOR=""
VIDEO=""
OUT=""
WIDTH=2592
HEIGHT=1944

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sensor)
      SENSOR="$2"
      shift 2
      ;;
    --video)
      VIDEO="$2"
      shift 2
      ;;
    --out)
      OUT="$2"
      shift 2
      ;;
    --width)
      WIDTH="$2"
      shift 2
      ;;
    --height)
      HEIGHT="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$SENSOR" || -z "$VIDEO" || -z "$OUT" ]]; then
  echo "Usage: $0 --sensor <gc5035|ov5678> --video </dev/videoN> --out <path> [--width N --height N]" >&2
  exit 2
fi

if [[ "$SENSOR" != "gc5035" && "$SENSOR" != "ov5678" ]]; then
  echo "Invalid --sensor '$SENSOR' (expected gc5035 or ov5678)" >&2
  exit 2
fi

OUT_DIR="$(dirname "$OUT")"
mkdir -p "$OUT_DIR"

sudo timeout 120 v4l2-ctl -d "$VIDEO" \
  --set-fmt-video="width=${WIDTH},height=${HEIGHT},pixelformat=BA10" \
  --stream-mmap=3 --stream-count=1 --stream-to="$OUT"

SIZE=$(stat -c '%s' "$OUT")
if [[ "$SIZE" -eq 0 ]]; then
  echo "Capture failed: $OUT is empty. Check media links with configure-camera-link.sh." >&2
  exit 1
fi

SHA=$(sha256sum "$OUT" | awk '{print $1}')
META_PATH="$OUT.meta.json"
UTC_NOW=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

cat > "$META_PATH" <<EOF
{
  "created_utc": "$UTC_NOW",
  "sensor": "$SENSOR",
  "video_node": "$VIDEO",
  "width": $WIDTH,
  "height": $HEIGHT,
  "raw_path": "$OUT",
  "size_bytes": $SIZE,
  "sha256": "$SHA",
  "capture_cmd": "sudo v4l2-ctl -d $VIDEO --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=BA10 --stream-mmap=3 --stream-count=1 --stream-to=$OUT"
}
EOF

echo "Captured: $OUT"
echo "Metadata: $META_PATH"
echo "Size: $SIZE"
echo "SHA256: $SHA"
