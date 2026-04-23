#!/usr/bin/env bash
set -euo pipefail

# Configure media links/formats for a single active sensor pipeline.
#
# Usage:
#   configure-camera-link.sh gc5035
#   configure-camera-link.sh ov5678

SENSOR="${1:-}"

if [[ -z "$SENSOR" ]]; then
  echo "usage: $0 {gc5035|ov5678}" >&2
  exit 2
fi

set_link() {
  local expr="$1"
  media-ctl -d /dev/media0 -l "$expr"
}

set_fmt_subdev() {
  local dev="$1"
  local pad="$2"
  local width="$3"
  local height="$4"
  v4l2-ctl -d "$dev" --set-subdev-fmt "pad=${pad},width=${width},height=${height},code=0x300a"
}

set_fmt_video() {
  local dev="$1"
  local width="$2"
  local height="$3"
  local pixfmt="$4"
  v4l2-ctl -d "$dev" --set-fmt-video="width=${width},height=${height},pixelformat=${pixfmt}"
}

case "$SENSOR" in
  gc5035)
    # Enable GC5035 capture route, disable OV5678 capture route.
    set_link "'Intel IPU6 CSI2 1':1 -> 'Intel IPU6 ISYS Capture 8':0 [1]"
    set_link "'Intel IPU6 CSI2 3':1 -> 'Intel IPU6 ISYS Capture 24':0 [0]"

    # Keep sensor and CSI formats coherent for the active route.
    set_fmt_subdev /dev/v4l-subdev4 0 2592 1944
    set_fmt_subdev /dev/v4l-subdev1 0 2592 1944
    set_fmt_subdev /dev/v4l-subdev1 1 2592 1944
    set_fmt_video /dev/video8 2592 1944 BA10
    ;;

  ov5678)
    # Enable OV5678 capture route, disable GC5035 capture route.
    set_link "'Intel IPU6 CSI2 3':1 -> 'Intel IPU6 ISYS Capture 24':0 [1]"
    set_link "'Intel IPU6 CSI2 1':1 -> 'Intel IPU6 ISYS Capture 8':0 [0]"

    # Keep sensor and CSI formats coherent for the active route.
    set_fmt_subdev /dev/v4l-subdev5 0 2592 1944
    set_fmt_subdev /dev/v4l-subdev3 0 2592 1944
    set_fmt_subdev /dev/v4l-subdev3 1 2592 1944
    set_fmt_video /dev/video24 2592 1944 BA10
    ;;

  *)
    echo "invalid sensor: $SENSOR" >&2
    exit 2
    ;;
esac
