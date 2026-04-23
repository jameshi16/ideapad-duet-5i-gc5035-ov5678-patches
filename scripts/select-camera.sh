#!/usr/bin/env bash
set -euo pipefail

probe_loopback() {
  local dev="$1"
  local width="${2:-1280}"
  local height="${3:-960}"
  local fps="${4:-15}"
  local tries=3
  local wait_s=2
  local tmp="/tmp/select-camera-probe-$(basename "$dev").yuv"

  for i in $(seq 1 "$tries"); do
    sudo rm -f "$tmp"
    if sudo timeout 20 ffmpeg -hide_banner -loglevel error \
      -f v4l2 -input_format yuyv422 -video_size "${width}x${height}" -framerate "$fps" \
      -i "$dev" -frames:v 3 -f rawvideo -pix_fmt yuyv422 -y "$tmp" >/dev/null 2>&1 \
      && python3 - "$dev" "$tmp" <<'PY'
import numpy as np
import os
import sys

dev = sys.argv[1]
path = sys.argv[2]

if not os.path.exists(path) or os.path.getsize(path) == 0:
    raise SystemExit(2)

buf = np.fromfile(path, dtype=np.uint8)
if buf.size == 0:
    raise SystemExit(3)

y = buf[0::2]
y_mean = float(y.mean())
y_max = int(y.max())
print(f"probe {dev}: y_mean={y_mean:.2f} y_max={y_max}")

if y_mean < 20.0 and y_max < 40:
    raise SystemExit(4)
PY
    then
      sudo rm -f "$tmp"
      return 0
    fi
    sleep "$wait_s"
  done

  sudo rm -f "$tmp"
  echo "loopback probe failed for $dev (still dark/unreadable)" >&2
  return 1
}

stop_pipelines() {
  sudo systemctl stop \
    processed-camera-gc5035.service \
    processed-camera-ov5678.service \
    processed-camera-gc5035-fullres.service \
    processed-camera-ov5678-fullres.service >/dev/null 2>&1 || true
}

case "${1:-}" in
  gc5035)
    stop_pipelines
    sudo /home/agent/ipu6-drivers/configure-camera-link.sh gc5035
    sudo systemctl start processed-camera-gc5035.service
    probe_loopback /dev/video42 1280 960 15
    ;;
  ov5678)
    stop_pipelines
    sudo /home/agent/ipu6-drivers/configure-camera-link.sh ov5678
    sudo systemctl start processed-camera-ov5678.service
    probe_loopback /dev/video43 1280 960 15
    ;;
  gc5035-fullres)
    stop_pipelines
    sudo /home/agent/ipu6-drivers/configure-camera-link.sh gc5035
    sudo systemctl start processed-camera-gc5035-fullres.service
    probe_loopback /dev/video42 2592 1944 5
    ;;
  ov5678-fullres)
    stop_pipelines
    sudo /home/agent/ipu6-drivers/configure-camera-link.sh ov5678
    sudo systemctl start processed-camera-ov5678-fullres.service
    probe_loopback /dev/video43 2592 1944 5
    ;;
  stop)
    stop_pipelines
    ;;
  status)
    systemctl --no-pager --full status \
      processed-camera-gc5035.service \
      processed-camera-ov5678.service \
      processed-camera-gc5035-fullres.service \
      processed-camera-ov5678-fullres.service
    ;;
  *)
    echo "usage: $0 {gc5035|ov5678|gc5035-fullres|ov5678-fullres|stop|status}" >&2
    exit 2
    ;;
esac
