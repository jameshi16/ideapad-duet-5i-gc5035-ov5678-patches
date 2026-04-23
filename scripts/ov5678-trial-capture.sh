#!/usr/bin/env bash
set -euo pipefail

LABEL=""
OUT_DIR="capture/trials"
HUE_ROI=""

usage() {
  cat <<'EOF'
Usage:
  ./ov5678-trial-capture.sh --label <name> [--out-dir <dir>] [--hue-roi x,y,w,h]

Collects a repeatable OV5678 trial bundle:
- dmesg mapping snippets, regulators, LED class snapshot
- OV5678 controls + service/journal status
- loopback frame capture + image_quality metrics against /home/agent/good.jpg
- towel ROI hue diagnostics (magenta-vs-green proportions)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --label)
      LABEL="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --hue-roi)
      HUE_ROI="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$LABEL" ]]; then
  echo "--label is required" >&2
  usage
  exit 2
fi

TS="$(date -u +%Y%m%dT%H%M%SZ)"
TRIAL_DIR="${OUT_DIR}/${TS}_${LABEL}"
mkdir -p "${TRIAL_DIR}"

echo "trial_dir=${TRIAL_DIR}"

{
  echo "label=${LABEL}"
  echo "timestamp_utc=${TS}"
  uname -a
} > "${TRIAL_DIR}/host.txt"

sudo dmesg > "${TRIAL_DIR}/dmesg.txt"
grep -iE "int3472|ovti5678|gcti5035|mapping type|unknown 0x02|privacy-led|dvdd|avdd" \
  "${TRIAL_DIR}/dmesg.txt" > "${TRIAL_DIR}/dmesg-int3472.txt" || true

{
  echo "=== /sys/class/leds ==="
  ls -la /sys/class/leds 2>&1 || true
} > "${TRIAL_DIR}/leds.txt"

{
  echo "name,state,num_users,microvolts"
  for r in /sys/class/regulator/regulator.*; do
    [[ -d "$r" ]] || continue
    name=$(cat "$r/name" 2>/dev/null || true)
    state=$(cat "$r/state" 2>/dev/null || true)
    users=$(cat "$r/num_users" 2>/dev/null || true)
    uv=$(cat "$r/microvolts" 2>/dev/null || true)
    echo "${name},${state},${users},${uv}"
  done
} > "${TRIAL_DIR}/regulators.csv"

sudo systemctl status --no-pager --full processed-camera-ov5678.service \
  > "${TRIAL_DIR}/ov5678-service-status.txt" 2>&1 || true
sudo journalctl -u processed-camera-ov5678.service -n 220 --no-pager \
  > "${TRIAL_DIR}/ov5678-service-journal.txt" 2>&1 || true

sudo v4l2-ctl -d /dev/v4l-subdev5 --all > "${TRIAL_DIR}/ov5678-controls.txt" 2>&1 || true
sudo v4l2-ctl -d /dev/v4l-subdev4 --all > "${TRIAL_DIR}/gc5035-controls.txt" 2>&1 || true

set +e
sudo ./select-camera.sh ov5678 > "${TRIAL_DIR}/select-ov5678.txt" 2>&1
OV_SELECT_RC=$?
set -e
echo "${OV_SELECT_RC}" > "${TRIAL_DIR}/ov5678-select-exit.txt"
if [[ "${OV_SELECT_RC}" -ne 0 ]]; then
  sudo systemctl reset-failed processed-camera-ov5678.service || true
  sudo systemctl restart processed-camera-loopback.service processed-camera-ov5678.service || true
  sleep 2
fi

OV_IMG="${TRIAL_DIR}/output_ov_${LABEL}.png"
OV_CAPTURE_RC=1
for _try in 1 2 3; do
  if sudo timeout 30 ffmpeg -hide_banner -loglevel error \
      -f v4l2 -input_format yuyv422 -video_size 1280x960 -i /dev/video43 \
      -frames:v 1 -y "${OV_IMG}" \
      >> "${TRIAL_DIR}/ffmpeg-ov5678.txt" 2>&1; then
    OV_CAPTURE_RC=0
    break
  fi
  sleep 2
done
echo "${OV_CAPTURE_RC}" > "${TRIAL_DIR}/ov5678-capture-exit.txt"
if [[ "${OV_CAPTURE_RC}" -ne 0 ]]; then
  echo "Failed to capture /dev/video43 frame after retries" >&2
  exit 1
fi

RAW_PATH="${TRIAL_DIR}/ov5678_${LABEL}.bin"
./capture_raw.sh --sensor ov5678 --video /dev/video24 --out "${RAW_PATH}" \
  > "${TRIAL_DIR}/capture-raw-ov5678.txt" 2>&1 || true

if [[ -z "${HUE_ROI}" ]]; then
  HUE_ROI="$(
    python3 - "${OV_IMG}" <<'PY'
import cv2
import sys

img = cv2.imread(sys.argv[1], cv2.IMREAD_COLOR)
if img is None:
    raise SystemExit(1)
h, w = img.shape[:2]
x = int(round(w * 0.02))
y = int(round(h * 0.52))
cw = int(round(w * 0.44))
ch = int(round(h * 0.42))
print(f"{x},{y},{cw},{ch}")
PY
  )"
fi
echo "${HUE_ROI}" > "${TRIAL_DIR}/hue-roi.txt"

REF_IMG="${TRIAL_DIR}/good_resized_for_trial.png"
python3 - "${OV_IMG}" "/home/agent/good.jpg" "${REF_IMG}" <<'PY'
import cv2
import sys

target = cv2.imread(sys.argv[1], cv2.IMREAD_COLOR)
ref = cv2.imread(sys.argv[2], cv2.IMREAD_COLOR)
if target is None or ref is None:
    raise SystemExit(1)
h, w = target.shape[:2]
ref = cv2.resize(ref, (w, h), interpolation=cv2.INTER_AREA)
cv2.imwrite(sys.argv[3], ref)
PY

python3 ./image_quality.py "${OV_IMG}" --reference "${REF_IMG}" \
  --hue-roi "${HUE_ROI}" > "${TRIAL_DIR}/ov5678-quality.json"

set +e
sudo ./select-camera.sh gc5035 > "${TRIAL_DIR}/select-gc5035.txt" 2>&1
GC_OK=$?
set -e
echo "${GC_OK}" > "${TRIAL_DIR}/gc5035-select-exit.txt"

echo "trial_complete=${TRIAL_DIR}"
