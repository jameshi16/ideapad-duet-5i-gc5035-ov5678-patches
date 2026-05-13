#!/usr/bin/env bash
set -euo pipefail

# Common camera control helper for GC5035 + OV5678 sensor subdevices.

GC_SUBDEV=${GC_SUBDEV:-/dev/v4l-subdev4}
OV_SUBDEV=${OV_SUBDEV:-/dev/v4l-subdev5}

SENSOR=""
ACTION=""
EXPOSURE=""
ANALOG_GAIN=""
DIGITAL_GAIN=""
VBLANK=""
TEST_PATTERN=""
PRESET=""
SHOW=0
PRESET_HANDLED=0

usage() {
  cat <<'EOF'
Usage:
  camera-controls.sh --sensor <gc5035|ov5678|all> [options]

Options:
  --show                         Show controls only
  --set-exposure <value>         Set V4L2 exposure
  --set-analog-gain <value>      Set V4L2 analogue_gain
  --set-digital-gain <value>     Set V4L2 digital_gain
  --set-vblank <value>           Set V4L2 vertical_blanking
  --set-test-pattern <0|1>       Set V4L2 test_pattern (0 disabled)
  --preset <name>                Apply preset: auto|indoor|daylight|lowlight|pipeline

Examples:
  camera-controls.sh --sensor gc5035 --show
  camera-controls.sh --sensor ov5678 --set-exposure 700 --set-analog-gain 256
  camera-controls.sh --sensor all --preset lowlight

Environment overrides:
  GC_SUBDEV=/dev/v4l-subdev4
  OV_SUBDEV=/dev/v4l-subdev5
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sensor)
      SENSOR="$2"
      shift 2
      ;;
    --show)
      SHOW=1
      shift
      ;;
    --set-exposure)
      EXPOSURE="$2"
      shift 2
      ;;
    --set-analog-gain)
      ANALOG_GAIN="$2"
      shift 2
      ;;
    --set-digital-gain)
      DIGITAL_GAIN="$2"
      shift 2
      ;;
    --set-vblank)
      VBLANK="$2"
      shift 2
      ;;
    --set-test-pattern)
      TEST_PATTERN="$2"
      shift 2
      ;;
    --preset)
      PRESET="$2"
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

if [[ -z "$SENSOR" ]]; then
  echo "--sensor is required" >&2
  usage
  exit 2
fi

apply_ov5678_auto() {
  local bus="/dev/i2c-3"

  if [[ ! -e "$bus" ]]; then
    sudo modprobe i2c-dev
  fi

  echo "Applying to ov5678: auto AEC/AGC (0x3503=0x00)"
  sudo python3 - <<'PY'
import fcntl
import os

I2C_SLAVE_FORCE = 0x0706
OV5678_ADDR = 0x36
bus = "/dev/i2c-3"

fd = os.open(bus, os.O_RDWR)
try:
    fcntl.ioctl(fd, I2C_SLAVE_FORCE, OV5678_ADDR)
    os.write(fd, bytes([0x35, 0x03, 0x00]))
finally:
    os.close(fd)
PY
}

apply_preset() {
  local sensor="$1"
  local preset="$2"
  case "$sensor:$preset" in
    gc5035:auto)
      ;;
    gc5035:daylight)
      EXPOSURE=${EXPOSURE:-500}
      ANALOG_GAIN=${ANALOG_GAIN:-256}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1023}
      ;;
    gc5035:indoor)
      EXPOSURE=${EXPOSURE:-800}
      ANALOG_GAIN=${ANALOG_GAIN:-320}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1023}
      ;;
    gc5035:lowlight)
      EXPOSURE=${EXPOSURE:-1200}
      ANALOG_GAIN=${ANALOG_GAIN:-640}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1023}
      ;;
    gc5035:pipeline)
      EXPOSURE=${EXPOSURE:-800}
      ANALOG_GAIN=${ANALOG_GAIN:-384}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1023}
      ;;
    ov5678:auto)
      apply_ov5678_auto
      PRESET_HANDLED=1
      ;;
    ov5678:daylight)
      EXPOSURE=${EXPOSURE:-350}
      ANALOG_GAIN=${ANALOG_GAIN:-128}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1024}
      ;;
    ov5678:indoor)
      EXPOSURE=${EXPOSURE:-550}
      ANALOG_GAIN=${ANALOG_GAIN:-220}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1024}
      ;;
    ov5678:lowlight)
      EXPOSURE=${EXPOSURE:-900}
      ANALOG_GAIN=${ANALOG_GAIN:-380}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1024}
      ;;
    ov5678:pipeline)
      EXPOSURE=${EXPOSURE:-512}
      ANALOG_GAIN=${ANALOG_GAIN:-1024}
      DIGITAL_GAIN=${DIGITAL_GAIN:-1024}
      ;;
    *)
      echo "Unsupported preset '$preset' for sensor '$sensor'" >&2
      exit 2
      ;;
  esac
}

show_controls() {
  local dev="$1"
  local label="$2"
  echo "--- $label ($dev) ---"
  sudo v4l2-ctl -d "$dev" --list-ctrls
}

apply_controls() {
  local dev="$1"
  local label="$2"

  if [[ "$SHOW" -eq 1 ]]; then
    show_controls "$dev" "$label"
    return
  fi

  local args=()
  [[ -n "$EXPOSURE" ]] && args+=("exposure=$EXPOSURE")
  [[ -n "$ANALOG_GAIN" ]] && args+=("analogue_gain=$ANALOG_GAIN")
  [[ -n "$DIGITAL_GAIN" ]] && args+=("digital_gain=$DIGITAL_GAIN")
  [[ -n "$VBLANK" ]] && args+=("vertical_blanking=$VBLANK")
  [[ -n "$TEST_PATTERN" ]] && args+=("test_pattern=$TEST_PATTERN")

  if [[ ${#args[@]} -eq 0 ]]; then
    if [[ "$PRESET_HANDLED" -eq 1 ]]; then
      return
    fi
    echo "No controls requested for $label; use --show or set options." >&2
    return
  fi

  local joined
  joined=$(IFS=,; echo "${args[*]}")
  echo "Applying to $label ($dev): $joined"
  sudo v4l2-ctl -d "$dev" --set-ctrl "$joined"
}

targets=()
case "$SENSOR" in
  gc5035)
    targets+=("gc5035:$GC_SUBDEV")
    ;;
  ov5678)
    targets+=("ov5678:$OV_SUBDEV")
    ;;
  all)
    targets+=("gc5035:$GC_SUBDEV" "ov5678:$OV_SUBDEV")
    ;;
  *)
    echo "Invalid --sensor '$SENSOR'" >&2
    exit 2
    ;;
esac

for t in "${targets[@]}"; do
  sensor_name=${t%%:*}
  dev=${t#*:}
  PRESET_HANDLED=0
  [[ -n "$PRESET" ]] && apply_preset "$sensor_name" "$PRESET"
  apply_controls "$dev" "$sensor_name"
done
