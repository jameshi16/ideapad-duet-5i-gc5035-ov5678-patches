#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

IPU6_REPO_URL="https://github.com/intel/ipu6-drivers.git"
LIBCAMERA_REPO_URL="https://github.com/raspberrypi/libcamera.git"
LIBCAMERA_TAG="v0.5.2"
V4L2LOOPBACK_REPO_URL="https://github.com/v4l2loopback/v4l2loopback.git"
IPU6_BASE_COMMIT="51fe72485032c779a261430a8100eaad5d8696b8"

WORKDIR="${PWD}/work"
IPU6_DIR=""
LIBCAMERA_DIR=""
V4L2LOOPBACK_DIR=""
SKIP_APT=0
PATCH_HOST_KERNEL=0

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --workdir <dir>
  --ipu6-dir <dir>
  --libcamera-dir <dir>
  --v4l2loopback-dir <dir>
  --skip-apt
  --patch-host-kernel
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workdir)
      WORKDIR="$2"
      shift 2
      ;;
    --ipu6-dir)
      IPU6_DIR="$2"
      shift 2
      ;;
    --libcamera-dir)
      LIBCAMERA_DIR="$2"
      shift 2
      ;;
    --v4l2loopback-dir)
      V4L2LOOPBACK_DIR="$2"
      shift 2
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --patch-host-kernel)
      PATCH_HOST_KERNEL=1
      shift
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

mkdir -p "$WORKDIR"

if [[ -z "$IPU6_DIR" ]]; then
  IPU6_DIR="${WORKDIR}/ipu6-drivers"
fi
if [[ -z "$LIBCAMERA_DIR" ]]; then
  LIBCAMERA_DIR="${WORKDIR}/libcamera-0.5.2"
fi
if [[ -z "$V4L2LOOPBACK_DIR" ]]; then
  V4L2LOOPBACK_DIR="${WORKDIR}/v4l2loopback"
fi

clone_or_fetch() {
  local repo_url="$1"
  local dir="$2"
  local ref="$3"
  if [[ -d "${dir}/.git" ]]; then
    git -C "$dir" fetch --depth=1 origin "$ref"
  else
    rm -rf "$dir"
    git clone --depth=1 --branch "$ref" "$repo_url" "$dir"
  fi
}

if [[ "$SKIP_APT" -eq 0 ]]; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential \
    git \
    dkms \
    meson \
    ninja-build \
    pkg-config \
    python3 \
    python3-pip \
    python3-yaml \
    python3-jinja2 \
    python3-ply \
    libyaml-dev \
    libgnutls28-dev \
    libevent-dev \
    libdrm-dev \
    libjpeg-dev \
    libtiff-dev \
    libdw-dev \
    libunwind-dev \
    liblttng-ust-dev \
    linux-headers-$(uname -r)
fi

if [[ -d "${IPU6_DIR}/.git" ]]; then
  git -C "$IPU6_DIR" fetch --depth=1 origin "$IPU6_BASE_COMMIT"
else
  git clone --depth=1 "$IPU6_REPO_URL" "$IPU6_DIR"
  git -C "$IPU6_DIR" fetch --depth=1 origin "$IPU6_BASE_COMMIT"
fi
git -C "$IPU6_DIR" checkout -B export-build "$IPU6_BASE_COMMIT"

for patch in "$SCRIPT_DIR"/patches/ipu6-drivers/*.patch; do
  [[ -e "$patch" ]] || continue
  git -C "$IPU6_DIR" apply "$patch"
done

cp "$SCRIPT_DIR"/scripts/*.py "$IPU6_DIR"/
cp "$SCRIPT_DIR"/scripts/*.sh "$IPU6_DIR"/
cp "$SCRIPT_DIR"/scripts/services/*.service "$IPU6_DIR"/
mkdir -p "$IPU6_DIR/ipa-config/simple"
cp "$SCRIPT_DIR"/scripts/tuning/*.yaml "$IPU6_DIR/ipa-config/simple"/

sudo dkms remove -m ipu6-drivers -v 0.0.0 --all || true
sudo dkms add "$IPU6_DIR"
sudo dkms autoinstall -m ipu6-drivers -v 0.0.0
sudo depmod -a

clone_or_fetch "$LIBCAMERA_REPO_URL" "$LIBCAMERA_DIR" "$LIBCAMERA_TAG"
git -C "$LIBCAMERA_DIR" checkout -B export-build "$LIBCAMERA_TAG"
for patch in "$SCRIPT_DIR"/patches/libcamera/*.patch; do
  [[ -e "$patch" ]] || continue
  git -C "$LIBCAMERA_DIR" apply "$patch"
done
meson setup "$LIBCAMERA_DIR/build" "$LIBCAMERA_DIR" --buildtype=release --prefix=/usr/local
ninja -C "$LIBCAMERA_DIR/build"
sudo ninja -C "$LIBCAMERA_DIR/build" install
sudo ldconfig

if [[ -d "${V4L2LOOPBACK_DIR}/.git" ]]; then
  git -C "$V4L2LOOPBACK_DIR" fetch --depth=1 origin master
  git -C "$V4L2LOOPBACK_DIR" checkout -B export-build FETCH_HEAD
else
  git clone --depth=1 "$V4L2LOOPBACK_REPO_URL" "$V4L2LOOPBACK_DIR"
fi
make -C "$V4L2LOOPBACK_DIR" -j"$(nproc)"
sudo make -C "$V4L2LOOPBACK_DIR" install
sudo depmod -a

if [[ "$PATCH_HOST_KERNEL" -eq 1 ]]; then
  echo "Applying host-kernel patch workflow via patch-int3472.sh..."
  sudo "$IPU6_DIR/patch-int3472.sh"
fi

cat <<EOF

Build complete.

Install service units:
  sudo cp "$IPU6_DIR"/processed-camera-*.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable processed-camera-loopback.service
  sudo systemctl enable processed-camera-gc5035.service
  sudo systemctl enable processed-camera-ov5678.service

Select camera:
  sudo "$IPU6_DIR/select-camera.sh" gc5035
  sudo "$IPU6_DIR/select-camera.sh" ov5678
EOF
