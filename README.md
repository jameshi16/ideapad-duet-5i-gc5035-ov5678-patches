# Ideapad Duet 5i GC5035 and OV5678 patches

This repository is the export package for both camera sensors, validated to work on the `6.18.9+deb13-amd64` kernel (sorry for the esoteric kernel version...).
You will need the kernel source code extracted somewhere for this to work.

Everything should work with `apply-and-build.sh`: clone/apply/build helper for ipu6-drivers + libcamera + v4l2loopback; this should pull all the relevant sources, patch them, and install them.
It should also work for secure boot systems.

## Quick start

Run from `export/`:

```bash
./apply-and-build.sh --workdir /tmp/ipu6-export-work --skip-apt
```

Without `--skip-apt`, the script runs `apt-get update` and installs common build/runtime dependencies.

## Script options

- `--workdir <dir>`: root directory for repo clones/builds.
- `--ipu6-dir <dir>`: use an existing/fixed ipu6-drivers checkout.
- `--libcamera-dir <dir>`: use an existing/fixed libcamera checkout.
- `--v4l2loopback-dir <dir>`: use an existing/fixed v4l2loopback checkout.
- `--skip-apt`: skip package installation.
- `--patch-host-kernel`: opt-in run of `patch-int3472.sh` after builds.

## Notes on host-kernel patches (opt-in)

`patches/host-kernel/` are intentionally separate from the ipu6 and libcamera patch sets.
They modify host kernel sources/modules and are not required for all systems.

Use only if your platform needs the INT3472/bridge adjustments. The helper script:

```bash
sudo ./patch-int3472.sh
```

is copied into the ipu6 tree by `apply-and-build.sh` and can be invoked automatically via `--patch-host-kernel`.

## Services and camera switching

After installation, copy service units and enable:

```bash
sudo cp processed-camera-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable processed-camera-loopback.service
sudo systemctl enable processed-camera-gc5035.service
sudo systemctl enable processed-camera-ov5678.service
```

Switch active sensor pipeline:

```bash
sudo ./select-camera.sh gc5035
sudo ./select-camera.sh ov5678
```

## Raw capture and conversion (Development only)

Capture raw:

```bash
./capture_raw.sh --sensor ov5678 --video /dev/video24 --out capture/ov5678.raw
```

Convert raw with direct RGB-IR defaults:

```bash
python3 ./bayer2png.py capture/ov5678.raw capture/ov5678.png 2592 1944 --sensor ov5678 --rgbir-mode direct
```
