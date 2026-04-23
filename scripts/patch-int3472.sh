#!/bin/bash

set -euo pipefail

KERNEL_SRC_DIR="/usr/src/linux-source-6.18"
INT3472_DIR="${KERNEL_SRC_DIR}/drivers/platform/x86/intel/int3472"
MEDIA_INTEL_DIR="${KERNEL_SRC_DIR}/drivers/media/pci/intel"
KDIR="/lib/modules/$(uname -r)/build"
MOD_DIR="/lib/modules/$(uname -r)/kernel/drivers/platform/x86/intel/int3472"
MEDIA_MOD_DIR="/lib/modules/$(uname -r)/kernel/drivers/media/pci/intel"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ORIG_DISCRETE="${SCRIPT_DIR}/discrete_original.c"
ORIG_QUIRKS="${SCRIPT_DIR}/discrete_quirks_original.c"
ORIG_IPU_BRIDGE="${SCRIPT_DIR}/ipu-bridge.original.c"

MOK_KEY=/var/lib/dkms/mok.key
MOK_PUB=/var/lib/dkms/mok.pub

restore_sources() {
    if [ -e "${ORIG_DISCRETE}" ]; then
        echo "Restoring discrete.c from repository baseline"
        cp "${ORIG_DISCRETE}" "${INT3472_DIR}/discrete.c"
    elif [ -e "${INT3472_DIR}/discrete.c.backup" ]; then
        echo "Restoring discrete.c from backup"
        cp "${INT3472_DIR}/discrete.c.backup" "${INT3472_DIR}/discrete.c"
    fi

    if [ -e "${ORIG_QUIRKS}" ]; then
        echo "Restoring discrete_quirks.c from repository baseline"
        cp "${ORIG_QUIRKS}" "${INT3472_DIR}/discrete_quirks.c"
    elif [ -e "${INT3472_DIR}/discrete_quirks.c.backup" ]; then
        echo "Restoring discrete_quirks.c from backup"
        cp "${INT3472_DIR}/discrete_quirks.c.backup" "${INT3472_DIR}/discrete_quirks.c"
    fi

    if [ -e "${ORIG_IPU_BRIDGE}" ]; then
        echo "Restoring ipu-bridge.c from repository baseline"
        cp "${ORIG_IPU_BRIDGE}" "${MEDIA_INTEL_DIR}/ipu-bridge.c"
    elif [ -e "${MEDIA_INTEL_DIR}/ipu-bridge.c.backup" ]; then
        echo "Restoring ipu-bridge.c from backup"
        cp "${MEDIA_INTEL_DIR}/ipu-bridge.c.backup" "${MEDIA_INTEL_DIR}/ipu-bridge.c"
    fi
}

backup_sources() {
    local f

    for f in discrete.c discrete_quirks.c; do
        if [ ! -e "${INT3472_DIR}/${f}.backup" ]; then
            cp "${INT3472_DIR}/${f}" "${INT3472_DIR}/${f}.backup"
            echo "Backed up ${f}"
        fi
    done

    if [ ! -e "${MEDIA_INTEL_DIR}/ipu-bridge.c.backup" ]; then
        cp "${MEDIA_INTEL_DIR}/ipu-bridge.c" "${MEDIA_INTEL_DIR}/ipu-bridge.c.backup"
        echo "Backed up ipu-bridge.c"
    fi
}

restore_modules() {
    local backups

    shopt -s nullglob
    backups=("${MOD_DIR}"/*.backup)
    if [ ${#backups[@]} -gt 0 ]; then
        echo "Restoring old kernel module backups..."
        for f in "${backups[@]}"; do
            mv "$f" "${f%.backup}"
        done
    fi
    shopt -u nullglob

    shopt -s nullglob
    backups=("${MEDIA_MOD_DIR}"/ipu-bridge.ko.backup "${MEDIA_MOD_DIR}"/ipu-bridge.ko.xz.backup)
    for f in "${backups[@]}"; do
        [ -e "$f" ] || continue
        mv "$f" "${f%.backup}"
    done
    shopt -u nullglob
}

backup_modules() {
    local modules

    shopt -s nullglob
    modules=("${MOD_DIR}"/*.ko "${MOD_DIR}"/*.ko.xz)
    if [ ${#modules[@]} -gt 0 ]; then
        echo "Backing up existing INT3472 modules..."
        for f in "${modules[@]}"; do
            mv "$f" "${f}.backup"
        done
    fi
    shopt -u nullglob

    shopt -s nullglob
    modules=("${MEDIA_MOD_DIR}"/ipu-bridge.ko "${MEDIA_MOD_DIR}"/ipu-bridge.ko.xz)
    for f in "${modules[@]}"; do
        [ -e "$f" ] || continue
        mv "$f" "${f}.backup"
    done
    shopt -u nullglob
}

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit 1
fi

if [ "${1:-}" = "restore" ]; then
    restore_sources
    restore_modules
    depmod -a
    exit 0
fi

restore_sources
restore_modules
backup_sources

echo "Patching discrete.c"
patch --forward --silent "${INT3472_DIR}/discrete.c" < "${SCRIPT_DIR}/int3472-gc5035-skylake.diff"

echo "Patching discrete_quirks.c"
patch --forward --silent "${INT3472_DIR}/discrete_quirks.c" < "${SCRIPT_DIR}/int3472-gc5035-skylake-quirks.diff"

echo "Patching ipu-bridge.c"
if patch --dry-run --forward --silent "${MEDIA_INTEL_DIR}/ipu-bridge.c" < "${SCRIPT_DIR}/ipu-bridge-gc5035.diff"; then
    patch --forward --silent "${MEDIA_INTEL_DIR}/ipu-bridge.c" < "${SCRIPT_DIR}/ipu-bridge-gc5035.diff"
else
    if grep -q 'IPU_SENSOR_CONFIG("GCTI5035"' "${MEDIA_INTEL_DIR}/ipu-bridge.c" && \
       grep -q 'IPU_SENSOR_CONFIG("OVTI5678"' "${MEDIA_INTEL_DIR}/ipu-bridge.c"; then
        echo "ipu-bridge.c already has required sensor entries; skipping patch"
    else
        echo "Failed to patch ipu-bridge.c and required entries are missing"
        exit 1
    fi
fi

echo "Building modules..."
make -C "${KDIR}" M="${INT3472_DIR}" modules
make -C "${KDIR}" M="${MEDIA_INTEL_DIR}" modules

backup_modules

echo "Copying new files..."
cp "${INT3472_DIR}"/*.ko "${MOD_DIR}"
cp "${MEDIA_INTEL_DIR}/ipu-bridge.ko" "${MEDIA_MOD_DIR}"

echo "Signing with MOK..."
for f in "${MOD_DIR}"/*.ko; do
    "${KDIR}/scripts/sign-file" sha256 "${MOK_KEY}" "${MOK_PUB}" "$f"
done
"${KDIR}/scripts/sign-file" sha256 "${MOK_KEY}" "${MOK_PUB}" "${MEDIA_MOD_DIR}/ipu-bridge.ko"

echo "Depmod..."
depmod -a

echo "Reloading modules..."
modprobe -r intel_skl_int3472_tps68470 intel_skl_int3472_discrete intel_skl_int3472_common || true
modprobe intel_skl_int3472_discrete || true
modprobe -r ov5678 gc5035 intel_ipu6_isys intel_ipu6_psys intel_ipu6 ipu_bridge || true
modprobe ipu_bridge || true
modprobe intel_ipu6 || true
modprobe intel_ipu6_isys || true
modprobe gc5035 || true
modprobe ov5678 || true
