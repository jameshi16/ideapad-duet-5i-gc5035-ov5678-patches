#!/usr/bin/env python3
"""Continuous raw-sensor -> processed -> v4l2loopback pipeline.

This script captures raw Bayer frames from a physical V4L2 node, processes
them in Python, and publishes corrected frames to a loopback camera node.

Default output is 1280x960 for app compatibility and reduced CPU usage.
"""

import argparse
import ctypes
import fcntl
import logging
import mmap
import os
import select
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass

import cv2
import gi
import numpy as np

from ov5678_rgbir import (
    OV5678RgbirSettings,
    parse_ir_cut,
    process_ov5678_rgbir,
    resolve_rgbir_pattern,
    rgbir_pattern_names,
)

gi.require_version("Gst", "1.0")
from gi.repository import Gst


LOG = logging.getLogger("camera-pipeline")
STOP = False


@dataclass
class SensorProfile:
    sensor: str
    black_level: int
    white_level: int
    default_bgr_gains: tuple[float, float, float]
    demosaic_code: int
    rgbir: bool
    cfa_period: int = 2
    default_adaptive_low_percentile: float = 1.0
    default_adaptive_high_percentile: float = 99.5
    default_adaptive_min_span: float = 48.0
    default_chroma_filter_sigma: float = 0.0
    default_luma_filter_sigma: float = 0.0
    default_rgbir_mode: str = "legacy"
    default_rgbir_gamma: float = 1.0
    default_rgbir_interp_sigma: float = 1.35
    default_ir_cut: str | float = "auto"
    default_ccm_profile: str = "none"
    default_awb_mode: str = "grayworld"
    default_awb_strength: float = 0.75
    default_awb_ema: float = 0.20
    default_color_saturation: float | None = None
    default_rgbir_pattern: str = "openrgbir"
    default_legacy_rgbir_pattern: str = "openrgbir"
    default_rgbir_pattern_y_offset: int = 0
    default_rgbir_pattern_x_offset: int = 0
    default_temporal_alpha: float = 0.0
    default_pink_fix_strength: float = 0.0


SENSOR_PROFILES = {
    "gc5035": SensorProfile(
        sensor="gc5035",
        black_level=64,
        white_level=1023,
        default_bgr_gains=(1.60, 0.72, 1.02),
        demosaic_code=cv2.COLOR_BayerGB2BGR,  # SGRBG10 = GRBG; BayerGB maps this correctly
        rgbir=False,
        cfa_period=2,
    ),
    "ov5678": SensorProfile(
        sensor="ov5678",
        black_level=64,
        white_level=1023,
        # Phase-calibrated pre-gains for windows-cjfl515 pattern (G=79, R=68.8, B=46.3 means).
        # Grayworld AWB makes per-frame fine corrections on top of these.
        default_bgr_gains=(1.70, 1.0, 1.15),
        demosaic_code=cv2.COLOR_BayerBG2BGR,
        rgbir=True,
        cfa_period=4,  # 4x4 RGB-IR pattern; downscale must preserve 4x4 period
        default_adaptive_low_percentile=0.5,
        default_adaptive_high_percentile=88.0,
        default_adaptive_min_span=12.0,
        default_chroma_filter_sigma=3.8,
        default_luma_filter_sigma=0.85,
        default_rgbir_mode="direct",
        default_rgbir_gamma=0.70,
        default_rgbir_interp_sigma=1.50,
        default_ir_cut=0.04,
        default_ccm_profile="none",
        default_awb_mode="grayworld",
        default_awb_strength=0.85,
        default_awb_ema=0.20,
        default_color_saturation=None,
        default_rgbir_pattern="windows-cjfl515",
        default_legacy_rgbir_pattern="windows-cjfl515",
        default_temporal_alpha=0.0,
        default_pink_fix_strength=0.0,
    ),
}


def on_signal(_signum, _frame):
    global STOP
    STOP = True


def setup_signals():
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)


def flow_name(flow):
    if hasattr(flow, "value_nick"):
        return flow.value_nick
    return str(flow)


V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_WRITE = 1
_IOC_READ = 2


class timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class v4l2_timecode(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("frames", ctypes.c_uint8),
        ("seconds", ctypes.c_uint8),
        ("minutes", ctypes.c_uint8),
        ("hours", ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]


class v4l2_requestbuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class v4l2_buffer_m(ctypes.Union):
    _fields_ = [
        ("offset", ctypes.c_uint32),
        ("userptr", ctypes.c_ulong),
        ("planes", ctypes.c_void_p),
        ("fd", ctypes.c_int32),
    ]


class v4l2_buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp", timeval),
        ("timecode", v4l2_timecode),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", v4l2_buffer_m),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


def _IOC(direction, type_, nr, size):
    return (
        (direction << _IOC_DIRSHIFT)
        | (ord(type_) << _IOC_TYPESHIFT)
        | (nr << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def _IOW(type_, nr, struct_type):
    return _IOC(_IOC_WRITE, type_, nr, ctypes.sizeof(struct_type))


def _IOWR(type_, nr, struct_type):
    return _IOC(_IOC_READ | _IOC_WRITE, type_, nr, ctypes.sizeof(struct_type))


VIDIOC_REQBUFS = _IOWR("V", 8, v4l2_requestbuffers)
VIDIOC_QUERYBUF = _IOWR("V", 9, v4l2_buffer)
VIDIOC_QBUF = _IOWR("V", 15, v4l2_buffer)
VIDIOC_DQBUF = _IOWR("V", 17, v4l2_buffer)
VIDIOC_STREAMON = _IOW("V", 18, ctypes.c_int)
VIDIOC_STREAMOFF = _IOW("V", 19, ctypes.c_int)

_I2C_SLAVE_FORCE = 0x0706
_OV5678_I2C_BUS = 3
_OV5678_I2C_ADDR = 0x36


def _ov5678_freeze_aec(settle_seconds: float = 1.5) -> None:
    """Disable OV5678 AEC/AGC after streaming starts to stop exposure hunting.

    Waits settle_seconds for AEC to converge on a scene-appropriate exposure,
    then writes 0x3503=0x04 (manual exposure + auto gain) via /dev/i2c-3.
    Without this, 0x3503=0x00 (Patch 3) keeps AEC running indefinitely, causing
    periodic brightness oscillations that are visible as pulsing.
    """
    bus_dev = f"/dev/i2c-{_OV5678_I2C_BUS}"
    if not os.path.exists(bus_dev):
        try:
            subprocess.run(["modprobe", "i2c-dev"], check=True, capture_output=True)
        except Exception as exc:
            LOG.warning("OV5678: modprobe i2c-dev failed: %s", exc)

    if settle_seconds > 0:
        LOG.info("OV5678: waiting %.1fs for AEC to settle before freezing", settle_seconds)
        time.sleep(settle_seconds)

    try:
        fd = os.open(bus_dev, os.O_RDWR)
        try:
            fcntl.ioctl(fd, _I2C_SLAVE_FORCE, _OV5678_I2C_ADDR)
            # 16-bit register address 0x3503, value 0x04: manual AEC (bit[2]),
            # auto AGC (bit[3]=0). This is the pre-Patch-3 manual-exposure value.
            os.write(fd, struct.pack("BBB", 0x35, 0x03, 0x04))
        finally:
            os.close(fd)
        LOG.info("OV5678: AEC frozen (0x3503=0x04, i2c-%d@0x%02x)",
                 _OV5678_I2C_BUS, _OV5678_I2C_ADDR)
    except Exception as exc:
        LOG.warning("OV5678: AEC freeze via i2c failed: %s", exc)


class RawCaptureStream:
    """Minimal mmap-based V4L2 capture for the IPU6 raw video nodes."""

    def __init__(self, device, buffer_count=4, timeout_ms=2000):
        self.device = device
        self.buffer_count = buffer_count
        self.timeout_ms = timeout_ms
        self.fd = None
        self.maps = []
        self.stream_type = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)

    def open(self):
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)

        req = v4l2_requestbuffers(
            count=self.buffer_count,
            type=V4L2_BUF_TYPE_VIDEO_CAPTURE,
            memory=V4L2_MEMORY_MMAP,
        )
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
        if req.count < 2:
            raise RuntimeError(
                f"{self.device}: insufficient V4L2 buffers ({req.count})"
            )

        for index in range(req.count):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = index
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, buf)
            mm = mmap.mmap(
                self.fd,
                buf.length,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=buf.m.offset,
            )
            self.maps.append(mm)
            fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)

        fcntl.ioctl(self.fd, VIDIOC_STREAMON, self.stream_type)

    def read_frame(self):
        if self.fd is None:
            raise RuntimeError("capture stream is not open")

        timeout_s = max(0.001, self.timeout_ms / 1000.0)
        ready, _, _ = select.select([self.fd], [], [], timeout_s)
        if not ready:
            return None

        # Drain all queued buffers and return only the most-recent frame to
        # avoid falling progressively behind when processing is slower than the
        # sensor rate.
        last_data = None
        while True:
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            try:
                fcntl.ioctl(self.fd, VIDIOC_DQBUF, buf)
            except BlockingIOError:
                break
            last_data = bytes(self.maps[buf.index][: buf.bytesused])
            fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)
        return last_data

    def close(self):
        if self.fd is None:
            return

        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, self.stream_type)
        except OSError:
            pass

        for mm in self.maps:
            try:
                mm.close()
            except OSError:
                pass
        self.maps.clear()

        try:
            req = v4l2_requestbuffers(
                count=0,
                type=V4L2_BUF_TYPE_VIDEO_CAPTURE,
                memory=V4L2_MEMORY_MMAP,
            )
            fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
        except OSError:
            pass

        os.close(self.fd)
        self.fd = None


def build_output_pipeline(device, width, height, out_fps):
    return Gst.parse_launch(
        " ! ".join(
            [
                (
                    "appsrc name=rgbsrc is-live=true block=false format=time do-timestamp=false "
                    f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={out_fps}/1"
                ),
                "queue max-size-buffers=2 leaky=downstream",
                "videoconvert",
                (
                    "video/x-raw,"
                    f"format=YUY2,width={width},height={height},framerate={out_fps}/1"
                ),
                f"v4l2sink device={device} sync=false",
            ]
        )
    )


def poll_bus_messages(pipeline, name):
    fatal = False
    bus = pipeline.get_bus()
    while True:
        msg = bus.pop_filtered(
            Gst.MessageType.ERROR | Gst.MessageType.WARNING | Gst.MessageType.EOS
        )
        if not msg:
            break

        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            LOG.error("%s pipeline error: %s (%s)", name, err, dbg)
            fatal = True
        elif msg.type == Gst.MessageType.WARNING:
            warn, dbg = msg.parse_warning()
            LOG.warning("%s pipeline warning: %s (%s)", name, warn, dbg)
        else:
            LOG.warning("%s pipeline EOS", name)
            fatal = True
    return fatal


def unpack_raw10_packed(raw_u8, width, height):
    groups = raw_u8.reshape(-1, 5).astype(np.uint16)
    out = np.empty((groups.shape[0], 4), dtype=np.uint16)
    out[:, 0] = (groups[:, 0] << 2) | ((groups[:, 4] >> 0) & 0x03)
    out[:, 1] = (groups[:, 1] << 2) | ((groups[:, 4] >> 2) & 0x03)
    out[:, 2] = (groups[:, 2] << 2) | ((groups[:, 4] >> 4) & 0x03)
    out[:, 3] = (groups[:, 3] << 2) | ((groups[:, 4] >> 6) & 0x03)
    return out.reshape(height, width)


def decode_raw_frame(raw_bytes, width, height):
    raw_u8 = np.frombuffer(raw_bytes, dtype=np.uint8)
    expected_16 = width * height * 2
    expected_packed10 = (width * height * 5) // 4

    if raw_u8.size == expected_16:
        return raw_u8.view("<u2").reshape(height, width)

    if raw_u8.size == expected_packed10:
        return unpack_raw10_packed(raw_u8, width, height)

    # Handle possible per-line stride for 16-bit Bayer.
    if raw_u8.size > expected_16 and raw_u8.size % height == 0:
        stride_bytes = raw_u8.size // height
        if stride_bytes >= width * 2 and stride_bytes % 2 == 0:
            rows = raw_u8.reshape(height, stride_bytes)
            active = rows[:, : width * 2].reshape(-1)
            return active.view("<u2").reshape(height, width)

    raise ValueError(
        "Unsupported raw frame size: "
        f"{raw_u8.size} bytes (expected {expected_16} or {expected_packed10})"
    )


def normalize_to_u8(raw16, profile, args, norm_state):
    rawf = raw16.astype(np.float32) - float(profile.black_level)
    np.maximum(rawf, 0.0, out=rawf)

    if args.normalize_mode == "adaptive":
        step = max(1, args.adaptive_sample_step)
        sample = rawf[::step, ::step]
        low_percentile = (
            profile.default_adaptive_low_percentile
            if args.adaptive_low_percentile is None
            else args.adaptive_low_percentile
        )
        high_percentile = (
            profile.default_adaptive_high_percentile
            if args.adaptive_high_percentile is None
            else args.adaptive_high_percentile
        )
        low_now = float(np.percentile(sample, low_percentile))
        high_now = float(np.percentile(sample, high_percentile))

        if high_now - low_now < args.adaptive_min_span:
            center = 0.5 * (high_now + low_now)
            low_now = max(0.0, center - args.adaptive_min_span * 0.5)
            high_now = low_now + args.adaptive_min_span

        if norm_state["low"] is None or norm_state["high"] is None:
            norm_state["low"] = low_now
            norm_state["high"] = high_now
        else:
            ema = args.adaptive_ema
            norm_state["low"] = norm_state["low"] * (1.0 - ema) + low_now * ema
            norm_state["high"] = norm_state["high"] * (1.0 - ema) + high_now * ema

        low = norm_state["low"]
        high = norm_state["high"]
    else:
        low = 0.0
        high = max(1.0, float(profile.white_level - profile.black_level))

    rawf -= low
    rawf *= 255.0 / max(1.0, high - low)
    np.clip(rawf, 0.0, 255.0, out=rawf)
    return rawf.astype(np.uint8), float(low), float(high)


def apply_bgr_gains(frame_bgr, bgr_gains):
    gain = np.array(bgr_gains, dtype=np.float32).reshape((1, 1, 3))
    out = frame_bgr.astype(np.float32) * gain
    np.clip(out, 0.0, 255.0, out=out)
    return out.astype(np.uint8)


def filter_grid_artifacts(frame_bgr, chroma_sigma, luma_sigma):
    if chroma_sigma <= 0.0 and luma_sigma <= 0.0:
        return frame_bgr

    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)
    if luma_sigma > 0.0:
        y = cv2.GaussianBlur(y, (0, 0), luma_sigma)
    if chroma_sigma > 0.0:
        cr = cv2.GaussianBlur(cr, (0, 0), chroma_sigma)
        cb = cv2.GaussianBlur(cb, (0, 0), chroma_sigma)
    return cv2.cvtColor(cv2.merge((y, cr, cb)), cv2.COLOR_YCrCb2BGR)


def limit_luma(frame_bgr, target_luma):
    if target_luma is None or target_luma <= 0.0:
        return frame_bgr
    current = estimate_luma_mean_u8(frame_bgr)
    if current <= target_luma:
        return frame_bgr
    out = frame_bgr.astype(np.float32) * (target_luma / max(current, 1e-6))
    return np.clip(out, 0.0, 255.0).astype(np.uint8)


def match_luma_bounded(frame_bgr, target_luma, max_boost=1.3):
    if target_luma is None or target_luma <= 0.0:
        return frame_bgr
    current = estimate_luma_mean_u8(frame_bgr)
    if current <= 1e-6:
        return frame_bgr
    scale = target_luma / current
    if scale > 1.0:
        highlight = float(np.percentile(frame_bgr, 99.5))
        highlight_cap = 248.0 / max(highlight, 1.0)
        scale = min(scale, max_boost, highlight_cap)
    out = frame_bgr.astype(np.float32) * scale
    return np.clip(out, 0.0, 255.0).astype(np.uint8)


def apply_pink_fix(frame_bgr, args):
    strength = args.pink_fix_strength
    if strength <= 0.0:
        return frame_bgr

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(np.float32)
    s = hsv[:, :, 1].astype(np.float32)
    v = hsv[:, :, 2].astype(np.float32)

    if args.pink_fix_hue_min <= args.pink_fix_hue_max:
        hue_mask = (
            (h >= float(args.pink_fix_hue_min))
            & (h <= float(args.pink_fix_hue_max))
        )
    else:
        hue_mask = (
            (h >= float(args.pink_fix_hue_min))
            | (h <= float(args.pink_fix_hue_max))
        )
    sat_mask = s >= float(args.pink_fix_s_min)
    val_mask = (v >= float(args.pink_fix_v_min)) & (v <= float(args.pink_fix_v_max))

    mask = (hue_mask & sat_mask & val_mask).astype(np.float32)
    if float(mask.mean()) <= 0.0:
        return frame_bgr

    mask = cv2.GaussianBlur(mask, (0, 0), 2.0)
    mask *= strength
    np.clip(mask, 0.0, 1.0, out=mask)

    h = np.mod(h + (float(args.pink_fix_hue_shift) * mask), 180.0)
    sat_gain = 1.0 + ((float(args.pink_fix_sat_scale) - 1.0) * mask)
    s *= sat_gain
    np.clip(s, 0.0, 255.0, out=s)

    hsv[:, :, 0] = h.astype(np.uint8)
    hsv[:, :, 1] = s.astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def apply_temporal_denoise(frame_bgr, state, alpha, reset_threshold):
    if alpha <= 0.0:
        return frame_bgr

    current = frame_bgr.astype(np.float32)
    current_luma = estimate_luma_mean_u8(frame_bgr)
    prev_luma = state.get("luma")
    prev_frame = state.get("frame")

    if (
        prev_frame is None
        or prev_luma is None
        or abs(current_luma - prev_luma) > reset_threshold
    ):
        state["frame"] = current
    else:
        state["frame"] = (current * alpha) + (prev_frame * (1.0 - alpha))

    state["luma"] = current_luma
    return np.clip(state["frame"], 0.0, 255.0).astype(np.uint8)


LEGACY_RGBIR_PATTERN_NAMES = ("openrgbir", "windows-cjfl515")

LEGACY_BAYER_CODES = {
    ("B", "G", "G", "R"): cv2.COLOR_BayerBG2BGR,
    ("G", "B", "R", "G"): cv2.COLOR_BayerGB2BGR,
    ("G", "R", "B", "G"): cv2.COLOR_BayerGR2BGR,
    ("R", "G", "G", "B"): cv2.COLOR_BayerRG2BGR,
}


def _legacy_rgbir_phase(pattern_name):
    pattern = resolve_rgbir_pattern(pattern_name)
    phase = np.empty((4, 4), dtype=object)
    for channel, coords in pattern.items():
        for y, x in coords:
            phase[y, x] = channel
    return pattern, phase


def _legacy_demosaic_code(phase):
    converted = phase.copy()
    ir_mask = phase == "IR"
    red_mask = phase == "R"
    converted[ir_mask] = "R"
    converted[red_mask] = "B"
    key = (
        converted[0, 0],
        converted[0, 1],
        converted[1, 0],
        converted[1, 1],
    )
    if key not in LEGACY_BAYER_CODES:
        raise ValueError(f"unsupported legacy OV5678 Bayer phase after remap: {key}")
    return LEGACY_BAYER_CODES[key]


def ov5678_rgbir_to_bggr(raw16, ir_cut, ir_clip, legacy_pattern):
    if legacy_pattern not in LEGACY_RGBIR_PATTERN_NAMES:
        raise ValueError(f"unsupported --legacy-rgbir-pattern: {legacy_pattern}")

    pattern, phase = _legacy_rgbir_phase(legacy_pattern)
    bayer = raw16.astype(np.float32)

    ir_coords = pattern["IR"]
    ir_y_parity = {y % 2 for y, _x in ir_coords}
    ir_x_parity = {x % 2 for _y, x in ir_coords}
    if len(ir_y_parity) != 1 or len(ir_x_parity) != 1:
        raise ValueError("legacy OV5678 RGB-IR pattern has unsupported IR parity layout")
    ir_y = next(iter(ir_y_parity))
    ir_x = next(iter(ir_x_parity))
    ir_half = bayer[ir_y::2, ir_x::2]

    if ir_cut > 0.0:
        ir_full = np.repeat(np.repeat(ir_half, 2, axis=0), 2, axis=1)
        clip_max = float(np.max(bayer)) / max(ir_clip, 1e-6)
        np.minimum(ir_full, clip_max, out=ir_full)
        bayer -= ir_full * ir_cut
        np.maximum(bayer, 0.0, out=bayer)

    # Convert selected 4x4 RGB-IR geometry into a regular Bayer plane by:
    # 1) estimating red at IR sites from red-leaning diagonals
    # 2) converting original red sites into interpolated blue
    pad = np.pad(bayer, 1, mode="edge")
    l_oblique = (pad[:-2, :-2] + pad[2:, 2:]) * 0.5
    r_oblique = (pad[2:, :-2] + pad[:-2, 2:]) * 0.5

    ir_mask = np.zeros_like(bayer, dtype=bool)
    ir_use_r = np.zeros_like(bayer, dtype=bool)
    for y, x in ir_coords:
        ir_mask[y::4, x::4] = True
        l_red = int(phase[(y - 1) % 4, (x - 1) % 4] == "R") + int(
            phase[(y + 1) % 4, (x + 1) % 4] == "R"
        )
        r_red = int(phase[(y - 1) % 4, (x + 1) % 4] == "R") + int(
            phase[(y + 1) % 4, (x - 1) % 4] == "R"
        )
        if r_red >= l_red:
            ir_use_r[y::4, x::4] = True

    ir_reconstructed = np.where(ir_use_r, r_oblique, l_oblique)
    bayer[ir_mask] = ir_reconstructed[ir_mask]

    # Reconstruct extra blue sites at the original RGB-IR red positions.
    p2 = np.pad(bayer, 2, mode="edge")
    cross = (p2[2:-2, :-4] + p2[2:-2, 4:] + p2[:-4, 2:-2] + p2[4:, 2:-2]) * 0.25
    red_mask = np.zeros_like(bayer, dtype=bool)
    for y, x in pattern["R"]:
        red_mask[y::4, x::4] = True
    bayer[red_mask] = cross[red_mask]

    return bayer.astype(np.uint16), _legacy_demosaic_code(phase)


def process_frame(raw16, profile, bgr_gains, ir_cut, ir_clip, args, norm_state):
    demosaic_code = profile.demosaic_code
    if profile.rgbir and args.rgbir_mode == "direct":
        high_percentile = (
            profile.default_adaptive_high_percentile
            if args.adaptive_high_percentile is None
            else args.adaptive_high_percentile
        )
        low_percentile = (
            profile.default_adaptive_low_percentile
            if args.adaptive_low_percentile is None
            else args.adaptive_low_percentile
        )
        settings = OV5678RgbirSettings(
            black_level=profile.black_level,
            white_level=profile.white_level,
            ir_cut=ir_cut,
            ir_clip=ir_clip,
            grid_correct=(args.raw_grid_correct == "on"),
            interp_sigma=args.rgbir_interp_sigma,
            gamma=args.rgbir_gamma,
            normalize_mode=args.normalize_mode,
            adaptive_low_percentile=low_percentile,
            adaptive_high_percentile=high_percentile,
            adaptive_sample_step=args.adaptive_sample_step,
            adaptive_ema=args.adaptive_ema,
            adaptive_min_span=(
                profile.default_adaptive_min_span
                if args.adaptive_min_span is None
                else args.adaptive_min_span
            ),
            ccm_profile=args.ccm_profile,
            awb_mode=args.awb_mode,
            awb_strength=args.awb_strength,
            awb_ema=args.awb_ema,
            color_saturation=args.color_saturation,
            rgbir_pattern=args.rgbir_pattern,
            pattern_y_offset=args.rgbir_pattern_y_offset,
            pattern_x_offset=args.rgbir_pattern_x_offset,
            pattern_flip_h=args.rgbir_pattern_flip_h,
            pattern_flip_v=args.rgbir_pattern_flip_v,
            dump_intermediates=args.dump_intermediates,
        )
        bgr, norm_low, norm_high, _ir = process_ov5678_rgbir(
            raw16,
            bgr_gains=bgr_gains,
            settings=settings,
            norm_state=norm_state,
        )
        chroma_sigma = (
            profile.default_chroma_filter_sigma
            if args.chroma_filter_sigma is None
            else args.chroma_filter_sigma
        )
        luma_sigma = (
            profile.default_luma_filter_sigma
            if args.luma_filter_sigma is None
            else args.luma_filter_sigma
        )
        bgr = filter_grid_artifacts(bgr, chroma_sigma, luma_sigma)
        return limit_luma(bgr, args.target_luma), norm_low, norm_high

    if profile.rgbir:
        legacy_ir_cut, _auto_ir = parse_ir_cut(ir_cut)
        raw16, demosaic_code = ov5678_rgbir_to_bggr(
            raw16,
            ir_cut=legacy_ir_cut,
            ir_clip=ir_clip,
            legacy_pattern=args.legacy_rgbir_pattern,
        )

    raw8, norm_low, norm_high = normalize_to_u8(
        raw16,
        profile=profile,
        args=args,
        norm_state=norm_state,
    )
    bgr = cv2.cvtColor(raw8, demosaic_code)
    bgr = apply_bgr_gains(bgr, bgr_gains)
    chroma_sigma = (
        profile.default_chroma_filter_sigma
        if args.chroma_filter_sigma is None
        else args.chroma_filter_sigma
    )
    luma_sigma = (
        profile.default_luma_filter_sigma
        if args.luma_filter_sigma is None
        else args.luma_filter_sigma
    )
    bgr = filter_grid_artifacts(bgr, chroma_sigma, luma_sigma)
    if profile.rgbir:
        bgr = apply_pink_fix(bgr, args)
        bgr = match_luma_bounded(bgr, args.target_luma)
    return bgr, norm_low, norm_high


def downscale_raw2x(raw16, cfa_period=2):
    """2x downscale preserving CFA phase.

    For cfa_period=2 (standard Bayer): averages the 4 same-channel pixels
    from the 4x4 input super-block into each output pixel.
    For cfa_period=4 (4x4 RGB-IR): averages from 8x8 input super-blocks.

    The previous implementation averaged all 4 cells in a 2x2 Bayer block
    together (G+R+B+G)/4, which destroys color information.
    """
    block = cfa_period * 2
    h = (raw16.shape[0] // block) * block
    w = (raw16.shape[1] // block) * block
    arr = raw16[:h, :w].astype(np.uint32)
    out = np.empty((h // 2, w // 2), dtype=np.uint16)
    for di in range(cfa_period):
        for dj in range(cfa_period):
            out[di::cfa_period, dj::cfa_period] = (
                arr[di::block, dj::block]
                + arr[di::block, dj + cfa_period :: block]
                + arr[di + cfa_period :: block, dj::block]
                + arr[di + cfa_period :: block, dj + cfa_period :: block]
            ) >> 2
    return out


def estimate_luma_mean_u8(frame_bgr):
    b = frame_bgr[:, :, 0].astype(np.uint16)
    g = frame_bgr[:, :, 1].astype(np.uint16)
    r = frame_bgr[:, :, 2].astype(np.uint16)
    y = ((29 * b) + (150 * g) + (77 * r)) >> 8
    return float(y.mean())


def make_buffer_from_frame(frame, pts_ns, dur_ns):
    if not frame.flags["C_CONTIGUOUS"]:
        frame = np.ascontiguousarray(frame)

    payload = frame.tobytes()
    gst_buf = Gst.Buffer.new_allocate(None, len(payload), None)
    gst_buf.fill(0, payload)
    gst_buf.pts = pts_ns
    gst_buf.duration = dur_ns
    return gst_buf


def run_pipeline(args):
    profile = SENSOR_PROFILES[args.sensor]
    bgr_gains = (
        args.b_gain if args.b_gain is not None else profile.default_bgr_gains[0],
        args.g_gain if args.g_gain is not None else profile.default_bgr_gains[1],
        args.r_gain if args.r_gain is not None else profile.default_bgr_gains[2],
    )

    out_pipe = build_output_pipeline(
        device=args.loopback,
        width=args.out_width,
        height=args.out_height,
        out_fps=args.out_fps,
    )

    appsrc = out_pipe.get_by_name("rgbsrc")
    if appsrc is None:
        raise RuntimeError("Failed to build output GStreamer pipeline")

    if out_pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        raise RuntimeError("Output pipeline failed to start")

    capture = RawCaptureStream(
        args.input,
        buffer_count=args.capture_buffers,
        timeout_ms=args.pull_timeout_ms,
    )
    try:
        capture.open()
    except Exception:
        out_pipe.set_state(Gst.State.NULL)
        raise

    do_freeze_aec = args.freeze_aec if args.freeze_aec is not None else (args.sensor == "ov5678")
    if do_freeze_aec:
        _ov5678_freeze_aec(settle_seconds=args.freeze_aec_delay)

    LOG.info(
        "Running: sensor=%s input=%s loopback=%s in=%dx%d out=%dx%d out-fps=%d",
        args.sensor,
        args.input,
        args.loopback,
        args.in_width,
        args.in_height,
        args.out_width,
        args.out_height,
        args.out_fps,
    )
    LOG.info(
        "Gains(B,G,R)=%.3f,%.3f,%.3f rgbir-mode=%s legacy-pattern=%s ir_cut=%s ir_clip=%.3f temporal_alpha=%.2f reset=%.1f pink_fix=%.2f",
        bgr_gains[0],
        bgr_gains[1],
        bgr_gains[2],
        args.rgbir_mode,
        args.legacy_rgbir_pattern,
        args.ir_cut,
        args.ir_clip,
        args.temporal_alpha,
        args.temporal_reset_threshold,
        args.pink_fix_strength,
    )

    frame_count = 0
    dropped = 0
    dark_run = 0
    consecutive_timeouts = 0
    start_ts = time.time()
    pipeline_start_ns = time.monotonic_ns()
    frame_duration_ns = int(1_000_000_000 / max(args.out_fps, 1))
    norm_state = {"low": None, "high": None}
    temporal_state = {"frame": None, "luma": None}
    raw_pair_buf = None  # holds first raw16 of an in-progress pair
    parity_means = [None, None]  # running EMA of processed luma per even/odd parity bucket

    try:
        while not STOP:
            if poll_bus_messages(out_pipe, "output"):
                raise RuntimeError("output pipeline entered fatal state")

            raw_bytes = capture.read_frame()
            if raw_bytes is None:
                consecutive_timeouts += 1
                if consecutive_timeouts >= args.max_consecutive_timeouts:
                    raise RuntimeError(
                        f"capture stream timed out after {consecutive_timeouts} consecutive misses"
                        " — forcing restart"
                    )
                continue
            consecutive_timeouts = 0

            try:
                raw16 = decode_raw_frame(raw_bytes, args.in_width, args.in_height)
            except ValueError as exc:
                dropped += 1
                LOG.warning("discarding malformed raw frame: %s", exc)
                continue

            if args.pair_average:
                if raw_pair_buf is None:
                    raw_pair_buf = raw16
                    continue  # wait for the second frame of this pair
                raw16 = (
                    (raw_pair_buf.astype(np.uint32) + raw16.astype(np.uint32)) >> 1
                ).astype(np.uint16)
                raw_pair_buf = None

            if args.downscale >= 2:
                raw16 = downscale_raw2x(raw16, cfa_period=profile.cfa_period)
            if args.downscale >= 4:
                raw16 = downscale_raw2x(raw16, cfa_period=profile.cfa_period)

            bgr, norm_low, norm_high = process_frame(
                raw16,
                profile=profile,
                bgr_gains=bgr_gains,
                ir_cut=args.ir_cut,
                ir_clip=args.ir_clip,
                args=args,
                norm_state=norm_state,
            )

            if (bgr.shape[1], bgr.shape[0]) != (args.out_width, args.out_height):
                bgr = cv2.resize(
                    bgr,
                    (args.out_width, args.out_height),
                    interpolation=cv2.INTER_AREA,
                )

            if args.parity_correct:
                cur_luma = float(estimate_luma_mean_u8(bgr))
                alpha = args.parity_ema_alpha
                if parity_means[0] is None:
                    parity_means[0] = cur_luma
                elif parity_means[1] is None:
                    if abs(cur_luma - parity_means[0]) > 5.0:
                        parity_means[1] = cur_luma
                    else:
                        parity_means[0] = parity_means[0] * (1.0 - alpha) + cur_luma * alpha
                else:
                    p = 0 if abs(cur_luma - parity_means[0]) <= abs(cur_luma - parity_means[1]) else 1
                    parity_means[p] = parity_means[p] * (1.0 - alpha) + cur_luma * alpha
                    sep = abs(parity_means[0] - parity_means[1])
                    if sep > 5.0 and parity_means[p] > 1.0:
                        target = (parity_means[0] + parity_means[1]) * 0.5
                        gain = float(np.clip(target / parity_means[p], 0.5, 2.0))
                        bgr = np.clip(bgr.astype(np.float32) * gain, 0, 255).astype(np.uint8)

            bgr = apply_temporal_denoise(
                bgr,
                state=temporal_state,
                alpha=args.temporal_alpha,
                reset_threshold=args.temporal_reset_threshold,
            )

            pts_ns = time.monotonic_ns() - pipeline_start_ns
            gst_out = make_buffer_from_frame(bgr, pts_ns, frame_duration_ns)
            flow = appsrc.emit("push-buffer", gst_out)
            if flow != Gst.FlowReturn.OK:
                dropped += 1
                LOG.warning("push-buffer returned %s", flow_name(flow))
            else:
                frame_count += 1

            luma_mean = estimate_luma_mean_u8(bgr)
            if luma_mean < args.dark_luma_threshold:
                dark_run += 1
            else:
                dark_run = 0

            if (
                args.dark_frame_window > 0
                and dark_run >= args.dark_frame_window
                and dark_run % args.dark_frame_window == 0
            ):
                LOG.warning(
                    "Output appears dark for %d frames (luma=%.2f<th=%.2f, norm=[%.1f, %.1f])",
                    dark_run,
                    luma_mean,
                    args.dark_luma_threshold,
                    norm_low,
                    norm_high,
                )

            if args.max_frames > 0 and frame_count >= args.max_frames:
                LOG.info("Reached max-frames=%d", args.max_frames)
                break

            if (
                args.log_every > 0
                and frame_count > 0
                and frame_count % args.log_every == 0
            ):
                elapsed = max(1e-6, time.time() - start_ts)
                fps = frame_count / elapsed
                LOG.info(
                    "Processed=%d dropped=%d avg-fps=%.2f",
                    frame_count,
                    dropped,
                    fps,
                )
    finally:
        try:
            appsrc.emit("end-of-stream")
        except Exception:
            pass
        capture.close()
        out_pipe.set_state(Gst.State.NULL)

    elapsed = max(1e-6, time.time() - start_ts)
    LOG.info(
        "Stopped: processed=%d dropped=%d avg-fps=%.2f",
        frame_count,
        dropped,
        frame_count / elapsed,
    )


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sensor", choices=sorted(SENSOR_PROFILES), required=True)
    p.add_argument(
        "--input", required=True, help="Physical raw capture node, e.g. /dev/video8"
    )
    p.add_argument(
        "--loopback", required=True, help="Loopback output node, e.g. /dev/video42"
    )

    p.add_argument("--in-width", type=int, default=2592)
    p.add_argument("--in-height", type=int, default=1944)
    p.add_argument("--capture-fps", type=int, default=30)
    p.add_argument(
        "--downscale",
        type=int,
        choices=(1, 2, 4),
        default=1,
        help="Raw-domain downscale factor before demosaic",
    )

    p.add_argument("--out-width", type=int, default=1280)
    p.add_argument("--out-height", type=int, default=960)
    p.add_argument("--out-fps", type=int, default=15)

    p.add_argument("--r-gain", type=float, default=None)
    p.add_argument("--g-gain", type=float, default=None)
    p.add_argument("--b-gain", type=float, default=None)
    p.add_argument("--ir-cut", default=None)
    p.add_argument("--ir-clip", type=float, default=1.3)
    p.add_argument(
        "--rgbir-mode",
        choices=("direct", "legacy"),
        default=None,
        help="OV5678 RGB-IR reconstruction path; defaults are sensor-specific",
    )
    p.add_argument(
        "--raw-grid-correct",
        choices=("on", "off"),
        default="on",
        help="Equalize same-channel 4x4 raw phases before interpolation",
    )
    p.add_argument(
        "--rgbir-interp-sigma",
        type=float,
        default=None,
        help="Sparse-plane interpolation sigma for direct OV5678 RGB-IR mode",
    )
    p.add_argument(
        "--rgbir-gamma",
        type=float,
        default=None,
        help="Display gamma exponent for direct OV5678 RGB-IR mode",
    )
    p.add_argument(
        "--ccm-profile",
        choices=("none", "ov5678-indoor", "ov5678-cjfl515"),
        default=None,
        help="Color correction matrix profile for direct OV5678 RGB-IR mode",
    )
    p.add_argument(
        "--awb-mode",
        choices=("fixed", "grayworld"),
        default=None,
        help="White balance mode for direct OV5678 RGB-IR mode",
    )
    p.add_argument("--awb-strength", type=float, default=None)
    p.add_argument(
        "--awb-ema",
        type=float,
        default=None,
        help="EMA alpha for grayworld AWB gain smoothing; 0=per-frame, default sensor-specific",
    )
    p.add_argument(
        "--color-saturation",
        type=float,
        default=None,
        help="Linear RGB saturation multiplier; profile default is used when omitted",
    )
    p.add_argument(
        "--rgbir-pattern",
        choices=rgbir_pattern_names(),
        default=None,
        help="OV5678 RGB-IR 4x4 pattern origin/profile",
    )
    p.add_argument(
        "--legacy-rgbir-pattern",
        choices=LEGACY_RGBIR_PATTERN_NAMES,
        default=None,
        help="OV5678 legacy-path 4x4 RGB-IR geometry",
    )
    p.add_argument("--rgbir-pattern-y-offset", type=int, default=None)
    p.add_argument("--rgbir-pattern-x-offset", type=int, default=None)
    p.add_argument("--rgbir-pattern-flip-h", action="store_true")
    p.add_argument("--rgbir-pattern-flip-v", action="store_true")
    p.add_argument(
        "--dump-intermediates",
        default=None,
        help="Directory for one-shot OV5678 intermediate debug images",
    )
    p.add_argument(
        "--chroma-filter-sigma",
        type=float,
        default=None,
        help="Gaussian sigma for chroma-only cleanup; defaults are sensor-specific",
    )
    p.add_argument(
        "--luma-filter-sigma",
        type=float,
        default=None,
        help="Gaussian sigma for luma-grid cleanup; defaults are sensor-specific",
    )

    p.add_argument(
        "--normalize-mode",
        choices=("fixed", "adaptive"),
        default="adaptive",
        help="Raw normalization mode before demosaic",
    )
    p.add_argument(
        "--adaptive-low-percentile",
        type=float,
        default=None,
        help="Adaptive low percentile; defaults are sensor-specific",
    )
    p.add_argument(
        "--adaptive-high-percentile",
        type=float,
        default=None,
        help="Adaptive high percentile; defaults are sensor-specific",
    )
    p.add_argument("--adaptive-sample-step", type=int, default=8)
    p.add_argument("--adaptive-ema", type=float, default=0.12)
    p.add_argument("--adaptive-min-span", type=float, default=None)
    p.add_argument("--target-luma", type=float, default=142.0)
    p.add_argument("--dark-luma-threshold", type=float, default=22.0)
    p.add_argument("--dark-frame-window", type=int, default=45)
    p.add_argument(
        "--pair-average",
        action="store_true",
        default=False,
        help="Average consecutive raw frame pairs before demosaic to cancel even/odd alternation (OV5678 pulsing fix).",
    )
    p.add_argument(
        "--parity-correct",
        action="store_true",
        default=False,
        help=(
            "Normalize even/odd raw frame levels via per-parity EMA gain correction. "
            "Suppresses OV5678 alternation without frame-pair averaging, keeping full framerate."
        ),
    )
    p.add_argument(
        "--parity-ema-alpha",
        type=float,
        default=0.05,
        help="EMA decay for parity mean tracking (0.05 = ~20-frame memory). Lower = slower adaptation.",
    )
    p.add_argument(
        "--temporal-alpha",
        type=float,
        default=None,
        help="Temporal EMA blend factor (0 disables). Defaults are sensor-specific",
    )
    p.add_argument(
        "--temporal-reset-threshold",
        type=float,
        default=18.0,
        help="Reset temporal state when luma changes by more than this amount",
    )
    p.add_argument(
        "--pink-fix-strength",
        type=float,
        default=None,
        help="Strength for OV5678 selective green-to-pink hue correction",
    )
    p.add_argument("--pink-fix-hue-min", type=int, default=42)
    p.add_argument("--pink-fix-hue-max", type=int, default=110)
    p.add_argument("--pink-fix-s-min", type=int, default=28)
    p.add_argument("--pink-fix-v-min", type=int, default=35)
    p.add_argument("--pink-fix-v-max", type=int, default=235)
    p.add_argument("--pink-fix-hue-shift", type=float, default=62.0)
    p.add_argument("--pink-fix-sat-scale", type=float, default=1.22)

    p.add_argument(
        "--freeze-aec",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Freeze OV5678 AEC/AGC via i2c after streaming starts (default: on for ov5678).",
    )
    p.add_argument(
        "--freeze-aec-delay",
        type=float,
        default=1.5,
        help="Seconds to let AEC settle before freezing it (default 1.5).",
    )
    p.add_argument("--pull-timeout-ms", type=int, default=2000)
    p.add_argument("--capture-buffers", type=int, default=4)
    p.add_argument(
        "--max-consecutive-timeouts",
        type=int,
        default=10,
        help="Raise a fatal error after this many consecutive read_frame() timeouts to trigger systemd restart",
    )
    p.add_argument("--log-every", type=int, default=60)
    p.add_argument("--max-frames", type=int, default=0, help="0 means run forever")
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    profile = SENSOR_PROFILES[args.sensor]
    if args.rgbir_mode is None:
        args.rgbir_mode = profile.default_rgbir_mode
    if args.ir_cut is None:
        args.ir_cut = profile.default_ir_cut
    if args.rgbir_interp_sigma is None:
        args.rgbir_interp_sigma = profile.default_rgbir_interp_sigma
    if args.rgbir_gamma is None:
        args.rgbir_gamma = profile.default_rgbir_gamma
    if args.ccm_profile is None:
        args.ccm_profile = profile.default_ccm_profile
    if args.awb_mode is None:
        args.awb_mode = profile.default_awb_mode
    if args.awb_strength is None:
        args.awb_strength = profile.default_awb_strength
    if args.awb_ema is None:
        args.awb_ema = profile.default_awb_ema
    if args.color_saturation is None:
        args.color_saturation = profile.default_color_saturation
    if args.rgbir_pattern is None:
        args.rgbir_pattern = profile.default_rgbir_pattern
    if args.legacy_rgbir_pattern is None:
        args.legacy_rgbir_pattern = profile.default_legacy_rgbir_pattern
    if args.rgbir_pattern_y_offset is None:
        args.rgbir_pattern_y_offset = profile.default_rgbir_pattern_y_offset
    if args.rgbir_pattern_x_offset is None:
        args.rgbir_pattern_x_offset = profile.default_rgbir_pattern_x_offset
    if args.adaptive_min_span is None:
        args.adaptive_min_span = profile.default_adaptive_min_span
    if args.temporal_alpha is None:
        args.temporal_alpha = profile.default_temporal_alpha
    if args.pink_fix_strength is None:
        args.pink_fix_strength = profile.default_pink_fix_strength
    if not 0.0 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.0, 1.0]")
    if args.temporal_reset_threshold < 0.0:
        raise ValueError("--temporal-reset-threshold must be >= 0.0")
    if not 0.0 <= args.pink_fix_strength <= 1.0:
        raise ValueError("--pink-fix-strength must be in [0.0, 1.0]")
    if not 0 <= args.pink_fix_hue_min <= 179:
        raise ValueError("--pink-fix-hue-min must be in [0, 179]")
    if not 0 <= args.pink_fix_hue_max <= 179:
        raise ValueError("--pink-fix-hue-max must be in [0, 179]")
    if not 0 <= args.pink_fix_s_min <= 255:
        raise ValueError("--pink-fix-s-min must be in [0, 255]")
    if not 0 <= args.pink_fix_v_min <= 255:
        raise ValueError("--pink-fix-v-min must be in [0, 255]")
    if not 0 <= args.pink_fix_v_max <= 255:
        raise ValueError("--pink-fix-v-max must be in [0, 255]")
    if args.pink_fix_v_min > args.pink_fix_v_max:
        raise ValueError("--pink-fix-v-min must be <= --pink-fix-v-max")
    if args.pink_fix_sat_scale <= 0.0:
        raise ValueError("--pink-fix-sat-scale must be > 0")

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    Gst.init(None)
    setup_signals()

    try:
        run_pipeline(args)
    except Exception as exc:
        LOG.exception("Pipeline failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
