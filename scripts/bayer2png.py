#!/usr/bin/env python3
"""Convert 16-bit little-endian RAW captures to PNG.

Supports:
- GC5035 (native GRBG Bayer)
- OV5678 RGB-IR (4x4 RGB-IR pattern converted to BGGR first)

Legacy usage remains compatible:
  bayer2png.py <input.bin> <output.png> [width] [height]

Examples:
  bayer2png.py raw.bin out.png 2592 1944 --sensor gc5035
  bayer2png.py raw.bin out.png 2592 1944 --sensor ov5678 --ir-output out_ir.png
"""

import argparse
import array
import datetime
import hashlib
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image

from ov5678_rgbir import (
    OV5678RgbirSettings,
    parse_ir_cut,
    process_ov5678_rgbir,
    resolve_rgbir_pattern,
    rgbir_pattern_names,
)


DEFAULT_WIDTH = 2592
DEFAULT_HEIGHT = 1944
LEGACY_RGBIR_PATTERN_NAMES = ("openrgbir", "windows-cjfl515")

if hasattr(Image, "Resampling"):
    RESAMPLE_BILINEAR = Image.Resampling.BILINEAR
    RESAMPLE_NEAREST = Image.Resampling.NEAREST
else:
    RESAMPLE_BILINEAR = getattr(Image, "BILINEAR", 2)
    RESAMPLE_NEAREST = getattr(Image, "NEAREST", 0)


def maybe_unpack_mipi_raw10(raw_bytes, width, height):
    """Decode RAW10 packed stream (5 bytes -> 4 pixels) into 16-bit values.

    V4L2 can expose MIPI RAW10 as packed bytes. If the file size matches the
    packed layout, unpack it to 16-bit sample values so the rest of the pipeline
    can stay unchanged.
    """

    packed_size = (width * height * 5) // 4
    if len(raw_bytes) != packed_size:
        return None

    out = array.array("H", [0]) * (width * height)
    o = 0
    mv = memoryview(raw_bytes)

    for i in range(0, packed_size, 5):
        b0 = mv[i + 0]
        b1 = mv[i + 1]
        b2 = mv[i + 2]
        b3 = mv[i + 3]
        b4 = mv[i + 4]

        out[o + 0] = (b0 << 2) | ((b4 >> 0) & 0x03)
        out[o + 1] = (b1 << 2) | ((b4 >> 2) & 0x03)
        out[o + 2] = (b2 << 2) | ((b4 >> 4) & 0x03)
        out[o + 3] = (b3 << 2) | ((b4 >> 6) & 0x03)
        o += 4

    return out


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def load_source_metadata(input_path):
    candidates = [
        input_path + ".meta.json",
        os.path.splitext(input_path)[0] + ".meta.json",
    ]

    for meta_path in candidates:
        if not os.path.exists(meta_path):
            continue
        try:
            with open(meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)
            return meta_path, meta
        except (OSError, json.JSONDecodeError):
            continue

    return None, None


def write_output_metadata(out_path, payload):
    meta_path = out_path + ".meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
    return meta_path


def check_source_metadata(args):
    meta_path, meta = load_source_metadata(args.input)
    if not meta:
        print("Note: no source metadata sidecar found; source sensor is unknown.")
        return

    meta_sensor = meta.get("sensor")
    meta_node = meta.get("video_node", "?")
    print(f"Source metadata: {meta_path} (sensor={meta_sensor}, node={meta_node})")

    if meta_sensor and meta_sensor != args.sensor and not args.ignore_meta_mismatch:
        raise ValueError(
            f"sensor mismatch: --sensor={args.sensor} but source metadata says {meta_sensor}. "
            "Use the matching sensor mode or pass --ignore-meta-mismatch."
        )


def read_raw(path, width, height):
    """Read RAW file and return unpacked 16-bit samples.

    Supported on-disk encodings:
    - 16-bit little-endian (2 bytes/pixel)
    - MIPI packed RAW10 (5 bytes per 4 pixels)
    """

    with open(path, "rb") as f:
        raw_bytes = f.read()

    expected16 = width * height * 2
    packed10 = (width * height * 5) // 4

    if len(raw_bytes) == packed10:
        vals = maybe_unpack_mipi_raw10(raw_bytes, width, height)
        if vals is None:
            raise ValueError("failed to unpack RAW10 stream")
        return vals

    if len(raw_bytes) < expected16:
        raise ValueError(
            f"unsupported/short raw size: got {len(raw_bytes)}, expected either "
            f"{expected16} (16-bit) or {packed10} (RAW10 packed)"
        )

    # Truncate potential trailing data and parse as 16-bit LE.
    raw_bytes = raw_bytes[:expected16]
    vals = array.array("H")
    vals.frombytes(raw_bytes)
    if sys.byteorder != "little":
        vals.byteswap()
    return vals


def normalize_dynamic_u8(values):
    lo = min(values)
    hi = max(values)
    if hi <= lo:
        return bytes(len(values))

    scale = 255.0 / float(hi - lo)
    out = bytearray(len(values))
    for i, v in enumerate(values):
        out[i] = int((v - lo) * scale + 0.5)
    return bytes(out)


def _l_oblique(bayer, x, y, width, height):
    if x == 0 or y == 0:
        return bayer[(y + 1) * width + (x + 1)]
    if x == width - 1 or y == height - 1:
        return bayer[(y - 1) * width + (x - 1)]
    return (bayer[(y - 1) * width + (x - 1)] + bayer[(y + 1) * width + (x + 1)]) // 2


def _r_oblique(bayer, x, y, width, height):
    if x == width - 1 and y == height - 1:
        return (
            bayer[(y - 1) * width + (x - 3)] + bayer[(y - 3) * width + (x - 1)]
        ) // 4
    if y == height - 1:
        return bayer[(y - 1) * width + (x + 1)]
    if x == width - 1:
        return bayer[(y + 1) * width + (x - 1)]
    return (bayer[(y + 1) * width + (x - 1)] + bayer[(y - 1) * width + (x + 1)]) // 2


def _oblique(bayer, x, y, width, height):
    if ((x + y) // 2) & 1:
        return _r_oblique(bayer, x, y, width, height)
    return _l_oblique(bayer, x, y, width, height)


def _cross(bayer, x, y, width, height):
    num_valid = 4

    if x != 0 and x < width - 2:
        bl = bayer[y * width + (x - 2)]
        br = bayer[y * width + (x + 2)]
    elif x == 0:
        num_valid -= 1
        bl = 0
        br = bayer[y * width + (x + 2)]
    else:
        num_valid -= 1
        bl = bayer[y * width + (x - 2)]
        br = 0

    if y != 0 and y < height - 2:
        bt = bayer[(y - 2) * width + x]
        bd = bayer[(y + 2) * width + x]
    elif y == 0:
        num_valid -= 1
        bt = 0
        bd = bayer[(y + 2) * width + x]
    else:
        num_valid -= 1
        bt = bayer[(y - 2) * width + x]
        bd = 0

    return (bl + br + bt + bd) // max(1, num_valid)


def _legacy_rgbir_phase(pattern_name):
    pattern = resolve_rgbir_pattern(pattern_name)
    phase = [["" for _ in range(4)] for _ in range(4)]
    for channel, coords in pattern.items():
        for y, x in coords:
            phase[y][x] = channel
    return pattern, phase


def _legacy_bayer_pattern(phase):
    converted = [row[:] for row in phase]
    for y in range(4):
        for x in range(4):
            if converted[y][x] == "IR":
                converted[y][x] = "R"
            elif converted[y][x] == "R":
                converted[y][x] = "B"

    key = (
        converted[0][0],
        converted[0][1],
        converted[1][0],
        converted[1][1],
    )
    if key == ("B", "G", "G", "R"):
        return "bggr"
    if key == ("G", "R", "B", "G"):
        return "grbg"
    raise ValueError(f"unsupported legacy OV5678 Bayer phase after remap: {key}")


def extract_ov5678_ir_half(raw, width, height, y_phase, x_phase):
    hw = width // 2
    hh = height // 2
    ir = [0] * (hw * hh)

    for y in range(hh):
        src_row = (2 * y + y_phase) * width + x_phase
        dst_row = y * hw
        for x in range(hw):
            ir[dst_row + x] = raw[src_row + 2 * x]

    return ir


def upsample_half_nn_to_full(half, width, height):
    hw = width // 2
    full = [0] * (width * height)

    for y in range(height):
        sy = y // 2
        src_row = sy * hw
        dst_row = y * width
        for x in range(width):
            full[dst_row + x] = half[src_row + (x // 2)]

    return full


def ov5678_rgbir_to_bggr(
    raw,
    width,
    height,
    ir_cut,
    ir_clip,
    apply_ir_cut=True,
    legacy_pattern="windows-cjfl515",
):
    if legacy_pattern not in LEGACY_RGBIR_PATTERN_NAMES:
        raise ValueError(f"unsupported --legacy-rgbir-pattern: {legacy_pattern}")

    pattern, phase = _legacy_rgbir_phase(legacy_pattern)
    ir_coords = pattern["IR"]
    ir_y_parity = {y % 2 for y, _x in ir_coords}
    ir_x_parity = {x % 2 for _y, x in ir_coords}
    if len(ir_y_parity) != 1 or len(ir_x_parity) != 1:
        raise ValueError("legacy OV5678 RGB-IR pattern has unsupported IR parity layout")
    ir_half = extract_ov5678_ir_half(
        raw,
        width,
        height,
        y_phase=next(iter(ir_y_parity)),
        x_phase=next(iter(ir_x_parity)),
    )
    bayer = array.array("H", raw)

    if apply_ir_cut:
        ir_full = upsample_half_nn_to_full(ir_half, width, height)
        raw_max = max(raw)
        clip_max = int(float(raw_max) / max(ir_clip, 1e-6))

        for i, v in enumerate(bayer):
            ir = ir_full[i]
            if ir > clip_max:
                ir = clip_max
            nv = int(v - ir * ir_cut + 0.5)
            bayer[i] = nv if nv > 0 else 0

    for y_off, x_off in ir_coords:
        l_red = int(phase[(y_off - 1) % 4][(x_off - 1) % 4] == "R") + int(
            phase[(y_off + 1) % 4][(x_off + 1) % 4] == "R"
        )
        r_red = int(phase[(y_off - 1) % 4][(x_off + 1) % 4] == "R") + int(
            phase[(y_off + 1) % 4][(x_off - 1) % 4] == "R"
        )
        use_r = r_red >= l_red
        for y in range(y_off, height, 4):
            row = y * width
            for x in range(x_off, width, 4):
                if use_r:
                    bayer[row + x] = _r_oblique(bayer, x, y, width, height)
                else:
                    bayer[row + x] = _l_oblique(bayer, x, y, width, height)

    for y_off, x_off in pattern["R"]:
        for y in range(y_off, height, 4):
            row = y * width
            for x in range(x_off, width, 4):
                bayer[row + x] = _cross(bayer, x, y, width, height)

    return bayer, ir_half, _legacy_bayer_pattern(phase)


def split_bayer_half(raw, width, height, pattern):
    hw = width // 2
    hh = height // 2

    r = [0] * (hw * hh)
    g1 = [0] * (hw * hh)
    g2 = [0] * (hw * hh)
    b = [0] * (hw * hh)

    for y in range(hh):
        row0 = (2 * y) * width
        row1 = (2 * y + 1) * width
        dst = y * hw
        for x in range(hw):
            p00 = raw[row0 + 2 * x]
            p01 = raw[row0 + 2 * x + 1]
            p10 = raw[row1 + 2 * x]
            p11 = raw[row1 + 2 * x + 1]

            if pattern == "grbg":
                g1[dst + x] = p00
                r[dst + x] = p01
                b[dst + x] = p10
                g2[dst + x] = p11
            elif pattern == "bggr":
                b[dst + x] = p00
                g1[dst + x] = p01
                g2[dst + x] = p10
                r[dst + x] = p11
            else:
                raise ValueError(f"Unsupported Bayer pattern: {pattern}")

    return r, g1, g2, b


def build_lut(black_level, white_level, gain, gamma, max_input=4095):
    denom = max(1.0, float(white_level - black_level))
    inv_gamma = 1.0 / gamma

    lut = bytearray(max_input + 1)
    for v in range(max_input + 1):
        x = v - black_level
        if x < 0:
            x = 0
        f = (x / denom) * gain
        if f > 1.0:
            f = 1.0
        lut[v] = int((f**inv_gamma) * 255.0 + 0.5)

    return lut


def normalize_u8(values, lut):
    max_idx = len(lut) - 1
    out = bytearray(len(values))
    for i, v in enumerate(values):
        out[i] = lut[v if v <= max_idx else max_idx]
    return bytes(out)


def raw_to_rgb_image(
    raw, width, height, pattern, r_gain, g_gain, b_gain, black_level, white_level, gamma
):
    r_half, g1_half, g2_half, b_half = split_bayer_half(raw, width, height, pattern)

    lut_r = build_lut(black_level, white_level, r_gain, gamma)
    lut_g = build_lut(black_level, white_level, g_gain, gamma)
    lut_b = build_lut(black_level, white_level, b_gain, gamma)

    hw = width // 2
    hh = height // 2
    r_img = Image.frombytes("L", (hw, hh), normalize_u8(r_half, lut_r))
    g1_img = Image.frombytes("L", (hw, hh), normalize_u8(g1_half, lut_g))
    g2_img = Image.frombytes("L", (hw, hh), normalize_u8(g2_half, lut_g))
    b_img = Image.frombytes("L", (hw, hh), normalize_u8(b_half, lut_b))

    g_img = Image.blend(g1_img, g2_img, 0.5)

    r_full = r_img.resize((width, height), RESAMPLE_BILINEAR)
    g_full = g_img.resize((width, height), RESAMPLE_BILINEAR)
    b_full = b_img.resize((width, height), RESAMPLE_BILINEAR)

    return Image.merge("RGB", (r_full, g_full, b_full))


def print_rgb_stats(rgb):
    r, g, b = rgb.split()
    rb = r.tobytes()
    gb = g.tobytes()
    bb = b.tobytes()
    print(f"  R: min={min(rb)} max={max(rb)} mean={sum(rb) / len(rb):.1f}")
    print(f"  G: min={min(gb)} max={max(gb)} mean={sum(gb) / len(gb):.1f}")
    print(f"  B: min={min(bb)} max={max(bb)} mean={sum(bb) / len(bb):.1f}")


def filter_bgr_ycrcb(frame_bgr, chroma_sigma, luma_sigma):
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


def limit_bgr_luma(frame_bgr, target_luma):
    if target_luma <= 0.0:
        return frame_bgr
    b = frame_bgr[:, :, 0].astype(np.float32)
    g = frame_bgr[:, :, 1].astype(np.float32)
    r = frame_bgr[:, :, 2].astype(np.float32)
    current = float((0.114 * b + 0.587 * g + 0.299 * r).mean())
    if current <= target_luma:
        return frame_bgr
    return np.clip(frame_bgr.astype(np.float32) * (target_luma / current), 0, 255).astype(np.uint8)


def convert(args):
    meta_path, source_meta = load_source_metadata(args.input)
    check_source_metadata(args)

    raw = read_raw(args.input, args.width, args.height)
    input_sha = sha256_file(args.input)

    if args.sensor == "gc5035":
        pattern = "grbg"
        work = raw
        ir_half = None
        default_r, default_g, default_b = (2.0, 1.0, 1.8)
        direct_rgbir = False
    else:
        if args.ir_cut is None:
            args.ir_cut = 0.04
        direct_rgbir = args.rgbir_mode == "direct"
        if direct_rgbir:
            pattern = None
            work = None
            ir_half = None
            default_r, default_g, default_b = (1.15, 3.30, 0.90)
        else:
            ir_cut_value, _auto_ir = parse_ir_cut(args.ir_cut)
            apply_ir_cut = (not args.no_ir_cut) and ir_cut_value > 0.0
            work, ir_half, pattern = ov5678_rgbir_to_bggr(
                raw,
                args.width,
                args.height,
                ir_cut=ir_cut_value,
                ir_clip=args.ir_clip,
                apply_ir_cut=apply_ir_cut,
                legacy_pattern=args.legacy_rgbir_pattern,
            )
            default_r, default_g, default_b = (1.0, 3.66, 0.85)

    r_gain = args.r_gain if args.r_gain is not None else default_r
    g_gain = args.g_gain if args.g_gain is not None else default_g
    b_gain = args.b_gain if args.b_gain is not None else default_b

    if direct_rgbir:
        ir_cut_arg = 0.0 if args.no_ir_cut else args.ir_cut
        raw_np = np.asarray(raw, dtype=np.uint16).reshape(args.height, args.width)
        settings = OV5678RgbirSettings(
            black_level=args.black_level,
            white_level=args.white_level,
            ir_cut=ir_cut_arg,
            ir_clip=args.ir_clip,
            grid_correct=(args.raw_grid_correct == "on"),
            interp_sigma=args.rgbir_interp_sigma,
            gamma=args.rgbir_gamma,
            normalize_mode=args.normalize_mode,
            adaptive_low_percentile=args.adaptive_low_percentile,
            adaptive_high_percentile=args.adaptive_high_percentile,
            adaptive_sample_step=args.adaptive_sample_step,
            adaptive_ema=args.adaptive_ema,
            adaptive_min_span=args.adaptive_min_span,
            ccm_profile=args.ccm_profile,
            awb_mode=args.awb_mode,
            awb_strength=args.awb_strength,
            color_saturation=args.color_saturation,
            rgbir_pattern=args.rgbir_pattern,
            pattern_y_offset=args.rgbir_pattern_y_offset,
            pattern_x_offset=args.rgbir_pattern_x_offset,
            pattern_flip_h=args.rgbir_pattern_flip_h,
            pattern_flip_v=args.rgbir_pattern_flip_v,
            dump_intermediates=args.dump_intermediates,
        )
        bgr, _norm_low, _norm_high, ir_half_np = process_ov5678_rgbir(
            raw_np,
            bgr_gains=(b_gain, g_gain, r_gain),
            settings=settings,
            norm_state={},
        )
        bgr = filter_bgr_ycrcb(bgr, args.chroma_filter_sigma, args.luma_filter_sigma)
        bgr = limit_bgr_luma(bgr, args.target_luma)
        rgb = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), "RGB")
        ir_half = array.array("H", ir_half_np.astype(np.uint16).ravel())
    else:
        rgb = raw_to_rgb_image(
            work,
            args.width,
            args.height,
            pattern,
            r_gain=r_gain,
            g_gain=g_gain,
            b_gain=b_gain,
            black_level=args.black_level,
            white_level=args.white_level,
            gamma=args.gamma,
        )
    rgb.save(args.output, "PNG")

    print(f"Saved: {args.output} ({args.width}x{args.height} RGB)")
    print(f"Input SHA256: {input_sha}")
    print_rgb_stats(rgb)

    ir_path = None
    if args.sensor == "ov5678" and ir_half is not None:
        ir_path = args.ir_output
        if not ir_path:
            base, ext = os.path.splitext(args.output)
            if not ext:
                ext = ".png"
            ir_path = base + "_ir" + ext

        ir_hw = args.width // 2
        ir_hh = args.height // 2
        ir_u8 = normalize_dynamic_u8(ir_half)
        ir_img = Image.frombytes("L", (ir_hw, ir_hh), ir_u8)
        ir_img = ir_img.resize((args.width, args.height), RESAMPLE_NEAREST)
        ir_img.save(ir_path, "PNG")
        print(f"Saved: {ir_path} ({args.width}x{args.height} IR)")

    meta_payload = {
        "created_utc": datetime.datetime.now(datetime.UTC)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
        "input_path": args.input,
        "input_sha256": input_sha,
        "output_path": args.output,
        "output_ir_path": ir_path,
        "sensor_mode": args.sensor,
        "width": args.width,
        "height": args.height,
        "effective_gains": {
            "r": r_gain,
            "g": g_gain,
            "b": b_gain,
        },
        "ir_cut": args.ir_cut,
        "ir_clip": args.ir_clip,
        "ir_cut_applied": (
            args.sensor == "ov5678"
            and (not args.no_ir_cut)
            and parse_ir_cut(args.ir_cut)[0] > 0.0
        ),
        "rgbir_mode": args.rgbir_mode if args.sensor == "ov5678" else None,
        "rgbir_pattern": args.rgbir_pattern if args.sensor == "ov5678" else None,
        "legacy_rgbir_pattern": (
            args.legacy_rgbir_pattern if args.sensor == "ov5678" else None
        ),
        "ccm_profile": args.ccm_profile if args.sensor == "ov5678" else None,
        "awb_mode": args.awb_mode if args.sensor == "ov5678" else None,
        "color_saturation": args.color_saturation if args.sensor == "ov5678" else None,
        "source_meta_path": meta_path,
        "source_meta": source_meta,
    }
    out_meta = write_output_metadata(args.output, meta_payload)
    print(f"Saved: {out_meta}")

    if ir_path:
        ir_meta_payload = dict(meta_payload)
        ir_meta_payload["output_path"] = ir_path
        ir_meta_payload["role"] = "ir"
        ir_meta = write_output_metadata(ir_path, ir_meta_payload)
        print(f"Saved: {ir_meta}")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Input RAW file (16-bit little-endian)")
    parser.add_argument("output", help="Output RGB PNG path")
    parser.add_argument("width", nargs="?", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("height", nargs="?", type=int, default=DEFAULT_HEIGHT)

    parser.add_argument(
        "--sensor",
        choices=("gc5035", "ov5678"),
        default="gc5035",
        help="Sensor model pipeline to use",
    )

    parser.add_argument("--r-gain", type=float, default=None, help="Red gain")
    parser.add_argument("--g-gain", type=float, default=None, help="Green gain")
    parser.add_argument("--b-gain", type=float, default=None, help="Blue gain")
    parser.add_argument(
        "--black-level", type=int, default=64, help="10-bit black pedestal"
    )
    parser.add_argument(
        "--white-level", type=int, default=1023, help="10-bit white point"
    )
    parser.add_argument("--gamma", type=float, default=2.2, help="Display gamma")

    parser.add_argument(
        "--ir-cut", default=None, help="OV5678 IR subtraction strength or 'auto'"
    )
    parser.add_argument(
        "--ir-clip", type=float, default=1.3, help="OV5678 IR clip coefficient"
    )
    parser.add_argument(
        "--no-ir-cut", action="store_true", help="Disable OV5678 IR subtraction"
    )
    parser.add_argument(
        "--ir-output", default=None, help="Optional OV5678 IR PNG output path"
    )
    parser.add_argument(
        "--rgbir-mode",
        choices=("direct", "legacy"),
        default="legacy",
        help="OV5678 RGB-IR reconstruction path",
    )
    parser.add_argument(
        "--raw-grid-correct",
        choices=("on", "off"),
        default="on",
        help="Equalize same-channel 4x4 raw phases before interpolation",
    )
    parser.add_argument("--rgbir-interp-sigma", type=float, default=1.50)
    parser.add_argument("--rgbir-gamma", type=float, default=0.70)
    parser.add_argument(
        "--ccm-profile",
        choices=("none", "ov5678-indoor", "ov5678-cjfl515"),
        default="none",
    )
    parser.add_argument("--awb-mode", choices=("fixed", "grayworld"), default="grayworld")
    parser.add_argument("--awb-strength", type=float, default=0.85)
    parser.add_argument("--color-saturation", type=float, default=None)
    parser.add_argument(
        "--rgbir-pattern",
        choices=rgbir_pattern_names(),
        default="windows-cjfl515",
    )
    parser.add_argument(
        "--legacy-rgbir-pattern",
        choices=LEGACY_RGBIR_PATTERN_NAMES,
        default="windows-cjfl515",
    )
    parser.add_argument("--rgbir-pattern-y-offset", type=int, default=0)
    parser.add_argument("--rgbir-pattern-x-offset", type=int, default=0)
    parser.add_argument("--rgbir-pattern-flip-h", action="store_true")
    parser.add_argument("--rgbir-pattern-flip-v", action="store_true")
    parser.add_argument("--normalize-mode", choices=("fixed", "adaptive"), default="adaptive")
    parser.add_argument("--adaptive-low-percentile", type=float, default=0.5)
    parser.add_argument("--adaptive-high-percentile", type=float, default=94.0)
    parser.add_argument("--adaptive-sample-step", type=int, default=8)
    parser.add_argument("--adaptive-ema", type=float, default=0.12)
    parser.add_argument("--adaptive-min-span", type=float, default=12.0)
    parser.add_argument("--dump-intermediates", default=None)
    parser.add_argument("--chroma-filter-sigma", type=float, default=1.6)
    parser.add_argument("--luma-filter-sigma", type=float, default=0.0)
    parser.add_argument("--target-luma", type=float, default=142.0)
    parser.add_argument(
        "--ignore-meta-mismatch",
        action="store_true",
        help="Proceed even if source metadata sensor differs from --sensor",
    )

    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    convert(args)


if __name__ == "__main__":
    main()
