#!/usr/bin/env python3
"""OV5678 RGB-IR raw-domain reconstruction helpers.

The OV5678 module used here exposes a 4x4 RGB-IR mosaic:

    B G R G
    G I G I
    R G B G
    G I G I

Treating that as normal Bayer creates the red/green grid artifacts seen in
processed output. This module keeps correction in the raw domain: optional IR
subtraction, same-channel phase equalization, sparse plane interpolation, then
tone/color mapping.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import cv2
import numpy as np


RGBIR_PATTERNS = {
    # Pattern used by openRGB-IR examples.
    "openrgbir": (
        ("B", "G", "R", "G"),
        ("G", "IR", "G", "IR"),
        ("R", "G", "B", "G"),
        ("G", "IR", "G", "IR"),
    ),
    # Vendor Windows graph_settings_OV5678_CJFL515_ADL.xml:
    # bayer_order="GIGI_RGBG_GIGI_BGRG"
    "windows-cjfl515": (
        ("G", "IR", "G", "IR"),
        ("R", "G", "B", "G"),
        ("G", "IR", "G", "IR"),
        ("B", "G", "R", "G"),
    ),
}


def _coords_from_pattern(rows: tuple[tuple[str, ...], ...]) -> dict[str, tuple[tuple[int, int], ...]]:
    coords: dict[str, list[tuple[int, int]]] = {"R": [], "G": [], "B": [], "IR": []}
    for y, row in enumerate(rows):
        for x, channel in enumerate(row):
            coords[channel].append((y, x))
    return {channel: tuple(points) for channel, points in coords.items()}


def resolve_rgbir_pattern(
    name: str,
    *,
    y_offset: int = 0,
    x_offset: int = 0,
    flip_h: bool = False,
    flip_v: bool = False,
) -> dict[str, tuple[tuple[int, int], ...]]:
    if name not in RGBIR_PATTERNS:
        raise ValueError(f"unknown OV5678 RGB-IR pattern: {name}")

    coords: dict[str, list[tuple[int, int]]] = {"R": [], "G": [], "B": [], "IR": []}
    rows = RGBIR_PATTERNS[name]
    for y, row in enumerate(rows):
        for x, channel in enumerate(row):
            py = 3 - y if flip_v else y
            px = 3 - x if flip_h else x
            coords[channel].append(((py + y_offset) % 4, (px + x_offset) % 4))
    return {channel: tuple(points) for channel, points in coords.items()}


RGBIR_PATTERN = _coords_from_pattern(RGBIR_PATTERNS["openrgbir"])


def rgbir_pattern_names() -> tuple[str, ...]:
    return tuple(sorted(RGBIR_PATTERNS))


COLOR_SATURATION = {
    "none": 1.0,
    "ov5678-indoor": 1.0,
    "ov5678-cjfl515": 1.0,
}


CCM_PROFILES = {
    "none": np.eye(3, dtype=np.float32),
    # Conservative indoor profile. White balance is still handled by gains;
    # this leaves color geometry stable while the rest of the path is proven.
    "ov5678-indoor": np.eye(3, dtype=np.float32),
    # Initial reference profile for the CJFL515 OV5678 module. This is kept
    # moderate because the RGB-IR path is sparse and easy to overdrive.
    "ov5678-cjfl515": np.eye(3, dtype=np.float32),
}


LEGACY_RGBIR_PATTERN = {
    "R": ((0, 2), (2, 0)),
    "G": ((0, 1), (0, 3), (1, 0), (1, 2), (2, 1), (2, 3), (3, 0), (3, 2)),
    "B": ((0, 0), (2, 2)),
    "IR": ((1, 1), (1, 3), (3, 1), (3, 3)),
}


@dataclass(frozen=True)
class OV5678RgbirSettings:
    black_level: int = 64
    white_level: int = 1023
    ir_cut: str | float = "auto"
    ir_clip: float = 1.3
    grid_correct: bool = True
    interp_sigma: float = 1.50
    gamma: float = 0.70
    normalize_mode: str = "adaptive"
    adaptive_low_percentile: float = 0.5
    adaptive_high_percentile: float = 94.0
    adaptive_sample_step: int = 8
    adaptive_ema: float = 0.12
    adaptive_min_span: float = 48.0
    ccm_profile: str = "ov5678-indoor"
    awb_mode: str = "grayworld"
    awb_strength: float = 0.85
    color_saturation: float | None = None
    rgbir_pattern: str = "openrgbir"
    pattern_y_offset: int = 0
    pattern_x_offset: int = 0
    pattern_flip_h: bool = False
    pattern_flip_v: bool = False
    dump_intermediates: str | None = None


def parse_ir_cut(value: str | float | int | None, *, auto_default: float = 0.04) -> tuple[float, bool]:
    if value is None or value == "auto":
        return auto_default, True
    return float(value), False


def _make_mask(shape: tuple[int, int], coords: tuple[tuple[int, int], ...]) -> np.ndarray:
    h, w = shape
    mask = np.zeros((h, w), dtype=bool)
    for y, x in coords:
        mask[y::4, x::4] = True
    return mask


def _sparse_interpolate(values: np.ndarray, mask: np.ndarray, sigma: float) -> np.ndarray:
    maskf = mask.astype(np.float32)
    sigma = max(float(sigma), 0.1)
    numerator = cv2.GaussianBlur(
        values * maskf,
        (0, 0),
        sigmaX=sigma,
        sigmaY=sigma,
        borderType=cv2.BORDER_REFLECT,
    )
    denominator = cv2.GaussianBlur(
        maskf,
        (0, 0),
        sigmaX=sigma,
        sigmaY=sigma,
        borderType=cv2.BORDER_REFLECT,
    )
    return numerator / np.maximum(denominator, 1e-6)


def _phase_equalize(values: np.ndarray, coords: tuple[tuple[int, int], ...]) -> None:
    medians = [float(np.median(values[y::4, x::4])) for y, x in coords]
    valid = [m for m in medians if m > 1.0]
    if not valid:
        return

    target = float(np.median(valid))
    for (y, x), median in zip(coords, medians):
        if median <= 1.0:
            continue
        gain = np.clip(target / median, 0.5, 2.0)
        values[y::4, x::4] *= gain


def _adaptive_range(rgb: np.ndarray, settings: OV5678RgbirSettings, norm_state: dict | None):
    if settings.normalize_mode == "fixed":
        return 0.0, float(max(1, settings.white_level - settings.black_level))

    step = max(1, int(settings.adaptive_sample_step))
    sample = rgb[::step, ::step, :]
    low_now = float(np.percentile(sample, settings.adaptive_low_percentile))
    high_now = float(np.percentile(sample, settings.adaptive_high_percentile))

    if high_now - low_now < settings.adaptive_min_span:
        center = 0.5 * (high_now + low_now)
        low_now = max(0.0, center - settings.adaptive_min_span * 0.5)
        high_now = low_now + settings.adaptive_min_span

    if norm_state is None:
        return low_now, high_now

    if norm_state.get("rgbir_low") is None or norm_state.get("rgbir_high") is None:
        norm_state["rgbir_low"] = low_now
        norm_state["rgbir_high"] = high_now
    else:
        ema = float(settings.adaptive_ema)
        norm_state["rgbir_low"] = norm_state["rgbir_low"] * (1.0 - ema) + low_now * ema
        norm_state["rgbir_high"] = norm_state["rgbir_high"] * (1.0 - ema) + high_now * ema

    return float(norm_state["rgbir_low"]), float(norm_state["rgbir_high"])


def _apply_grayworld_awb(rgb: np.ndarray, strength: float) -> np.ndarray:
    gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    lo = float(np.percentile(gray, 15.0))
    hi = float(np.percentile(gray, 90.0))
    mask = (gray > lo) & (gray < hi)
    if not np.any(mask):
        return rgb

    means = rgb[mask].mean(axis=0)
    if np.any(means <= 1e-6):
        return rgb

    target = float(np.median(means))
    gains = np.clip(target / means, 0.55, 1.85)
    strength = float(np.clip(strength, 0.0, 1.0))
    gains = 1.0 + (gains - 1.0) * strength
    return rgb * gains.reshape(1, 1, 3)


def _apply_saturation(rgb: np.ndarray, saturation: float) -> np.ndarray:
    saturation = float(max(0.0, saturation))
    if abs(saturation - 1.0) < 1e-6:
        return rgb

    luma = (
        0.299 * rgb[..., 0]
        + 0.587 * rgb[..., 1]
        + 0.114 * rgb[..., 2]
    )[..., None]
    return np.maximum(luma + (rgb - luma) * saturation, 0.0)


def _dump_intermediates(path: str, rgb: np.ndarray, ir: np.ndarray, raw: np.ndarray) -> None:
    os.makedirs(path, exist_ok=True)
    raw_vis = np.clip(raw * 255.0 / max(1.0, float(np.percentile(raw, 99.5))), 0, 255)
    ir_vis = np.clip(ir * 255.0 / max(1.0, float(np.percentile(ir, 99.5))), 0, 255)
    cv2.imwrite(os.path.join(path, "ov5678_raw_luma.png"), raw_vis.astype(np.uint8))
    cv2.imwrite(os.path.join(path, "ov5678_ir.png"), ir_vis.astype(np.uint8))
    cv2.imwrite(os.path.join(path, "ov5678_rgb_linear.png"), cv2.cvtColor(
        np.clip(rgb * 255.0 / max(1.0, float(np.percentile(rgb, 99.5))), 0, 255).astype(np.uint8),
        cv2.COLOR_RGB2BGR,
    ))


def process_ov5678_rgbir(
    raw16: np.ndarray,
    *,
    bgr_gains: tuple[float, float, float],
    settings: OV5678RgbirSettings,
    norm_state: dict | None = None,
) -> tuple[np.ndarray, float, float, np.ndarray]:
    """Return processed BGR frame, norm low/high, and half-resolution IR plane."""

    raw = raw16.astype(np.float32) - float(settings.black_level)
    np.maximum(raw, 0.0, out=raw)

    pattern = resolve_rgbir_pattern(
        settings.rgbir_pattern,
        y_offset=settings.pattern_y_offset,
        x_offset=settings.pattern_x_offset,
        flip_h=settings.pattern_flip_h,
        flip_v=settings.pattern_flip_v,
    )

    ir_mask = _make_mask(raw.shape, pattern["IR"])
    ir_full = _sparse_interpolate(raw, ir_mask, sigma=1.5)
    ir_cut, _auto_ir = parse_ir_cut(settings.ir_cut)
    if ir_cut > 0.0:
        clip_max = float(np.max(raw)) / max(float(settings.ir_clip), 1e-6)
        ir_sub = np.minimum(ir_full, clip_max)
        work = raw - ir_sub * ir_cut
        np.maximum(work, 0.0, out=work)
    else:
        work = raw.copy()

    planes = []
    for channel in ("R", "G", "B"):
        coords = pattern[channel]
        mask = _make_mask(work.shape, coords)
        if settings.grid_correct:
            _phase_equalize(work, coords)
        planes.append(_sparse_interpolate(work, mask, sigma=settings.interp_sigma))

    rgb = np.dstack(planes).astype(np.float32)
    rgb_gains = np.array(
        [bgr_gains[2], bgr_gains[1], bgr_gains[0]], dtype=np.float32
    ).reshape(1, 1, 3)
    rgb *= rgb_gains

    ccm = CCM_PROFILES.get(settings.ccm_profile)
    if ccm is None:
        raise ValueError(f"unknown OV5678 CCM profile: {settings.ccm_profile}")
    rgb = np.maximum(rgb @ ccm.T, 0.0)
    if settings.awb_mode == "grayworld":
        rgb = _apply_grayworld_awb(rgb, settings.awb_strength)
    elif settings.awb_mode != "fixed":
        raise ValueError(f"unknown OV5678 AWB mode: {settings.awb_mode}")
    saturation = (
        COLOR_SATURATION.get(settings.ccm_profile, 1.0)
        if settings.color_saturation is None
        else settings.color_saturation
    )
    rgb = _apply_saturation(rgb, saturation)

    low, high = _adaptive_range(rgb, settings, norm_state)
    out = (rgb - low) * (255.0 / max(1.0, high - low))
    np.clip(out, 0.0, 255.0, out=out)
    out = np.power(out / 255.0, float(settings.gamma)) * 255.0
    out = np.clip(out, 0.0, 255.0).astype(np.uint8)

    if settings.dump_intermediates and not (norm_state or {}).get("rgbir_dumped"):
        _dump_intermediates(settings.dump_intermediates, rgb, ir_full, raw)
        if norm_state is not None:
            norm_state["rgbir_dumped"] = True

    bgr = cv2.cvtColor(out, cv2.COLOR_RGB2BGR)
    return bgr, low, high, raw16[1::2, 1::2]


def rgbir_phase_stats(
    raw16: np.ndarray,
    black_level: int = 64,
    pattern_name: str = "openrgbir",
    y_offset: int = 0,
    x_offset: int = 0,
    flip_h: bool = False,
    flip_v: bool = False,
) -> dict:
    raw = raw16.astype(np.float32) - float(black_level)
    np.maximum(raw, 0.0, out=raw)
    pattern = resolve_rgbir_pattern(
        pattern_name,
        y_offset=y_offset,
        x_offset=x_offset,
        flip_h=flip_h,
        flip_v=flip_v,
    )
    stats = {}
    for channel, coords in pattern.items():
        values = np.concatenate([raw[y::4, x::4].ravel() for y, x in coords])
        stats[channel] = {
            "mean": float(values.mean()),
            "median": float(np.median(values)),
            "p90": float(np.percentile(values, 90)),
        }
    return stats
