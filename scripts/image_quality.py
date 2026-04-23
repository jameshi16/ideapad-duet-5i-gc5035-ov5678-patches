#!/usr/bin/env python3
"""Report deterministic quality metrics for processed camera images."""

import argparse
import json

import cv2
import numpy as np


def parse_crop(value):
    parts = [int(x) for x in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("crop must be x,y,w,h")
    x, y, w, h = parts
    if w <= 0 or h <= 0:
        raise argparse.ArgumentTypeError("crop width and height must be positive")
    return x, y, w, h


def load_rgb(path):
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"failed to read image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def image_metrics(rgb):
    rgbf = rgb.astype(np.float32)
    gray = (
        0.299 * rgbf[..., 0]
        + 0.587 * rgbf[..., 1]
        + 0.114 * rgbf[..., 2]
    )

    midtone = (gray > 30.0) & (gray < 220.0)
    if np.any(midtone):
        mid_rgb = rgbf[midtone]
        mid_mean = mid_rgb.mean(axis=0)
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)
        lab_mid = lab[midtone].astype(np.float32)
        cast_score = float(
            abs(lab_mid[:, 1].mean() - 128.0)
            + abs(lab_mid[:, 2].mean() - 128.0)
        )
    else:
        mid_mean = np.array([0.0, 0.0, 0.0])
        cast_score = 0.0

    gx = np.abs(np.diff(gray, axis=1)).mean() if gray.shape[1] > 1 else 0.0
    gy = np.abs(np.diff(gray, axis=0)).mean() if gray.shape[0] > 1 else 0.0
    row_means = gray.mean(axis=1)
    row_smooth = cv2.GaussianBlur(row_means[:, None], (1, 0), 15).ravel()
    ycrcb = cv2.cvtColor(rgb, cv2.COLOR_RGB2YCrCb)
    chroma = ycrcb[..., 1:3].astype(np.float32)
    chroma_hf = 0.0
    if chroma.shape[0] > 1:
        chroma_hf += float(np.abs(np.diff(chroma, axis=0)).mean())
    if chroma.shape[1] > 1:
        chroma_hf += float(np.abs(np.diff(chroma, axis=1)).mean())

    grid4_luma = 0.0
    grid4_chroma = 0.0
    if gray.shape[0] >= 8 and gray.shape[1] >= 8:
        row_phase = np.array([gray[p::4, :].mean() for p in range(4)])
        col_phase = np.array([gray[:, p::4].mean() for p in range(4)])
        grid4_luma = float(row_phase.std() + col_phase.std())
        cr = chroma[..., 0]
        cb = chroma[..., 1]
        for plane in (cr, cb):
            row_phase = np.array([plane[p::4, :].mean() for p in range(4)])
            col_phase = np.array([plane[:, p::4].mean() for p in range(4)])
            grid4_chroma += float(row_phase.std() + col_phase.std())

    return {
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
        "mean_luma": float(gray.mean()),
        "dark_frac_luma_le_5": float((gray <= 5.0).mean()),
        "clip_any_frac_rgb_ge_250": float((rgb >= 250).any(axis=2).mean()),
        "edge_strength": float(gx + gy),
        "banding_score": float((row_means - row_smooth).std()),
        "chroma_hf_score": chroma_hf,
        "grid4_luma_score": grid4_luma,
        "grid4_chroma_score": grid4_chroma,
        "mean_rgb": [float(x) for x in rgbf.mean(axis=(0, 1))],
        "midtone_mean_rgb": [float(x) for x in mid_mean],
        "midtone_lab_cast_score": cast_score,
    }


def crop_image(rgb, crop):
    if crop is None:
        return rgb
    x, y, w, h = crop
    return rgb[y : y + h, x : x + w]


def _circular_mean_deg(values):
    if values.size == 0:
        return None
    radians = np.deg2rad(values)
    sin_mean = float(np.sin(radians).mean())
    cos_mean = float(np.cos(radians).mean())
    if abs(sin_mean) < 1e-8 and abs(cos_mean) < 1e-8:
        return None
    angle = np.rad2deg(np.arctan2(sin_mean, cos_mean))
    if angle < 0.0:
        angle += 360.0
    return float(angle)


def _circular_distance_deg(a_deg, b_deg):
    if a_deg is None or b_deg is None:
        return None
    delta = abs(a_deg - b_deg) % 360.0
    return float(min(delta, 360.0 - delta))


def hue_region_stats(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue = hsv[..., 0].astype(np.float32) * 2.0
    sat = hsv[..., 1].astype(np.float32)
    val = hsv[..., 2].astype(np.float32)

    valid = (sat >= 35.0) & (val >= 20.0) & (val <= 245.0)
    valid_count = int(valid.sum())
    total = int(valid.size)
    if valid_count == 0:
        return {
            "valid_frac": 0.0,
            "valid_pixels": 0,
            "green_frac": 0.0,
            "magenta_frac": 0.0,
            "magenta_over_green": None,
            "mean_hue_deg": None,
        }

    hue_valid = hue[valid]
    green = (hue_valid >= 70.0) & (hue_valid <= 170.0)
    magenta = (hue_valid >= 285.0) | (hue_valid <= 20.0)

    green_frac = float(green.mean())
    magenta_frac = float(magenta.mean())
    return {
        "valid_frac": float(valid_count / max(1, total)),
        "valid_pixels": valid_count,
        "green_frac": green_frac,
        "magenta_frac": magenta_frac,
        "magenta_over_green": (
            float(magenta_frac / green_frac) if green_frac > 1e-6 else None
        ),
        "mean_hue_deg": _circular_mean_deg(hue_valid),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("--reference")
    parser.add_argument("--crop", type=parse_crop, help="Optional x,y,w,h crop")
    parser.add_argument("--hue-roi", type=parse_crop, help="Optional hue ROI x,y,w,h")
    args = parser.parse_args()

    image_rgb = load_rgb(args.image)
    payload = {
        "image": args.image,
        "metrics": image_metrics(crop_image(image_rgb, args.crop)),
    }
    if args.reference:
        reference_rgb = load_rgb(args.reference)
        payload["reference"] = args.reference
        payload["reference_metrics"] = image_metrics(
            crop_image(reference_rgb, args.crop)
        )
    else:
        reference_rgb = None

    if args.hue_roi:
        hue_payload = {
            "crop": {
                "x": args.hue_roi[0],
                "y": args.hue_roi[1],
                "w": args.hue_roi[2],
                "h": args.hue_roi[3],
            },
            "image_stats": hue_region_stats(crop_image(image_rgb, args.hue_roi)),
        }
        if reference_rgb is not None:
            ref_stats = hue_region_stats(crop_image(reference_rgb, args.hue_roi))
            hue_payload["reference_stats"] = ref_stats
            hue_payload["mean_hue_distance_deg"] = _circular_distance_deg(
                hue_payload["image_stats"]["mean_hue_deg"],
                ref_stats["mean_hue_deg"],
            )
        payload["hue_roi"] = hue_payload

    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
