#!/usr/bin/env python3
"""Offline OV5678 RGB-IR diagnostics and output scoring."""

import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np

from camera_pipeline import decode_raw_frame, downscale_raw2x
from image_quality import image_metrics, load_rgb
from ov5678_rgbir import (
    OV5678RgbirSettings,
    process_ov5678_rgbir,
    rgbir_pattern_names,
    rgbir_phase_stats,
)


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
    b = frame_bgr[:, :, 0].astype("float32")
    g = frame_bgr[:, :, 1].astype("float32")
    r = frame_bgr[:, :, 2].astype("float32")
    current = float((0.114 * b + 0.587 * g + 0.299 * r).mean())
    if current <= target_luma:
        return frame_bgr
    return cv2.convertScaleAbs(frame_bgr, alpha=target_luma / current, beta=0)


def make_settings(args, **overrides):
    values = {
        "black_level": args.black_level,
        "white_level": args.white_level,
        "ir_cut": args.ir_cut,
        "ir_clip": args.ir_clip,
        "grid_correct": (args.raw_grid_correct == "on"),
        "interp_sigma": args.rgbir_interp_sigma,
        "gamma": args.rgbir_gamma,
        "adaptive_low_percentile": args.adaptive_low_percentile,
        "adaptive_high_percentile": args.adaptive_high_percentile,
        "adaptive_min_span": args.adaptive_min_span,
        "ccm_profile": args.ccm_profile,
        "awb_mode": args.awb_mode,
        "awb_strength": args.awb_strength,
        "color_saturation": args.color_saturation,
        "rgbir_pattern": args.rgbir_pattern,
        "pattern_y_offset": args.rgbir_pattern_y_offset,
        "pattern_x_offset": args.rgbir_pattern_x_offset,
        "pattern_flip_h": args.rgbir_pattern_flip_h,
        "pattern_flip_v": args.rgbir_pattern_flip_v,
        "dump_intermediates": args.dump_intermediates,
    }
    values.update(overrides)
    return OV5678RgbirSettings(**values)


def read_capture_raw(path, args):
    raw = decode_raw_frame(Path(path).read_bytes(), args.width, args.height)
    if args.downscale == 2:
        raw = downscale_raw2x(raw, cfa_period=4)
    return raw


def process_raw(raw, args, settings):
    bgr, norm_low, norm_high, _ir_half = process_ov5678_rgbir(
        raw,
        bgr_gains=(args.b_gain, args.g_gain, args.r_gain),
        settings=settings,
        norm_state={},
    )
    bgr = filter_bgr_ycrcb(bgr, args.chroma_filter_sigma, args.luma_filter_sigma)
    bgr = limit_bgr_luma(bgr, args.target_luma)

    if args.out_width and args.out_height:
        bgr = cv2.resize(
            bgr,
            (args.out_width, args.out_height),
            interpolation=cv2.INTER_AREA,
        )

    return bgr, norm_low, norm_high


def process_capture(path, args):
    raw = read_capture_raw(path, args)
    settings = make_settings(args)
    bgr, norm_low, norm_high = process_raw(raw, args, settings)

    output_path = None
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
        output_path = os.path.join(args.output_dir, Path(path).stem + "_ov5678.png")
        cv2.imwrite(output_path, bgr)

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    payload = {
        "input": str(path),
        "output": output_path,
        "norm_low": norm_low,
        "norm_high": norm_high,
        "pattern": {
            "name": settings.rgbir_pattern,
            "y_offset": settings.pattern_y_offset,
            "x_offset": settings.pattern_x_offset,
            "flip_h": settings.pattern_flip_h,
            "flip_v": settings.pattern_flip_v,
        },
        "phase_stats": rgbir_phase_stats(
            raw,
            black_level=args.black_level,
            pattern_name=settings.rgbir_pattern,
            y_offset=settings.pattern_y_offset,
            x_offset=settings.pattern_x_offset,
            flip_h=settings.pattern_flip_h,
            flip_v=settings.pattern_flip_v,
        ),
        "metrics": image_metrics(rgb),
    }
    if args.reference:
        payload["reference"] = args.reference
        payload["reference_metrics"] = image_metrics(load_rgb(args.reference))
    return payload


def mean_saturation(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    return float(hsv[..., 1].mean())


def score_against_reference(rgb, reference_rgb):
    ref = cv2.resize(reference_rgb, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_AREA)
    metrics = image_metrics(rgb)
    ref_metrics = image_metrics(ref)
    sat = mean_saturation(rgb)
    ref_sat = mean_saturation(ref)

    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
    ref_lab = cv2.cvtColor(ref, cv2.COLOR_RGB2LAB).astype(np.float32)
    gray = (
        0.299 * ref[..., 0].astype(np.float32)
        + 0.587 * ref[..., 1].astype(np.float32)
        + 0.114 * ref[..., 2].astype(np.float32)
    )
    mask = (gray > 20.0) & (gray < 235.0)
    if np.any(mask):
        lab_delta = float(np.mean(np.abs(lab[mask] - ref_lab[mask])))
    else:
        lab_delta = 255.0

    score = (
        abs(metrics["mean_luma"] - ref_metrics["mean_luma"]) * 0.8
        + abs(sat - ref_sat) * 1.6
        + lab_delta * 0.7
        + metrics["clip_any_frac_rgb_ge_250"] * 500.0
        + metrics["grid4_chroma_score"] * 120.0
        + max(0.0, 45.0 - sat) * 2.0
    )
    return {
        "score": float(score),
        "mean_saturation": sat,
        "reference_mean_saturation": ref_sat,
        "lab_delta_mean": lab_delta,
        "metrics": metrics,
        "reference_metrics": ref_metrics,
    }


def fit_color_matrix(rgb, reference_rgb):
    ref = cv2.resize(reference_rgb, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_AREA)
    src = rgb.astype(np.float32) / 255.0
    dst = ref.astype(np.float32) / 255.0
    gray = 0.299 * dst[..., 0] + 0.587 * dst[..., 1] + 0.114 * dst[..., 2]
    sat = cv2.cvtColor(ref, cv2.COLOR_RGB2HSV)[..., 1].astype(np.float32)
    mask = (gray > 0.08) & (gray < 0.92) & (sat > 15.0)
    if mask.sum() < 1000:
        mask = (gray > 0.08) & (gray < 0.92)

    src_samples = src[mask].reshape(-1, 3)
    dst_samples = dst[mask].reshape(-1, 3)
    if src_samples.shape[0] > 200000:
        step = max(1, src_samples.shape[0] // 200000)
        src_samples = src_samples[::step]
        dst_samples = dst_samples[::step]

    matrix, _residuals, _rank, _singular = np.linalg.lstsq(src_samples, dst_samples, rcond=None)
    # process_ov5678_rgbir uses rgb @ ccm.T, so transpose the least-squares
    # right-matrix before reporting constants suitable for CCM_PROFILES.
    return matrix.T


def candidate_iter(args):
    names = rgbir_pattern_names() if args.search_patterns == "all" else (args.search_patterns,)
    flips = [(False, False)]
    if args.search_flips:
        flips = [(False, False), (True, False), (False, True), (True, True)]
    for name in names:
        for y_offset in range(4):
            for x_offset in range(4):
                for flip_h, flip_v in flips:
                    yield {
                        "rgbir_pattern": name,
                        "pattern_y_offset": y_offset,
                        "pattern_x_offset": x_offset,
                        "pattern_flip_h": flip_h,
                        "pattern_flip_v": flip_v,
                    }


def search_capture(path, args):
    if not args.reference:
        raise ValueError("--reference is required for --search-patterns")
    raw = read_capture_raw(path, args)
    reference = load_rgb(args.reference)
    results = []
    os.makedirs(args.output_dir, exist_ok=True) if args.output_dir else None

    for candidate in candidate_iter(args):
        settings = make_settings(args, **candidate, dump_intermediates=None)
        bgr, norm_low, norm_high = process_raw(raw, args, settings)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        scored = score_against_reference(rgb, reference)
        result = {
            "input": str(path),
            "norm_low": norm_low,
            "norm_high": norm_high,
            "candidate": candidate,
            **scored,
        }
        results.append(result)

    results.sort(key=lambda item: item["score"])

    if args.output_dir:
        stem = Path(path).stem
        for rank, item in enumerate(results[: args.save_top], start=1):
            candidate = item["candidate"]
            settings = make_settings(args, **candidate, dump_intermediates=None)
            bgr, _norm_low, _norm_high = process_raw(raw, args, settings)
            name = (
                f"{stem}_rank{rank:02d}_{candidate['rgbir_pattern']}"
                f"_y{candidate['pattern_y_offset']}x{candidate['pattern_x_offset']}"
                f"_fh{int(candidate['pattern_flip_h'])}fv{int(candidate['pattern_flip_v'])}.png"
            )
            item["output"] = os.path.join(args.output_dir, name)
            cv2.imwrite(item["output"], bgr)

    payload = {"input": str(path), "reference": args.reference, "results": results[: args.top]}
    if args.fit_reference_profile and results:
        best = results[0]["candidate"]
        settings = make_settings(args, **best, dump_intermediates=None)
        bgr, _norm_low, _norm_high = process_raw(raw, args, settings)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        matrix = fit_color_matrix(rgb, reference)
        payload["fit"] = {
            "candidate": best,
            "ccm_for_profiles": matrix.tolist(),
        }
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="RAW frame files")
    parser.add_argument("--width", type=int, default=2592)
    parser.add_argument("--height", type=int, default=1944)
    parser.add_argument("--downscale", type=int, choices=(1, 2), default=2)
    parser.add_argument("--out-width", type=int, default=1280)
    parser.add_argument("--out-height", type=int, default=960)
    parser.add_argument("--output-dir")
    parser.add_argument("--reference", default="../good.jpg")
    parser.add_argument("--black-level", type=int, default=64)
    parser.add_argument("--white-level", type=int, default=1023)
    parser.add_argument("--r-gain", type=float, default=1.05)
    parser.add_argument("--g-gain", type=float, default=3.30)
    parser.add_argument("--b-gain", type=float, default=0.80)
    parser.add_argument("--ir-cut", default=0.04)
    parser.add_argument("--ir-clip", type=float, default=1.3)
    parser.add_argument("--raw-grid-correct", choices=("on", "off"), default="on")
    parser.add_argument("--rgbir-interp-sigma", type=float, default=1.50)
    parser.add_argument("--rgbir-gamma", type=float, default=0.55)
    parser.add_argument("--adaptive-low-percentile", type=float, default=0.5)
    parser.add_argument("--adaptive-high-percentile", type=float, default=96.0)
    parser.add_argument("--adaptive-min-span", type=float, default=12.0)
    parser.add_argument("--chroma-filter-sigma", type=float, default=1.6)
    parser.add_argument("--luma-filter-sigma", type=float, default=0.0)
    parser.add_argument("--target-luma", type=float, default=142.0)
    parser.add_argument(
        "--ccm-profile",
        choices=("none", "ov5678-indoor", "ov5678-cjfl515"),
        default="ov5678-cjfl515",
    )
    parser.add_argument("--awb-mode", choices=("fixed", "grayworld"), default="grayworld")
    parser.add_argument("--awb-strength", type=float, default=0.85)
    parser.add_argument("--color-saturation", type=float, default=None)
    parser.add_argument(
        "--rgbir-pattern",
        choices=rgbir_pattern_names(),
        default="windows-cjfl515",
    )
    parser.add_argument("--rgbir-pattern-y-offset", type=int, default=0)
    parser.add_argument("--rgbir-pattern-x-offset", type=int, default=0)
    parser.add_argument("--rgbir-pattern-flip-h", action="store_true")
    parser.add_argument("--rgbir-pattern-flip-v", action="store_true")
    parser.add_argument(
        "--search-patterns",
        choices=("all", *rgbir_pattern_names()),
        help="Score pattern/offset candidates against --reference",
    )
    parser.add_argument("--search-flips", action="store_true")
    parser.add_argument("--fit-reference-profile", action="store_true")
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--save-top", type=int, default=4)
    parser.add_argument("--dump-intermediates")
    args = parser.parse_args()

    if args.search_patterns:
        print(json.dumps([search_capture(path, args) for path in args.inputs], indent=2))
    else:
        print(json.dumps([process_capture(path, args) for path in args.inputs], indent=2))


if __name__ == "__main__":
    main()
