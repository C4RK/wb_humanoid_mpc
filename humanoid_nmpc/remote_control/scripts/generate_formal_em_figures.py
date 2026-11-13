#!/usr/bin/env python3
"""Generate Chapter 7 figures and summaries for the formal EM experiments.

The calibration figure uses the ten complete archived sessions S01--S10.
The collision figure uses the protected S06 diagnostic.  Each frame in that
file contains both the pre-filter clearance and the protected clearance, so
the comparison is paired without requiring a second human motion.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont


FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

COLORS = {
    "text": "#202124",
    "muted": "#5f6368",
    "grid": "#dfe3e7",
    "border": "#9aa0a6",
    "panel": "#fbfcfd",
    "upper": "#1565c0",
    "forearm": "#e65100",
    "raw": "#c62828",
    "safe": "#00897b",
    "correction": "#6a1b9a",
    "threshold": "#37474f",
}


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold else FONT_REGULAR
    if not path.exists():
        path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size)


def save_figure(image: Image.Image, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    image.save(output_dir / f"{stem}.png", dpi=(300, 300), optimize=True)
    image.save(output_dir / f"{stem}.pdf", "PDF", resolution=300.0)


def text_center(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    size: int,
    *,
    fill: str = COLORS["text"],
    bold: bool = False,
) -> None:
    draw.text(xy, value, font=font(size, bold), fill=fill, anchor="mm")


def load_calibrations(formal_dir: Path) -> list[dict[str, float | str | bool]]:
    rows: list[dict[str, float | str | bool]] = []
    for session_number in range(1, 11):
        session = f"S{session_number:02d}"
        path = formal_dir / session / f"wmet_dual_calibration_{session}.yaml"
        calibration = yaml.safe_load(path.read_text(encoding="utf-8"))["calibration"]
        quality = calibration["quality"]
        elbow_raw = np.asarray(quality["elbow_raw_errors_deg"], dtype=float)
        elbow_corrected = np.asarray(
            quality["elbow_corrected_errors_deg"], dtype=float
        )
        rows.append(
            {
                "session": session,
                "dry_run_only": bool(calibration["dry_run_only"]),
                "recorded_forearm_publish_limit_deg": float(
                    calibration.get("forearm_reference_publish_limit_deg", 15.0)
                ),
                "upper_static_position_rms_mm": 1000.0
                * quality["upper_position_rms_m"],
                "forearm_static_position_rms_mm": 1000.0
                * quality["forearm_position_rms_m"],
                "upper_static_angle_rms_deg": quality["upper_angle_rms_deg"],
                "forearm_static_angle_rms_deg": quality["forearm_angle_rms_deg"],
                "twist_motion_range_deg": quality["twist_axis_motion_range_deg"],
                "twist_axis_rms_deg": quality["twist_axis_rms_error_deg"],
                "upper_reference_max_deg": quality["upper_reference_max_error_deg"],
                "forearm_reference_max_deg": quality[
                    "forearm_reference_max_error_deg"
                ],
                "twist_repeatability_max_deg": quality[
                    "forearm_twist_max_error_deg"
                ],
                "position_model_rms_mm": 1000.0
                * quality["upper_position_model_rms_m"],
                "position_model_max_mm": 1000.0
                * quality["upper_position_model_max_m"],
                "elbow_raw_rms_deg": float(np.sqrt(np.mean(elbow_raw**2))),
                "elbow_corrected_rms_deg": float(
                    np.sqrt(np.mean(elbow_corrected**2))
                ),
                "elbow_corrected_max_deg": float(
                    np.max(np.abs(elbow_corrected))
                ),
            }
        )
    return rows


def draw_grouped_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    ylabel: str,
    sessions: list[str],
    series: list[tuple[str, list[float], str]],
    ymax: float,
    tick: float,
    thresholds: list[tuple[float, str]] = [],
) -> None:
    x0, y0, x1, y1 = box
    draw.rectangle(box, fill=COLORS["panel"], outline=COLORS["border"], width=2)
    draw.text((x0 + 20, y0 + 18), title, font=font(30, True), fill=COLORS["text"])
    plot_left, plot_right = x0 + 92, x1 - 25
    plot_top, plot_bottom = y0 + 78, y1 - 82
    plot_height = plot_bottom - plot_top
    for value in np.arange(0.0, ymax + tick * 0.1, tick):
        py = plot_bottom - value / ymax * plot_height
        draw.line((plot_left, py, plot_right, py), fill=COLORS["grid"], width=2)
        draw.text(
            (plot_left - 12, py),
            f"{value:g}",
            font=font(20),
            fill=COLORS["muted"],
            anchor="rm",
        )
    for value, label in thresholds:
        threshold_color = COLORS["threshold"]
        if "upper-arm" in label:
            threshold_color = COLORS["upper"]
        elif "forearm" in label:
            threshold_color = COLORS["forearm"]
        py = plot_bottom - value / ymax * plot_height
        draw.line(
            (plot_left, py, plot_right, py),
            fill=threshold_color,
            width=3,
        )
        draw.text(
            (plot_right - 5, py - 6),
            label,
            font=font(19),
            fill=threshold_color,
            anchor="rb",
        )
    group_width = (plot_right - plot_left) / len(sessions)
    bar_gap = 7
    bar_width = min(46, (group_width - 28) / len(series) - bar_gap)
    for group_index, session in enumerate(sessions):
        center = plot_left + (group_index + 0.5) * group_width
        total_width = len(series) * bar_width + (len(series) - 1) * bar_gap
        start = center - total_width / 2
        for series_index, (_, values, color) in enumerate(series):
            value = values[group_index]
            left = start + series_index * (bar_width + bar_gap)
            right = left + bar_width
            top = plot_bottom - min(value, ymax) / ymax * plot_height
            draw.rectangle((left, top, right, plot_bottom), fill=color)
        draw.text(
            (center, plot_bottom + 14),
            session,
            font=font(21),
            fill=COLORS["text"],
            anchor="ma",
        )
    draw.text(
        (plot_left, plot_top - 10),
        ylabel,
        font=font(19),
        fill=COLORS["muted"],
        anchor="lb",
    )
    legend_x = plot_left
    legend_y = y1 - 27
    for name, _, color in series:
        draw.rectangle((legend_x, legend_y - 10, legend_x + 25, legend_y + 10), fill=color)
        draw.text(
            (legend_x + 35, legend_y),
            name,
            font=font(19),
            fill=COLORS["text"],
            anchor="lm",
        )
        legend_x += 35 + int(draw.textlength(name, font=font(19))) + 28


def draw_calibration_figure(rows: list[dict], output_dir: Path) -> None:
    width, height = 2800, 1700
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    text_center(draw, (width / 2, 58), "Formal repeated EM calibration quality", 48, bold=True)
    text_center(
        draw,
        (width / 2, 110),
        "S01--S10 contain complete reference NPZ and calibration YAML files",
        25,
        fill=COLORS["muted"],
    )
    sessions = [str(row["session"]) for row in rows]
    left, right, top, bottom, gap = 70, 70, 155, 60, 52
    panel_width = (width - left - right - gap) // 2
    panel_height = (height - top - bottom - gap) // 2
    boxes = [
        (left, top, left + panel_width, top + panel_height),
        (left + panel_width + gap, top, width - right, top + panel_height),
        (left, top + panel_height + gap, left + panel_width, height - bottom),
        (left + panel_width + gap, top + panel_height + gap, width - right, height - bottom),
    ]
    draw_grouped_panel(
        draw,
        boxes[0],
        "(a) Static position noise",
        "RMS (mm)",
        sessions,
        [
            ("Upper arm", [r["upper_static_position_rms_mm"] for r in rows], COLORS["upper"]),
            ("Forearm", [r["forearm_static_position_rms_mm"] for r in rows], COLORS["forearm"]),
        ],
        3.5,
        0.5,
        [(3.0, "3 mm limit")],
    )
    draw_grouped_panel(
        draw,
        boxes[1],
        "(b) Reference-direction residual",
        "Maximum (deg)",
        sessions,
        [
            ("Upper arm", [r["upper_reference_max_deg"] for r in rows], COLORS["upper"]),
            ("Forearm", [r["forearm_reference_max_deg"] for r in rows], COLORS["forearm"]),
        ],
        24.0,
        4.0,
        [
            (15.0, "upper-arm hard limit"),
            (20.0, "forearm publish limit"),
        ],
    )
    draw_grouped_panel(
        draw,
        boxes[2],
        "(c) Upper-arm affine position model",
        "Error (mm)",
        sessions,
        [
            ("RMS", [r["position_model_rms_mm"] for r in rows], COLORS["upper"]),
            ("Maximum", [r["position_model_max_mm"] for r in rows], COLORS["forearm"]),
        ],
        70.0,
        10.0,
        [(35.0, "RMS limit"), (65.0, "maximum limit")],
    )
    draw_grouped_panel(
        draw,
        boxes[3],
        "(d) Elbow reference fit",
        "RMS (deg)",
        sessions,
        [
            ("Before correction", [r["elbow_raw_rms_deg"] for r in rows], COLORS["raw"]),
            ("After correction", [r["elbow_corrected_rms_deg"] for r in rows], COLORS["safe"]),
        ],
        15.0,
        3.0,
    )
    save_figure(image, output_dir, "em_formal_calibration_quality")


def split_polyline(
    x: np.ndarray,
    y: np.ndarray,
    t: np.ndarray,
    *,
    max_gap_s: float = 0.1,
) -> list[list[tuple[float, float]]]:
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for index, (px, py) in enumerate(zip(x, y)):
        if index and t[index] - t[index - 1] > max_gap_s:
            if len(current) > 1:
                segments.append(current)
            current = []
        current.append((float(px), float(py)))
    if len(current) > 1:
        segments.append(current)
    return segments


def draw_collision_figure(collision_path: Path, output_dir: Path) -> dict[str, float]:
    with np.load(collision_path, allow_pickle=False) as data:
        stamp_ns = np.asarray(data["stamp_ns"], dtype=np.int64)
        raw_mm = 1000.0 * np.asarray(data["collision_raw_clearance_m"], dtype=float)
        safe_mm = 1000.0 * np.asarray(data["collision_safe_clearance_m"], dtype=float)
        active = np.asarray(data["collision_filter_active"], dtype=bool)
        held = np.asarray(data["collision_filter_held"], dtype=bool)
        adjustment_deg = np.rad2deg(
            np.asarray(data["collision_roll_adjustment_rad"], dtype=float)
        )
        limiting = np.asarray(data["collision_limiting_link"]).astype(str)
        margin_mm = 1000.0 * float(data["torso_collision_margin_m"])
    time_s = (stamp_ns - stamp_ns[0]) * 1e-9

    width, height = 2400, 1450
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    text_center(draw, (width / 2, 58), "Torso-collision protection: paired frame comparison", 48, bold=True)
    text_center(
        draw,
        (width / 2, 110),
        "Pre-filter target and protected target are evaluated on the same 8,460 frames",
        25,
        fill=COLORS["muted"],
    )
    left, right, top, bottom = 150, 55, 175, 100
    gap = 95
    panel_height = (height - top - bottom - gap) / 2
    duration = float(time_s[-1])

    def panel(
        y_top: float,
        title: str,
        y_min: float,
        y_max: float,
        y_tick: float,
        ylabel: str,
    ) -> tuple[float, float, float, float]:
        x0, x1 = left, width - right
        y0, y1 = y_top, y_top + panel_height
        draw.rectangle((x0, y0, x1, y1), fill=COLORS["panel"], outline=COLORS["border"], width=2)
        draw.text((x0 + 20, y0 + 14), title, font=font(29, True), fill=COLORS["text"])
        plot = (x0 + 90, y0 + 65, x1 - 22, y1 - 65)
        px0, py0, px1, py1 = plot
        for value in np.arange(y_min, y_max + y_tick * 0.1, y_tick):
            py = py1 - (value - y_min) / (y_max - y_min) * (py1 - py0)
            draw.line((px0, py, px1, py), fill=COLORS["grid"], width=2)
            draw.text((px0 - 12, py), f"{value:g}", font=font(19), fill=COLORS["muted"], anchor="rm")
        for seconds in np.arange(0.0, duration + 0.1, 30.0):
            px = px0 + seconds / duration * (px1 - px0)
            draw.line((px, py0, px, py1), fill=COLORS["grid"], width=2)
            draw.text((px, py1 + 12), f"{seconds:g}", font=font(19), fill=COLORS["muted"], anchor="ma")
        draw.text((px0, py0 - 10), ylabel, font=font(19), fill=COLORS["muted"], anchor="lb")
        draw.text(((px0 + px1) / 2, y1 - 22), "Time (s)", font=font(21), fill=COLORS["muted"], anchor="mm")
        return plot

    clearance_plot = panel(top, "(a) Torso clearance", -10.0, 130.0, 20.0, "mm")
    px0, py0, px1, py1 = clearance_plot
    x = px0 + time_s / duration * (px1 - px0)
    raw_y = py1 - (raw_mm + 10.0) / 140.0 * (py1 - py0)
    safe_y = py1 - (safe_mm + 10.0) / 140.0 * (py1 - py0)
    for points in split_polyline(x, raw_y, time_s):
        draw.line(points, fill=COLORS["raw"], width=3)
    for points in split_polyline(x, safe_y, time_s):
        draw.line(points, fill=COLORS["safe"], width=4)
    margin_y = py1 - (margin_mm + 10.0) / 140.0 * (py1 - py0)
    draw.line((px0, margin_y, px1, margin_y), fill=COLORS["threshold"], width=3)
    legend_y = py0 + 16
    legend_x = px0 + 30
    for label, color in (("Pre-filter", COLORS["raw"]), ("Protected", COLORS["safe"]), ("25 mm margin", COLORS["threshold"])):
        draw.line((legend_x, legend_y, legend_x + 55, legend_y), fill=color, width=5)
        draw.text((legend_x + 68, legend_y), label, font=font(20), fill=COLORS["text"], anchor="lm")
        legend_x += 68 + int(draw.textlength(label, font=font(20))) + 45

    correction_plot = panel(top + panel_height + gap, "(b) Safety intervention", 0.0, 16.0, 2.0, "deg")
    px0, py0, px1, py1 = correction_plot
    correction_y = py1 - adjustment_deg / 16.0 * (py1 - py0)
    for points in split_polyline(x, correction_y, time_s):
        draw.line(points, fill=COLORS["correction"], width=4)
    draw.text((px0 + 30, py0 + 18), "Shoulder-roll correction", font=font(20), fill=COLORS["correction"], anchor="lm")

    positive = active
    return {
        "frames": float(len(time_s)),
        "duration_s": duration,
        "effective_rate_hz": float((len(time_s) - 1) / duration),
        "margin_mm": margin_mm,
        "raw_min_mm": float(np.min(raw_mm)),
        "raw_median_mm": float(np.median(raw_mm)),
        "raw_p05_mm": float(np.quantile(raw_mm, 0.05)),
        "safe_min_mm": float(np.min(safe_mm)),
        "safe_median_mm": float(np.median(safe_mm)),
        "safe_p05_mm": float(np.quantile(safe_mm, 0.05)),
        "active_frames": float(np.count_nonzero(active)),
        "active_fraction_percent": float(100.0 * np.mean(active)),
        "held_frames": float(np.count_nonzero(held)),
        "roll_adjustment_active_median_deg": float(np.median(adjustment_deg[positive])),
        "roll_adjustment_active_p95_deg": float(np.quantile(adjustment_deg[positive], 0.95)),
        "roll_adjustment_max_deg": float(np.max(adjustment_deg)),
        "limiting_upper_arm_frames": float(np.count_nonzero(limiting[active] == "upper_arm")),
        "limiting_forearm_frames": float(np.count_nonzero(limiting[active] == "forearm")),
        "limiting_hand_frames": float(np.count_nonzero(limiting[active] == "hand")),
    } | {"_image": image}


def write_calibration_csv(rows: list[dict], output_dir: Path) -> None:
    path = output_dir / "em_formal_calibration_summary.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_collision_csv(summary: dict[str, float], output_dir: Path) -> None:
    path = output_dir / "collision_guard_summary.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "value"))
        for key, value in summary.items():
            if not key.startswith("_"):
                writer.writerow((key, value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--formal-em-dir",
        type=Path,
        default=Path("/home/kunwang/humanoid_mpc_ws/calibration_logs/formal_em"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "figures",
    )
    args = parser.parse_args()
    rows = load_calibrations(args.formal_em_dir)
    draw_calibration_figure(rows, args.output_dir)
    write_calibration_csv(rows, args.output_dir)

    collision_candidates = sorted(
        (args.formal_em_dir / "S06" / "collision_guard_on").glob(
            "dual_retargeting_*.npz"
        )
    )
    if len(collision_candidates) != 1:
        raise RuntimeError(
            f"Expected exactly one S06 collision record, found {len(collision_candidates)}"
        )
    collision_summary = draw_collision_figure(
        collision_candidates[0], args.output_dir
    )
    collision_image = collision_summary.pop("_image")
    save_figure(collision_image, args.output_dir, "collision_guard_clearance")
    write_collision_csv(collision_summary, args.output_dir)
    print(args.output_dir / "em_formal_calibration_quality.pdf")
    print(args.output_dir / "collision_guard_clearance.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
