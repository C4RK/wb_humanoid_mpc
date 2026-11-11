#!/usr/bin/env python3
"""Generate the three thesis figures for the formal G1 R0/R1/R2 trials.

The bridge CSV files contain absolute-alignment samples before normal tracking.
For the frozen formal protocol, the normal tracking segment is therefore the
last 3,001 rows for R0 and the last 12,001 rows for R1/R2.  The file grouping
below is the audited grouping used by Chapter 7.  Figures deliberately avoid
dates and raw filenames; the generated manifest preserves data provenance.

Only Pillow and the Python standard library are required.  This keeps the
figure build usable in the project's existing Conda environment, where NumPy
and Matplotlib are not guaranteed to be installed.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
from typing import Iterable, Sequence

from PIL import Image, ImageDraw, ImageFont


JOINTS = ("pitch", "roll", "yaw", "elbow", "wrist")
JOINT_LABELS = {
    "pitch": "Shoulder pitch",
    "roll": "Shoulder roll",
    "yaw": "Shoulder yaw",
    "elbow": "Elbow",
    "wrist": "Wrist roll",
}
JOINT_SHORT_LABELS = {
    "pitch": "Pitch",
    "roll": "Roll",
    "yaw": "Yaw",
    "elbow": "Elbow",
    "wrist": "Wrist",
}
ACTUAL_COLUMNS = {
    "pitch": "actual_left_shoulder_pitch_rad",
    "roll": "actual_left_shoulder_roll_rad",
    "yaw": "actual_left_shoulder_yaw_rad",
    "elbow": "actual_left_elbow_rad",
    "wrist": "actual_left_wrist_roll_rad",
}

PROTOCOL_FILES = {
    "R0": (
        "g1_arm_tracking_20261110T152251Z.csv",
        "g1_arm_tracking_20261110T153952Z.csv",
        "g1_arm_tracking_20261110T154205Z.csv",
    ),
    "R1": (
        "g1_arm_tracking_20261110T153110Z.csv",
        "g1_arm_tracking_20261110T162625Z.csv",
        "g1_arm_tracking_20261110T164319Z.csv",
    ),
    "R2": (
        "g1_arm_tracking_20261110T153633Z.csv",
        "g1_arm_tracking_20261110T163243Z.csv",
        "g1_arm_tracking_20261110T163802Z.csv",
    ),
}
TRACKING_ROWS = {"R0": 3001, "R1": 12001, "R2": 12001}
PROTOCOL_LABELS = {
    "R0": "R0 static hold",
    "R1": "R1 dynamic, 5 deg/s",
    "R2": "R2 dynamic, 8 deg/s",
}

COLORS = {
    "text": "#202124",
    "muted": "#5f6368",
    "grid": "#dfe3e7",
    "border": "#9aa0a6",
    "background": "#ffffff",
    "panel": "#fbfcfd",
    "command": "#1565c0",
    "actual": "#e65100",
    "R0": "#5e35b1",
    "R1": "#00897b",
    "R2": "#ef6c00",
    "threshold": "#c62828",
}

FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


@dataclass
class Trial:
    protocol: str
    repeat: int
    path: Path
    time_s: list[float]
    command_deg: dict[str, list[float]]
    actual_deg: dict[str, list[float]]
    target_age_ms: list[float]

    def errors_deg(self, joint: str) -> list[float]:
        return [
            command - actual
            for command, actual in zip(
                self.command_deg[joint], self.actual_deg[joint]
            )
        ]

    def rmse(self, joint: str) -> float:
        errors = self.errors_deg(joint)
        return math.sqrt(sum(value * value for value in errors) / len(errors))

    def p95(self, joint: str) -> float:
        return percentile([abs(value) for value in self.errors_deg(joint)], 0.95)

    def composite_rmse(self) -> float:
        return statistics.mean(self.rmse(joint) for joint in JOINTS)


def percentile(values: Sequence[float], fraction: float) -> float:
    """Linear percentile, matching NumPy's default interpolation."""
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def load_trial(protocol: str, repeat: int, path: Path) -> Trial:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    expected = TRACKING_ROWS[protocol]
    if len(rows) < expected:
        raise ValueError(
            f"{path} has {len(rows)} rows; {protocol} needs {expected}"
        )
    rows = rows[-expected:]
    start_ns = int(rows[0]["monotonic_time_ns"])
    time_s = [
        (int(row["monotonic_time_ns"]) - start_ns) / 1e9 for row in rows
    ]
    command = {
        joint: [math.degrees(float(row[f"command_{joint}_rad"])) for row in rows]
        for joint in JOINTS
    }
    actual = {
        joint: [
            math.degrees(float(row[ACTUAL_COLUMNS[joint]])) for row in rows
        ]
        for joint in JOINTS
    }
    return Trial(
        protocol=protocol,
        repeat=repeat,
        path=path,
        time_s=time_s,
        command_deg=command,
        actual_deg=actual,
        target_age_ms=[float(row["udp_target_age_ms"]) for row in rows],
    )


def load_trials(log_dir: Path) -> dict[str, list[Trial]]:
    groups: dict[str, list[Trial]] = {}
    for protocol, filenames in PROTOCOL_FILES.items():
        groups[protocol] = [
            load_trial(protocol, repeat, log_dir / filename)
            for repeat, filename in enumerate(filenames, start=1)
        ]
    return groups


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold else FONT_REGULAR
    if not path.exists():
        path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size)


def text_center(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    size: int,
    fill: str = COLORS["text"],
    bold: bool = False,
) -> None:
    draw.text(xy, value, font=font(size, bold), fill=fill, anchor="mm")


def vertical_text(
    image: Image.Image,
    center: tuple[int, int],
    value: str,
    size: int,
    fill: str = COLORS["text"],
) -> None:
    """Draw a compact bottom-to-top axis label without overlapping ticks."""
    label_font = font(size)
    scratch = Image.new("RGBA", (1000, 120), (255, 255, 255, 0))
    scratch_draw = ImageDraw.Draw(scratch)
    bbox = scratch_draw.textbbox((0, 0), value, font=label_font)
    scratch_draw.text((8, 8), value, font=label_font, fill=fill)
    cropped = scratch.crop((0, 0, bbox[2] + 16, bbox[3] - bbox[1] + 20))
    rotated = cropped.rotate(90, expand=True, resample=Image.Resampling.BICUBIC)
    position = (
        int(center[0] - rotated.width / 2),
        int(center[1] - rotated.height / 2),
    )
    image.paste(rotated, position, rotated)


def save_figure(image: Image.Image, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    png_path = output_dir / f"{stem}.png"
    pdf_path = output_dir / f"{stem}.pdf"
    image.save(png_path, dpi=(300, 300), optimize=True)
    image.save(pdf_path, "PDF", resolution=300.0)
    print(png_path)
    print(pdf_path)


def nice_bounds(values: Iterable[float]) -> tuple[float, float, float]:
    values = list(values)
    low, high = min(values), max(values)
    span = max(high - low, 1.0)
    raw_step = span / 4.0
    magnitude = 10 ** math.floor(math.log10(raw_step))
    normalized = raw_step / magnitude
    if normalized <= 1.0:
        step = magnitude
    elif normalized <= 2.0:
        step = 2.0 * magnitude
    elif normalized <= 5.0:
        step = 5.0 * magnitude
    else:
        step = 10.0 * magnitude
    lower = math.floor((low - 0.08 * span) / step) * step
    upper = math.ceil((high + 0.08 * span) / step) * step
    if upper <= lower:
        upper = lower + step
    return lower, upper, step


def draw_legend_line(
    draw: ImageDraw.ImageDraw,
    x: int,
    y: int,
    color: str,
    label: str,
    line_width: int = 6,
) -> int:
    draw.line((x, y, x + 65, y), fill=color, width=line_width)
    draw.text(
        (x + 78, y), label, font=font(27), fill=COLORS["text"], anchor="lm"
    )
    return x + 78 + int(draw.textlength(label, font=font(27))) + 50


def representative_trial(trials: Sequence[Trial]) -> Trial:
    """Return the median composite-RMSE repeat without visual cherry-picking."""
    ordered = sorted(trials, key=lambda trial: trial.composite_rmse())
    return ordered[len(ordered) // 2]


def draw_timeseries(groups: dict[str, list[Trial]], output_dir: Path) -> None:
    representatives = {
        protocol: representative_trial(groups[protocol])
        for protocol in ("R1", "R2")
    }
    width, height = 2600, 2550
    image = Image.new("RGB", (width, height), COLORS["background"])
    draw = ImageDraw.Draw(image)
    text_center(
        draw,
        (width / 2, 60),
        "Formal G1 tracking: commanded and measured joint angles",
        48,
        bold=True,
    )
    text_center(
        draw,
        (width / 2, 118),
        "Representative repeat selected by median five-joint composite RMSE",
        28,
        fill=COLORS["muted"],
    )
    legend_x = 980
    legend_x = draw_legend_line(
        draw, legend_x, 165, COLORS["command"], "Safe command"
    )
    draw_legend_line(draw, legend_x, 165, COLORS["actual"], "Encoder angle")

    left_margin, right_margin = 185, 65
    column_gap, row_gap = 105, 28
    top, bottom = 245, 120
    panel_width = (width - left_margin - right_margin - column_gap) / 2
    panel_height = (height - top - bottom - row_gap * 4) / 5

    for column, protocol in enumerate(("R1", "R2")):
        cx = (
            left_margin
            + column * (panel_width + column_gap)
            + panel_width / 2
        )
        text_center(draw, (cx, top - 42), PROTOCOL_LABELS[protocol], 32, bold=True)

    for row_index, joint in enumerate(JOINTS):
        row_values: list[float] = []
        for protocol in ("R1", "R2"):
            trial = representatives[protocol]
            row_values.extend(trial.command_deg[joint])
            row_values.extend(trial.actual_deg[joint])
        y_min, y_max, y_step = nice_bounds(row_values)

        row_y = top + row_index * (panel_height + row_gap)
        draw.text(
            (20, row_y + panel_height / 2),
            JOINT_SHORT_LABELS[joint],
            font=font(28, True),
            fill=COLORS["text"],
            anchor="lm",
        )

        for column, protocol in enumerate(("R1", "R2")):
            trial = representatives[protocol]
            x0 = left_margin + column * (panel_width + column_gap)
            y0 = row_y
            x1, y1 = x0 + panel_width, y0 + panel_height
            draw.rectangle((x0, y0, x1, y1), fill=COLORS["panel"], outline=COLORS["border"], width=2)

            tick = y_min
            while tick <= y_max + y_step * 0.1:
                py = y1 - (tick - y_min) / (y_max - y_min) * panel_height
                draw.line((x0, py, x1, py), fill=COLORS["grid"], width=2)
                draw.text(
                    (x0 - 12, py),
                    f"{tick:g}",
                    font=font(20),
                    fill=COLORS["muted"],
                    anchor="rm",
                )
                tick += y_step

            duration = trial.time_s[-1]
            for seconds in (0, 60, 120, 180, 240):
                if seconds > duration + 1.0:
                    continue
                px = x0 + seconds / 240.0 * panel_width
                draw.line((px, y0, px, y1), fill=COLORS["grid"], width=2)
                if row_index == len(JOINTS) - 1:
                    draw.text(
                        (px, y1 + 12),
                        str(seconds),
                        font=font(20),
                        fill=COLORS["muted"],
                        anchor="ma",
                    )

            stride = max(1, len(trial.time_s) // 3500)

            def points(values: Sequence[float]) -> list[tuple[float, float]]:
                indices = list(range(0, len(values), stride))
                if indices[-1] != len(values) - 1:
                    indices.append(len(values) - 1)
                return [
                    (
                        x0 + trial.time_s[index] / 240.0 * panel_width,
                        y1
                        - (values[index] - y_min)
                        / (y_max - y_min)
                        * panel_height,
                    )
                    for index in indices
                ]

            draw.line(
                points(trial.command_deg[joint]),
                fill=COLORS["command"],
                width=5,
                joint="curve",
            )
            draw.line(
                points(trial.actual_deg[joint]),
                fill=COLORS["actual"],
                width=3,
                joint="curve",
            )

    text_center(draw, (width / 2, height - 35), "Tracking time (s)", 28)
    draw.text(
        (22, 205), "Angle (deg)", font=font(25), fill=COLORS["muted"]
    )
    save_figure(image, output_dir, "g1_formal_tracking_timeseries")


def protocol_metric(
    trials: Sequence[Trial], metric: str
) -> tuple[list[float], list[float]]:
    means, standard_deviations = [], []
    for joint in JOINTS:
        values = [getattr(trial, metric)(joint) for trial in trials]
        means.append(statistics.mean(values))
        standard_deviations.append(statistics.stdev(values))
    return means, standard_deviations


def draw_bar_panel(
    draw: ImageDraw.ImageDraw,
    bounds: tuple[float, float, float, float],
    title: str,
    data: dict[str, tuple[list[float], list[float]]],
    y_max: float,
) -> None:
    x0, y0, x1, y1 = bounds
    draw.rectangle(bounds, fill=COLORS["panel"], outline=COLORS["border"], width=2)
    text_center(draw, ((x0 + x1) / 2, y0 - 42), title, 34, bold=True)
    plot_height = y1 - y0
    for tick_index in range(0, 7):
        tick = y_max * tick_index / 6
        py = y1 - tick / y_max * plot_height
        draw.line((x0, py, x1, py), fill=COLORS["grid"], width=2)
        draw.text(
            (x0 - 12, py),
            f"{tick:.1f}",
            font=font(21),
            fill=COLORS["muted"],
            anchor="rm",
        )

    group_width = (x1 - x0) / len(JOINTS)
    bar_width = group_width * 0.20
    protocols = ("R0", "R1", "R2")
    for joint_index, joint in enumerate(JOINTS):
        group_center = x0 + (joint_index + 0.5) * group_width
        for protocol_index, protocol in enumerate(protocols):
            means, deviations = data[protocol]
            value = means[joint_index]
            deviation = deviations[joint_index]
            center = group_center + (protocol_index - 1) * bar_width * 1.15
            top = y1 - value / y_max * plot_height
            draw.rectangle(
                (center - bar_width / 2, top, center + bar_width / 2, y1),
                fill=COLORS[protocol],
            )
            error_top = y1 - (value + deviation) / y_max * plot_height
            error_bottom = y1 - max(0.0, value - deviation) / y_max * plot_height
            draw.line((center, error_top, center, error_bottom), fill=COLORS["text"], width=3)
            draw.line((center - 10, error_top, center + 10, error_top), fill=COLORS["text"], width=3)
            draw.line((center - 10, error_bottom, center + 10, error_bottom), fill=COLORS["text"], width=3)
        text_center(
            draw,
            (group_center, y1 + 32),
            JOINT_LABELS[joint].replace("Shoulder ", ""),
            22,
        )


def draw_error_summary(groups: dict[str, list[Trial]], output_dir: Path) -> None:
    width, height = 2500, 1350
    image = Image.new("RGB", (width, height), COLORS["background"])
    draw = ImageDraw.Draw(image)
    text_center(
        draw,
        (width / 2, 60),
        "Formal R0/R1/R2 tracking-error summary",
        48,
        bold=True,
    )
    text_center(
        draw,
        (width / 2, 115),
        "Bars: mean of three independent repeats; error bars: sample standard deviation",
        28,
        fill=COLORS["muted"],
    )

    legend_x = 880
    for protocol in ("R0", "R1", "R2"):
        draw.rectangle(
            (legend_x, 151, legend_x + 34, 185), fill=COLORS[protocol]
        )
        draw.text(
            (legend_x + 48, 168),
            protocol,
            font=font(26),
            fill=COLORS["text"],
            anchor="lm",
        )
        legend_x += 180

    rmse = {protocol: protocol_metric(trials, "rmse") for protocol, trials in groups.items()}
    p95 = {protocol: protocol_metric(trials, "p95") for protocol, trials in groups.items()}
    left = (150, 270, 1190, 1160)
    right = (1390, 270, 2430, 1160)
    y_max = 3.0
    draw_bar_panel(draw, left, "RMSE", rmse, y_max)
    draw_bar_panel(draw, right, "P95 absolute error", p95, y_max)
    vertical_text(
        image,
        (38, int((left[1] + left[3]) / 2)),
        "Angle error (deg)",
        27,
    )
    text_center(
        draw,
        (width / 2, height - 45),
        "All protocol mean values are at most 2.27 deg; the configured hard stop was 5 deg.",
        27,
        fill=COLORS["muted"],
    )
    save_figure(image, output_dir, "g1_formal_error_summary")


def empirical_cdf_points(values: Sequence[float], count: int = 2500):
    ordered = sorted(value for value in values if math.isfinite(value) and value >= 0.0)
    if not ordered:
        raise ValueError("No finite target-age samples")
    indices = sorted(
        {
            int(round(index * (len(ordered) - 1) / (count - 1)))
            for index in range(count)
        }
    )
    return [(ordered[index], 100.0 * (index + 1) / len(ordered)) for index in indices]


def draw_target_age_cdf(groups: dict[str, list[Trial]], output_dir: Path) -> None:
    width, height = 2300, 1400
    image = Image.new("RGB", (width, height), COLORS["background"])
    draw = ImageDraw.Draw(image)
    text_center(
        draw,
        (width / 2, 62),
        "Formal trials: empirical CDF of UDP target age",
        48,
        bold=True,
    )
    text_center(
        draw,
        (width / 2, 116),
        "Target age measures command freshness at the bridge, not pure network round-trip time",
        27,
        fill=COLORS["muted"],
    )

    x0, y0, x1, y1 = 190, 245, 2220, 1210
    draw.rectangle((x0, y0, x1, y1), fill=COLORS["panel"], outline=COLORS["border"], width=2)
    x_min, x_max = 0.1, 1000.0

    def map_x(value: float) -> float:
        clipped = min(max(value, x_min), x_max)
        return x0 + (math.log10(clipped) - math.log10(x_min)) / (
            math.log10(x_max) - math.log10(x_min)
        ) * (x1 - x0)

    def map_y(percent: float) -> float:
        return y1 - percent / 100.0 * (y1 - y0)

    for percent in (0, 20, 40, 60, 80, 90, 95, 100):
        py = map_y(percent)
        draw.line((x0, py, x1, py), fill=COLORS["grid"], width=2)
        draw.text(
            (x0 - 14, py),
            str(percent),
            font=font(22),
            fill=COLORS["muted"],
            anchor="rm",
        )

    x_ticks = (0.1, 1, 10, 100, 250, 750, 1000)
    for value in x_ticks:
        px = map_x(value)
        draw.line((px, y0, px, y1), fill=COLORS["grid"], width=2)
        label = f"{value:g}"
        draw.text(
            (px, y1 + 14),
            label,
            font=font(22),
            fill=COLORS["muted"],
            anchor="ma",
        )

    for threshold, label, anchor in (
        (250.0, "250 ms reference", "ra"),
        (750.0, "750 ms stale stop", "ra"),
    ):
        px = map_x(threshold)
        draw.line((px, y0, px, y1), fill=COLORS["threshold"], width=4)
        offset = -10 if anchor == "ra" else 10
        draw.text(
            (px + offset, y0 + 12),
            label,
            font=font(22, True),
            fill=COLORS["threshold"],
            anchor=anchor,
        )

    legend_y = 175
    legend_x = 500
    for protocol in ("R0", "R1", "R2"):
        values = [age for trial in groups[protocol] for age in trial.target_age_ms]
        p95 = percentile(values, 0.95)
        over_250 = 100.0 * sum(value > 250.0 for value in values) / len(values)
        points = [
            (map_x(value), map_y(percent))
            for value, percent in empirical_cdf_points(values)
        ]
        draw.line(points, fill=COLORS[protocol], width=7, joint="curve")
        label = f"{protocol}: P95={p95:.2f} ms, >250 ms={over_250:.3f}%"
        legend_x = draw_legend_line(
            draw, legend_x, legend_y, COLORS[protocol], label, line_width=7
        )

    text_center(draw, ((x0 + x1) / 2, height - 70), "UDP target age (ms, logarithmic scale)", 29)
    vertical_text(
        image,
        (43, int((y0 + y1) / 2)),
        "Cumulative probability (%)",
        28,
    )
    save_figure(image, output_dir, "g1_formal_target_age_cdf")


def write_manifest(groups: dict[str, list[Trial]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    representatives = {
        protocol: representative_trial(groups[protocol])
        for protocol in ("R1", "R2")
    }
    path = output_dir / "g1_formal_figure_manifest.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "protocol",
            "repeat",
            "raw_csv",
            "formal_tracking_rows",
            "formal_duration_s",
            "composite_rmse_deg",
            "selected_for_timeseries",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for protocol in ("R0", "R1", "R2"):
            for trial in groups[protocol]:
                writer.writerow(
                    {
                        "protocol": protocol,
                        "repeat": trial.repeat,
                        "raw_csv": trial.path.name,
                        "formal_tracking_rows": len(trial.time_s),
                        "formal_duration_s": f"{trial.time_s[-1]:.6f}",
                        "composite_rmse_deg": f"{trial.composite_rmse():.6f}",
                        "selected_for_timeseries": (
                            "yes"
                            if representatives.get(protocol) is trial
                            else "no"
                        ),
                    }
                )
    print(path)


def main() -> int:
    repository = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log-dir", type=Path, default=repository / "calibration_logs"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "figures",
    )
    args = parser.parse_args()

    groups = load_trials(args.log_dir)
    draw_timeseries(groups, args.output_dir)
    draw_error_summary(groups, args.output_dir)
    draw_target_age_cdf(groups, args.output_dir)
    write_manifest(groups, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
