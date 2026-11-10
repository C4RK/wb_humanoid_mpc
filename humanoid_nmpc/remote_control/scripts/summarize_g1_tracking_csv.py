#!/usr/bin/env python3
"""Create reproducible, whole-file summaries of G1 arm tracking CSV logs.

The bridge CSV format does not contain phase or operator-action labels.  This
script therefore reports objective whole-file statistics only.  For absolute
alignment runs, the result includes the alignment samples that precede normal
tracking.  Trial labels and completion status must be added from a terminal
transcript rather than inferred from the numbers here.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path
import re
import statistics


JOINTS = ("pitch", "roll", "yaw", "elbow", "wrist")
ACTUAL_COLUMNS = {
    "pitch": "actual_left_shoulder_pitch_rad",
    "roll": "actual_left_shoulder_roll_rad",
    "yaw": "actual_left_shoulder_yaw_rad",
    "elbow": "actual_left_elbow_rad",
    "wrist": "actual_left_wrist_roll_rad",
}
STAMP_RE = re.compile(r"g1_arm_tracking_(\d{8})T(\d{6})Z\.csv$")


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    return ordered[int(fraction * (len(ordered) - 1))]


def degrees(values: list[float]) -> list[float]:
    return [math.degrees(value) for value in values]


def summarize(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} contains no data rows")

    match = STAMP_RE.search(path.name)
    date_utc = match.group(1) if match else "unknown"
    time_utc = match.group(2) if match else "unknown"
    duration_s = (
        int(rows[-1]["monotonic_time_ns"])
        - int(rows[0]["monotonic_time_ns"])
    ) / 1e9
    target_ages = [float(row["udp_target_age_ms"]) for row in rows]
    imu_tilts = [
        math.degrees(
            math.hypot(float(row["imu_roll_rad"]), float(row["imu_pitch_rad"]))
        )
        for row in rows
    ]

    result: dict[str, object] = {
        "file": path.name,
        "date_utc": date_utc,
        "time_utc": time_utc,
        "classification": "unreviewed_development",
        "scope": "whole_csv_including_alignment_if_present",
        "frames": len(rows),
        "duration_s": duration_s,
        "column_count": len(rows[0]),
        "target_age_median_ms": statistics.median(target_ages),
        "target_age_p95_ms": percentile(target_ages, 0.95),
        "target_age_max_ms": max(target_ages),
        "target_age_gt_250ms_frames": sum(age > 250.0 for age in target_ages),
        "target_age_gt_250ms_percent": (
            100.0 * sum(age > 250.0 for age in target_ages) / len(target_ages)
        ),
        "imu_tilt_max_deg": max(imu_tilts),
        "pitch_ff_max_abs_nm": max(
            abs(float(row.get("command_pitch_gravity_ff_nm", 0.0)))
            for row in rows
        ),
        "roll_ff_max_abs_nm": max(
            abs(float(row.get("command_roll_gravity_ff_nm", 0.0)))
            for row in rows
        ),
    }

    for joint in JOINTS:
        source = degrees(
            [float(row[f"source_target_{joint}_rad"]) for row in rows]
        )
        command = degrees([float(row[f"command_{joint}_rad"]) for row in rows])
        actual = degrees([float(row[ACTUAL_COLUMNS[joint]]) for row in rows])
        errors = [command_value - actual_value for command_value, actual_value in zip(
            command, actual
        )]
        result.update({
            f"{joint}_source_min_deg": min(source),
            f"{joint}_source_max_deg": max(source),
            f"{joint}_command_min_deg": min(command),
            f"{joint}_command_max_deg": max(command),
            f"{joint}_actual_min_deg": min(actual),
            f"{joint}_actual_max_deg": max(actual),
            f"{joint}_rmse_deg": math.sqrt(
                sum(error * error for error in errors) / len(errors)
            ),
            f"{joint}_p95_abs_error_deg": percentile(
                [abs(error) for error in errors], 0.95
            ),
            f"{joint}_max_abs_error_deg": max(abs(error) for error in errors),
        })
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--sha256-output", type=Path)
    args = parser.parse_args()

    summaries = [summarize(path) for path in sorted(args.paths)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    if args.sha256_output is not None:
        args.sha256_output.parent.mkdir(parents=True, exist_ok=True)
        with args.sha256_output.open("w", encoding="utf-8") as stream:
            for path in sorted(args.paths):
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                stream.write(f"{digest}  {path.as_posix()}\n")
    print(f"Saved {len(summaries)} summaries to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
