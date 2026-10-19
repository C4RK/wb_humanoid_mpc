#!/usr/bin/env python3
"""Check whether expected ArUco IDs are readable in a phone video.

This is a preflight visibility check. It does not require camera calibration and
does not estimate metric marker poses. It writes an annotated MP4 and a JSON
summary containing detection rates, marker image sizes, and longest gaps.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def make_detector(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV was built without the aruco module; install opencv-contrib-python"
        )
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unknown ArUco dictionary: {dictionary_name}")
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers
    parameters = cv2.aruco.DetectorParameters_create()
    return lambda image: cv2.aruco.detectMarkers(
        image, dictionary, parameters=parameters
    )


def marker_edge_pixels(corners) -> float:
    points = np.asarray(corners, dtype=float).reshape(4, 2)
    edge_lengths = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
    return float(np.mean(edge_lengths))


def update_longest_gaps(expected_ids, present, current_gaps, longest_gaps) -> None:
    for marker_id in expected_ids:
        if marker_id in present:
            current_gaps[marker_id] = 0
        else:
            current_gaps[marker_id] += 1
            longest_gaps[marker_id] = max(
                longest_gaps[marker_id], current_gaps[marker_id]
            )


def check_video(args) -> int:
    video_path = Path(args.video).expanduser().resolve()
    if not video_path.is_file():
        raise FileNotFoundError(video_path)

    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else video_path.with_name(video_path.stem + "_aruco_check.mp4")
    )
    summary_path = (
        Path(args.summary).expanduser().resolve()
        if args.summary
        else output_path.with_suffix(".json")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    detect_markers = make_detector(args.dictionary)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(
            f"cannot decode {video_path}; copy the original file locally or convert "
            "it to H.264 MP4 without resizing"
        )

    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0.0:
        fps = 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if width <= 0 or height <= 0:
        raise RuntimeError("video reports an invalid frame size")

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot create annotated video {output_path}")

    expected_ids = tuple(dict.fromkeys(args.ids))
    detection_counts = {marker_id: 0 for marker_id in expected_ids}
    edge_samples = {marker_id: [] for marker_id in expected_ids}
    current_gaps = {marker_id: 0 for marker_id in expected_ids}
    longest_gaps = {marker_id: 0 for marker_id in expected_ids}
    all_detected_ids = set()
    all_expected_count = 0
    total_frames = 0

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            total_frames += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _rejected = detect_markers(gray)
            flat_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
            present = set(flat_ids)
            all_detected_ids.update(present)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for marker_corners, marker_id in zip(corners, flat_ids):
                    if marker_id in edge_samples:
                        detection_counts[marker_id] += 1
                        edge_samples[marker_id].append(
                            marker_edge_pixels(marker_corners)
                        )

            if all(marker_id in present for marker_id in expected_ids):
                all_expected_count += 1
            update_longest_gaps(
                expected_ids, present, current_gaps, longest_gaps
            )

            y = 32
            for marker_id in expected_ids:
                visible = marker_id in present
                color = (0, 200, 0) if visible else (0, 0, 255)
                text = f"ID {marker_id}: {'VISIBLE' if visible else 'NOT SEEN'}"
                cv2.putText(
                    frame,
                    text,
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    2,
                    cv2.LINE_AA,
                )
                y += 32
            writer.write(frame)
    finally:
        capture.release()
        writer.release()

    if total_frames == 0:
        raise RuntimeError("the video contained no decodable frames")

    per_id = {}
    for marker_id in expected_ids:
        samples = np.asarray(edge_samples[marker_id], dtype=float)
        per_id[str(marker_id)] = {
            "detected_frames": detection_counts[marker_id],
            "detection_rate": detection_counts[marker_id] / total_frames,
            "median_edge_pixels": (
                float(np.median(samples)) if samples.size else None
            ),
            "p05_edge_pixels": (
                float(np.percentile(samples, 5)) if samples.size else None
            ),
            "longest_gap_frames": longest_gaps[marker_id],
            "longest_gap_seconds_nominal": longest_gaps[marker_id] / fps,
        }

    summary = {
        "video": str(video_path),
        "annotated_video": str(output_path),
        "dictionary": args.dictionary,
        "nominal_fps": fps,
        "frame_size": [width, height],
        "total_frames": total_frames,
        "expected_ids": list(expected_ids),
        "all_detected_ids": sorted(all_detected_ids),
        "all_expected_detection_rate": all_expected_count / total_frames,
        "per_id": per_id,
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"Video: {video_path}")
    print(f"Frames: {total_frames}, nominal fps: {fps:.3f}")
    print(f"All IDs seen in the video: {sorted(all_detected_ids)}")
    for marker_id in expected_ids:
        item = per_id[str(marker_id)]
        median_text = (
            f"{item['median_edge_pixels']:.1f} px"
            if item["median_edge_pixels"] is not None
            else "n/a"
        )
        print(
            f"ID {marker_id}: detected {item['detection_rate']*100:.1f}% of "
            f"frames, median edge {median_text}, longest gap "
            f"{item['longest_gap_seconds_nominal']:.2f} s"
        )
    print(
        f"Both expected IDs visible together: "
        f"{summary['all_expected_detection_rate']*100:.1f}% of frames"
    )
    print(f"Annotated video: {output_path}")
    print(f"JSON summary: {summary_path}")

    missing = [marker_id for marker_id in expected_ids if detection_counts[marker_id] == 0]
    if missing:
        print(f"RESULT: FAIL - expected IDs never detected: {missing}")
        return 2
    print("RESULT: expected IDs were detected; inspect rates, gaps, and overlay video.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check expected ArUco IDs in a recorded phone video."
    )
    parser.add_argument("video", help="input phone video (MP4 or MOV)")
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=[10, 20],
        help="expected marker IDs (default: 10 20)",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary constant (default: DICT_4X4_50)",
    )
    parser.add_argument("--output", help="annotated MP4 output path")
    parser.add_argument("--summary", help="JSON summary output path")
    args = parser.parse_args()

    try:
        return check_video(args)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

