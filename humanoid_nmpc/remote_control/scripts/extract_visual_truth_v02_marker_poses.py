#!/usr/bin/env python3
"""Extract 5 Hz ArUco poses from the two V02 evaluation videos."""

from pathlib import Path

import cv2
import numpy as np


ROOT = Path('/home/kunwang/humanoid_mpc_ws/calibration_logs/'
            'visual_truth/V00/preflight')
VIDEOS = {
    'A': ROOT / 'phone_A_evaluation_V02.mp4',
    'B': ROOT / 'phone_B_evaluation_V02.mp4',
}
CALIBRATION = {
    'A': (
        np.array([[1583.653, 0.0, 985.813],
                  [0.0, 1566.056, 575.853],
                  [0.0, 0.0, 1.0]], dtype=float),
        np.array([0.15492, -0.69415, 0.01211, 0.01261, 0.58210])),
    'B': (
        np.array([[1590.757, 0.0, 922.572],
                  [0.0, 1569.927, 552.685],
                  [0.0, 0.0, 1.0]], dtype=float),
        np.array([0.15938, -0.95536, 0.00205, -0.01059, 1.28774])),
}
OUTPUT = Path('/tmp/v02_marker_poses_5hz.npz')
MARKER_SIZE_M = 0.058
HALF_SIZE = MARKER_SIZE_M / 2.0
OBJECT_POINTS = np.array([
    [-HALF_SIZE, HALF_SIZE, 0.0],
    [HALF_SIZE, HALF_SIZE, 0.0],
    [HALF_SIZE, -HALF_SIZE, 0.0],
    [-HALF_SIZE, -HALF_SIZE, 0.0],
], dtype=np.float32)


def marker_pose(corners, camera_matrix, distortion):
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    result = cv2.solvePnPGeneric(
        OBJECT_POINTS, image_points, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    ok, rotation_vectors, translations = result[:3]
    if not ok:
        return None
    candidates = []
    for rotation_vector, translation in zip(rotation_vectors, translations):
        translation = np.asarray(translation, dtype=float).reshape(3)
        if translation[2] <= 0.0:
            continue
        projection, _ = cv2.projectPoints(
            OBJECT_POINTS, rotation_vector, translation,
            camera_matrix, distortion)
        reprojection = float(np.sqrt(np.mean(np.sum(
            (projection.reshape(4, 2) - image_points) ** 2, axis=1))))
        rotation, _ = cv2.Rodrigues(rotation_vector)
        candidates.append((reprojection, rotation, translation))
    return min(candidates, key=lambda item: item[0]) if candidates else None


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    records = []

    for camera_name, video_path in VIDEOS.items():
        camera_matrix, distortion = CALIBRATION[camera_name]
        capture = cv2.VideoCapture(str(video_path))
        fps = capture.get(cv2.CAP_PROP_FPS)
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        step = max(1, int(round(fps / 5.0)))
        print(f'{camera_name}: fps={fps:.6f}, frames={frame_count}, step={step}')
        for frame_index in range(0, frame_count, step):
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = capture.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                continue
            for marker_id, marker_corners in zip(ids.reshape(-1), corners):
                marker_id = int(marker_id)
                if marker_id not in (10, 20):
                    continue
                pose = marker_pose(
                    marker_corners, camera_matrix, distortion)
                if pose is None or pose[0] > 3.0:
                    continue
                records.append((
                    camera_name, frame_index / fps, marker_id,
                    pose[0], pose[1], pose[2]))
        capture.release()

    np.savez_compressed(
        OUTPUT,
        camera=np.array([item[0] for item in records]),
        time_s=np.array([item[1] for item in records]),
        marker_id=np.array([item[2] for item in records]),
        reprojection_px=np.array([item[3] for item in records]),
        rotation_camera_marker=np.array([item[4] for item in records]),
        translation_camera_marker_m=np.array([item[5] for item in records]),
    )
    print(f'Saved {len(records)} valid marker poses to {OUTPUT}')


if __name__ == '__main__':
    main()
