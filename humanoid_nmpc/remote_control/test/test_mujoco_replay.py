import importlib.util
import math
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'mujoco_replay.py'
SPEC = importlib.util.spec_from_file_location('mujoco_replay', SCRIPT_PATH)
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


def test_parse_log_accepts_dual_and_legacy_angle_formats():
    text = """
[INFO] [100.000000000]: pitch=+1.0 roll=-2.0 yaw=+3.0 elbow=+4.0 wrist=-5.0 deg; upper radius error=2 mm
[INFO] [101.000000000]: pitch=+6.0° roll=-7.0° yaw=+8.0° elbow=+9.0° wrist=-10.0°
"""

    timestamps, angles = REPLAY.parse_log(text)

    assert timestamps == [0.0, 1.0]
    np.testing.assert_allclose(
        np.degrees(angles),
        [[1.0, -2.0, 3.0, 4.0, -5.0],
         [6.0, -7.0, 8.0, 9.0, -10.0]],
    )


def test_interpolate_trajectory_preserves_endpoints():
    timestamps = [0.0, 1.0]
    angles = [[0.0] * 5, [math.radians(30.0)] * 5]

    output_t, output_q = REPLAY.interpolate_trajectory(
        timestamps, angles, sample_rate_hz=30.0)

    assert len(output_t) == 31
    assert output_t[0] == 0.0
    assert output_t[-1] == 1.0
    np.testing.assert_allclose(output_q[0], angles[0])
    np.testing.assert_allclose(output_q[-1], angles[-1])
