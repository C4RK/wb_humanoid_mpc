#!/usr/bin/env python3
"""
mujoco_replay.py

Replays arm joint angles from retargeting_node log output in a MuJoCo
simulation of the Unitree G1 (23-DOF).

Usage
-----
  # paste/pipe the log lines into a file, then:
  python3 mujoco_replay.py --log trajectory.txt

  # or paste log text directly (prompts you):
  python3 mujoco_replay.py --stdin

  # choose model:
  python3 mujoco_replay.py --log trajectory.txt --model menagerie
  python3 mujoco_replay.py --log trajectory.txt --model unitree

  # save a video instead of interactive viewer:
  python3 mujoco_replay.py --log trajectory.txt --save replay.mp4

Log format expected (output of retargeting_node with 1 Hz throttle):
  [INFO] [...]: pitch=-6.3° roll=-2.3° yaw=+0.0° elbow=+71.3°

All four values are in degrees; the script converts them to radians.

Joints controlled (left arm only):
  left_shoulder_pitch_joint   (index 0 of arm)
  left_shoulder_roll_joint    (index 1)
  left_shoulder_yaw_joint     (index 2)
  left_elbow_joint            (index 3)
All other joints are held at their URDF default (keyframe "home" if defined,
else qpos0 from the model).
"""

import re
import math
import time
import argparse
import sys
import numpy as np


# ---------------------------------------------------------------------------
# Model paths
# ---------------------------------------------------------------------------

MODELS = {
    'menagerie': '/home/kunwang/humanoid-motion-planning/mujoco_menagerie/unitree_g1/scene.xml',
    'unitree':   '/home/kunwang/unitree_mujoco/unitree_robots/g1/scene_23dof.xml',
}

# Left-arm joint names in the G1 URDF (must match the XML exactly)
ARM_JOINTS = [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
]

# Default (rest) pose for the whole robot — overridden by the keyframe below
# if one is present in the model.
LEG_DEFAULT = [
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,   # left leg
    -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,   # right leg
]

# Playback speed multiplier (1.0 = real-time, 2.0 = double speed)
PLAYBACK_SPEED = 1.0


# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------

_LOG_RE = re.compile(
    r'pitch=([+-]?\d+\.?\d*)°\s+'
    r'roll=([+-]?\d+\.?\d*)°\s+'
    r'yaw=([+-]?\d+\.?\d*)°\s+'
    r'elbow=([+-]?\d+\.?\d*)°'
)

_STAMP_RE = re.compile(r'\[(\d+)\.(\d+)\]')   # ROS stamp: [sec.nanosec]


def parse_log(text: str) -> tuple[list, list]:
    """
    Parse retargeting_node log lines.

    Returns
    -------
    timestamps : list of float (seconds; 0-based, relative to first line)
    angles     : list of [pitch, roll, yaw, elbow] in RADIANS
    """
    timestamps = []
    angles = []

    for line in text.splitlines():
        m = _LOG_RE.search(line)
        if not m:
            continue
        pitch_deg, roll_deg, yaw_deg, elbow_deg = map(float, m.groups())

        # Try to extract ROS timestamp for correct inter-frame timing
        ts_m = _STAMP_RE.search(line)
        if ts_m:
            t = int(ts_m.group(1)) + int(ts_m.group(2)) * 1e-9
        else:
            t = len(timestamps) * 1.0   # fall back to 1 Hz if no stamp

        timestamps.append(t)
        angles.append([
            math.radians(pitch_deg),
            math.radians(roll_deg),
            math.radians(yaw_deg),
            math.radians(elbow_deg),
        ])

    if not timestamps:
        raise ValueError('No valid log lines found. Check the format.')

    # Make timestamps relative to the first sample
    t0 = timestamps[0]
    timestamps = [t - t0 for t in timestamps]
    return timestamps, angles


# ---------------------------------------------------------------------------
# MuJoCo replay
# ---------------------------------------------------------------------------

def find_joint_qpos_indices(model, joint_names: list) -> list:
    """Return qpos indices for each joint name."""
    indices = []
    for name in joint_names:
        jid = model.joint(name).id
        indices.append(model.jnt_qposadr[jid])
    return indices


def replay(scene_xml: str, timestamps: list, angles: list,
           save_path: str | None = None):
    """
    Load the MuJoCo scene and replay the joint angle sequence.

    If save_path is given (e.g. 'replay.mp4'), renders offscreen and saves
    a video; otherwise opens an interactive viewer window.
    """
    import mujoco
    import mujoco.viewer

    print(f'Loading model: {scene_xml}')
    model = mujoco.MjModel.from_xml_path(scene_xml)
    data  = mujoco.MjData(model)

    # Use keyframe "home" if defined (sets a reasonable standing pose)
    home_key = -1
    for i in range(model.nkey):
        name = model.keyframe(i).name
        if 'home' in name.lower() or 'stand' in name.lower():
            home_key = i
            break
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
        print(f'Using keyframe "{model.keyframe(home_key).name}" as base pose.')
    else:
        mujoco.mj_resetData(model, data)
        print('No home keyframe found — using model default.')

    # Find qpos addresses for the four left-arm joints
    try:
        arm_qpos_idx = find_joint_qpos_indices(model, ARM_JOINTS)
    except Exception as e:
        print(f'[ERROR] Could not find arm joints: {e}')
        print('Available joints:')
        for i in range(model.njnt):
            print(f'  {model.joint(i).name}')
        return

    print(f'Left-arm qpos indices: {arm_qpos_idx}')
    print(f'Loaded {len(timestamps)} frames  '
          f'(duration: {timestamps[-1]:.1f} s)\n')
    print('Joint angle ranges in this trajectory:')
    arr = np.degrees(angles)
    for i, name in enumerate(ARM_JOINTS):
        short = name.replace('left_', '').replace('_joint', '')
        print(f'  {short:25s}  '
              f'min={arr[:,i].min():+6.1f}°  max={arr[:,i].max():+6.1f}°')
    print()

    if save_path:
        _save_video(model, data, arm_qpos_idx, timestamps, angles, save_path)
    else:
        _interactive(model, data, arm_qpos_idx, timestamps, angles)


def _set_arm(data, arm_qpos_idx, frame_angles):
    """Write four left-arm qpos values."""
    for idx, val in zip(arm_qpos_idx, frame_angles):
        data.qpos[idx] = val


def _interactive(model, data, arm_qpos_idx, timestamps, angles):
    """Replay in MuJoCo's interactive viewer."""
    import mujoco.viewer

    frame = [0]
    paused = [False]

    def key_callback(keycode):
        if keycode == ord(' '):
            paused[0] = not paused[0]
        elif keycode == ord('R'):
            frame[0] = 0
            print('Replay restarted.')

    print('Controls:')
    print('  SPACE  — pause / resume')
    print('  R      — restart')
    print('  Ctrl+C / close window — quit\n')

    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=key_callback) as viewer:
        t_start = time.time()
        last_frame = -1

        while viewer.is_running():
            if not paused[0]:
                elapsed = (time.time() - t_start) * PLAYBACK_SPEED

                # Find the frame closest to current elapsed time
                f = frame[0]
                while f + 1 < len(timestamps) and timestamps[f + 1] <= elapsed:
                    f += 1
                frame[0] = f

                if f != last_frame:
                    _set_arm(data, arm_qpos_idx, angles[f])
                    mujoco.mj_forward(model, data)
                    last_frame = f

                    # Print progress every ~0.5 s of wall time
                    if f % max(1, len(timestamps) // 60) == 0:
                        pct = 100 * f / max(1, len(timestamps) - 1)
                        print(f'\r  Frame {f+1:3d}/{len(timestamps)}'
                              f'  t={timestamps[f]:5.1f}s  {pct:.0f}%',
                              end='', flush=True)

                if f >= len(timestamps) - 1:
                    print('\n[Replay complete. Press R to restart or close to exit.]')
                    paused[0] = True

            viewer.sync()
            time.sleep(0.005)

    print('\nViewer closed.')


def _save_video(model, data, arm_qpos_idx, timestamps, angles, path):
    """Render each frame offscreen and save to a video file."""
    import mujoco

    try:
        import imageio
    except ImportError:
        print('[ERROR] imageio is required for video saving.')
        print('  pip install imageio imageio-ffmpeg')
        return

    width, height = 1280, 720
    renderer = mujoco.Renderer(model, height=height, width=width)

    frames = []
    print(f'Rendering {len(angles)} frames at {width}×{height}...')
    for i, frame_angles in enumerate(angles):
        _set_arm(data, arm_qpos_idx, frame_angles)
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera='track_pelvis')
        pixels = renderer.render()
        frames.append(pixels)
        if (i + 1) % 10 == 0:
            print(f'\r  {i+1}/{len(angles)}', end='', flush=True)

    print(f'\nSaving to {path} ...')
    # Compute fps from average frame interval
    if len(timestamps) > 1:
        avg_dt = timestamps[-1] / (len(timestamps) - 1)
        fps = max(1.0, min(60.0, 1.0 / avg_dt)) * PLAYBACK_SPEED
    else:
        fps = 10.0

    with imageio.get_writer(path, fps=fps) as writer:
        for frame in frames:
            writer.append_data(frame)
    print(f'Video saved: {path}')
    renderer.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Replay retargeting_node log as MuJoCo animation.')
    parser.add_argument('--log',   help='Path to log file (one line per sample)')
    parser.add_argument('--stdin', action='store_true',
                        help='Read log lines from stdin / paste')
    parser.add_argument('--model', choices=list(MODELS.keys()),
                        default='menagerie',
                        help='Which MuJoCo model to use (default: menagerie)')
    parser.add_argument('--save',  metavar='FILE',
                        help='Save replay to video file instead of viewer')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    args = parser.parse_args()

    global PLAYBACK_SPEED
    PLAYBACK_SPEED = args.speed

    if args.log:
        with open(args.log) as f:
            text = f.read()
    elif args.stdin:
        print('Paste log lines, then press Ctrl+D (Linux) or Ctrl+Z (Windows):')
        text = sys.stdin.read()
    else:
        # Try to find a log file automatically
        import os
        candidates = ['trajectory.txt', 'retargeting.log', 'log.txt']
        for c in candidates:
            if os.path.exists(c):
                print(f'Found log file: {c}')
                with open(c) as f:
                    text = f.read()
                break
        else:
            parser.print_help()
            print('\n[ERROR] Provide --log <file> or --stdin.')
            sys.exit(1)

    timestamps, angles = parse_log(text)
    print(f'Parsed {len(angles)} frames from log.')

    scene_xml = MODELS[args.model]
    replay(scene_xml, timestamps, angles, save_path=args.save)


if __name__ == '__main__':
    main()
