#!/usr/bin/env python3

"""
pose_sequence_node.py

Simulates the EM tracker for MuJoCo testing — no hardware needed.
Publishes a smooth sequence of arm poses to /em/pose at 50 Hz.

Sequence (each pose held for a few seconds):
  1. Arm hanging straight down          (robot default)
  2. Arm raised forward horizontal      (shoulder pitch)
  3. Arm raised sideways horizontal     (shoulder roll)
  4. Arm at side, elbow bent forward    (elbow joint)
  5. Back to hanging

This lets you verify the full pipeline:
  pose_sequence_node → /em/pose → retargeting_node → /arm_joint_target → MPC → MuJoCo
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# Each pose: (label, position [x,y,z] in meters, quaternion [w,x,y,z])
# Positions are in the shoulder/transmitter frame.
# All poses use fully extended arm (L1+L2 = 0.54m) except pose 4.
#
# Quaternion: must point forearm_dir along FOREARM_LOCAL_AXIS in world frame.
# With FOREARM_LOCAL_AXIS = [0,1,0] and forearm pointing in same direction as arm:
#   arm down    → forearm = [0,0,-1] → rotate local Y to world -Z → q=(0.707,0.707,0,0)
#   arm forward → forearm = [1,0,0]  → rotate local Y to world +X → q=(0.5,0.5,0.5,0.5)
#   arm sideways→ forearm = [0,1,0]  → local Y already there     → q=(1,0,0,0) identity
#   elbow bent  → forearm = [1,0,0]  (upper arm down, forearm forward)

POSES = [
    {
        'label':    'Arm hanging straight down (robot default)',
        'position': [0.0, 0.0, -0.54],
        # FOREARM_LOCAL_AXIS=[0,1,0]: need R@[0,1,0]=[0,0,-1] → rotate -90° around X
        # q=(w=0.707, x=-0.707, y=0, z=0)
        'quat':     [0.707, -0.707, 0.0, 0.0],   # forearm_dir = [0,0,-1] ✓
        'hold_sec': 4.0,
    },
    {
        'label':    'Arm raised forward horizontal',
        'position': [0.54, 0.0, 0.0],
        # need R@[0,1,0]=[1,0,0] → rotate -90° around Z
        # q=(w=0.707, x=0, y=0, z=-0.707)
        'quat':     [0.707, 0.0, 0.0, -0.707],   # forearm_dir = [1,0,0] ✓
        'hold_sec': 4.0,
    },
    {
        'label':    'Arm raised sideways horizontal (T-pose)',
        'position': [0.0, 0.54, 0.0],
        # need R@[0,1,0]=[0,1,0] → identity
        'quat':     [1.0, 0.0, 0.0, 0.0],        # forearm_dir = [0,1,0] ✓
        'hold_sec': 4.0,
    },
    {
        'label':    'Arm at side, elbow bent 90 forward (waiter pose = robot zero)',
        'position': [0.26, 0.0, -0.28],
        # upper arm down, forearm forward: same as pose 2
        # q=(w=0.707, x=0, y=0, z=-0.707)
        'quat':     [0.707, 0.0, 0.0, -0.707],   # forearm_dir = [1,0,0] ✓
        'hold_sec': 4.0,
    },
    {
        'label':    'Back to arm hanging down',
        'position': [0.0, 0.0, -0.54],
        'quat':     [0.707, -0.707, 0.0, 0.0],   # forearm_dir = [0,0,-1] ✓
        'hold_sec': 4.0,
    },
]

PUBLISH_RATE_HZ = 50.0


class PoseSequenceNode(Node):

    def __init__(self):
        super().__init__('pose_sequence_node')

        self._pub = self.create_publisher(PoseStamped, '/em/pose', 10)

        self._pose_idx = 0
        self._elapsed  = 0.0
        self._dt       = 1.0 / PUBLISH_RATE_HZ

        self.create_timer(self._dt, self._tick)

        self.get_logger().info('Pose sequence node started. Publishing to /em/pose at 50 Hz.')
        self._announce_pose()

    def _tick(self):
        pose_def = POSES[self._pose_idx]

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'em_transmitter'

        p = pose_def['position']
        msg.pose.position.x = p[0]
        msg.pose.position.y = p[1]
        msg.pose.position.z = p[2]

        q = pose_def['quat']
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]

        self._pub.publish(msg)

        self._elapsed += self._dt
        if self._elapsed >= pose_def['hold_sec']:
            self._elapsed  = 0.0
            self._pose_idx = (self._pose_idx + 1) % len(POSES)
            self._announce_pose()

    def _announce_pose(self):
        pose_def = POSES[self._pose_idx]
        self.get_logger().info(
            f'[{self._pose_idx + 1}/{len(POSES)}] {pose_def["label"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
