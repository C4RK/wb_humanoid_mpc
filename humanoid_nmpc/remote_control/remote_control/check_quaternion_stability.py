#!/usr/bin/env python3
"""
check_quaternion_stability.py

Hold your arm STILL and watch if forearm_dir stays stable.

  - If forearm_dir is stable → quaternion is fine (sign flips are harmless)
  - If forearm_dir JUMPS to a different direction → quaternion itself is unstable

Run:
    python3 check_quaternion_stability.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


FOREARM_LOCAL_AXIS = np.array([0.0, 1.0, 0.0])


def quat_to_matrix(q):
    """q = [w, x, y, z]"""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


class CheckNode(Node):

    def __init__(self):
        super().__init__('check_quaternion_stability')
        self._prev_dir = None
        self._sub = self.create_subscription(
            PoseStamped, '/em/pose', self._cb, 10)
        self.get_logger().info(
            'Listening to /em/pose ... Hold your arm STILL and watch forearm_dir.')

    def _cb(self, msg):
        o = msg.pose.orientation
        q = [o.w, o.x, o.y, o.z]

        R = quat_to_matrix(q)
        forearm_dir = R @ FOREARM_LOCAL_AXIS

        # Detect jumps
        jump = ''
        if self._prev_dir is not None:
            delta = np.linalg.norm(forearm_dir - self._prev_dir)
            if delta > 0.3:   # >0.3 = >~17° jump in one frame
                jump = f'  *** JUMP! Δ={delta:.3f} ***'

        self._prev_dir = forearm_dir.copy()

        p = msg.pose.position
        print(
            f'pos=[{p.x:+.3f},{p.y:+.3f},{p.z:+.3f}]  '
            f'q=[{q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}]  '
            f'forearm_dir=[{forearm_dir[0]:+.5f},{forearm_dir[1]:+.5f},{forearm_dir[2]:+.5f}]'
            f'{jump}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
