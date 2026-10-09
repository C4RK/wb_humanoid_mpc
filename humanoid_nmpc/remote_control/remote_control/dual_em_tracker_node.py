#!/usr/bin/env python3

"""Publish a synchronized upper-arm/forearm WEMT receiver pair."""

from datetime import datetime, timezone
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from wemt_api import DeviceConfig, WemtAPI


class DualEmTrackerNode(Node):
    """Connect to two WEMT modules and give paired poses one ROS timestamp."""

    def __init__(self):
        super().__init__('dual_em_tracker_node')

        # Verified on the EM hotspot: the new receiver is used on the upper arm
        # and the original receiver remains on the forearm.
        self.declare_parameter('upper_arm_ip', '10.42.0.143')
        self.declare_parameter('forearm_ip', '10.42.0.144')
        self.declare_parameter('upper_arm_id', 'upper_arm')
        self.declare_parameter('forearm_id', 'forearm')
        self.declare_parameter('upper_arm_port', 801)
        self.declare_parameter('forearm_port', 801)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_pose_age_ms', 100.0)
        self.declare_parameter('max_pair_skew_ms', 30.0)

        upper_ip = str(self.get_parameter('upper_arm_ip').value).strip()
        forearm_ip = str(self.get_parameter('forearm_ip').value).strip()
        self.upper_id = str(self.get_parameter('upper_arm_id').value)
        self.forearm_id = str(self.get_parameter('forearm_id').value)
        upper_port = int(self.get_parameter('upper_arm_port').value)
        forearm_port = int(self.get_parameter('forearm_port').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.max_pose_age_s = float(self.get_parameter('max_pose_age_ms').value) / 1000.0
        self.max_pair_skew_s = float(self.get_parameter('max_pair_skew_ms').value) / 1000.0

        if not upper_ip:
            raise ValueError('upper_arm_ip is required')
        if not forearm_ip:
            raise ValueError('forearm_ip is required')
        if self.upper_id == self.forearm_id:
            raise ValueError('upper_arm_id and forearm_id must be different')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.upper_publisher = self.create_publisher(
            PoseStamped, '/em/upper_arm_pose', 10)
        self.forearm_publisher = self.create_publisher(
            PoseStamped, '/em/forearm_pose', 10)

        devices = [
            DeviceConfig(id=self.upper_id, host=upper_ip, port=upper_port),
            DeviceConfig(id=self.forearm_id, host=forearm_ip, port=forearm_port),
        ]
        self.get_logger().info(
            f'Connecting upper_arm={upper_ip}:{upper_port}, '
            f'forearm={forearm_ip}:{forearm_port}')
        # The API default averages eight position samples but leaves orientation
        # unfiltered.  That creates a position/orientation delay mismatch, so raw
        # samples are published and filtering is done after pairing.
        self.api = WemtAPI(devices=devices, smoothing_window=1)
        self.api.start_tracking()

        self._last_raw_stamps = None
        self._published_pairs = 0
        self._timer = self.create_timer(1.0 / publish_rate, self._read_and_publish)
        self.get_logger().info(
            'Waiting for both receivers; output topics are '
            '/em/upper_arm_pose and /em/forearm_pose.')

    def _read_and_publish(self):
        try:
            upper = self.api.pose(self.upper_id)
            forearm = self.api.pose(self.forearm_id)
        except Exception as exc:
            self.get_logger().warn(
                f'WEMT read error: {exc}', throttle_duration_sec=1.0)
            return

        unavailable = []
        for label, pose in [('upper arm', upper), ('forearm', forearm)]:
            if not pose.connected or pose.timestamp is None or pose.updated_at is None:
                unavailable.append(label)
            elif pose.error:
                self.get_logger().warn(
                    f'{label} receiver error: {pose.error}', throttle_duration_sec=1.0)
                return
        if unavailable:
            self.get_logger().warn(
                'Waiting for receiver data: ' + ', '.join(unavailable),
                throttle_duration_sec=1.0)
            return

        now_utc = datetime.now(timezone.utc)
        upper_age = (now_utc - upper.updated_at).total_seconds()
        forearm_age = (now_utc - forearm.updated_at).total_seconds()
        if max(upper_age, forearm_age) > self.max_pose_age_s:
            self.get_logger().warn(
                f'Stale WEMT data: upper={upper_age*1000:.0f} ms, '
                f'forearm={forearm_age*1000:.0f} ms',
                throttle_duration_sec=1.0)
            return

        skew_s = abs((upper.updated_at - forearm.updated_at).total_seconds())
        if skew_s > self.max_pair_skew_s:
            self.get_logger().warn(
                f'Receiver pair skew {skew_s*1000:.1f} ms exceeds '
                f'{self.max_pair_skew_s*1000:.0f} ms; pair dropped.',
                throttle_duration_sec=1.0)
            return

        raw_stamps = (upper.timestamp, forearm.timestamp)
        if self._last_raw_stamps is not None:
            # Wait until both receivers have supplied a new hardware frame.  This
            # prevents one old pose being paired repeatedly with newer poses.
            if (raw_stamps[0] == self._last_raw_stamps[0] or
                    raw_stamps[1] == self._last_raw_stamps[1]):
                return

        stamp = self.get_clock().now().to_msg()
        upper_msg = self._pose_message(upper, stamp)
        forearm_msg = self._pose_message(forearm, stamp)
        self.upper_publisher.publish(upper_msg)
        self.forearm_publisher.publish(forearm_msg)
        self._last_raw_stamps = raw_stamps
        self._published_pairs += 1

        if self._published_pairs == 1:
            self.get_logger().info(
                f'First synchronized pair published (arrival skew {skew_s*1000:.1f} ms).')

    @staticmethod
    def _pose_message(pose, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'em_transmitter'
        msg.pose.position.x = float(pose.position_mm[0]) / 1000.0
        msg.pose.position.y = float(pose.position_mm[1]) / 1000.0
        msg.pose.position.z = float(pose.position_mm[2]) / 1000.0
        q = pose.quaternion_wxyz
        norm = math.sqrt(sum(float(v) ** 2 for v in q))
        if norm < 1e-12:
            raise ValueError(f'{pose.device_id} returned a zero quaternion')
        msg.pose.orientation.w = float(q[0]) / norm
        msg.pose.orientation.x = float(q[1]) / norm
        msg.pose.orientation.y = float(q[2]) / norm
        msg.pose.orientation.z = float(q[3]) / norm
        return msg

    def destroy_node(self):
        self.get_logger().info('Stopping both WEMT receivers...')
        self.api.stop_tracking()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualEmTrackerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
