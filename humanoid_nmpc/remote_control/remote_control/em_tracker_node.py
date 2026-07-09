#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from wemt_api import WemtAPI


class EmTrackerNode(Node):
    """
    ROS2 node that reads pose data from a WEMT electromagnetic tracker
    and publishes it as a PoseStamped message on /em/pose.

    The WEMT device gives the receiver pose relative to the transmitter.
    In our setup:
      - Transmitter: mounted on the user's shoulder (acts as origin)
      - Receiver:    mounted on the user's forearm
    So the published position is the forearm position in the shoulder frame.
    """

    def __init__(self):
        super().__init__('em_tracker_node')

        # --- ROS2 Parameters ---
        # These can be overridden at launch time without changing the code:
        #   ros2 run remote_control em_tracker_node --ros-args -p device_ip:=192.168.1.5
        self.declare_parameter('device_ip', '192.168.137.10')
        self.declare_parameter('publish_rate_hz', 50.0)

        device_ip = self.get_parameter('device_ip').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        # --- Publisher ---
        # Publishes the receiver pose relative to the transmitter.
        # frame_id is set to 'em_transmitter' to make the coordinate frame explicit.
        self.pose_publisher = self.create_publisher(PoseStamped, '/em/pose', 10)

        # --- Connect to WEMT hardware ---
        self.get_logger().info(f'Connecting to WEMT device at {device_ip} ...')
        try:
            self.api = WemtAPI(device_ip=device_ip)
            self.api.start_tracking()
            self.get_logger().info('WEMT tracking started successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to WEMT device: {e}')
            raise

        # --- Timer ---
        # Fires every (1 / publish_rate) seconds → calls read_and_publish()
        # At 50 Hz: fires every 0.02 seconds = 20 ms
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.read_and_publish)

        self.get_logger().info(f'Publishing /em/pose at {publish_rate:.0f} Hz.')

    # ------------------------------------------------------------------

    def read_and_publish(self):
        """
        Called by the timer at 50 Hz.
        Reads one pose from the WEMT device and publishes it.
        """
        try:
            # Wait up to 20 ms for a new pose measurement.
            # If no pose arrives in time, an exception is raised and we skip this cycle.
            pose = self.api.wait_for_pose(timeout_s=0.02)
        except Exception as e:
            # throttle_duration_sec=1.0 means this warning prints at most once per second.
            # Without throttling, a dropped frame would flood the terminal.
            self.get_logger().warn(
                f'Missed WEMT frame: {e}',
                throttle_duration_sec=1.0
            )
            return

        msg = PoseStamped()

        # Timestamp from the ROS2 clock (used for synchronization with other nodes)
        msg.header.stamp = self.get_clock().now().to_msg()

        # Coordinate frame: all positions are relative to the transmitter origin.
        # The calibration and retargeting nodes must use this same frame.
        msg.header.frame_id = 'em_transmitter'

        # --- Position: convert mm → meters ---
        # WEMT gives position in millimeters. ROS2 standard unit is meters.
        msg.pose.position.x = pose.position_mm[0] / 1000.0
        msg.pose.position.y = pose.position_mm[1] / 1000.0
        msg.pose.position.z = pose.position_mm[2] / 1000.0

        # --- Orientation: WEMT format is (w, x, y, z), ROS2 format is also (x, y, z, w) ---
        # WEMT API returns quaternion_wxyz = [w, x, y, z]
        # ROS2 geometry_msgs uses fields .x .y .z .w  (same values, just named separately)
        q = pose.quaternion_wxyz
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]

        self.pose_publisher.publish(msg)

    # ------------------------------------------------------------------

    def destroy_node(self):
        """
        Called automatically when the node shuts down (Ctrl+C or rclpy.shutdown()).
        Always stop tracking before disconnecting to avoid leaving the device in a bad state.
        """
        self.get_logger().info('Stopping WEMT tracking...')
        self.api.stop_tracking()
        super().destroy_node()


# -----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = EmTrackerNode()
    try:
        # rclpy.spin() blocks here and processes timer callbacks continuously.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
