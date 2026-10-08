#!/usr/bin/env python3
"""Forward ROS arm targets to the host-side Unitree safety bridge over UDP."""

import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_TARGET_INDICES = (13, 14, 15, 16, 17)
PACKET = struct.Struct("!4sIQ5d")
PACKET_MAGIC = b"G1A1"


class ArmTargetUdpSender(Node):
    def __init__(self):
        super().__init__("arm_target_udp_sender")
        self.declare_parameter("destination_host", "127.0.0.1")
        self.declare_parameter("destination_port", 15000)
        self.declare_parameter("source_topic", "/arm_joint_target")

        host = str(self.get_parameter("destination_host").value)
        port = int(self.get_parameter("destination_port").value)
        topic = str(self.get_parameter("source_topic").value)

        self._destination = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = 0
        self._last_log = 0.0
        self.create_subscription(JointState, topic, self._on_target, 10)
        self.get_logger().info(
            f"Forwarding {topic} left-arm targets to udp://{host}:{port}"
        )

    def _on_target(self, message: JointState) -> None:
        if len(message.position) <= ARM_TARGET_INDICES[-1]:
            self.get_logger().warning(
                f"Ignoring JointState with {len(message.position)} positions; "
                f"need at least {ARM_TARGET_INDICES[-1] + 1}.",
                throttle_duration_sec=1.0,
            )
            return

        target = tuple(float(message.position[index]) for index in ARM_TARGET_INDICES)
        if not all(math.isfinite(value) for value in target):
            self.get_logger().warning(
                "Ignoring non-finite arm target.", throttle_duration_sec=1.0
            )
            return

        packet = PACKET.pack(
            PACKET_MAGIC,
            self._sequence,
            time.monotonic_ns(),
            *target,
        )
        self._socket.sendto(packet, self._destination)
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF

        now = time.monotonic()
        if now - self._last_log >= 1.0:
            degrees = [math.degrees(value) for value in target]
            self.get_logger().info(
                "sent pitch={:+.1f} roll={:+.1f} yaw={:+.1f} "
                "elbow={:+.1f} wrist={:+.1f} deg".format(*degrees)
            )
            self._last_log = now

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTargetUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
