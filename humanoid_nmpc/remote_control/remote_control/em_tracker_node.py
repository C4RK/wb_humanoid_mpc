#!/usr/bin/env python3

import math
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
        # 把设备IP地址，设备id，设备端口和发布频率定义为外部参数。
        self.declare_parameter('device_ip', '192.168.137.10')
        self.declare_parameter('device_id', 'module-1')   # default from WMET API
        self.declare_parameter('device_port', 801)         # default TCP port from WMET API
        self.declare_parameter('publish_rate_hz', 50.0)

        device_ip        = self.get_parameter('device_ip').value
        self.device_id   = self.get_parameter('device_id').value
        device_port      = self.get_parameter('device_port').value
        publish_rate     = self.get_parameter('publish_rate_hz').value

        # --- Publisher ---
        # Publishes the receiver pose relative to the transmitter.
        # frame_id is set to 'em_transmitter' to make the coordinate frame explicit.
        #创建发布者，话题为/em/pose，消息类型是 Posetamped，带时间戳的空间位姿消息。
        self.pose_publisher = self.create_publisher(PoseStamped, '/em/pose', 10)

        # --- Connect to WEMT hardware ---
        self.get_logger().info(f'Connecting to WMET device at {device_ip}:{device_port} (id={self.device_id}) ...')
        try:
            # WemtAPI communicates over TCP (port 801 by default), not HTTP.
            self.api = WemtAPI(device_ip=device_ip, device_id=self.device_id, device_port=device_port)
            self.api.start_tracking()
            self.get_logger().info('WMET tracking started successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to WEMT device: {e}')
            raise

        # --- Timer ---
        # Fires every (1 / publish_rate) seconds → calls read_and_publish()
        # At 50 Hz: fires every 0.02 seconds = 20 ms
        #启动50hz的定时器。每20ms就会自动调用一次读取发布函数。
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
            # wait_for_pose requires device_id as the first argument (from WMET API).
            # timeout_s=0.1 gives 100 ms — enough for one cycle at 50 Hz with margin.
            #0.1秒的超时阈值
            pose = self.api.wait_for_pose(self.device_id, timeout_s=0.1)
        except TimeoutError:#捕获超时异常
            self.get_logger().warn(
                'WMET pose timeout — no data received.',
                throttle_duration_sec=1.0#高频日志限流，不至于刷屏
            )
            return
        except Exception as e:#捕获其他异常
            self.get_logger().warn(
                f'WMET read error: {e}',
                throttle_duration_sec=1.0
            )
            return

        # The Pose object has an error field set by the device if something went wrong.
        #硬件自带的错误标志
        if pose.error:
            self.get_logger().warn(f'WMET device error: {pose.error}', throttle_duration_sec=1.0)
            return

        msg = PoseStamped()

        # Timestamp from the ROS2 clock (used for synchronization with other nodes)
        msg.header.stamp = self.get_clock().now().to_msg()#给数据打上时间戳

        # Coordinate frame: all positions are relative to the transmitter origin.
        # The calibration and retargeting nodes must use this same frame.
        msg.header.frame_id = 'em_transmitter'#给数据贴上坐标系标签

        # --- Position: convert mm → meters ---
        # WEMT gives position in millimeters. ROS2 standard unit is meters.
        msg.pose.position.x = pose.position_mm[0] / 1000.0#将毫米转换成米
        msg.pose.position.y = pose.position_mm[1] / 1000.0
        msg.pose.position.z = pose.position_mm[2] / 1000.0

        # --- Orientation: WEMT format is (w, x, y, z), ROS2 format is also (x, y, z, w) ---
        # WEMT API returns quaternion_wxyz = [w, x, y, z]
        # ROS2 geometry_msgs uses fields .x .y .z .w  (same values, just named separately)
        q = pose.quaternion_wxyz #四元数对齐
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]

        self.pose_publisher.publish(msg)

        # Print position and distance in centimeters to terminal for live monitoring
        x = msg.pose.position.x * 100.0
        y = msg.pose.position.y * 100.0
        z = msg.pose.position.z * 100.0
        dist_cm = math.sqrt(x**2 + y**2 + z**2)
        print(f"x={x:7.2f}cm  y={y:7.2f}cm  z={z:7.2f}cm  |  dist={dist_cm:.2f}cm", end='\r')

    # ------------------------------------------------------------------

    def destroy_node(self):
        """
        Called automatically when the node shuts down (Ctrl+C or rclpy.shutdown()).
        Always stop tracking before disconnecting to avoid leaving the device in a bad state.
        """
        self.get_logger().info('Stopping WEMT tracking...')
        self.api.stop_tracking()
        super().destroy_node()#安全退出与资源释放。


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
