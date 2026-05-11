import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from humanoid_mpc_msgs.msg import WalkingVelocityCommand

class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mpc_bridge')
        # 订阅标准速度话题 (来自 RViz 或滑块)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        # 发布 MPC 专用话题
        self.publisher_ = self.create_publisher(WalkingVelocityCommand, '/humanoid/walking_velocity_command', 10)

    def listener_callback(self, msg):
        new_msg = WalkingVelocityCommand()
        # 核心翻译逻辑
        new_msg.linear_velocity_x = msg.linear.x
        new_msg.linear_velocity_y = msg.linear.y
        new_msg.angular_velocity_z = msg.angular.z
        new_msg.desired_pelvis_height = 0.6  # 保持稳定的站立高度

        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Forwarding: x={new_msg.linear_velocity_x}, h={new_msg.desired_pelvis_height}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()