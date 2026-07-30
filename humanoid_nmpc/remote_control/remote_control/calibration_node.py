#!/usr/bin/env python3

"""
calibration_node.py

Determines the user's arm lengths (upper_arm and forearm) from two calibration
poses using the WMET EM tracker. Results are saved to a YAML file and reused
in every session without repeating the calibration.

Calibration procedure (two poses):

  Pose 1 — Arm hanging straight down:
    User stands upright, arm fully relaxed at side, elbow straight.
    The entire arm (upper_arm + forearm) hangs vertically.
    → total_arm_length = |position_mm| / 1000

  Pose 2 — Elbow bent 90°, upper arm hanging, forearm horizontal:
    User keeps upper arm hanging straight down, bends elbow to exactly 90°,
    forearm points horizontally forward.
    → upper_arm_length = |position.z|   (vertical component = upper arm)
    → forearm_length   = sqrt(x² + y²)  (horizontal component = forearm)

Coordinate frame: transmitter is at shoulder (origin).
"""

import os
import math
import yaml
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime, timezone


# Where the calibration file is saved.
# ~/.ros/wmet_calibration.yaml — always accessible regardless of workspace.
CALIBRATION_FILE = os.path.expanduser('~/.ros/wmet_calibration.yaml')

# How many pose samples to average per calibration pose (at 50 Hz → 2 seconds).
NUM_SAMPLES = 100#采样数量

# Maximum allowed difference between total arm length (Pose 1) and
# upper_arm + forearm (Pose 2). If exceeded, calibration is likely wrong.
VALIDATION_TOLERANCE_M = 0.03  # 3 cm误差上限


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')#向ros系统注册一个名为calibration_node的节点

        # Stores the most recent pose received from the EM tracker.
        self.latest_pose = None#创建变量，暂存来自电磁的姿态数据
        self._pose_lock = threading.Lock()#创建线程锁，互斥。一共两个线程。接收写入数据；计算臂长

        # Subscribe to the EM tracker topic published by em_tracker_node.py
        self.pose_subscriber = self.create_subscription(#创建ros2订阅者
            PoseStamped,#消息类型
            '/em/pose',#话题名称
            self._on_pose_received,#回调函数。只要电磁节点往/em/pose发消息，就会触发这个回调函数，把数据存入latest_pose
            10#QoS队列深度
        )

        self.get_logger().info('Calibration node started. Waiting for /em/pose ...')

        # Run the calibration sequence in a background thread so that
        # rclpy.spin() continues running and subscriber callbacks fire normally.
        self._calibration_thread = threading.Thread(#后台子线程。跑_run_calibration_sequence校准步骤，完成和用户的交互。
            target=self._run_calibration_sequence,
            daemon=True#主程序退出，这个程序也自动销毁。
        )
        self._calibration_thread.start()

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_pose_received(self, msg: PoseStamped):
        """Stores the latest pose. Called by rclpy at 50 Hz."""
        with self._pose_lock:#收到数据自动触发。打开线程锁。
            self.latest_pose = msg

    # ------------------------------------------------------------------
    # Calibration sequence (runs in background thread)
    # ------------------------------------------------------------------

    def _run_calibration_sequence(self):
        """
        Guides the user through two calibration poses interactively.
        Runs in a separate thread; uses input() to wait for the user.
        """

        # ---- Check for existing calibration ----检查历史的校准文件。
        if os.path.exists(CALIBRATION_FILE):
            print(f'\n[Calibration] Existing calibration found at {CALIBRATION_FILE}')
            cal = self._load_calibration()
            print(f'  upper_arm_length = {cal["upper_arm_length_m"]*100:.1f} cm')
            print(f'  forearm_length   = {cal["forearm_length_m"]*100:.1f} cm')
            answer = input('\nUse existing calibration? [Y/n]: ').strip().lower()
            if answer != 'n':
                print('[Calibration] Using existing calibration. Node will now exit.')
                return

        # ---- Wait for first pose to arrive ----阻塞等待传感器的首帧数据。
        print('\n[Calibration] Waiting for EM tracker data on /em/pose ...')
        while rclpy.ok():
            with self._pose_lock:
                if self.latest_pose is not None:
                    break
            import time; time.sleep(0.1)
        print('[Calibration] EM tracker data received.\n')

        # ---- Pose 1: Arm hanging straight down ----
        print('=' * 55)
        print('POSE 1: Arm hanging straight down')
        print('  Stand upright. Let your arm hang fully relaxed.')
        print('  Keep elbow straight. Do NOT move during recording.')
        print('=' * 55)
        input('Press ENTER when ready...')#阻塞等待，用户站好后按回车

        samples_1 = self._collect_samples(NUM_SAMPLES)#在 2 秒内采集 100 帧新数据
        P1 = self._average_position(samples_1)#求算术平均值得到消除噪声后的手腕三维坐标
        total_arm_length = math.sqrt(P1[0]**2 + P1[1]**2 + P1[2]**2)

        print(f'  Recorded position: x={P1[0]*100:.1f} cm, y={P1[1]*100:.1f} cm, z={P1[2]*100:.1f} cm')
        print(f'  → Total arm length: {total_arm_length*100:.1f} cm\n')

        # ---- Pose 2: Elbow bent 90°, upper arm down, forearm horizontal ----
        print('=' * 55)
        print('POSE 2: Elbow bent 90°')
        print('  Keep upper arm hanging straight down.')
        print('  Bend elbow to exactly 90°.')
        print('  Forearm points horizontally forward.')
        print('  Do NOT move during recording.')
        print('=' * 55)
        input('Press ENTER when ready...')

        samples_2 = self._collect_samples(NUM_SAMPLES)
        P2 = self._average_position(samples_2)

        # In this pose:
        #   upper arm hangs down → z-component = -upper_arm_length
        #   forearm points forward → x,y-components = forearm_length
        upper_arm_length = abs(P2[2])#大臂
        forearm_length   = math.sqrt(P2[0]**2 + P2[1]**2)#小臂

        print(f'  Recorded position: x={P2[0]*100:.1f} cm, y={P2[1]*100:.1f} cm, z={P2[2]*100:.1f} cm')
        print(f'  → upper_arm_length: {upper_arm_length*100:.1f} cm')
        print(f'  → forearm_length:   {forearm_length*100:.1f} cm\n')

        # ---- Validation ----
        computed_total = upper_arm_length + forearm_length#根据大臂和小臂 计算总臂长
        error = abs(computed_total - total_arm_length)#再和第一个动作计算的总臂长 作差
        print(f'Validation: Pose1 total = {total_arm_length*100:.1f} cm, '
              f'Pose2 sum = {computed_total*100:.1f} cm, '
              f'difference = {error*100:.1f} cm')#得出误差为

        if error > VALIDATION_TOLERANCE_M:#如果误差超过上限
            print(f'\n[WARNING] Difference ({error*100:.1f} cm) exceeds tolerance '
                  f'({VALIDATION_TOLERANCE_M*100:.0f} cm).')
            print('  Possible causes:')
            print('  - Arm was not fully straight in Pose 1')#手臂在动作1 没有完全伸直
            print('  - Elbow was not exactly 90° in Pose 2')#手臂在动作2 没有完全垂直
            answer = input('Save anyway? [y/N]: ').strip().lower()
            if answer != 'y':
                print('[Calibration] Calibration cancelled. Please try again.')
                return#直接退出函数，节点运行结束。这里可以考虑更改为while循环
        else:
            print('[Calibration] Validation passed.')

        # ---- Save ----
        self._save_calibration(upper_arm_length, forearm_length, total_arm_length)#数据保存
        print(f'\n[Calibration] Saved to {CALIBRATION_FILE}')
        print('[Calibration] You can now start retargeting_node.py.')

    # ------------------------------------------------------------------
    # Sample collection helpers
    # ------------------------------------------------------------------

    def _collect_samples(self, n: int) -> list:
        """
        Collects n pose samples from /em/pose.
        Waits for each new sample (skips duplicates by tracking the last stamp).
        Returns a list of PoseStamped messages.
        """
        import time
        samples = []#创建一个列表，用来存放收集到的 PoseStamped 消息对象
        last_stamp = None#用来记录上一帧数据的 ROS2 时间戳，这是实现去重的关键变量
        print(f'  Recording {n} samples ', end='', flush=True)

        while len(samples) < n:
            with self._pose_lock:#上锁
                msg = self.latest_pose
            if msg is not None:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if stamp != last_stamp:
                    samples.append(msg)#将msg添加到stamp中
                    last_stamp = stamp#更新stamp
                    if len(samples) % 10 == 0:
                        print('.', end='', flush=True)#打点 显示在屏幕中
            time.sleep(0.01)  # poll at 100 Hz, data arrives at 50 Hz

        print(f' done ({n} samples)')
        return samples

    def _average_position(self, samples: list) -> list:#计算平均
        """Returns the mean [x, y, z] position in meters from a list of PoseStamped."""
        x = sum(s.pose.position.x for s in samples) / len(samples)
        y = sum(s.pose.position.y for s in samples) / len(samples)
        z = sum(s.pose.position.z for s in samples) / len(samples)
        return [x, y, z]

    # ------------------------------------------------------------------
    # YAML load / save 
    # ------------------------------------------------------------------
    #数据写入
    def _save_calibration(self, upper_arm_m: float, forearm_m: float, total_m: float):#接收三个浮点数
        """Writes calibration results to YAML file."""
        os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)#创建目录
        data = {#创建字典结构
            'calibration': {
                'upper_arm_length_m': round(upper_arm_m, 4),
                'forearm_length_m':   round(forearm_m, 4),
                'total_arm_length_m': round(total_m, 4),
                'calibrated_at': datetime.now(timezone.utc).isoformat(),
            }
        }
        with open(CALIBRATION_FILE, 'w') as f:#写入模式
            yaml.dump(data, f, default_flow_style=False)#块状可读文本
    #数据读取
    def _load_calibration(self) -> dict:
        """Reads calibration results from YAML file. Returns the inner dict."""
        with open(CALIBRATION_FILE, 'r') as f:#只读
            data = yaml.safe_load(f)
        return data['calibration']#返回字典


# -----------------------------------------------------------------------
# Static helper — used by other nodes (retargeting_node.py) 
# -----------------------------------------------------------------------

def load_calibration() -> dict:#没有self ，外部节点可以调用。
    """
    Load calibration from the YAML file.
    Import this function in retargeting_node.py:

        from remote_control.calibration_node import load_calibration
        cal = load_calibration()
        upper_arm = cal['upper_arm_length_m']
        forearm   = cal['forearm_length_m']

    Raises FileNotFoundError if calibration has not been run yet.
    """
    if not os.path.exists(CALIBRATION_FILE):#防错。如果没有文档，主动抛出异常。
        raise FileNotFoundError(
            f'Calibration file not found at {CALIBRATION_FILE}. '
            'Please run calibration_node first.'
        )
    with open(CALIBRATION_FILE, 'r') as f:
        data = yaml.safe_load(f)
    return data['calibration']#读取并解析yaml数据


# -----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
