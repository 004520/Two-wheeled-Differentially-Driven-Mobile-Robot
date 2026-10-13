#!/usr/bin/env python3
"""A5 按位移停车版：累计位移 >= 5.0 m 就停车，不依赖时间。"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

TARGET_DISTANCE_M = 5.0
LINEAR_SPEED = 0.2
TOLERANCE_M = 0.10
POSE_TIMEOUT_S = 10.0

def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class A5DistTest(Node):
    def __init__(self):
        super().__init__('a5_dist_test')
        self._sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 50)
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        self._latest = None

    def _odom_cb(self, msg):
        self._latest = msg

    def _get_pose(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest is not None:
                break
        if self._latest is None:
            return None
        p = self._latest.pose.pose.position
        q = self._latest.pose.pose.orientation
        return (p.x, p.y, yaw_from_quaternion(q))

    def _send_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._pub.publish(msg)

    def run(self):
        start = self._get_pose()
        if start is None:
            self.get_logger().error('拿不到起始位姿')
            return
        self.get_logger().info(f'起始: x={start[0]:.4f} y={start[1]:.4f}')

        self.get_logger().info(f'直行中，目标位移 {TARGET_DISTANCE_M} m ...')
        t0 = time.time()
        while True:
            self._send_vel(LINEAR_SPEED, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self._get_pose()
            if cur is None:
                continue
            dx = cur[0] - start[0]
            dy = cur[1] - start[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= TARGET_DISTANCE_M:
                break
            if time.time() - t0 > 60.0:
                self.get_logger().warn('超时 60 秒，强制停车')
                break

        # 停车
        for _ in range(20):
            self._send_vel(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(2.0)

        end = self._get_pose()
        if end is None:
            self.get_logger().error('拿不到终点位姿')
            return

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.sqrt(dx*dx + dy*dy)
        err = abs(dist - TARGET_DISTANCE_M)

        self.get_logger().info(f'终点: x={end[0]:.4f} y={end[1]:.4f}')
        self.get_logger().info(f'位移: dist={dist:.4f} m, 误差={err:.4f} m ({err/TARGET_DISTANCE_M*100:.2f}%)')
        if err < TOLERANCE_M:
            self.get_logger().info('A5 通过 ✅')
        else:
            self.get_logger().warn('A5 不通过 ❌')

def main():
    rclpy.init()
    node = A5DistTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._send_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
