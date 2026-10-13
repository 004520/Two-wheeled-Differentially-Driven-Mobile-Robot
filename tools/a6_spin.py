#!/usr/bin/env python3
"""A6 原地旋转精度验收（P0 阶段 3，核心验收项）

流程：
  1. 订阅 /odometry/filtered，记录起始 yaw
  2. 以 10 Hz 循环向 /cmd_vel 发速度：linear.x = 0, angular.z = 0.5 rad/s，持续 12.6 s（≈360°）
  3. 停车（多发几帧零速），静置 2 s
  4. 记录终点 yaw
  5. 计算回零误差并判定

关于「回零误差」的计算：
  yaw 在 [-pi, pi] 之间回绕，转满一整圈后 yaw 应回到起始值。
  设 d = normalize(yaw_end - yaw_start)，则：
      恰好 360°  -> d = 0        -> 误差 0
      只转了 350° -> d = -10°    -> 误差 10°
      转了 370°   -> d = +10°    -> 误差 10°
  因此 **误差 = |normalize(yaw_end - yaw_start)|**（对"名义一整圈"成立）。

用法（launch 保持运行，新开终端即可）：
  source /home/chuyun/mycar_ws/install/setup.bash
  python3 /home/chuyun/mycar_ws/tools/a6_spin.py

Ctrl+C 会立即停车。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ---- 可调参数 ----------------------------------------------------------
ANGULAR_SPEED = 0.8         # 角速度 rad/s（正 = 逆时针）
RUN_SECONDS = 7.85          # 0.8 rad/s * 7.85 s ≈ 6.28 rad = 360°
SETTLE_SECONDS = 2.0        # 停车后静置时间
CMD_RATE_HZ = 10.0          # cmd_vel 发送频率
TOLERANCE_DEG = 5.0         # A6 通过阈值：yaw 回零误差 < 5°
POSE_TIMEOUT_S = 10.0       # 等待 /odometry/filtered 首帧的超时
# ------------------------------------------------------------------------


def yaw_from_quaternion(q):
    """四元数 -> yaw（rad）"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    """角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class A6SpinTest(Node):
    def __init__(self):
        super().__init__('a6_test')
        self._pose = None            # (x, y, yaw)
        self._stop_requested = False

        self._sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 50)

        # /cmd_vel 必须用 BEST_EFFORT：micro-ROS 端的订阅是 best effort，
        # 默认 RELIABLE 发布者与之不兼容，消息发不出去。
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

    # ---------------- 回调 ----------------
    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    # ---------------- 工具 ----------------
    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline and not self._stop_requested:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_first_pose(self, timeout=POSE_TIMEOUT_S):
        deadline = time.time() + timeout
        while time.time() < deadline and self._pose is None and not self._stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pose

    def _publish_stop(self, frames=10):
        stop = Twist()  # 全零
        for _ in range(frames):
            self._pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def _publish_cmd(self, linear_x, angular_z, seconds):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        period = 1.0 / CMD_RATE_HZ
        start = time.time()
        while (time.time() - start) < seconds and not self._stop_requested:
            self._pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    # ---------------- 主流程 ----------------
    def run(self):
        print('=' * 62)
        print('A6 原地旋转精度验收（目标 360.00 deg，通过阈值 误差 < 5.00 deg）')
        print('=' * 62)
        print('[A6] 等待 /odometry/filtered 首帧 ...')

        if self._wait_first_pose() is None:
            print('[A6] 未收到 /odometry/filtered，请确认 EKF 已启动')
            return False

        start_pose = self._pose
        yaw_start = start_pose[2]
        print('[A6] 起始位姿: x=%.4f m  y=%.4f m  yaw=%.4f rad (%.2f deg)'
              % (start_pose[0], start_pose[1], yaw_start, math.degrees(yaw_start)))

        print('[A6] 原地旋转中: linear.x=0, angular.z=%.2f rad/s, 持续 %.1f s ...'
              % (ANGULAR_SPEED, RUN_SECONDS))
        self._publish_cmd(0.0, ANGULAR_SPEED, RUN_SECONDS)

        print('[A6] 停车 ...')
        self._publish_stop()
        print('[A6] 静置 %.1f s ...' % SETTLE_SECONDS)
        self._spin_for(SETTLE_SECONDS)

        end_pose = self._pose
        yaw_end = end_pose[2]
        print('[A6] 终点位姿: x=%.4f m  y=%.4f m  yaw=%.4f rad (%.2f deg)'
              % (end_pose[0], end_pose[1], yaw_end, math.degrees(yaw_end)))

        delta = normalize_angle(yaw_end - yaw_start)        # 回绕后的残差
        error_rad = abs(delta)                              # 相对 360° 的回零误差
        error_deg = math.degrees(error_rad)

        print('-' * 62)
        print('  起始 yaw          = %+.4f rad (%+.2f deg)'
              % (yaw_start, math.degrees(yaw_start)))
        print('  终点 yaw          = %+.4f rad (%+.2f deg)'
              % (yaw_end, math.degrees(yaw_end)))
        print('  yaw 变化(回绕后)  = %+.4f rad (%+.2f deg)'
              % (delta, math.degrees(delta)))
        print('  名义旋转量        = %+.4f rad (%+.2f deg)'
              % (math.copysign(2.0 * math.pi, ANGULAR_SPEED),
                 math.degrees(math.copysign(2.0 * math.pi, ANGULAR_SPEED))))
        print('  回零误差          = %.4f rad (%.2f deg)' % (error_rad, error_deg))
        print('-' * 62)

        passed = error_deg < TOLERANCE_DEG
        print('[A6] 判定: %s（误差 %.2f deg %s 阈值 %.2f deg）'
              % ('通过 ✅' if passed else '不通过 ❌',
                 error_deg, '<' if passed else '>=', TOLERANCE_DEG))
        if not passed:
            print('[A6] 提示: 纯编码器积分通常 10~30 deg；若本结果与改前相当，')
            print('           说明 IMU 仍未真正参与融合，请检查 /diagnostics 与')
            print('           angular_velocity_covariance 是否仍为 -1。')
        print('=' * 62)
        return passed


def main():
    rclpy.init()
    node = A6SpinTest()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        print('\n[A6] 收到 Ctrl+C，正在停车 ...')
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys_exit = main()
    raise SystemExit(sys_exit)
