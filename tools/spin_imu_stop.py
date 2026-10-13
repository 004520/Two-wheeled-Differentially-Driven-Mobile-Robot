#!/usr/bin/env python3
"""IMU 积分停车测试

原地旋转，停车判据是「IMU 积分角度达到 360°」，不依赖任何计时：
  1. 订阅 /imu，用消息到达本脚本的本地时钟算 dt，对 angular_velocity.z 积分
  2. 旋转前静止采样 0.5 s 求 z 轴零偏，积分时扣除
  3. 以 10 Hz 发 angular.z = 0.8 rad/s，直到 |积分角度| >= 2π 才发零速停车

回调在独立 executor 线程里跑，主线程只负责 10 Hz 发指令，两者互不阻塞。

用法（launch 保持运行，新开终端即可）：
  source /home/chuyun/mycar_ws/install/setup.bash
  python3 /home/chuyun/mycar_ws/tools/spin_imu_stop.py

Ctrl+C 会立即停车。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# ---- 可调参数 ----------------------------------------------------------
ANGULAR_SPEED = 0.8          # 角速度 rad/s（正 = 逆时针）
LINEAR_SPEED = 0.0           # 原地旋转
CMD_RATE_HZ = 10.0           # cmd_vel 发送频率
STOP_FRAMES = 20             # 停车帧数
STOP_INTERVAL_S = 0.05       # 停车帧间隔
SETTLE_SECONDS = 1.0         # 停车后静置时间
NOMINAL_RAD = 2.0 * math.pi  # 停车判据：积分角度达到 360°
SAFETY_TIMEOUT_S = 30.0      # 兜底安全超时（仅防飞车，正常不会触发）
BIAS_SAMPLE_SECONDS = 0.5    # 静止采样零偏的时长
BIAS_MOVE_THRESHOLD = 0.1    # 采样期间 |wz| 超过此值(rad/s)即认为车在动
BIAS_MIN_SAMPLES = 5         # 有效零偏所需的最少样本数
BIAS_CALIB_TIMEOUT_S = 2.0   # 采样阶段整体超时
DT_MAX_S = 0.5               # dt 超出 [0, 0.5] s 时跳过该次积分
FIRST_FRAME_TIMEOUT_S = 10.0
# ------------------------------------------------------------------------


class SpinImuStop(Node):
    def __init__(self):
        super().__init__('spin_imu_stop')
        self._lock = threading.Lock()

        # ---- IMU 积分 ----
        self._imu_angle = 0.0
        self._imu_last_time = None      # 上一帧到达本脚本的本地时刻（monotonic）
        self._imu_dt_sum = 0.0
        self._imu_seen = False
        self._imu_frames = 0            # 参与积分的帧数
        self._imu_skipped = 0           # dt 异常跳过的帧数
        self._integrating = False

        # ---- 零偏 ----
        self._bias_z = 0.0
        self._bias_samples = []
        self._calibrating = False
        self._bias_move_detected = False

        # ---- 判据与计时 ----
        self._target_reached = False
        self._t_target = None           # 积分角度达到 360° 的时刻
        self._t_cmd_first = None        # 第一条非零速 cmd_vel 的时刻

        self._stop_requested = False

        # 下位机侧 /imu 是 best effort，订阅必须匹配
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Imu, '/imu', self._imu_cb, sub_qos)

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

    # ---------------- 回调（executor 线程） ----------------
    def _imu_cb(self, msg):
        # 不用 header.stamp：下位机 stamp 分辨率只有 20 ms，低于 IMU 帧间隔，dt 会系统性偏小
        now = time.monotonic()
        wz = msg.angular_velocity.z
        with self._lock:
            self._imu_seen = True
            if self._calibrating:
                # 静止采样阶段：只收集样本，不积分
                self._bias_samples.append(wz)
                self._imu_last_time = now
                if abs(wz) > BIAS_MOVE_THRESHOLD:
                    self._bias_move_detected = True
                return
            if not self._integrating:
                return
            if self._imu_last_time is None:
                self._imu_last_time = now
                return
            dt = now - self._imu_last_time
            self._imu_last_time = now
            if dt <= 0.0 or dt > DT_MAX_S:
                self._imu_skipped += 1
                return
            self._imu_angle += (wz - self._bias_z) * dt
            self._imu_frames += 1
            self._imu_dt_sum += dt
            if not self._target_reached and abs(self._imu_angle) >= NOMINAL_RAD:
                self._target_reached = True
                self._t_target = now

    # ---------------- 状态 ----------------
    def wait_first_frame(self, timeout=FIRST_FRAME_TIMEOUT_S):
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop_requested:
            with self._lock:
                if self._imu_seen:
                    return True
            time.sleep(0.05)
        with self._lock:
            return self._imu_seen

    def start_tracking(self):
        with self._lock:
            self._imu_angle = 0.0
            self._imu_last_time = None
            self._imu_dt_sum = 0.0
            self._imu_frames = 0
            self._imu_skipped = 0
            self._bias_z = 0.0
            self._bias_samples = []
            self._calibrating = True      # 先进静止采样阶段，此间不积分
            self._bias_move_detected = False
            self._target_reached = False
            self._t_target = None
            self._t_cmd_first = None
            self._integrating = True

    def calibrate_bias(self, duration=BIAS_SAMPLE_SECONDS):
        """静止采样 IMU z 轴角速度求零偏，返回是否应用了补偿"""
        start = time.time()
        deadline = start + duration + BIAS_CALIB_TIMEOUT_S
        while time.time() < deadline and not self._stop_requested:
            with self._lock:
                enough = (len(self._bias_samples) >= BIAS_MIN_SAMPLES
                          and (time.time() - start) >= duration)
            if enough:
                break
            time.sleep(0.02)

        with self._lock:
            self._calibrating = False
            samples = list(self._bias_samples)
            moved = self._bias_move_detected

        if moved or len(samples) < BIAS_MIN_SAMPLES:
            print('[IMU] 采样期间检测到车体运动（|wz| > %.2f rad/s）或样本不足，'
                  '跳过零偏补偿' % BIAS_MOVE_THRESHOLD)
            return False

        bias = sum(samples) / len(samples)
        self._bias_z = bias
        return True

    def snapshot(self):
        with self._lock:
            return {
                'angle': self._imu_angle,
                'bias': self._bias_z,
                'frames': self._imu_frames,
                'skipped': self._imu_skipped,
                'dt_mean': (self._imu_dt_sum / self._imu_frames
                            if self._imu_frames > 0 else None),
                't_cmd_first': self._t_cmd_first,
                't_target': self._t_target,
            }

    # ---------------- 指令 ----------------
    def _publish_stop(self):
        stop = Twist()  # 全零
        for _ in range(STOP_FRAMES):
            self._pub.publish(stop)
            time.sleep(STOP_INTERVAL_S)

    def _publish_cmd(self, angular_z):
        """以 10 Hz 发 cmd_vel，直到积分角度达到 360° 或安全超时"""
        twist = Twist()
        twist.linear.x = float(LINEAR_SPEED)
        twist.angular.z = float(angular_z)

        period = 1.0 / CMD_RATE_HZ
        start = time.monotonic()
        next_tick = start
        while not self._stop_requested:
            self._pub.publish(twist)
            now = time.monotonic()
            with self._lock:
                if self._t_cmd_first is None:
                    self._t_cmd_first = now
                reached = self._target_reached
            if reached:
                return 'target'
            if (now - start) >= SAFETY_TIMEOUT_S:
                return 'timeout'
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
        return 'abort'

    # ---------------- 主流程 ----------------
    def run(self):
        print('=' * 62)
        print('IMU 积分停车测试')
        print('=' * 62)
        print('[IMU] 等待 /imu 首帧 ...')
        if not self.wait_first_frame():
            print('[IMU] 未收到 /imu，请确认 micro-ROS 与 EKF 已启动')
            return False

        self.start_tracking()
        print('[IMU] 静止采样零偏 %.1f s（车必须完全静止）...' % BIAS_SAMPLE_SECONDS)
        self.calibrate_bias()
        bias = self.snapshot()['bias']
        print('-' * 62)
        print('  IMU 零偏       = %+0.4f rad/s' % bias)
        print('-' * 62)

        print('[IMU] 发第一条 cmd_vel ...')
        reason = self._publish_cmd(ANGULAR_SPEED)
        if reason == 'timeout':
            print('[IMU] 安全超时 %.1f s，强制停车' % SAFETY_TIMEOUT_S)
        elif reason == 'abort':
            print('[IMU] 收到停止请求，进入停车流程')
        else:
            print('[IMU] IMU 积分达 360°，停车')

        print('[IMU] 停车 ...')
        self._publish_stop()
        time.sleep(SETTLE_SECONDS)

        s = self.snapshot()
        elapsed = None
        if s['t_cmd_first'] is not None and s['t_target'] is not None:
            elapsed = s['t_target'] - s['t_cmd_first']

        print('-' * 62)
        print('  IMU 积分角度   = %+0.4f rad (%+0.2f deg)' % (s['angle'], math.degrees(s['angle'])))
        print('  实际耗时       = %s' % ('未检出' if elapsed is None else '%0.3f s' % elapsed))
        if elapsed and elapsed > 0.0:
            print('  平均角速度     = %+0.4f rad/s' % (s['angle'] / elapsed))
        else:
            print('  平均角速度     = 未检出')
        if s['dt_mean'] is not None:
            print('  平均 dt        = %0.1f ms（等效 %0.1f Hz）'
                  % (s['dt_mean'] * 1000.0, 1.0 / s['dt_mean']))
        print('  IMU 积分帧数   = %d（dt 异常跳过 %d 帧）' % (s['frames'], s['skipped']))
        print('-' * 62)

        try:
            answer = input('  请用胶带标记法量车实际转了多少度，输入：____ deg\n  实际角度 = ')
        except (EOFError, KeyboardInterrupt):
            answer = ''
        answer = answer.strip()
        if answer:
            try:
                real_deg = float(answer)
                print('-' * 62)
                print('  实际角度       = %+0.2f deg' % real_deg)
                print('  IMU 角度       = %+0.2f deg' % math.degrees(s['angle']))
                print('  差值           = %+0.2f deg' % (real_deg - math.degrees(s['angle'])))
                if abs(real_deg) > 1e-3:
                    print('  实际/IMU 比值  = %0.4f' % (real_deg / math.degrees(s['angle'])))
            except ValueError:
                print('  输入非数字，跳过对比')
        print('=' * 62)
        return reason == 'target'


def main():
    rclpy.init()
    node = SpinImuStop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 回调在后台线程处理，主线程专心按 10 Hz 发指令
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        print('\n[IMU] 收到 Ctrl+C，正在停车 ...')
    finally:
        with node._lock:
            node._integrating = False
            node._calibrating = False
        node._stop_requested = True
        try:
            node._publish_stop()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        try:
            executor.remove_node(node)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
