#!/usr/bin/env python3
"""A6 原地旋转 + IMU 积分对比测试

一个节点同时做三件事：
  1. 订阅 /imu，对 angular_velocity.z 按「消息到达本脚本」的本地时钟 dt 做积分，累计 IMU 角度
  2. 订阅 /odometry/filtered，四元数转 yaw 并累加「连续、未回绕」的转角
  3. 以 10 Hz 向 /cmd_vel 发 angular.z = 0.8 rad/s

停车不靠计时：每帧 odom 更新累计转角，达到 (2π - PRE_STOP_OFFSET) 才发零速，
以此规避 micro-ROS + WiFi 链路 30~200 ms 的执行延迟；另有 30 s 安全超时兜底。

回调在独立 executor 线程里跑，主线程负责 10 Hz 定时发指令，两者互不阻塞。

用法（launch 保持运行，新开终端即可）：
  source /home/chuyun/mycar_ws/install/setup.bash
  python3 /home/chuyun/mycar_ws/tools/a6_spin_imu_check.py

Ctrl+C 会立即停车。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# ---- 可调参数 ----------------------------------------------------------
ANGULAR_SPEED = 0.8          # 角速度 rad/s（正 = 逆时针）
LINEAR_SPEED = 0.0           # 原地旋转，线速度为 0
SETTLE_SECONDS = 2.0         # 停车后静置时间
CMD_RATE_HZ = 10.0           # cmd_vel 发送频率
STOP_FRAMES = 20             # 停车帧数
STOP_INTERVAL_S = 0.05       # 停车帧间隔
NOMINAL_RAD = 2.0 * math.pi  # 名义旋转量 360°
PRE_STOP_OFFSET = math.radians(18)   # 提前停车量 18°，补偿物理惯性过冲
SAFETY_TIMEOUT_S = 30.0      # 安全超时：到不了 360° 就强制停车并报错
TOLERANCE_DEG = 5.0          # 通过阈值：EKF 与 IMU 误差均需 < 5°
FIRST_FRAME_TIMEOUT_S = 10.0
BASELINE_TIMEOUT_S = 2.0     # 等待 odom 基准帧的超时
DT_MAX_S = 0.5               # dt 超出 [0, 0.5] s（丢帧/时间跳变）时跳过该次积分
BIAS_SAMPLE_SECONDS = 0.5     # 旋转前静止采样 IMU z 轴角速度的时长
BIAS_MOVE_THRESHOLD = 0.1     # 采样期间 |wz| 超过此值(rad/s)即认为车在动，跳过补偿
BIAS_MIN_SAMPLES = 5          # 有效零偏所需的最少样本数
BIAS_CALIB_TIMEOUT_S = 2.0    # 采样阶段的整体超时（在 0.5 s 基础上再宽限）
START_MOVE_THRESHOLD_RAD = 0.01    # 判定「开始转动」的 yaw 变化阈值
STOPPED_YAW_TOL_RAD = 0.0017       # 判定「停稳」的窗口内 yaw 变化上限（0.1°）
STOPPED_WINDOW_S = 0.3             # 连续满足上述上限多久算停稳
STOP_DETECT_TIMEOUT_S = 5.0        # 等待停稳的超时
# ------------------------------------------------------------------------


def stop_threshold_rad():
    """触发停车的累计转角阈值"""
    return NOMINAL_RAD - PRE_STOP_OFFSET


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


def stamp_to_sec(stamp):
    """builtin_interfaces/Time -> float 秒"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fmt_rad_deg(rad):
    return '%+0.4f rad (%+0.2f deg)' % (rad, math.degrees(rad))


def delta_or_none(t_from, t_to):
    """两个 monotonic 时刻之差；任一端缺失返回 None"""
    if t_from is None or t_to is None:
        return None
    return t_to - t_from


def fmt_ms(seconds):
    """秒 -> 'x.xxx s (xxx ms)'；None 表示未检出"""
    if seconds is None:
        return '未检出'
    return '%0.3f s (%0.0f ms)' % (seconds, seconds * 1000.0)


class A6ImuCheck(Node):
    def __init__(self):
        super().__init__('a6_imu_check')
        self._lock = threading.Lock()

        # ---- EKF 位姿 ----
        self._odom_yaw = None
        self._odom_seen = False
        self._yaw_start_val = None      # 起始 yaw（基准帧确立）
        self._yaw_prev = None           # 上一帧 yaw，用于逐帧累加
        self._yaw_accum = 0.0           # 累计转角（连续、未回绕）

        # ---- IMU 积分 ----
        self._imu_angle = 0.0
        self._imu_last_time = None      # 上一帧到达本脚本的本地时刻（monotonic）
        self._imu_dt_sum = 0.0          # 有效 dt 之和，用于算平均 dt
        self._imu_seen = False

        # ---- IMU z 轴零偏 ----
        self._bias_z = 0.0              # 零偏估计值 rad/s（未补偿时为 0）
        self._bias_samples = []         # 静止采样得到的 wz 样本
        self._calibrating = False       # 处于零偏采样阶段
        self._bias_move_detected = False  # 采样期间检测到车体运动
        self._imu_acked_frames = 0      # 真正参与积分的帧数
        self._imu_skipped_frames = 0    # 因 dt 异常被跳过的帧数
        self._integrating = False

        # ---- 停车判据与计时 ----
        self._target_reached = False
        self._target_time = None        # 累计转角达到停车阈值的时刻
        self._start_move_time = None    # 首次 yaw 变化超过阈值的时刻
        self._t_cmd_first = None        # 第一条非零速 cmd_vel
        self._t_cmd_last = None         # 最后一条非零速 cmd_vel
        self._t_stop_first = None       # 第一条零速 cmd_vel
        self._t_stopped = None          # 判定车停稳的时刻

        self._stop_requested = False

        # 发布侧均为 best effort（micro-ROS 订阅也是 best effort），订阅必须匹配
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, sub_qos)
        self.create_subscription(Imu, '/imu', self._imu_cb, sub_qos)

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

    # ---------------- 回调（executor 线程） ----------------
    def _odom_cb(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._odom_yaw = yaw
            self._odom_seen = True
            if not self._integrating or self._stop_requested:
                return
            if self._yaw_prev is None:
                # 基准帧：只记录起始 yaw，不累加
                self._yaw_prev = yaw
                self._yaw_start_val = yaw
                return
            # 逐帧累加，避免跨 ±π 回绕导致转角丢失
            self._yaw_accum += normalize_angle(yaw - self._yaw_prev)
            self._yaw_prev = yaw
            if (self._start_move_time is None
                    and abs(self._yaw_accum) >= START_MOVE_THRESHOLD_RAD):
                self._start_move_time = time.monotonic()
            if (not self._target_reached
                    and abs(self._yaw_accum) >= stop_threshold_rad()):
                self._target_reached = True
                self._target_time = time.monotonic()

    def _imu_cb(self, msg):
        # 不使用 msg.header.stamp：下位机 stamp 分辨率只有 20 ms，而 IMU 帧间隔约 24 ms(40 Hz)，
        # 用 stamp 算 dt 会系统性偏小，导致积分少算十几到几十度。
        # 改用「消息到达本脚本」的本地单调时钟计时。
        now = time.monotonic()
        wz = msg.angular_velocity.z
        with self._lock:
            self._imu_seen = True
            if self._calibrating:
                # 静止采样阶段：只收集样本，不做积分
                self._bias_samples.append(wz)
                self._imu_last_time = now
                if abs(wz) > BIAS_MOVE_THRESHOLD:
                    self._bias_move_detected = True
                return
            if not self._integrating:
                return
            if self._imu_last_time is None:
                # 第一帧只记录到达时刻，不积分
                self._imu_last_time = now
                return
            dt = now - self._imu_last_time
            self._imu_last_time = now
            if dt <= 0.0 or dt > DT_MAX_S:
                self._imu_skipped_frames += 1
                return
            self._imu_angle += (wz - self._bias_z) * dt
            self._imu_acked_frames += 1
            self._imu_dt_sum += dt

    # ---------------- 状态读写 ----------------
    def wait_first_frames(self, timeout=FIRST_FRAME_TIMEOUT_S):
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop_requested:
            with self._lock:
                ok = self._odom_seen and self._imu_seen
            if ok:
                return True
            time.sleep(0.05)
        with self._lock:
            return self._odom_seen and self._imu_seen

    def start_tracking(self):
        """旋转开始前清零；下一帧 odom 作为基准帧，下一帧 IMU 作为「只记时间戳」的首帧"""
        with self._lock:
            self._imu_angle = 0.0
            self._imu_last_time = None
            self._imu_dt_sum = 0.0
            self._imu_acked_frames = 0
            self._imu_skipped_frames = 0
            self._bias_z = 0.0
            self._bias_samples = []
            self._calibrating = True        # 先进入静止采样阶段，此间不积分
            self._bias_move_detected = False
            self._yaw_prev = None
            self._yaw_start_val = None
            self._yaw_accum = 0.0
            self._target_reached = False
            self._target_time = None
            self._start_move_time = None
            self._integrating = True

    def stop_tracking(self):
        with self._lock:
            self._integrating = False

    def calibrate_bias(self, duration=BIAS_SAMPLE_SECONDS):
        """静止采样 IMU z 轴角速度求零偏。返回是否实际应用了补偿。"""
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
            print('[CHECK] 采样期间检测到车体运动（|wz| > %.2f rad/s）或样本不足，'
                  '跳过 IMU 零偏补偿' % BIAS_MOVE_THRESHOLD)
            return False

        bias = sum(samples) / len(samples)
        self._bias_z = bias
        print('[CHECK] IMU z 轴零偏 = %+0.4f rad/s（%d 个样本 / %.2f s）'
              % (bias, len(samples), duration))
        return True

    def wait_baseline(self, timeout=BASELINE_TIMEOUT_S):
        """等待基准帧（确立起始 yaw）"""
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop_requested:
            with self._lock:
                ready = self._yaw_start_val is not None
            if ready:
                return True
            time.sleep(0.02)
        return False

    def snapshot(self):
        with self._lock:
            return {
                'start_yaw': self._yaw_start_val,
                'end_yaw': self._odom_yaw,
                'yaw_accum': self._yaw_accum,
                'imu_angle': self._imu_angle,
                'imu_frames': self._imu_acked_frames,
                'imu_skipped': self._imu_skipped_frames,
                'imu_dt_mean': (self._imu_dt_sum / self._imu_acked_frames
                                if self._imu_acked_frames > 0 else None),
                't_cmd_first': self._t_cmd_first,
                't_cmd_last': self._t_cmd_last,
                't_start_move': self._start_move_time,
                't_target': self._target_time,
                't_stop_first': self._t_stop_first,
                't_stopped': self._t_stopped,
            }

    def wait_until_stopped(self, timeout=STOP_DETECT_TIMEOUT_S):
        """yaw 在 STOPPED_WINDOW_S 内变化不超过 STOPPED_YAW_TOL_RAD 即认为停稳"""
        deadline = time.monotonic() + timeout
        with self._lock:
            last_accum = self._yaw_accum
        last_change = time.monotonic()
        while time.monotonic() < deadline and not self._stop_requested:
            with self._lock:
                accum = self._yaw_accum
            now = time.monotonic()
            if abs(accum - last_accum) > STOPPED_YAW_TOL_RAD:
                last_change = now
                last_accum = accum
            elif (now - last_change) >= STOPPED_WINDOW_S:
                with self._lock:
                    self._t_stopped = now
                return now
            time.sleep(0.02)
        return None

    # ---------------- 指令 ----------------
    def _publish_stop(self):
        """停车：连续多发几帧零速，确保小车收到"""
        stop = Twist()  # 全零
        with self._lock:
            if self._t_stop_first is None:
                self._t_stop_first = time.monotonic()
        for _ in range(STOP_FRAMES):
            self._pub.publish(stop)
            time.sleep(STOP_INTERVAL_S)

    def _publish_cmd(self, angular_z):
        """以 10 Hz 发 cmd_vel，直到累计转角达阈值或安全超时。
        回调由独立 executor 线程处理，此处不做阻塞 spin。
        返回 'target' / 'timeout' / 'abort'。"""
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
                self._t_cmd_last = now
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
        print('A6 + IMU 积分对比测试（按 EKF yaw 停车）')
        print('=' * 62)
        print('[CHECK] 等待 /odometry/filtered 与 /imu 首帧 ...')

        if not self.wait_first_frames():
            print('[CHECK] 未同时收到 /odometry/filtered 与 /imu，请确认 EKF 与 micro-ROS 已启动')
            return False

        self.start_tracking()
        if not self.wait_baseline():
            print('[CHECK] 未拿到 /odometry/filtered 基准帧，退出')
            return False

        print('[CHECK] 静止采样 IMU z 轴零偏 %.1f s ...' % BIAS_SAMPLE_SECONDS)
        self.calibrate_bias()

        start_yaw = self.snapshot()['start_yaw']
        print('[CHECK] 起始 yaw: %s' % fmt_rad_deg(start_yaw))
        print('[CHECK] 旋转中: angular.z=%+0.2f rad/s, linear.x=%0.1f, 停车阈值 %s'
              % (ANGULAR_SPEED, LINEAR_SPEED, fmt_rad_deg(stop_threshold_rad())))

        reason = self._publish_cmd(ANGULAR_SPEED)
        if reason == 'timeout':
            print('[CHECK] 已达到安全超时 %.1f s，强制停车' % SAFETY_TIMEOUT_S)
        elif reason == 'abort':
            print('[CHECK] 收到停止请求，进入停车流程')
        else:
            print('[CHECK] 累计转角达到停车阈值')

        print('[CHECK] 停车 ...')
        self._publish_stop()
        self.wait_until_stopped()
        # 停车瞬间快照：此刻 EKF 尚未被 rf2o 的静止假漂移拉动
        s_motion = self.snapshot()
        print('[CHECK] 静置 %0.1f s ...' % SETTLE_SECONDS)
        time.sleep(SETTLE_SECONDS)

        self.stop_tracking()
        s = self.snapshot()

        end_yaw = s['end_yaw']
        imu_angle = s['imu_angle']
        ekf_delta = normalize_angle(end_yaw - start_yaw)
        # 360° 等价于 0，所以用回绕后的差值算误差
        ekf_error = abs(normalize_angle(ekf_delta - NOMINAL_RAD))
        imu_error = abs(normalize_angle(imu_angle - NOMINAL_RAD))
        diff = normalize_angle(ekf_delta - imu_angle)
        # 拆分指标：δ_motion = 停车瞬间的 EKF−IMU（衡量滤波器滞后）
        #           δ_static = 静置 2 s 后的 EKF−IMU（含 rf2o 静止漂移）
        ekf_delta_motion = normalize_angle(s_motion['end_yaw'] - start_yaw)
        diff_motion = normalize_angle(ekf_delta_motion - s_motion['imu_angle'])
        diff_static = diff
        static_injected = normalize_angle(diff_static - diff_motion)

        print('-' * 62)
        print('  起始 yaw        = %s' % fmt_rad_deg(start_yaw))
        print('  终点 yaw        = %s' % fmt_rad_deg(end_yaw))
        print('  EKF yaw 变化    = %s' % fmt_rad_deg(ekf_delta))
        print('  累计转角(连续)  = %s' % fmt_rad_deg(s['yaw_accum']))
        print('  IMU 积分角度    = %s' % fmt_rad_deg(imu_angle))
        print('  名义旋转量      = %s' % fmt_rad_deg(NOMINAL_RAD))
        print('-' * 62)
        print('  EKF 误差        = %0.2f deg' % math.degrees(ekf_error))
        print('  IMU 误差        = %0.2f deg' % math.degrees(imu_error))
        print('  EKF - IMU 差值  = %0.2f deg（= δ_static）' % math.degrees(diff))
        print('  --- δ 拆分（定位 rf2o 静止假漂移）---')
        print('  δ_motion（停车瞬间）  = %+0.2f deg   [EKF %+0.2f / IMU %+0.2f]'
              % (math.degrees(diff_motion),
                 math.degrees(ekf_delta_motion), math.degrees(s_motion['imu_angle'])))
        print('  δ_static（静置 %0.1f s）  = %+0.2f deg   [EKF %+0.2f / IMU %+0.2f]'
              % (SETTLE_SECONDS, math.degrees(diff_static),
                 math.degrees(ekf_delta), math.degrees(imu_angle)))
        print('  静置期灌入量(静−动)  = %+0.2f deg' % math.degrees(static_injected))
        print('  IMU 积分帧数    = %d（dt 异常跳过 %d 帧）' % (s['imu_frames'], s['imu_skipped']))
        if s['imu_dt_mean'] is None:
            print('  IMU 积分用本地时钟 dt，平均 dt = 未检出')
        else:
            print('  IMU 积分用本地时钟 dt，平均 dt = %0.1f ms（等效 %0.1f Hz）'
                  % (s['imu_dt_mean'] * 1000.0, 1.0 / s['imu_dt_mean']))
        print('-' * 62)
        print('  --- 链路延迟观测（monotonic clock）---')
        print('  首条 -> 末条 cmd_vel        = %s'
              % fmt_ms(delta_or_none(s['t_cmd_first'], s['t_cmd_last'])))
        print('  首条 cmd_vel -> 车开始转动  = %s'
              % fmt_ms(delta_or_none(s['t_cmd_first'], s['t_start_move'])))
        print('  首条 cmd_vel -> 达停车阈值  = %s'
              % fmt_ms(delta_or_none(s['t_cmd_first'], s['t_target'])))
        print('  停车命令 -> 车停稳          = %s'
              % fmt_ms(delta_or_none(s['t_stop_first'], s['t_stopped'])))
        print('-' * 62)

        ekf_passed = math.degrees(ekf_error) < TOLERANCE_DEG
        imu_passed = math.degrees(imu_error) < TOLERANCE_DEG
        passed = ekf_passed and imu_passed and reason == 'target'
        if reason == 'timeout':
            print('  错误：安全超时 %.1f s 内累计仅转过 %0.2f deg，未达 360°，已强制停车'
                  % (SAFETY_TIMEOUT_S, math.degrees(s['yaw_accum'])))
        print('  判定：EKF %s / IMU %s' % ('✅' if ekf_passed else '❌',
                                          '✅' if imu_passed else '❌'))
        print('=' * 62)
        return passed


def main():
    rclpy.init()
    node = A6ImuCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 回调在后台线程处理，主线程专心按 10 Hz 发指令
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        print('\n[CHECK] 收到 Ctrl+C，正在停车 ...')
    finally:
        node.stop_tracking()
        try:
            node._publish_stop()
        except Exception:
            pass
        node._stop_requested = True
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
