#!/usr/bin/env python3
"""为 rf2o 雷达里程计填充协方差并转发。

订阅 /odom_laser，仅替换 pose.covariance 和 twist.covariance，
其余字段原样发布到 /odom_laser_fused。本节点不发布 TF。
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


INPUT_TOPIC = '/odom_laser'
OUTPUT_TOPIC = '/odom_laser_fused'
IMU_TOPIC = '/imu_compensated'   # 陀螺角速度参考（已由 imu_bias_relay 做零偏补偿）

POSE_COVARIANCE = [0.0] * 36
POSE_COVARIANCE[0] = 1e-3   # x
POSE_COVARIANCE[7] = 1e-3   # y
POSE_COVARIANCE[35] = 1e-2  # yaw

TWIST_COVARIANCE = [0.0] * 36
TWIST_COVARIANCE[0] = 1e-2   # vx
TWIST_COVARIANCE[35] = 1e6   # vyaw（EKF 不融合，仅明确标记为高不确定性）

# 冻结期（判定为静止）对外发布的 yaw 协方差：置为极大值，让 EKF 在静止期
# 完全忽略 odom1 的 yaw。
# 依据（2026-09-26 rosbag 离线分析）：车停住后陀螺 wz ≈ 0，odom0 的 yaw 已关，
# 此时 odom1 是唯一还能推动 EKF yaw 的输入；而 rf2o 在停车瞬间会灌入约
# +10.6° 的静止假漂移，EKF 在静置 2 s 内照单全收 → δ ≈ +4.6°。
# 切断这条通路后，静止期 EKF 的 yaw 不再被 rf2o 拉走。
STATIC_YAW_COVARIANCE = 1e6

# --- 静止冻结参数 ---
# rf2o 静止时 twist.angular.z 实测约 -0.032 ~ +0.028 rad/s（均值 ≈0.001），
# 自转指令 0.8 rad/s 远高于阈值，故 0.02 rad/s 能干净区分「静止 / 运动」。
STATIC_WZ_THRESHOLD = 0.02   # rad/s：低于此值视为本帧静止
STATIC_FRAMES_REQUIRED = 3   # 连续 3 帧低于阈值才进入静止（滞回，防噪声误触发）

# --- rf2o 失败帧（pose 卡死）检测与陀螺外推 ---
# rf2o 在 Eigensolver 解算失败时不更新位姿，实测占 31%~34% 的帧，且与 rf2o
# 日志中的失败记录 98% 吻合。其输出因此呈阶梯状：成功帧的增量覆盖「距上次成功」
# 的整段时间，阶梯与真值直线的偏差 RMS ≈ 4.16°（实测抖动）。
# 这里用陀螺角速度补出卡死帧的 yaw，把阶梯填平。
# 判据用陀螺 |wz| 而非 rf2o 自己的 twist：后者是 4 次滑动平均且失败时不更新，
# 车刚停住时会残留在停车前的数值上，有误判风险（实测静止段误命中 0 次）。
STUCK_EPS = 1e-6            # pose 增量小于此值即认为「未更新」（m / rad）
MOVING_WZ_THRESHOLD = 0.05  # rad/s：陀螺角速度超过此值认为车在转动
MAX_EXTRAP_FRAMES = 2       # 连续外推上限，超过则退回原逻辑，防止异常时长时间外推
DT_MAX_S = 0.5              # dt 上限；stamp 差值异常时回退到本地单调时钟


def normalize_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_euler(x, y, z, w):
    """四元数 -> (roll, pitch, yaw)，ZYX 顺序。"""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """(roll, pitch, yaw) -> 四元数，ZYX 顺序。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OdomLaserCovRelay(Node):
    def __init__(self):
        super().__init__('odom_laser_cov_relay')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._publisher = self.create_publisher(Odometry, OUTPUT_TOPIC, qos)
        self._subscription = self.create_subscription(
            Odometry,
            INPUT_TOPIC,
            self._odom_callback,
            qos,
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self._imu_callback,
            qos,
        )

        # 失败帧检测 / 陀螺外推状态
        self._imu_wz = 0.0          # 最新陀螺角速度（rad/s）
        self._prev_x = None         # 上一帧的 pose（原始值，用于卡死判定）
        self._prev_y = None
        self._prev_yaw_raw = None
        self._prev_yaw_out = None   # 上一帧实际输出的 yaw（外推基准）
        self._prev_stamp = None
        self._prev_mono = None
        self._extrap_streak = 0     # 当前连续外推帧数
        self._extrapolated = False  # 本帧是否走了外推
        self._extrap_total = 0      # 累计外推帧数（仅日志用）

        # 静止冻结状态
        self._static_streak = 0     # 连续低于阈值的帧数
        self._is_static = False     # 当前是否处于冻结
        self._frozen_yaw = 0.0      # 冻结时锁定的 yaw
        self._yaw_offset = 0.0      # 冻结期间被丢弃的假漂移，退出时折算为常值偏移
        self._frozen_frames = 0     # 本次冻结已持续的帧数（仅日志用）

        self.get_logger().info(
            f'Relaying {INPUT_TOPIC} to {OUTPUT_TOPIC} with configured covariance; '
            f'yaw freeze: |wz| < {STATIC_WZ_THRESHOLD} rad/s for '
            f'{STATIC_FRAMES_REQUIRED} frames; '
            f'stuck-frame extrapolation: |wz_imu| > {MOVING_WZ_THRESHOLD} rad/s, '
            f'max {MAX_EXTRAP_FRAMES} frames'
        )

    def _imu_callback(self, msg: Imu) -> None:
        """只取陀螺 z 轴角速度，作为外推用的角速度参考。"""
        self._imu_wz = msg.angular_velocity.z

    def _odom_callback(self, msg: Odometry) -> None:
        msg.pose.covariance = POSE_COVARIANCE.copy()
        msg.twist.covariance = TWIST_COVARIANCE.copy()

        wz = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        roll, pitch, yaw_raw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        # ---- 失败帧（pose 卡死）检测 ----
        # 本帧时间戳优先用 header.stamp；异常（首帧 / 跳变 / 超量程）时回退本地时钟
        now_mono = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None
        if self._prev_stamp is not None:
            dt = stamp - self._prev_stamp
        if dt is None or not (0.0 < dt < DT_MAX_S):
            dt = (now_mono - self._prev_mono) if self._prev_mono is not None else 0.0

        self._extrapolated = False
        if self._prev_yaw_raw is not None:
            dx = msg.pose.pose.position.x - self._prev_x
            dy = msg.pose.pose.position.y - self._prev_y
            dyaw = normalize_angle(yaw_raw - self._prev_yaw_raw)
            pose_stuck = (abs(dx) < STUCK_EPS and abs(dy) < STUCK_EPS
                          and abs(dyaw) < STUCK_EPS)
            if not pose_stuck:
                self._extrap_streak = 0
            elif (abs(self._imu_wz) > MOVING_WZ_THRESHOLD
                    and not self._is_static
                    and self._extrap_streak < MAX_EXTRAP_FRAMES
                    and dt > 0.0):
                # 卡死且车在转：本帧 yaw 由陀螺外推补出（位置不外推）
                self._extrapolated = True
                self._extrap_streak += 1
                self._extrap_total += 1
                if self._extrap_total % 100 == 0:
                    self.get_logger().info(
                        f'Extrapolated {self._extrap_total} stuck frames '
                        f'(latest dt={dt * 1000:.1f} ms, '
                        f'wz_imu={self._imu_wz:+.4f} rad/s)')

        if abs(wz) < STATIC_WZ_THRESHOLD:
            self._static_streak += 1
        else:
            self._static_streak = 0
            if self._is_static:
                # 退出冻结：冻结期间 rf2o 内部仍在累积假漂移，这里把差额一次性
                # 折算成常值偏移，使输出 yaw 在退出瞬间连续（否则会跳变数十度，
                # 反而把假漂移以「一帧大增量」的形式灌进 EKF）。
                self._yaw_offset = normalize_angle(self._frozen_yaw - yaw_raw)
                self._is_static = False
                self.get_logger().info(
                    f'Yaw unfreeze after {self._frozen_frames} frames, '
                    f'offset {math.degrees(self._yaw_offset):+.3f} deg'
                )
                self._frozen_frames = 0

        if not self._is_static and self._static_streak >= STATIC_FRAMES_REQUIRED:
            # 进入冻结：锁定当前输出 yaw，此后不再跟随 rf2o 的漂移
            self._is_static = True
            self._frozen_frames = 0
            self._frozen_yaw = normalize_angle(yaw_raw + self._yaw_offset)
            self.get_logger().info(
                f'Yaw freeze engaged at {math.degrees(self._frozen_yaw):+.3f} deg'
            )

        if self._is_static:
            self._frozen_frames += 1
            yaw_out = self._frozen_yaw
        else:
            yaw_out = yaw_raw + self._yaw_offset

        if self._extrapolated and not self._is_static and self._prev_yaw_out is not None:
            # 填平阶梯：以上一帧输出 yaw 为基准，按陀螺角速度推一帧
            yaw_out = self._prev_yaw_out + self._imu_wz * dt

        if self._is_static:
            # 冻结期：yaw 协方差抬到极大值，EKF 在静止期不再采纳 odom1 的 yaw。
            # 退出冻结后自动恢复正常值（回调开头已按 POSE_COVARIANCE 填充）。
            cov = POSE_COVARIANCE.copy()
            cov[35] = STATIC_YAW_COVARIANCE
            msg.pose.covariance = cov

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw_out)
        q.x, q.y, q.z, q.w = qx, qy, qz, qw

        # 记录本帧状态，供下一帧的卡死判定与外推基准使用
        self._prev_x = msg.pose.pose.position.x
        self._prev_y = msg.pose.pose.position.y
        self._prev_yaw_raw = yaw_raw
        self._prev_yaw_out = yaw_out
        self._prev_stamp = stamp
        self._prev_mono = now_mono

        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomLaserCovRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
