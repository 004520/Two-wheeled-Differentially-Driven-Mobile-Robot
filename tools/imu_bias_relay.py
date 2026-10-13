#!/usr/bin/env python3
"""IMU 陀螺 z 轴零偏补偿 relay。

订阅 /imu，节点启动后静止采样 BIAS_SAMPLE_SECONDS 秒，取
angular_velocity.z 的平均值作为零偏；此后把每帧的 angular_velocity.z
减去零偏后发布到 /imu_compensated。

其余字段（orientation、angular_velocity.x/y、linear_acceleration、
covariance、header）全部原样透传。本节点不发布 TF。

背景：实测 IMU z 轴零偏 ≈ +0.018 rad/s（三次静止采样 0.0179/0.0183/0.0184），
A6 脚本自采样补偿了它，但 EKF 直接消费原始 /imu，整圈自转（7~9 s）会被零偏
推高 +7~9°，是比 rf2o 更大的航向误差源。

采样窗口从「首帧到达本节点」开始计时，而不是从节点启动时刻，避免节点比
micro-ROS 先起来时把采样窗口耗在等待上。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


INPUT_TOPIC = '/imu'
OUTPUT_TOPIC = '/imu_compensated'

BIAS_SAMPLE_SECONDS = 3.0   # 静止采样时长（s）
BIAS_MIN_SAMPLES = 20       # 有效样本下限；IMU ~40 Hz，3 s 约 120 帧
BIAS_STD_WARN = 0.02        # 采样期 wz 标准差超过此值，提示采样时车可能在动
QOS_DEPTH = 50              # IMU ~40 Hz，队列放宽以减少 BEST_EFFORT 丢帧


class ImuBiasRelay(Node):
    def __init__(self):
        super().__init__('imu_bias_relay')

        # /imu 发布侧为 best effort，订阅必须匹配；depth 放宽防丢帧
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
        )

        self._publisher = self.create_publisher(Imu, OUTPUT_TOPIC, qos)
        self._subscription = self.create_subscription(
            Imu, INPUT_TOPIC, self._imu_callback, qos)

        # ---- 零偏采样状态 ----
        self._bias = 0.0            # 待补偿的 wz 零偏（rad/s）
        self._calibrating = True    # 采样阶段：只收集，不发布
        self._sample_t0 = None      # 首帧到达时刻（monotonic）
        self._samples = []          # 采样阶段的 wz 样本

        self.get_logger().info(
            f'Subscribing {INPUT_TOPIC}, publishing {OUTPUT_TOPIC}; '
            f'static sampling {BIAS_SAMPLE_SECONDS:.1f} s for gyro-z bias, '
            f'no output until sampling done'
        )

    def _imu_callback(self, msg: Imu) -> None:
        now = time.monotonic()

        if self._calibrating:
            if self._sample_t0 is None:
                self._sample_t0 = now
            self._samples.append(msg.angular_velocity.z)

            if now - self._sample_t0 < BIAS_SAMPLE_SECONDS:
                return  # 采样未满，本帧不发布

            # 采样结束：算零偏并打印
            self._calibrating = False
            samples = self._samples
            self._samples = []

            n = len(samples)
            mean = sum(samples) / n
            self._bias = mean

            if n >= 2:
                var = sum((v - mean) ** 2 for v in samples) / (n - 1)
                std = math.sqrt(var)
            else:
                std = 0.0

            self.get_logger().info(
                f'IMU z 轴零偏 = {mean:+.4f} rad/s '
                f'({n} samples / {now - self._sample_t0:.2f} s, std {std:.4f})'
            )
            if n < BIAS_MIN_SAMPLES:
                self.get_logger().warn(
                    f'样本数 {n} < {BIAS_MIN_SAMPLES}，零偏估计可能不可靠')
            if std > BIAS_STD_WARN:
                self.get_logger().warn(
                    f'采样期 wz 标准差 {std:.4f} rad/s > {BIAS_STD_WARN}，'
                    f'采样时车可能在动，零偏估计可能不准')

        # 补偿 wz，其余字段原样透传（采样结束的那一帧也按补偿后发出，少丢一帧）
        msg.angular_velocity.z -= self._bias
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasRelay()
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
