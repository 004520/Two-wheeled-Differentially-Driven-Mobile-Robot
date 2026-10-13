"""``avoidance_decider`` -- extension point #2: turn obstacles into a velocity.

Consumes ``/vision/obstacles`` (``vision_msgs/Detection3DArray``, robot frame)
and produces a *proposal* on ``/avoidance_cmd`` (``geometry_msgs/Twist``).

Safety contract
---------------
* By default this node **never** publishes ``/cmd_vel``.  It only fills
  ``/avoidance_cmd`` so that a higher-level arbiter (teleop mux, Nav2
  ``velocity_smoother``, a custom selector, ...) can decide.
* Publishing directly to ``/cmd_vel`` requires ``enable_cmd_vel=true`` *and* an
  explicit ``cmd_vel_topic``.  Never enable both this node and a Nav2
  controller on the same topic.

A "keep-out" corridor is used: obstacles inside ``fov_deg`` and closer than
``slow_distance`` slow the robot down, obstacles closer than ``stop_distance``
stop it and steer away from the closest one.

Two guards keep that decision honest:

* ``min_score`` -- detections whose ``hypothesis.score`` is below this value are
  ignored, so a low-confidence false positive cannot trigger a stop.  ``<= 0``
  disables the check.
* ``stale_timeout`` -- if no ``/vision/obstacles`` message arrives for this long
  the node publishes a single zero ``Twist`` so a stop/turn command cannot stay
  stuck on the topic after the detector dies.  ``<= 0`` disables the check.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray


class AvoidanceDecider(Node):
    """Very small reactive avoider, deliberately easy to replace."""

    def __init__(self) -> None:
        super().__init__("avoidance_decider")

        self.declare_parameter("obstacles_topic", "/vision/obstacles")
        self.declare_parameter("avoidance_topic", "/avoidance_cmd")
        self.declare_parameter("enable_cmd_vel", False)
        self.declare_parameter("cmd_vel_topic", "")

        self.declare_parameter("fov_deg", 70.0)
        self.declare_parameter("min_range", 0.10)
        self.declare_parameter("slow_distance", 0.80)
        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("max_linear", 0.15)
        self.declare_parameter("max_angular", 0.80)
        self.declare_parameter("turn_gain", 1.2)

        self.declare_parameter("target_classes", ["person", "bottle", "blue_block", "green_block"])
        self.declare_parameter("publish_when_clear", False)
        self.declare_parameter("min_score", 0.4)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("log_interval", 10.0)

        gp = self.get_parameter
        self.obstacles_topic = gp("obstacles_topic").get_parameter_value().string_value
        self.avoidance_topic = gp("avoidance_topic").get_parameter_value().string_value
        self.enable_cmd_vel = gp("enable_cmd_vel").get_parameter_value().bool_value
        self.cmd_vel_topic = gp("cmd_vel_topic").get_parameter_value().string_value

        self.fov = math.radians(gp("fov_deg").get_parameter_value().double_value)
        self.min_range = gp("min_range").get_parameter_value().double_value
        self.slow_distance = gp("slow_distance").get_parameter_value().double_value
        self.stop_distance = gp("stop_distance").get_parameter_value().double_value
        self.max_linear = gp("max_linear").get_parameter_value().double_value
        self.max_angular = gp("max_angular").get_parameter_value().double_value
        self.turn_gain = gp("turn_gain").get_parameter_value().double_value

        self.target_classes = list(
            gp("target_classes").get_parameter_value().string_array_value
        )
        self.publish_when_clear = (
            gp("publish_when_clear").get_parameter_value().bool_value
        )
        self.min_score = gp("min_score").get_parameter_value().double_value
        self.stale_timeout = gp("stale_timeout").get_parameter_value().double_value
        self.log_interval = gp("log_interval").get_parameter_value().double_value

        self._avoidance_pub = self.create_publisher(Twist, self.avoidance_topic, 10)
        self._cmd_vel_pub = None
        if self.enable_cmd_vel:
            topic = self.cmd_vel_topic or "/cmd_vel"
            self._cmd_vel_pub = self.create_publisher(Twist, topic, 10)
            self.get_logger().warn(
                f"enable_cmd_vel=true -> publishing directly to {topic}. "
                "Make sure no other controller owns this topic."
            )

        self.create_subscription(
            Detection3DArray, self.obstacles_topic, self._obstacles_callback, 10
        )

        # Liveness: ``None`` == "nothing received yet".  A timer watches for the
        # obstacle stream going quiet and emits a single zero Twist so a stale
        # stop/turn command cannot stick on the topic.
        self._last_seen: float | None = None
        self._stale = False
        self._stale_timer = None
        if self.stale_timeout > 0.0:
            self._stale_timer = self.create_timer(
                max(0.05, 0.5 * self.stale_timeout), self._stale_check
            )

        self._last_log_time = time.monotonic()
        self.get_logger().info(
            f"avoidance_decider ready | in={self.obstacles_topic} "
            f"out={self.avoidance_topic} | stop={self.stop_distance} "
            f"slow={self.slow_distance} fov={math.degrees(self.fov):.0f}deg | "
            f"min_score={self.min_score} stale_timeout={self.stale_timeout}s | "
            f"cmd_vel={'on' if self._cmd_vel_pub else 'off'}"
        )

    # ------------------------------------------------------------------
    def _obstacles_callback(self, msg: Detection3DArray) -> None:
        now = time.monotonic()
        if self._stale:
            self._stale = False
            self.get_logger().info(
                f"{self.obstacles_topic} recovered after stale window"
            )
        self._last_seen = now

        closest = None
        for det in msg.detections:
            label = det.results[0].hypothesis.class_id if det.results else ""
            score = float(det.results[0].hypothesis.score) if det.results else 0.0
            if self.target_classes and label not in self.target_classes:
                continue
            if self.min_score > 0.0 and score < self.min_score:
                continue

            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            distance = math.hypot(x, y)
            if distance < self.min_range:
                continue
            if abs(math.atan2(y, x)) > self.fov:
                continue
            if closest is None or distance < closest[0]:
                closest = (distance, y, label)

        if closest is None:
            if self.publish_when_clear:
                self._publish(0.0, 0.0, "clear")
            return

        distance, lateral, label = closest
        if distance <= self.stop_distance:
            # Stop and turn away from the shortest side.
            turn = -1.0 if lateral > 0.0 else 1.0
            self._publish(0.0, turn * self.max_angular, f"STOP {label} {distance:.2f}m")
        elif distance <= self.slow_distance:
            span = max(1e-3, self.slow_distance - self.stop_distance)
            ratio = (distance - self.stop_distance) / span  # 0 .. 1
            linear = self.max_linear * ratio
            turn = self._lateral_to_turn(lateral, distance)
            self._publish(linear, turn, f"SLOW {label} {distance:.2f}m")
        else:
            if self.publish_when_clear:
                self._publish(0.0, 0.0, "clear")

    def _stale_check(self) -> None:
        """Emit a single zero Twist once the obstacle stream has gone quiet.

        Without this the last non-zero command would sit on
        ``/avoidance_cmd`` forever when the detector dies, so a mux that holds
        the last value would keep steering/stopping against a phantom obstacle.
        """
        if self._last_seen is None or self._stale:
            return
        age = time.monotonic() - self._last_seen
        if age <= self.stale_timeout:
            return
        self._stale = True
        self.get_logger().warn(
            f"no {self.obstacles_topic} for {age:.2f}s "
            f"(> stale_timeout={self.stale_timeout:.2f}s) -> clearing command"
        )
        self._publish(0.0, 0.0, f"STALE {age:.2f}s")

    def _lateral_to_turn(self, lateral: float, distance: float) -> float:
        """Steer away from an obstacle offset ``lateral`` metres to the left."""
        offset = max(-1.0, min(1.0, lateral / max(distance, 1e-3)))
        turn = -offset * self.turn_gain
        return max(-self.max_angular, min(self.max_angular, turn))

    def _publish(self, linear_x: float, angular_z: float, reason: str) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self._avoidance_pub.publish(twist)
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(twist)

        now = time.monotonic()
        if self.log_interval > 0.0 and now - self._last_log_time >= self.log_interval:
            self._last_log_time = now
            self.get_logger().info(
                f"{reason} -> linear.x={linear_x:.3f} angular.z={angular_z:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AvoidanceDecider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
