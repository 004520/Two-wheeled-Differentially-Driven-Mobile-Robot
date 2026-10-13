"""``fake_camera_publisher`` -- bench-test camera for ``vision_detector``.

Useful when the real car (and therefore ``mycar_camera``) is offline.

Two modes:

* **image mode** -- cycles through a list of image files::

      ros2 run mycar_vision fake_camera_publisher --ros-args \
        -p images:="['/path/bus.jpg']"

* **synthetic mode** (default, no ``images``) -- draws a scene with one blue and
  one green block so the classical colour path can be exercised deterministically.

It publishes the exact same topic/type/QoS as ``mycar_camera``
(``/camera/image_raw/compressed``, ``CompressedImage``, BEST_EFFORT, depth 1).
"""

from __future__ import annotations

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

SYNTHETIC_SCENE = [
    # (label, colour BGR, x, y, w, h) -- all in a 640x480 canvas
    ("blue_block", (230, 120, 30), 60, 300, 90, 90),
    ("green_block", (60, 200, 60), 200, 320, 100, 100),
    ("blue_block", (230, 120, 30), 430, 250, 70, 70),
]


def _video_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class FakeCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_camera_publisher")

        self.declare_parameter("topic", "/camera/image_raw/compressed")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        # NB: an empty Python list would be declared as BYTE_ARRAY by rclpy,
        # so the string-array default needs at least one (empty) element.
        self.declare_parameter("images", [""])
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("loop", True)

        gp = self.get_parameter
        self.topic = gp("topic").get_parameter_value().string_value
        self.frame_id = gp("frame_id").get_parameter_value().string_value
        self.fps = max(0.1, gp("fps").get_parameter_value().double_value)
        self.width = max(64, gp("width").get_parameter_value().integer_value)
        self.height = max(64, gp("height").get_parameter_value().integer_value)
        self.images = [p for p in gp("images").get_parameter_value().string_array_value if p]
        self.jpeg_quality = max(1, min(100, gp("jpeg_quality").get_parameter_value().integer_value))
        self.loop = gp("loop").get_parameter_value().bool_value

        self._loaded = []
        for path in self.images:
            expanded = os.path.expanduser(path)
            frame = cv2.imread(expanded)
            if frame is None:
                self.get_logger().warn(f"Cannot read image: {expanded}")
                continue
            self._loaded.append(cv2.resize(frame, (self.width, self.height)))
        if self.images and not self._loaded:
            self.get_logger().warn("No usable image; falling back to synthetic mode")

        self._synthetic = self._render_synthetic()
        self._index = 0
        self._count = 0
        self._pub = self.create_publisher(CompressedImage, self.topic, _video_qos())
        self.create_timer(1.0 / self.fps, self._tick)

        mode = f"{len(self._loaded)} image(s)" if self._loaded else "synthetic scene"
        self.get_logger().info(
            f"fake_camera_publisher -> {self.topic} @ {self.fps:.1f} fps "
            f"({self.width}x{self.height}, {mode})"
        )

    def _render_synthetic(self) -> np.ndarray:
        canvas = np.full((self.height, self.width, 3), 96, dtype=np.uint8)
        canvas[: self.height // 3] = (150, 150, 150)  # "wall"
        sx = self.width / 640.0
        sy = self.height / 480.0
        for _, color, x, y, w, h in SYNTHETIC_SCENE:
            p1 = (int(x * sx), int(y * sy))
            p2 = (int((x + w) * sx), int((y + h) * sy))
            cv2.rectangle(canvas, p1, p2, color, -1)
            cv2.rectangle(canvas, p1, p2, (255, 255, 255), 2)
        cv2.putText(
            canvas,
            "SYNTHETIC CAMERA",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def _next_frame(self) -> np.ndarray:
        if not self._loaded:
            return self._synthetic
        frame = self._loaded[self._index]
        self._index += 1
        if self._index >= len(self._loaded):
            self._index = 0 if self.loop else len(self._loaded) - 1
        return frame

    def _tick(self) -> None:
        frame = self._next_frame()
        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self._pub.publish(msg)

        self._count += 1
        if self._count % max(1, int(self.fps * 10)) == 0:
            self.get_logger().info(f"published {self._count} frames")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FakeCameraPublisher()
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
