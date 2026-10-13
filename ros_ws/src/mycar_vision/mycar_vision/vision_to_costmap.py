"""``vision_to_costmap`` -- extension point #3: Nav2 plug-in adapter.

Converts ``/vision/obstacles`` (``vision_msgs/Detection3DArray``) into formats
that Nav2's ``obstacle_layer`` already understands:

* ``sensor_msgs/LaserScan`` on ``/vision/obstacles_scan``
* ``sensor_msgs/PointCloud2`` on ``/vision/obstacles_cloud``

Both are expressed in ``output_frame`` (default: whatever the obstacle array
says, falling back to ``base_link``).  Nav2 needs them in the global frame, so
either set ``output_frame`` to ``odom``/``map`` *and* feed a matching TF, or add
this topic to the local costmap's observation sources.

Nothing is written to the costmap directly -- the costmap stays owned by Nav2.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from vision_msgs.msg import Detection3DArray


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def xyz_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Pack an ``(N, 3)`` float array into a dense ``PointCloud2``."""
    points = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class VisionToCostmap(Node):
    """Publish vision obstacles as LaserScan + PointCloud2 for Nav2."""

    def __init__(self) -> None:
        super().__init__("vision_to_costmap")

        self.declare_parameter("obstacles_topic", "/vision/obstacles")
        self.declare_parameter("scan_topic", "/vision/obstacles_scan")
        self.declare_parameter("cloud_topic", "/vision/obstacles_cloud")
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_cloud", True)
        self.declare_parameter("output_frame", "")  # empty -> use the input frame
        self.declare_parameter("scan_bins", 720)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("target_classes", ["person", "bottle", "blue_block", "green_block"])
        self.declare_parameter("log_interval", 10.0)

        gp = self.get_parameter
        self.obstacles_topic = gp("obstacles_topic").get_parameter_value().string_value
        self.scan_topic = gp("scan_topic").get_parameter_value().string_value
        self.cloud_topic = gp("cloud_topic").get_parameter_value().string_value
        self.publish_scan = gp("publish_scan").get_parameter_value().bool_value
        self.publish_cloud = gp("publish_cloud").get_parameter_value().bool_value
        self.output_frame = gp("output_frame").get_parameter_value().string_value
        self.scan_bins = max(90, int(gp("scan_bins").get_parameter_value().integer_value))
        self.range_min = gp("range_min").get_parameter_value().double_value
        self.range_max = gp("range_max").get_parameter_value().double_value
        self.min_score = gp("min_score").get_parameter_value().double_value
        self.target_classes = list(
            gp("target_classes").get_parameter_value().string_array_value
        )
        self.log_interval = gp("log_interval").get_parameter_value().double_value

        self._scan_pub = (
            self.create_publisher(LaserScan, self.scan_topic, _sensor_qos())
            if self.publish_scan and self.scan_topic
            else None
        )
        self._cloud_pub = (
            self.create_publisher(PointCloud2, self.cloud_topic, _sensor_qos())
            if self.publish_cloud and self.cloud_topic
            else None
        )
        self.create_subscription(
            Detection3DArray, self.obstacles_topic, self._obstacles_callback, 10
        )

        self._log_counter = 0
        self.get_logger().info(
            f"vision_to_costmap ready | in={self.obstacles_topic} "
            f"scan={self.scan_topic if self._scan_pub else 'off'} "
            f"cloud={self.cloud_topic if self._cloud_pub else 'off'}"
        )

    # ------------------------------------------------------------------
    def _obstacles_callback(self, msg: Detection3DArray) -> None:
        frame_id = self.output_frame or msg.header.frame_id or "base_link"

        scan_points: List[Tuple[float, float]] = []
        cloud_points: List[Tuple[float, float, float]] = []

        for det in msg.detections:
            label = det.results[0].hypothesis.class_id if det.results else ""
            score = float(det.results[0].hypothesis.score) if det.results else 0.0
            if self.target_classes and label not in self.target_classes:
                continue
            if score < self.min_score:
                continue

            x = float(det.bbox.center.position.x)
            y = float(det.bbox.center.position.y)
            height = max(0.02, float(det.bbox.size.z))
            radius = math.hypot(x, y)
            if radius < self.range_min or radius > self.range_max:
                continue

            angle = math.atan2(y, x)
            half_width = max(0.05, 0.5 * float(det.bbox.size.x))
            spread = math.atan2(half_width, max(radius, self.range_min))
            for offset in (-spread, -0.5 * spread, 0.0, 0.5 * spread, spread):
                scan_points.append((angle + offset, radius))
                cloud_points.append(
                    (radius * math.cos(angle + offset), radius * math.sin(angle + offset), 0.0)
                )
                cloud_points.append(
                    (
                        radius * math.cos(angle + offset),
                        radius * math.sin(angle + offset),
                        min(height, 0.4),
                    )
                )

        if self._scan_pub is not None:
            self._scan_pub.publish(self._build_scan(scan_points, frame_id, msg.header.stamp))
        if self._cloud_pub is not None:
            points = np.asarray(cloud_points, dtype=np.float32) if cloud_points else np.zeros((0, 3), np.float32)
            self._cloud_pub.publish(xyz_to_pointcloud2(points, frame_id, msg.header.stamp))

        self._log_counter += 1
        if self.log_interval > 0.0 and self._log_counter % max(1, int(self.log_interval)) == 0:
            self.get_logger().info(
                f"{len(msg.detections)} detections -> {len(scan_points)} scan beams, "
                f"{len(cloud_points)} cloud points"
            )

    def _build_scan(self, scan_points, frame_id: str, stamp) -> LaserScan:
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = frame_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.scan_bins
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)

        ranges = np.full(self.scan_bins, np.inf, dtype=np.float32)
        for angle, radius in scan_points:
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            index = int(round((wrapped - scan.angle_min) / scan.angle_increment))
            if index < 0:
                index += self.scan_bins
            if index >= self.scan_bins:
                index -= self.scan_bins
            if radius < ranges[index]:
                ranges[index] = radius
        scan.ranges = [float(v) for v in ranges]
        return scan


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VisionToCostmap()
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
