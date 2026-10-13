"""``vision_radar_fusion`` -- extension point #1: distance estimation + fusion.

Consumes ``/vision/detections`` (``Detection2DArray``) and ``/scan``
(``LaserScan``), attaches a metric range to every detection and publishes
``vision_msgs/Detection3DArray`` on ``/vision/obstacles``.

Range sources, in priority order
--------------------------------
1. **radar / lidar** -- the closest valid ``/scan`` return inside the bearing
   window spanned by the 2D box (default).
2. **monocular fallback** -- ``Z = fy * H_real / h_pixel`` using the per-class
   ``object_heights`` table (only when ``enable_mono_depth`` is true).

This node is intentionally stateless w.r.t. motion: it assumes the camera looks
forward and is roughly aligned with ``base_link`` unless ``use_tf`` is true, in
which case ``camera_optical_frame -> output_frame`` is resolved with tf2.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox3D
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection3D
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from .detection_utils import DEFAULT_REAL_HEIGHTS
from .detection_utils import MonoCameraModel

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None  # type: ignore[assignment]


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


#: A monocular depth is refused when the 2D box touches a frame border by less
#: than this many pixels.  A box glued to the top (or bottom) edge means the
#: object sticks out of the image, so ``h_pixel`` is truncated and
#: ``Z = fy * H / h`` over-estimates the range -- a person 1 m away reads as
#: ~3 m once the head is cut off.
_MONO_CLIP_MARGIN_PX = 1.0


class VisionRadarFusion(Node):
    """Attach a range to every vision detection and re-publish as 3D."""

    def __init__(self) -> None:
        super().__init__("vision_radar_fusion")

        self.declare_parameter("detection_topic", "/vision/detections")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacles_topic", "/vision/obstacles")

        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("use_tf", False)
        self.declare_parameter("camera_optical_frame", "camera_link")
        self.declare_parameter("tf_timeout", 0.05)

        # pinhole model, defaults for the 640x480 / ~60 deg stream
        self.declare_parameter("fx", 554.3)
        self.declare_parameter("fy", 554.3)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        self.declare_parameter("enable_mono_depth", True)
        self.declare_parameter("mono_depth_classes", ["person", "bottle"])
        self.declare_parameter("publish_without_range", False)

        self.declare_parameter("scan_min_range", 0.05)
        self.declare_parameter("scan_max_range", 8.0)
        self.declare_parameter("bearing_window_margin_deg", 2.0)
        # Default matches config/vision_radar_fusion.yaml: "min" uses argmin's
        # own bin angle, which lands on the object's right edge for a rigid
        # target and biases the bearing by -half its angular width.
        self.declare_parameter("range_statistic", "percentile")  # "min" | "percentile"
        self.declare_parameter("range_percentile", 20.0)

        # Monocular fallback hardening.  YOLO frequently locks onto a *partial*
        # box (cap + label band) of a tall thin object, so ``size_y`` is too small
        # and ``Z = fy * H / h`` over-estimates the range (a 0.70 m bottle read as
        # ~1.18 m).  Taking the **max** unclipped pixel height seen over the last
        # few frames is the same as "use the frame where the whole object was
        # framed", which collapses that far peak.
        self.declare_parameter("mono_use_max_box", True)
        self.declare_parameter("mono_max_box_window_s", 1.5)

        self.declare_parameter("target_classes", ["person", "bottle", "blue_block", "green_block"])
        self.declare_parameter(
            "object_heights",
            [f"{k}:{v}" for k, v in DEFAULT_REAL_HEIGHTS.items()],
        )
        self.declare_parameter("log_interval", 10.0)
        self.declare_parameter("stats_interval", 1.0)
        self.declare_parameter("fusion_stats_topic", "/vision/fusion_stats")

        gp = self.get_parameter
        self.detection_topic = gp("detection_topic").get_parameter_value().string_value
        self.scan_topic = gp("scan_topic").get_parameter_value().string_value
        self.obstacles_topic = gp("obstacles_topic").get_parameter_value().string_value
        self.output_frame = gp("output_frame").get_parameter_value().string_value
        self.use_tf = gp("use_tf").get_parameter_value().bool_value
        self.camera_optical_frame = (
            gp("camera_optical_frame").get_parameter_value().string_value
        )
        self.tf_timeout = gp("tf_timeout").get_parameter_value().double_value

        self.camera = MonoCameraModel(
            fx=gp("fx").get_parameter_value().double_value,
            fy=gp("fy").get_parameter_value().double_value,
            cx=gp("cx").get_parameter_value().double_value,
            cy=gp("cy").get_parameter_value().double_value,
        )
        self.enable_mono_depth = gp("enable_mono_depth").get_parameter_value().bool_value
        self.mono_depth_classes = list(
            gp("mono_depth_classes").get_parameter_value().string_array_value
        )
        self.publish_without_range = (
            gp("publish_without_range").get_parameter_value().bool_value
        )
        self.scan_min_range = gp("scan_min_range").get_parameter_value().double_value
        self.scan_max_range = gp("scan_max_range").get_parameter_value().double_value
        self.bearing_margin = math.radians(
            gp("bearing_window_margin_deg").get_parameter_value().double_value
        )
        self.range_statistic = gp("range_statistic").get_parameter_value().string_value
        self.range_percentile = (
            gp("range_percentile").get_parameter_value().double_value
        )
        self.mono_use_max_box = (
            gp("mono_use_max_box").get_parameter_value().bool_value
        )
        self.mono_max_box_window_s = (
            gp("mono_max_box_window_s").get_parameter_value().double_value
        )
        self.target_classes = list(
            gp("target_classes").get_parameter_value().string_array_value
        )
        self.object_heights = self._parse_heights(
            list(gp("object_heights").get_parameter_value().string_array_value)
        )
        self.log_interval = gp("log_interval").get_parameter_value().double_value
        self.stats_interval = gp("stats_interval").get_parameter_value().double_value
        self.fusion_stats_topic = (
            gp("fusion_stats_topic").get_parameter_value().string_value
        )

        # -- state ---------------------------------------------------------
        self._latest_scan: Optional[LaserScan] = None
        self._angles: Optional[np.ndarray] = None
        self._ranges: Optional[np.ndarray] = None
        #: geometry of the cached ``_angles`` so a driver that changes its
        #: angular layout with a constant beam count still invalidates it
        self._scan_angle_min: Optional[float] = None
        self._scan_angle_increment: Optional[float] = None

        self._tf_buffer = None
        self._tf_listener = None
        if self.use_tf:
            if tf2_ros is None:
                self.get_logger().warn("use_tf=true but tf2_ros is unavailable; disabled")
                self.use_tf = False
            else:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._stats = {
            "frames": 0,
            "detections": 0,
            "radar": 0,
            "mono": 0,
            "mono_clipped": 0,
            "dropped": 0,
            "published": 0,
        }
        #: snapshot of :attr:`_stats` so the log line can stay per-interval even
        #: though the counters themselves are cumulative
        self._stats_logged = dict(self._stats)
        #: per-class ``(timestamp, max_unclipped_pixel_height)`` used by the
        #: "multi-box max" monocular fallback; expires after
        #: ``mono_max_box_window_s``.
        self._mono_best: Dict[str, Tuple[float, float]] = {}
        self._last_log_time = time.monotonic()

        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, _sensor_qos()
        )
        self.create_subscription(
            Detection2DArray, self.detection_topic, self._detections_callback, 10
        )
        self._obstacle_pub = self.create_publisher(
            Detection3DArray, self.obstacles_topic, 10
        )
        self._stats_pub = self.create_publisher(String, self.fusion_stats_topic, 10)
        if self.stats_interval > 0.0:
            self.create_timer(self.stats_interval, self._publish_stats)

        self.get_logger().info(
            f"vision_radar_fusion ready | {self.detection_topic} + {self.scan_topic} "
            f"-> {self.obstacles_topic} | frame={self.output_frame} "
            f"mono_depth={self.enable_mono_depth} ({self.mono_depth_classes}) "
            f"tf={self.use_tf} | range_statistic={self.range_statistic} "
            f"stats={self.fusion_stats_topic}@{self.stats_interval:.1f}s"
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _parse_heights(entries: List[str]) -> Dict[str, float]:
        heights: Dict[str, float] = dict(DEFAULT_REAL_HEIGHTS)
        for entry in entries:
            if ":" not in entry:
                continue
            label, _, value = entry.partition(":")
            try:
                heights[label.strip()] = float(value)
            except ValueError:
                continue
        return heights

    # ------------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan) -> None:
        count = len(msg.ranges)
        if count == 0:
            return
        if (
            self._angles is None
            or self._angles.shape[0] != count
            or self._scan_angle_min != msg.angle_min
            or self._scan_angle_increment != msg.angle_increment
        ):
            self._angles = msg.angle_min + np.arange(count) * msg.angle_increment
            self._scan_angle_min = msg.angle_min
            self._scan_angle_increment = msg.angle_increment
        self._ranges = np.asarray(msg.ranges, dtype=np.float32)
        self._latest_scan = msg

    def _detections_callback(self, msg: Detection2DArray) -> None:
        self._stats["frames"] += 1
        out = Detection3DArray()
        out.header = msg.header
        out.header.frame_id = self.output_frame

        for det2d in msg.detections:
            self._stats["detections"] += 1
            label, score, source = self._split_id(det2d)
            if self.target_classes and label not in self.target_classes:
                continue

            point, range_source = self._estimate_range(det2d, label)
            if range_source in ("radar", "mono"):
                self._stats[range_source] += 1
            elif range_source == "mono_clipped":
                self._stats["mono_clipped"] += 1

            if point is None:
                if not self.publish_without_range:
                    self._stats["dropped"] += 1
                    continue
            else:
                self._stats["published"] += 1

            out.detections.append(
                self._make_detection3d(
                    det2d,
                    label,
                    score,
                    source,
                    range_source,
                    point or (0.0, 0.0, 0.0),
                )
            )

        self._obstacle_pub.publish(out)
        self._maybe_log()

    # ------------------------------------------------------------------
    @staticmethod
    def _split_id(det2d: Detection2D) -> Tuple[str, float, str]:
        """Recover ``(label, score, source)`` from the vision_detector id."""
        label = ""
        score = 0.0
        if det2d.results:
            label = det2d.results[0].hypothesis.class_id
            score = float(det2d.results[0].hypothesis.score)
        source = "unknown"
        parts = det2d.id.split("#") if det2d.id else []
        if len(parts) >= 2:
            source = parts[1]
            if not label:
                label = parts[0]
        return label, score, source

    def _estimate_range(
        self, det2d: Detection2D, label: str
    ) -> Tuple[Optional[Tuple[float, float, float]], str]:
        """Return ``((x, y, z), source)`` in the output frame, or ``(None, 'none')``."""
        center_x = det2d.bbox.center.position.x
        half_w = 0.5 * max(0.0, det2d.bbox.size_x)
        left_u = center_x - half_w
        right_u = center_x + half_w

        # Bearing in the optical frame: positive to the right.
        bearing_left = self.camera.bearing(left_u)
        bearing_right = self.camera.bearing(right_u)
        # Robot frame (REP-103): y is to the left, so the sign flips.
        a_min = -bearing_right
        a_max = -bearing_left

        radar = self._radar_range(a_min, a_max)
        if radar is not None:
            distance, theta = radar
            x = distance * math.cos(theta)
            y = distance * math.sin(theta)
            return (x, y, 0.0), "radar"

        if not self.enable_mono_depth:
            return None, "none"
        # Some classes are far too small for ``h_pixel`` to carry any range
        # information (a 5 cm block is < 10 px tall beyond 1.4 m, so +/-1 px of
        # box jitter is already +/-0.4 m).  Keep those radar-only.
        if self.mono_depth_classes and label not in self.mono_depth_classes:
            return None, "none"

        real_height = self.object_heights.get(label)
        if real_height is None:
            return None, "none"
        pixel_height = float(det2d.bbox.size_y)
        if self.mono_use_max_box:
            is_clipped = self._box_is_clipped(det2d, pixel_height)
            eff_h = self._effective_mono_height(label, pixel_height, is_clipped)
            if eff_h is None:
                # current frame clipped and no usable unclipped history: refuse
                # rather than publish a clipped-frame estimate.
                return None, "mono_clipped"
            pixel_height = eff_h
        else:
            if self._box_is_clipped(det2d, pixel_height):
                # ``h_pixel`` is truncated by the image border, so
                # ``Z = fy * H / h`` over-estimates the range badly (a person 1 m
                # away reads as ~3 m).  Refuse the estimate rather than publish a
                # distance the planner would trust.
                return None, "mono_clipped"
        depth = self.camera.distance_from_pixel_height(pixel_height, real_height)
        if depth is None:
            return None, "none"

        bearing_center = self.camera.bearing(center_x)
        x = depth * math.cos(bearing_center)
        y = -depth * math.sin(bearing_center)
        return (x, y, 0.0), "mono"

    def _box_is_clipped(self, det2d: Detection2D, pixel_height: float) -> bool:
        """True when the 2D box touches the top or bottom border of the frame.

        ``Detection2D`` carries no image dimensions, so the frame height is
        taken as ``2 * cy`` -- for the calibrated pinhole model used here the
        principal point sits at the centre of the image.
        """
        center_v = det2d.bbox.center.position.y
        top = center_v - 0.5 * pixel_height
        bottom = center_v + 0.5 * pixel_height
        frame_bottom = 2.0 * self.camera.cy - 1.0
        return (
            top <= _MONO_CLIP_MARGIN_PX
            or bottom >= frame_bottom - _MONO_CLIP_MARGIN_PX
        )

    def _effective_mono_height(
        self, label: str, pixel_height: float, is_clipped: bool
    ) -> Optional[float]:
        """Max unclipped pixel height for ``label`` over the recent time window.

        YOLO often frames only part of a tall object (cap + label), yielding a
        small ``size_y`` and a far over-estimate of the range.  Keeping the
        largest *unclipped* height seen recently is equivalent to "trust the
        frame where the full object was captured", which collapses the spurious
        far peak of the measurement distribution.

        Returns ``None`` when there is no usable (unclipped) sample in the
        window yet -- the caller then refuses the monocular estimate instead of
        publishing a clipped-frame distance.
        """
        now = time.monotonic()
        best = self._mono_best.get(label)
        if best is not None and (now - best[0]) > self.mono_max_box_window_s:
            best = None
        # Only unclipped boxes feed the running max: a clipped box under-reports
        # the true height and would drag the max toward a false far range.
        if not is_clipped:
            if best is None or pixel_height > best[1]:
                best = (now, pixel_height)
                self._mono_best[label] = best
        return best[1] if best is not None else None

    def _radar_range(self, a_min: float, a_max: float) -> Optional[Tuple[float, float]]:
        """Closest scan return inside ``[a_min, a_max]`` -> ``(range, angle)``."""
        if self._latest_scan is None or self._angles is None or self._ranges is None:
            return None

        margin = self.bearing_margin
        # Angles wrap at +/-pi.  A window that straddles the scan seam (e.g. a
        # target dead ahead while the scan's angle_min is 0) must still match the
        # beams at 359 deg, so compare *wrapped* angular differences rather than
        # raw values -- otherwise the beam covering the target is silently
        # excluded and the window picks up whatever sits behind it.
        centre = 0.5 * (a_min + a_max)
        half_width = 0.5 * (a_max - a_min) + margin
        delta = (self._angles - centre + np.pi) % (2.0 * np.pi) - np.pi
        mask = np.abs(delta) <= half_width
        if not np.any(mask):
            return None

        window_angles = self._angles[mask]
        window_ranges = self._ranges[mask]
        valid = (
            np.isfinite(window_ranges)
            & (window_ranges >= self.scan_min_range)
            & (window_ranges <= self.scan_max_range)
        )
        if not np.any(valid):
            return None

        valid_angles = window_angles[valid]
        valid_ranges = window_ranges[valid]
        if self.range_statistic == "percentile":
            value = float(np.percentile(valid_ranges, self.range_percentile))
            # Circular mean: the matched beams may straddle the 0/360 seam, and a
            # plain mean of e.g. {359 deg, 0 deg, 1 deg} yields 120 deg instead
            # of ~0 deg, which then corrupts the derived (x, y) position.
            theta = float(
                math.atan2(
                    float(np.mean(np.sin(valid_angles))),
                    float(np.mean(np.cos(valid_angles))),
                )
            )
        else:
            index = int(np.argmin(valid_ranges))
            value = float(valid_ranges[index])
            theta = float(valid_angles[index])
        return value, theta

    def _make_detection3d(
        self,
        det2d: Detection2D,
        label: str,
        score: float,
        source: str,
        range_source: str = "none",
        point: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Detection3D:
        x, y, z = self._to_output_frame(point)

        size_x, size_y = self._footprint(det2d, (x, y, z))
        height = float(self.object_heights.get(label, 0.1))

        det = Detection3D()
        det.header = det2d.header
        det.header.frame_id = self.output_frame
        # "<label>#<detector-source>#<range-source>#<n>".  The range source
        # (radar | mono | mono_clipped | none) used to be invisible, so a
        # measured radar echo and a guessed monocular depth looked identical
        # downstream.
        index = det2d.id.split("#")[-1] if det2d.id else "0"
        det.id = f"{label}#{source}#{range_source}#{index}"

        box = BoundingBox3D()
        box.center.position.x = float(x)
        box.center.position.y = float(y)
        box.center.position.z = float(z + 0.5 * height)
        box.size.x = float(size_x)
        box.size.y = float(size_y)
        box.size.z = float(height)
        det.bbox = box

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = float(score)
        det.results.append(hypothesis)
        return det

    def _footprint(self, det2d: Detection2D, point) -> Tuple[float, float]:
        """Metric footprint from the angular box width at the measured range."""
        radial = math.hypot(point[0], point[1])
        if radial <= 1e-3:
            return 0.1, 0.1
        left = self.camera.bearing(det2d.bbox.center.position.x - 0.5 * det2d.bbox.size_x)
        right = self.camera.bearing(det2d.bbox.center.position.x + 0.5 * det2d.bbox.size_x)
        width = max(0.02, radial * abs(math.tan(right) - math.tan(left)))
        return width, max(0.02, 0.6 * width)

    def _to_output_frame(self, point) -> Tuple[float, float, float]:
        """Apply ``camera_optical_frame -> output_frame`` when ``use_tf`` is on.

        With ``use_tf=false`` (default) the point is assumed to already be in
        ``output_frame`` (camera looking forward, no roll/pitch/yaw offset).
        """
        if not self.use_tf or self._tf_buffer is None:
            return point
        try:
            transform = self._tf_buffer.lookup_transform(
                self.output_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:  # noqa: BLE001 - TFExtrapolationException etc.
            self.get_logger().warn(
                f"TF lookup failed ({exc.__class__.__name__}); using raw camera frame",
                throttle_duration_sec=5.0,
            )
            return point

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # quaternion (x, y, z, w) -> rotation matrix
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        rot = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )
        rotated = rot @ np.array(point, dtype=np.float64)
        return (
            float(rotated[0] + translation.x),
            float(rotated[1] + translation.y),
            float(rotated[2] + translation.z),
        )

    def _maybe_log(self) -> None:
        if self.log_interval <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_log_time < self.log_interval:
            return
        self._last_log_time = now
        delta = {
            key: value - self._stats_logged.get(key, 0)
            for key, value in self._stats.items()
        }
        self._stats_logged = dict(self._stats)
        self.get_logger().info(
            f"arrays {delta['frames']} published {delta['published']} | "
            f"radar {delta['radar']} mono {delta['mono']} "
            f"unranged {delta['dropped']} mono_clipped {delta['mono_clipped']}"
        )

    def _publish_stats(self) -> None:
        """Publish the cumulative fusion counters as JSON.

        Exposed on ``fusion_stats_topic`` so that "how many of the published
        obstacles actually got a radar range" is observable at runtime instead
        of only appearing in a log line.
        """
        payload = dict(self._stats)
        payload["stamp"] = self.get_clock().now().nanoseconds / 1e9
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._stats_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VisionRadarFusion()
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
