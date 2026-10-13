"""``vision_detector``: YOLO11n + classical colour detection for mycar.

Pipeline per frame
------------------
1. decode the incoming ``sensor_msgs/CompressedImage`` with ``cv2.imdecode``
2. **YOLO11n** (``imgsz=640``, CPU) -> ``person`` (class 0) / ``bottle`` (39)
3. **classical HSV segmentation** -> ``blue_block`` / ``green_block``
4. **guard**: drop ``green_block`` boxes that sit inside a YOLO ``bottle`` box
   (a green bottle label must not be reported as a green block)
5. publish the fused ``vision_msgs/Detection2DArray`` and, optionally, an
   annotated JPEG image

Threading model
---------------
A single ``MutuallyExclusiveCallbackGroup`` is used.  While a frame is being
processed the busy flag is set and any further frame is **dropped** (the QoS
profile keeps only the newest sample).  This is the classic "drop new frames
instead of queueing them" behaviour requested for slow CPU inference.

This node is **perception only** -- it never publishes ``/cmd_vel``.  Downstream
consumers live in ``vision_radar_fusion``, ``avoidance_decider`` and
``vision_to_costmap``.
"""

from __future__ import annotations

import os
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from .color_detector import ColorBlockDetector
from .detection_utils import ANNOTATION_COLORS
from .detection_utils import BOTTLE
from .detection_utils import BOTTLE_EDGE
from .detection_utils import PERSON
from .detection_utils import SUPPRESSED_COLOR
from .detection_utils import Detection
from .detection_utils import suppress_blocks_in_containers

def _load_cv_bridge():
    """Import ``cv_bridge`` lazily.

    It is only needed for the optional *raw* annotated image.  Importing it
    unconditionally spews a scary (but harmless) NumPy-1.x-ABI diagnostic on
    hosts running NumPy 2 with a NumPy-1-built ``cv_bridge``, so it is probed
    on demand and the node degrades gracefully when it is unusable.
    """
    try:  # pragma: no cover - depends on the host installation
        from cv_bridge import CvBridge as _CvBridge

        return _CvBridge
    except Exception:  # noqa: BLE001 - numpy/OpenCV ABI mismatches, missing pkg
        return None


def _input_qos() -> QoSProfile:
    """Best-effort / depth 1 -- matches ``mycar_camera``'s video publisher."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _video_qos() -> QoSProfile:
    """Best-effort / depth 1 for the annotated video stream."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _detection_qos() -> QoSProfile:
    """Reliable / depth 10 for the (tiny) detection array."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class VisionDetector(Node):
    """Fuse a YOLO11n detector with a classical colour detector."""

    def __init__(self) -> None:
        super().__init__("vision_detector")

        self._declare_parameters()
        self._read_parameters()

        # -- model ---------------------------------------------------------
        self._model = None
        self._load_model()

        # -- sub-detectors -------------------------------------------------
        self._color_detector: Optional[ColorBlockDetector] = None
        if self.detect_colored_blocks:
            self._color_detector = ColorBlockDetector(
                classes=self.color_classes,
                min_area=self.color_min_area,
                max_area=self.color_max_area,
                min_fill=self.color_min_fill,
                min_solidity=self.color_min_solidity,
                morph_kernel=self.color_morph_kernel,
                edge_min_area=self.edge_min_area,
                edge_aspect_min=self.edge_aspect_min,
                edge_aspect_max=self.edge_aspect_max,
                edge_min_circularity=self.edge_min_circularity,
                edge_canny_low=self.edge_canny_low,
                edge_canny_high=self.edge_canny_high,
                edge_blur=self.edge_blur,
                edge_close_kernel=self.edge_close_kernel,
                edge_fill=self.edge_fill,
            )

        self._bridge = None
        if self.publish_annotated_raw:
            bridge_class = _load_cv_bridge()
            if bridge_class is not None:
                self._bridge = bridge_class()
            else:
                self.get_logger().warn(
                    "publish_annotated_raw=true but cv_bridge is unavailable "
                    "(numpy/OpenCV ABI mismatch?); raw annotated image disabled."
                )

        # -- ROS interfaces ------------------------------------------------
        self._detection_pub = self.create_publisher(
            Detection2DArray, self.detection_topic, _detection_qos()
        )
        self._annotated_pub = (
            self.create_publisher(CompressedImage, self.annotated_topic, _video_qos())
            if self.publish_annotated and self.annotated_topic
            else None
        )
        self._annotated_raw_pub = (
            self.create_publisher(Image, self.annotated_raw_topic, _video_qos())
            if self._bridge is not None and self.annotated_raw_topic
            else None
        )

        self.create_subscription(
            CompressedImage, self.image_topic, self._image_callback, _input_qos()
        )

        # -- bookkeeping ---------------------------------------------------
        self._processing = False
        self._frames_in = 0
        self._frames_processed = 0
        self._frames_dropped = 0
        self._decode_failures = 0
        self._infer_failures = 0
        self._close_warnings = 0
        self._last_processed_time: Optional[float] = None
        self._fps = 0.0
        self._last_log_time = time.monotonic()
        self._last_detection_count = 0

        self.get_logger().info(
            f"vision_detector ready | model={self.model_path} | "
            f"in={self.image_topic} out={self.detection_topic} "
            f"annotated={self.annotated_topic if self._annotated_pub else 'off'} | "
            f"imgsz={self.image_size} device={self.device or 'auto'} "
            f"conf={self.confidence} iou={self.iou}"
        )

    # ------------------------------------------------------------------
    # parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # --- requested by the project spec -------------------------------
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("detection_topic", "/vision/detections")
        self.declare_parameter("annotated_topic", "/vision/image/compressed")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("bottle_class_id", 39)
        self.declare_parameter("detect_colored_blocks", True)
        self.declare_parameter("color_min_area", 800)

        # --- small, documented extras ------------------------------------
        self.declare_parameter("model_dir", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("color_classes", ["blue_block", "green_block"])
        self.declare_parameter("color_max_area", 0)
        self.declare_parameter("color_min_fill", 0.45)
        self.declare_parameter("color_min_solidity", 0.55)
        self.declare_parameter("color_morph_kernel", 5)
        self.declare_parameter("bottle_containment_ratio", 0.35)
        self.declare_parameter("bottle_center_margin", 0.10)
        self.declare_parameter("annotated_quality", 70)
        self.declare_parameter("publish_annotated_raw", False)
        self.declare_parameter("annotated_raw_topic", "/vision/image_raw")
        self.declare_parameter("log_interval", 10.0)

        # --- distant small-object fallback (magnified ROI second pass) -----
        self.declare_parameter("roi_zoom_enabled", False)
        self.declare_parameter("roi_zoom", 2.0)
        self.declare_parameter("roi_center_x", 0.5)
        self.declare_parameter("roi_center_y", 0.75)

        # --- third layer: outline-only bottle fallback --------------------
        self.declare_parameter("detect_bottle_edges", False)
        self.declare_parameter("edge_min_area", 200)
        self.declare_parameter("edge_aspect_min", 1.5)
        self.declare_parameter("edge_aspect_max", 4.0)
        self.declare_parameter("edge_min_circularity", 0.3)
        self.declare_parameter("edge_canny_low", 20)
        self.declare_parameter("edge_canny_high", 70)
        self.declare_parameter("edge_blur", 5)
        self.declare_parameter("edge_close_kernel", 5)
        self.declare_parameter("edge_fill", True)

    def _read_parameters(self) -> None:
        gp = self.get_parameter

        self.model = gp("model").get_parameter_value().string_value
        self.image_topic = gp("image_topic").get_parameter_value().string_value
        self.detection_topic = gp("detection_topic").get_parameter_value().string_value
        self.annotated_topic = gp("annotated_topic").get_parameter_value().string_value
        self.confidence = gp("confidence").get_parameter_value().double_value
        self.iou = gp("iou").get_parameter_value().double_value
        self.image_size = gp("image_size").get_parameter_value().integer_value
        self.device = gp("device").get_parameter_value().string_value.strip()
        self.publish_annotated = gp("publish_annotated").get_parameter_value().bool_value
        self.person_class_id = gp("person_class_id").get_parameter_value().integer_value
        self.bottle_class_id = gp("bottle_class_id").get_parameter_value().integer_value
        self.detect_colored_blocks = (
            gp("detect_colored_blocks").get_parameter_value().bool_value
        )
        self.color_min_area = gp("color_min_area").get_parameter_value().integer_value

        self.model_dir = gp("model_dir").get_parameter_value().string_value
        self.frame_id = gp("frame_id").get_parameter_value().string_value
        self.color_classes = list(
            gp("color_classes").get_parameter_value().string_array_value
        )
        self.color_max_area = gp("color_max_area").get_parameter_value().integer_value
        self.color_min_fill = gp("color_min_fill").get_parameter_value().double_value
        self.color_min_solidity = (
            gp("color_min_solidity").get_parameter_value().double_value
        )
        self.color_morph_kernel = (
            gp("color_morph_kernel").get_parameter_value().integer_value
        )
        self.bottle_containment_ratio = (
            gp("bottle_containment_ratio").get_parameter_value().double_value
        )
        self.bottle_center_margin = (
            gp("bottle_center_margin").get_parameter_value().double_value
        )
        self.annotated_quality = int(
            gp("annotated_quality").get_parameter_value().integer_value
        )
        self.publish_annotated_raw = (
            gp("publish_annotated_raw").get_parameter_value().bool_value
        )
        self.annotated_raw_topic = (
            gp("annotated_raw_topic").get_parameter_value().string_value
        )
        self.log_interval = gp("log_interval").get_parameter_value().double_value
        self.roi_zoom_enabled = gp("roi_zoom_enabled").get_parameter_value().bool_value
        self.roi_zoom = gp("roi_zoom").get_parameter_value().double_value
        self.roi_center_x = gp("roi_center_x").get_parameter_value().double_value
        self.roi_center_y = gp("roi_center_y").get_parameter_value().double_value
        self.detect_bottle_edges = (
            gp("detect_bottle_edges").get_parameter_value().bool_value
        )
        self.edge_min_area = gp("edge_min_area").get_parameter_value().integer_value
        self.edge_aspect_min = gp("edge_aspect_min").get_parameter_value().double_value
        self.edge_aspect_max = gp("edge_aspect_max").get_parameter_value().double_value
        self.edge_min_circularity = (
            gp("edge_min_circularity").get_parameter_value().double_value
        )
        self.edge_canny_low = gp("edge_canny_low").get_parameter_value().integer_value
        self.edge_canny_high = gp("edge_canny_high").get_parameter_value().integer_value
        self.edge_blur = gp("edge_blur").get_parameter_value().integer_value
        self.edge_close_kernel = (
            gp("edge_close_kernel").get_parameter_value().integer_value
        )
        self.edge_fill = gp("edge_fill").get_parameter_value().bool_value

        self.annotated_quality = max(1, min(100, self.annotated_quality))
        self.confidence = max(0.0, min(1.0, self.confidence))
        self.iou = max(0.0, min(1.0, self.iou))
        if self.image_size <= 0:
            self.image_size = 640
        self.roi_zoom = max(1.0, float(self.roi_zoom))
        self.roi_center_x = min(1.0, max(0.0, float(self.roi_center_x)))
        self.roi_center_y = min(1.0, max(0.0, float(self.roi_center_y)))

    # ------------------------------------------------------------------
    # model loading
    # ------------------------------------------------------------------
    def _model_search_paths(self, name: str) -> List[str]:
        paths: List[str] = []
        if self.model_dir:
            paths.append(os.path.join(os.path.expanduser(self.model_dir), name))
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("mycar_vision")
            paths.append(os.path.join(share, "models", name))
        except Exception:  # noqa: BLE001 - not installed yet while running from source
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        paths.append(os.path.join(os.path.dirname(here), "models", name))
        paths.append(os.path.join(os.getcwd(), name))
        paths.append(os.path.expanduser(f"~/.cache/mycar_vision/{name}"))
        return paths

    def _resolve_model_path(self) -> str:
        name = self.model.strip()
        if not name:
            name = "yolo11n.pt"
        if os.path.isabs(name):
            return name if os.path.exists(name) else name
        for candidate in self._model_search_paths(name):
            if os.path.exists(candidate):
                return candidate
        self.get_logger().warn(
            f"Model '{name}' not found locally; ultralytics will try to download it."
        )
        return name

    def _load_model(self) -> None:
        self.model_path = self._resolve_model_path()
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - depends on the host
            raise RuntimeError(
                "The 'ultralytics' package is required: pip3 install --user "
                "'ultralytics>=8.3.0'"
            ) from exc

        started = time.time()
        self._model = YOLO(self.model_path)
        self._infer_device = self.device if self.device else None
        names = getattr(self._model, "names", {}) or {}
        person_name = names.get(self.person_class_id, "?")
        bottle_name = names.get(self.bottle_class_id, "?")
        self.get_logger().info(
            f"Loaded {os.path.basename(self.model_path)} in "
            f"{time.time() - started:.2f}s | person id={self.person_class_id} "
            f"({person_name}) bottle id={self.bottle_class_id} ({bottle_name})"
        )

    # ------------------------------------------------------------------
    # frame handling
    # ------------------------------------------------------------------
    def _image_callback(self, msg: CompressedImage) -> None:
        self._frames_in += 1
        if self._processing:
            # Drop the incoming frame: the previous one is still in flight.
            self._frames_dropped += 1
            return

        self._processing = True
        try:
            self._process(msg)
        except Exception as exc:  # noqa: BLE001 - never kill the node on one frame
            self._infer_failures += 1
            self.get_logger().error(f"Failed to process frame: {exc!r}")
        finally:
            self._processing = False

    def _process(self, msg: CompressedImage) -> None:
        frame = self._decode(msg)
        if frame is None:
            return

        started = time.perf_counter()
        yolo_detections = self._run_yolo(frame)
        if self.roi_zoom_enabled:
            zoomed = self._run_yolo_roi_zoom(frame, yolo_detections)
            if zoomed:
                yolo_detections = yolo_detections + zoomed
        color_detections, suppressed = self._run_color(frame, yolo_detections)
        # Layer 3 -- only reached while no bottle has been found yet.
        edge_detections = self._run_edges(
            frame, yolo_detections + color_detections
        )
        infer_ms = (time.perf_counter() - started) * 1000.0

        detections = sorted(
            yolo_detections + color_detections + edge_detections,
            key=lambda d: d.area,
            reverse=True,
        )

        header = msg.header
        if self.frame_id:
            header.frame_id = self.frame_id

        self._detection_pub.publish(self._to_detection_array(detections, header))
        self._last_detection_count = len(detections)

        if self._annotated_pub is not None:
            self._publish_annotated(frame, detections, suppressed, infer_ms, header)

        self._frames_processed += 1
        self._update_stats()
        self._maybe_log(detections, infer_ms)

    def _decode(self, msg: CompressedImage) -> Optional[np.ndarray]:
        data = bytes(msg.data)
        if not data:
            self._decode_failures += 1
            return None
        buffer = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            self._decode_failures += 1
            self.get_logger().warn(
                f"cv2.imdecode failed on a {len(data)}-byte frame",
                throttle_duration_sec=10.0,
            )
            return None
        return frame

    # -- detectors ---------------------------------------------------------
    def _run_yolo(self, frame: np.ndarray) -> List[Detection]:
        if self._model is None:
            return []

        results = self._model.predict(
            source=frame,
            imgsz=self.image_size,
            conf=self.confidence,
            iou=self.iou,
            device=self._infer_device,
            verbose=False,
        )
        if not results:
            return []

        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        height, width = frame.shape[:2]
        wanted = {self.person_class_id: PERSON, self.bottle_class_id: BOTTLE}
        detections: List[Detection] = []
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)

        for (x1, y1, x2, y2), score, class_id in zip(xyxy, confs, classes):
            label = wanted.get(int(class_id))
            if label is None:
                continue  # only person / bottle are forwarded downstream
            detections.append(
                Detection(
                    label=label,
                    score=float(score),
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    source="yolo",
                ).clamp(width, height)
            )
        return detections

    def _run_yolo_roi_zoom(
        self, frame: np.ndarray, yolo_detections: List[Detection]
    ) -> List[Detection]:
        """Second YOLO pass over a magnified sub-window, for distant bottles.

        Letterboxing to a fixed ``imgsz`` means the network sees an object at
        ``object_px * imgsz / max(roi_w, roi_h)``.  Cropping only the lower band
        keeps the full frame width, so blowing that band back up is undone by
        the letterbox and buys *nothing* -- the window has to shrink in BOTH
        dimensions.  A ``w/k x h/k`` window resized back to ``w x h`` magnifies
        the object by exactly ``k``, at the price of seeing only ``1/k**2`` of
        the scene.

        That is why this is a narrow forward-looking magnifying glass rather
        than a whole-frame pass, and why it only runs when the first pass found
        no bottle at all (otherwise it would double the frame cost every time).
        """
        if self._model is None or not self.roi_zoom_enabled:
            return []
        if any(d.label == BOTTLE for d in yolo_detections):
            return []

        height, width = frame.shape[:2]
        k = max(1.0, float(self.roi_zoom))
        crop_w = max(1, min(width, int(round(width / k))))
        crop_h = max(1, min(height, int(round(height / k))))
        cx = int(round(self.roi_center_x * width))
        cy = int(round(self.roi_center_y * height))
        x0 = max(0, min(width - crop_w, cx - crop_w // 2))
        y0 = max(0, min(height - crop_h, cy - crop_h // 2))

        roi = frame[y0:y0 + crop_h, x0:x0 + crop_w]
        if roi.size == 0:
            return []

        magnified = cv2.resize(roi, (width, height), interpolation=cv2.INTER_LINEAR)
        found = self._run_yolo(magnified)
        for det in found:
            det.x1 = min(width, max(0.0, det.x1 / k + x0))
            det.x2 = min(width, max(0.0, det.x2 / k + x0))
            det.y1 = min(height, max(0.0, det.y1 / k + y0))
            det.y2 = min(height, max(0.0, det.y2 / k + y0))
        return found

    def _run_color(
        self, frame: np.ndarray, yolo_detections: List[Detection]
    ) -> Tuple[List[Detection], List[Detection]]:
        if self._color_detector is None:
            return [], []

        blocks = self._color_detector.detect(frame)
        if not blocks:
            return [], []

        bottles = [d for d in yolo_detections if d.label == BOTTLE]
        if not bottles:
            return blocks, []

        # Bottle-label guard: a colour patch inside a detected bottle is not a block.
        kept, suppressed = suppress_blocks_in_containers(
            blocks,
            bottles,
            containment_ratio=self.bottle_containment_ratio,
            center_margin_ratio=self.bottle_center_margin,
        )
        if suppressed and self._close_warnings < 3:
            self._close_warnings += 1
            self.get_logger().info(
                f"bottle guard suppressed {len(suppressed)} colour block(s)"
            )
        return kept, suppressed

    def _run_edges(
        self, frame: np.ndarray, earlier: List[Detection]
    ) -> List[Detection]:
        """Layer 3: outline-only fallback for colourless transparent bottles.

        Runs only when no earlier layer (YOLO, the ROI zoom, HSV) has already
        produced a bottle, and any outline landing inside something an earlier
        layer reported is dropped -- so a transparent bottle is never counted
        twice and a person's silhouette is not mistaken for one.

        This is inherently the noisiest layer: door frames, chair legs and
        strong floor seams are all tall closed contours too.  It is off by
        default and its filters need tuning against the real scene.
        """
        if self._color_detector is None or not self.detect_bottle_edges:
            return []
        if any(d.label in (BOTTLE, BOTTLE_EDGE) for d in earlier):
            return []

        found = self._color_detector.detect_by_edges(frame)
        if not found:
            return []

        kept, _ = suppress_blocks_in_containers(
            found,
            earlier,
            containment_ratio=self.bottle_containment_ratio,
            center_margin_ratio=self.bottle_center_margin,
        )
        return list(kept)

    # -- publishing --------------------------------------------------------
    def _to_detection_array(self, detections, header) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header = header
        for index, det in enumerate(detections):
            out = Detection2D()
            out.header = header
            # ``id`` carries the provenance so downstream nodes can tell YOLO
            # boxes from classical ones without a custom message type.
            out.id = f"{det.label}#{det.source}#{index}"
            center_x, center_y = det.center
            out.bbox.center.position.x = float(center_x)
            out.bbox.center.position.y = float(center_y)
            out.bbox.center.theta = 0.0
            out.bbox.size_x = float(det.width)
            out.bbox.size_y = float(det.height)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det.label
            hypothesis.hypothesis.score = float(det.score)
            out.results.append(hypothesis)
            msg.detections.append(out)
        return msg

    def _publish_annotated(self, frame, detections, suppressed, infer_ms, header):
        canvas = self._annotate(frame, detections, suppressed, infer_ms)
        ok, encoded = cv2.imencode(
            ".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), self.annotated_quality]
        )
        if not ok:
            self.get_logger().warn("Failed to encode the annotated frame", throttle_duration_sec=10.0)
            return

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self._annotated_pub.publish(msg)

        if self._annotated_raw_pub is not None and self._bridge is not None:
            try:
                raw = self._bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
                raw.header = header
                self._annotated_raw_pub.publish(raw)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"cv2_to_imgmsg failed, disabling raw output: {exc!r}"
                )
                self._annotated_raw_pub = None

    def _annotate(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        suppressed: List[Detection],
        infer_ms: float,
    ) -> np.ndarray:
        canvas = frame.copy()

        for det in detections:
            color = ANNOTATION_COLORS.get(det.label, (200, 200, 200))
            p1 = (int(round(det.x1)), int(round(det.y1)))
            p2 = (int(round(det.x2)), int(round(det.y2)))
            cv2.rectangle(canvas, p1, p2, color, 2)
            caption = f"{det.label} {det.score:.2f}"
            self._draw_caption(canvas, caption, p1, color)

        for det in suppressed:
            p1 = (int(round(det.x1)), int(round(det.y1)))
            p2 = (int(round(det.x2)), int(round(det.y2)))
            cv2.rectangle(canvas, p1, p2, SUPPRESSED_COLOR, 1)
            self._draw_caption(canvas, "green_block (in bottle)", p1, SUPPRESSED_COLOR)

        dropped_ratio = (
            100.0 * self._frames_dropped / self._frames_in if self._frames_in else 0.0
        )
        self._draw_hud(
            canvas,
            [
                f"FPS {self._fps:.1f}  infer {infer_ms:.0f} ms",
                f"det {len(detections)}  drop {self._frames_dropped}"
                f" ({dropped_ratio:.0f}%)",
                f"{self.image_size}px  {os.path.basename(self.model_path)}",
            ],
        )
        return canvas

    @staticmethod
    def _draw_caption(canvas, text: str, anchor: Tuple[int, int], color) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        thickness = 1
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        y = max(anchor[1] - 4, th + 2)
        x = anchor[0]
        cv2.rectangle(canvas, (x, y - th - 3), (x + tw + 3, y + baseline), color, -1)
        cv2.putText(
            canvas,
            text,
            (x + 2, y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_hud(canvas, lines) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        for index, text in enumerate(lines):
            y = 20 + index * 20
            cv2.putText(
                canvas, text, (8, y), font, 0.5, (0, 0, 0), 3, cv2.LINE_AA
            )
            cv2.putText(
                canvas, text, (8, y), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA
            )

    # -- stats -------------------------------------------------------------
    def _update_stats(self) -> None:
        now = time.monotonic()
        if self._last_processed_time is not None:
            delta = now - self._last_processed_time
            if delta > 0.0:
                instant = 1.0 / delta
                self._fps = instant if self._fps <= 0.0 else 0.8 * self._fps + 0.2 * instant
        self._last_processed_time = now

    def _maybe_log(self, detections, infer_ms: float) -> None:
        if self.log_interval <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_log_time < self.log_interval:
            return

        elapsed = max(1e-6, now - self._last_log_time)
        self._last_log_time = now
        dropped_ratio = 100.0 * self._frames_dropped / self._frames_in if self._frames_in else 0.0
        labels = {}
        for det in detections:
            labels[det.label] = labels.get(det.label, 0) + 1
        summary = ", ".join(f"{k}x{v}" for k, v in sorted(labels.items())) or "none"

        self.get_logger().info(
            f"in {self._frames_in} proc {self._frames_processed} "
            f"drop {self._frames_dropped} ({dropped_ratio:.0f}%) "
            f"| {self._frames_processed / elapsed:.1f} fps processed "
            f"| {infer_ms:.0f} ms/frame | {summary}"
        )
        self._frames_in = 0
        self._frames_processed = 0
        self._frames_dropped = 0

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VisionDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"[vision_detector] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
