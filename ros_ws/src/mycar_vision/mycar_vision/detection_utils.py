"""Shared geometry / colour / camera helpers for ``mycar_vision``.

This module deliberately contains **no ROS imports** so that the maths can be
unit tested with plain Python (see ``test/test_detection_utils.py``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Class-name tables
# ---------------------------------------------------------------------------

PERSON = "person"
BOTTLE = "bottle"
#: Colourless transparent bottle found from its outline (third detector layer).
BOTTLE_EDGE = "bottle_edge"
BLUE_BLOCK = "blue_block"
GREEN_BLOCK = "green_block"

#: Colour classes handled by the classical (non-YOLO) detector.
COLOR_CLASSES = (BLUE_BLOCK, GREEN_BLOCK)

#: HSV thresholds in the OpenCV convention (H: 0-179, S: 0-255, V: 0-255).
#:
#: To add a class simply append another entry, e.g.::
#:
#:     "red_block": {
#:         "hsv_lower": (0, 110, 60),
#:         "hsv_upper": (10, 255, 255),
#:         "bgr": (40, 40, 230),
#:     },
COLOR_PROFILES: Dict[str, Dict[str, object]] = {
    BLUE_BLOCK: {
        "hsv_lower": (100, 110, 60),
        "hsv_upper": (130, 255, 255),
        "bgr": (230, 120, 30),
    },
    GREEN_BLOCK: {
        "hsv_lower": (40, 90, 60),
        "hsv_upper": (85, 255, 255),
        "bgr": (60, 200, 60),
    },
    # Distant-bottle fallback.  YOLO stops seeing a 0.22 m bottle at ~50 px,
    # i.e. ~1.6 m, but a colour blob is still a few hundred pixels there, so
    # the HSV path keeps working well past the YOLO range.
    #
    # The hue range MUST be calibrated against the real bottle
    # (see tools/calibrate_bottle_hsv.py) -- it is only active when "bottle"
    # is listed in the `color_classes` parameter, which is off by default.
    #
    # The block-oriented filters are relaxed here on purpose: a bottle is small
    # (min_area 40 instead of 800), thin (min_fill 0.25) and would be erased by
    # the default 5x5 morphology (morph_kernel 3).
    BOTTLE: {
        "hsv_lower": (95, 80, 40),
        "hsv_upper": (135, 255, 255),
        "bgr": (0, 215, 255),
        "min_area": 40,
        "min_fill": 0.25,
        "min_solidity": 0.35,
        "morph_kernel": 3,
    },
}

#: BGR colours used when drawing the annotated image.
ANNOTATION_COLORS: Dict[str, Tuple[int, int, int]] = {
    PERSON: (0, 140, 255),        # orange
    BOTTLE: (0, 215, 255),        # amber
    BOTTLE_EDGE: (255, 180, 0),   # cyan-ish, distinct from the YOLO bottle
    BLUE_BLOCK: (230, 120, 30),   # blue
    GREEN_BLOCK: (60, 200, 60),   # green
}
SUPPRESSED_COLOR: Tuple[int, int, int] = (0, 0, 255)

#: Assumed real-world height of each class in metres, used by the monocular
#: fallback inside :mod:`mycar_vision.vision_radar_fusion`.
DEFAULT_REAL_HEIGHTS: Dict[str, float] = {
    PERSON: 1.65,
    BOTTLE: 0.22,
    BOTTLE_EDGE: 0.22,
    BLUE_BLOCK: 0.05,
    GREEN_BLOCK: 0.05,
}


# ---------------------------------------------------------------------------
# Detection container
# ---------------------------------------------------------------------------


@dataclass
class Detection:
    """One 2D detection in pixel coordinates (``x`` right, ``y`` down)."""

    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    source: str = "color"  # "yolo" | "color"
    #: Metres. Filled in by ``vision_radar_fusion``; ``None`` from the detector.
    distance: Optional[float] = None

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> Tuple[float, float]:
        return (0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2))

    def clamp(self, image_width: int, image_height: int) -> "Detection":
        """Clip the box into the image and keep ``x1 <= x2`` / ``y1 <= y2``."""
        x_lo, x_hi = sorted((self.x1, self.x2))
        y_lo, y_hi = sorted((self.y1, self.y2))
        self.x1 = float(min(max(x_lo, 0.0), max(0, image_width - 1)))
        self.x2 = float(min(max(x_hi, 0.0), max(0, image_width - 1)))
        self.y1 = float(min(max(y_lo, 0.0), max(0, image_height - 1)))
        self.y2 = float(min(max(y_hi, 0.0), max(0, image_height - 1)))
        return self


# ---------------------------------------------------------------------------
# Box geometry
# ---------------------------------------------------------------------------


def intersection_area(a: Detection, b: Detection) -> float:
    """Area of the intersection of two axis-aligned boxes."""
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    return (x2 - x1) * (y2 - y1)


def iou(a: Detection, b: Detection) -> float:
    """Intersection-over-union of two detections."""
    inter = intersection_area(a, b)
    if inter <= 0.0:
        return 0.0
    union = a.area + b.area - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def containment(inner: Detection, outer: Detection) -> float:
    """Fraction of ``inner``'s area that lies inside ``outer``."""
    if inner.area <= 0.0:
        return 0.0
    return intersection_area(inner, outer) / inner.area


def expand_box(box: Detection, ratio: float) -> Detection:
    """Grow a box by ``ratio`` of its own size on every side."""
    dx = box.width * ratio
    dy = box.height * ratio
    return Detection(
        label=box.label,
        score=box.score,
        x1=box.x1 - dx,
        y1=box.y1 - dy,
        x2=box.x2 + dx,
        y2=box.y2 + dy,
        source=box.source,
    )


def point_inside(u: float, v: float, box: Detection) -> bool:
    return box.x1 <= u <= box.x2 and box.y1 <= v <= box.y2


def suppress_blocks_in_containers(
    blocks: Sequence[Detection],
    containers: Sequence[Detection],
    containment_ratio: float = 0.35,
    center_margin_ratio: float = 0.10,
) -> Tuple[List[Detection], List[Detection]]:
    """Split colour blocks into (kept, suppressed) using container boxes.

    A block is considered part of a container when

    * its centre falls inside the container box grown by
      ``center_margin_ratio`` (typical case: a bottle label is a small patch in
      the middle of the bottle), **or**
    * at least ``containment_ratio`` of the block's area overlaps the
      container (typical case: a green bottle body swallowed by the box).

    Returns ``(kept, suppressed)`` preserving the input order.
    """
    kept: List[Detection] = []
    suppressed: List[Detection] = []
    grown = [expand_box(c, center_margin_ratio) for c in containers]

    for block in blocks:
        cx, cy = block.center
        blocked = False
        for raw, pad in zip(containers, grown):
            if point_inside(cx, cy, pad) or containment(block, raw) >= containment_ratio:
                blocked = True
                break
        (suppressed if blocked else kept).append(block)
    return kept, suppressed


# ---------------------------------------------------------------------------
# Monocular camera model (extension point for distance estimation)
# ---------------------------------------------------------------------------


@dataclass
class MonoCameraModel:
    """Simple pinhole model for a monocular range fallback.

    Defaults match the 640x480 / ~60 deg horizontal FOV stream published by
    ``mycar_camera`` (``fx = 320 / tan(30 deg) ~= 554``).  Replace with real
    calibration values before trusting the numbers.
    """

    fx: float = 554.3
    fy: float = 554.3
    cx: float = 320.0
    cy: float = 240.0

    def bearing(self, u: float) -> float:
        """Horizontal angle of pixel column ``u``, positive to the right (rad)."""
        if self.fx <= 0.0:
            return 0.0
        return math.atan2(u - self.cx, self.fx)

    def distance_from_pixel_height(
        self, pixel_height: float, real_height: float
    ) -> Optional[float]:
        """Depth along the optical axis: ``Z = fy * H_real / h_pixel``."""
        if pixel_height <= 1e-6 or real_height <= 0.0 or self.fy <= 0.0:
            return None
        return self.fy * real_height / pixel_height

    def distance_from_bottom_edge(
        self, bottom_v: float, ground_v: float, real_height: float
    ) -> Optional[float]:
        """Depth from the distance between the box bottom and the ground row."""
        delta = bottom_v - ground_v
        if abs(delta) <= 1e-6 or self.fy <= 0.0 or real_height <= 0.0:
            return None
        return abs(self.fy * real_height / delta)

    def estimate(
        self,
        detection: Detection,
        real_height: float,
        use_bottom_edge: bool = False,
        ground_v: Optional[float] = None,
    ) -> Optional[float]:
        """Best-effort monocular range for one detection."""
        if use_bottom_edge and ground_v is not None:
            return self.distance_from_bottom_edge(detection.y2, ground_v, real_height)
        return self.distance_from_pixel_height(detection.height, real_height)
