"""Classical HSV colour-block detector.

Thin, dependency-light wrapper around ``cv2.inRange`` + contour filtering.  It
is kept in its own module so that:

* the ROS node stays readable, and
* the logic can be tested without ``rclpy`` (see ``test/test_detection_utils.py``).
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np

from .detection_utils import BOTTLE_EDGE, COLOR_PROFILES, Detection


class ColorBlockDetector:
    """Detect solid coloured blocks (blue_block / green_block) via HSV masks."""

    def __init__(
        self,
        classes: Sequence[str] = ("blue_block", "green_block"),
        min_area: int = 800,
        max_area: int = 0,
        min_fill: float = 0.45,
        min_solidity: float = 0.55,
        morph_kernel: int = 5,
        # --- edge fallback (colourless transparent bottles) ---------------
        edge_min_area: int = 200,
        edge_aspect_min: float = 1.5,
        edge_aspect_max: float = 4.0,
        edge_min_circularity: float = 0.3,
        # 20/70 rather than the usual 50/150: a colourless bottle against a
        # plain floor is a very low-contrast target and 50/150 extracts almost
        # no edge at all (measured: 0 usable contours at every distance).
        edge_canny_low: int = 20,
        edge_canny_high: int = 70,
        edge_blur: int = 5,
        edge_close_kernel: int = 5,
        edge_fill: bool = True,
    ) -> None:
        """
        Args:
            classes: Colour profile names to look for (keys of ``COLOR_PROFILES``).
            min_area: Minimum contour area in pixels.
            max_area: Maximum contour area in pixels (``0`` disables the check).
            min_fill: Minimum ``rect_area_ratio``.  ``contour_area / bbox_area``.
                Solid cubes score ~1.0; thin reflections / laser lines score low.
            min_solidity: Minimum ``contour_area / convex_hull_area``.
            morph_kernel: Odd kernel size for the open/close morphology.
        """
        self.min_area = int(min_area)
        self.max_area = int(max_area)
        self.min_fill = float(min_fill)
        self.min_solidity = float(min_solidity)

        self.edge_min_area = int(edge_min_area)
        self.edge_aspect_min = float(edge_aspect_min)
        self.edge_aspect_max = float(edge_aspect_max)
        self.edge_min_circularity = float(edge_min_circularity)
        self.edge_canny_low = int(edge_canny_low)
        self.edge_canny_high = int(edge_canny_high)
        self.edge_blur = int(edge_blur)
        self.edge_close_kernel = int(edge_close_kernel)
        self.edge_fill = bool(edge_fill)

        size = max(1, int(morph_kernel))
        if size % 2 == 0:
            size += 1
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))

        self._profiles: Dict[str, Dict[str, object]] = {}
        for name in classes:
            profile = COLOR_PROFILES.get(name)
            if profile is None:
                raise KeyError(f"Unknown colour class '{name}' (see COLOR_PROFILES)")
            self._profiles[name] = self._resolve_profile(profile)

    # -- internals ---------------------------------------------------------

    @staticmethod
    def _kernel_for(size: object) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))

    def _resolve_profile(self, profile: Dict[str, object]) -> Dict[str, object]:
        """Merge a class' own filter settings on top of the detector defaults.

        ``COLOR_PROFILES`` may carry ``min_area`` / ``max_area`` / ``min_fill`` /
        ``min_solidity`` / ``morph_kernel``.  Anything it does not specify falls
        back to the constructor arguments, so the existing blue/green block
        behaviour is bit-for-bit unchanged while a small, thin, distant target
        such as a bottle can relax the filters.
        """
        cfg: Dict[str, object] = dict(profile)
        cfg.setdefault("min_area", self.min_area)
        cfg.setdefault("max_area", self.max_area)
        cfg.setdefault("min_fill", self.min_fill)
        cfg.setdefault("min_solidity", self.min_solidity)
        cfg["kernel"] = (
            self._kernel_for(profile["morph_kernel"])
            if "morph_kernel" in profile
            else self._kernel
        )
        return cfg

    def _mask_for(self, hsv: np.ndarray, profile: Dict[str, object]) -> np.ndarray:
        lower = np.array(profile["hsv_lower"], dtype=np.uint8)
        upper = np.array(profile["hsv_upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = profile.get("kernel", self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _contours_to_detections(
        self, label: str, contours: Sequence[np.ndarray], profile: Dict[str, object]
    ) -> List[Detection]:
        min_area = float(profile.get("min_area", self.min_area))
        max_area = float(profile.get("max_area", self.max_area))
        min_fill = float(profile.get("min_fill", self.min_fill))
        min_solidity = float(profile.get("min_solidity", self.min_solidity))

        found: List[Detection] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            if max_area > 0 and area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            if area / float(w * h) < min_fill:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            if hull_area > 0.0 and area / hull_area < min_solidity:
                continue

            found.append(
                Detection(
                    label=label,
                    score=round(min(1.0, area / float(w * h)), 3),
                    x1=float(x),
                    y1=float(y),
                    x2=float(x + w),
                    y2=float(y + h),
                    source="color",
                )
            )
        return found

    # -- public API --------------------------------------------------------

    def detect(self, bgr: np.ndarray) -> List[Detection]:
        """Return all colour-block detections in a BGR image."""
        if bgr is None or bgr.size == 0:
            return []

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        detections: List[Detection] = []
        for label, profile in self._profiles.items():
            mask = self._mask_for(hsv, profile)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            detections.extend(self._contours_to_detections(label, contours, profile))
        return detections

    def detect_by_edges(self, bgr: np.ndarray) -> List[Detection]:
        """Find a bottle from its **outline only** -- no colour, no shadow.

        A colourless transparent bottle carries almost no chromatic signal, so
        the HSV path is useless for it and YOLO only sees it when it is large
        enough to build up contrast.  What does survive all the way out is the
        silhouette: a closed, tall, moderately rounded contour.

        Pipeline: greyscale -> Gaussian blur -> Canny -> morphological close
        (bridges the gaps Canny leaves) -> contours -> shape filters on area,
        aspect ratio and circularity.

        ``score`` is the contour circularity ``4*pi*A/P**2`` (1.0 for a circle,
        ~0.5-0.6 for a bottle-shaped rectangle, near 0 for a thin sliver), so a
        higher score means a more compact, bottle-like blob.
        """
        if bgr is None or bgr.size == 0:
            return []

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blur = max(1, int(self.edge_blur))
        if blur % 2 == 0:
            blur += 1
        gray = cv2.GaussianBlur(gray, (blur, blur), 0)

        edges = cv2.Canny(gray, self.edge_canny_low, self.edge_canny_high)

        if self.edge_close_kernel > 1:
            edges = cv2.morphologyEx(
                edges, cv2.MORPH_CLOSE, self._kernel_for(self.edge_close_kernel)
            )

        if self.edge_fill:
            # Canny returns *open arcs* on a transparent bottle, so the raw
            # contours are thin slivers whose circularity is ~0.05 and every
            # one gets rejected.  Flooding each enclosed region turns the arcs
            # into the blob they bound, which is what the shape filters are
            # actually meant to judge.  Without this the layer detects nothing.
            seed, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            canvas = np.zeros_like(edges)
            cv2.drawContours(canvas, seed, -1, 255, thickness=-1)
            edges = canvas

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        found: List[Detection] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.edge_min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            # orientation independent: a bottle is tall whether it stands or lies
            aspect = max(w, h) / float(min(w, h))
            if not (self.edge_aspect_min <= aspect <= self.edge_aspect_max):
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self.edge_min_circularity:
                continue

            found.append(
                Detection(
                    label=BOTTLE_EDGE,
                    score=round(min(1.0, circularity), 3),
                    x1=float(x),
                    y1=float(y),
                    x2=float(x + w),
                    y2=float(y + h),
                    source="edge",
                )
            )
        return found

    def overlay_masks(self, bgr: np.ndarray) -> np.ndarray:
        """Debug helper: colour-code the masks on top of the image."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        canvas = bgr.copy()
        for label, profile in self._profiles.items():
            mask = self._mask_for(hsv, profile)
            layer = np.zeros_like(canvas)
            layer[:] = profile["bgr"]
            canvas = np.where(
                cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR).astype(bool), layer, canvas
            )
        return canvas
