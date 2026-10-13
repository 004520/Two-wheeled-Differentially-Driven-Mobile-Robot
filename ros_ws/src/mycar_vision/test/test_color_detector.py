"""Unit tests for the classical colour / outline detector.

Run with::

    cd ~/mycar_ws/src/mycar_vision
    python3 -m pytest test -q
"""

import cv2
import numpy as np

from mycar_vision.color_detector import BOTTLE_EDGE, ColorBlockDetector


def math_close(a, b, tol):
    return abs(a - b) <= tol


def _blue_block_frame():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[300:390, 60:150] = (230, 120, 30)   # blue block
    return frame


def _green_block_frame():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[320:420, 200:300] = (60, 200, 60)   # green block
    return frame


# ----------------------------------------------------------- colour path --
def test_detects_blue_and_green_blocks():
    detector = ColorBlockDetector(min_area=800)
    found = {d.label: d for d in detector.detect(_blue_block_frame())}
    assert "blue_block" in found
    assert math_close(found["blue_block"].width, 90, 3)


def test_ignores_colour_outside_profiles():
    # a red patch must not be picked up by the default blue/green profiles
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[300:390, 60:150] = (40, 40, 230)    # red
    assert ColorBlockDetector(min_area=800).detect(frame) == []


def test_rejects_small_colour_regions():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[10:14, 10:14] = (230, 120, 30)      # 16 px, below min_area
    assert ColorBlockDetector(min_area=800).detect(frame) == []


def test_green_block_detected():
    found = {d.label: d for d in ColorBlockDetector(min_area=800).detect(
        _green_block_frame())}
    assert "green_block" in found


# ----------------------------------------------------------- edge path --
def test_detect_by_edges_finds_tall_silhouette():
    """A tall rectangle (bottle-shaped) should be reported as BOTTLE_EDGE."""
    frame = np.full((480, 640, 3), 128, dtype=np.uint8)
    cv2.rectangle(frame, (300, 300), (324, 396), (200, 200, 200), -1)  # 24x96
    found = [d for d in ColorBlockDetector().detect_by_edges(frame)
             if d.label == BOTTLE_EDGE]
    assert found, "tall silhouette should be detected"
    # the largest found box should be roughly the 24x96 rectangle
    big = max(found, key=lambda d: d.area)
    assert math_close(big.height, 96, 6)
    assert math_close(big.width, 24, 6)


def test_detect_by_edges_ignores_blank_image():
    frame = np.full((480, 640, 3), 128, dtype=np.uint8)  # no edges at all
    assert ColorBlockDetector().detect_by_edges(frame) == []


def test_detect_by_edges_rejects_small_blob():
    frame = np.full((480, 640, 3), 128, dtype=np.uint8)
    cv2.circle(frame, (320, 240), 5, (200, 200, 200), -1)  # area ~78 < min_area
    found = [d for d in ColorBlockDetector().detect_by_edges(frame)
             if d.label == BOTTLE_EDGE]
    assert found == []


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-q"]))
