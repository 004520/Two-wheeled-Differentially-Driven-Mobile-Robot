"""Unit tests for the ROS-free helpers.

Run with::

    cd ~/mycar_ws/src/mycar_vision
    python3 -m pytest test -q
"""

import math

import numpy as np

from mycar_vision.color_detector import ColorBlockDetector
from mycar_vision.detection_utils import (
    BOTTLE,
    BLUE_BLOCK,
    GREEN_BLOCK,
    Detection,
    MonoCameraModel,
    containment,
    intersection_area,
    iou,
    suppress_blocks_in_containers,
)


def _box(label, x1, y1, x2, y2, score=1.0):
    return Detection(label=label, score=score, x1=x1, y1=y1, x2=x2, y2=y2)


# ---------------------------------------------------------------- geometry --
def test_intersection_and_iou():
    a = _box("a", 0, 0, 10, 10)
    b = _box("b", 5, 5, 15, 15)
    assert intersection_area(a, b) == 25.0
    assert math.isclose(containment(a, b), 0.25)
    assert math.isclose(iou(a, b), 25.0 / 175.0)


def test_no_intersection():
    a = _box("a", 0, 0, 10, 10)
    b = _box("b", 20, 20, 30, 30)
    assert intersection_area(a, b) == 0.0
    assert iou(a, b) == 0.0


# ------------------------------------------------------- bottle-label guard --
def test_green_label_inside_bottle_is_suppressed():
    """A small green patch in the middle of a bottle must NOT be a block."""
    bottle = _box(BOTTLE, 100, 100, 200, 400)
    green_label = _box(GREEN_BLOCK, 130, 220, 170, 280)
    kept, suppressed = suppress_blocks_in_containers([green_label], [bottle])
    assert kept == []
    assert suppressed == [green_label]


def test_green_block_swallowed_by_bottle_box_is_suppressed():
    """A green bottle body fully covered by the YOLO box is also suppressed."""
    bottle = _box(BOTTLE, 100, 100, 200, 400)
    green_body = _box(GREEN_BLOCK, 105, 110, 195, 390)
    kept, suppressed = suppress_blocks_in_containers([green_body], [bottle])
    assert kept == []
    assert suppressed == [green_body]


def test_green_block_beside_bottle_is_kept():
    block = _box(GREEN_BLOCK, 300, 300, 360, 360)
    bottle = _box(BOTTLE, 100, 100, 200, 400)
    kept, suppressed = suppress_blocks_in_containers([block], [bottle])
    assert kept == [block]
    assert suppressed == []


def test_blue_block_unaffected_without_bottles():
    blue = _box(BLUE_BLOCK, 10, 10, 90, 90)
    kept, suppressed = suppress_blocks_in_containers([blue], [])
    assert kept == [blue]
    assert suppressed == []


# --------------------------------------------------------- monocular model --
def test_mono_distance_scales_inversely_with_height():
    camera = MonoCameraModel(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    near = camera.distance_from_pixel_height(200.0, 1.6)
    far = camera.distance_from_pixel_height(100.0, 1.6)
    assert math.isclose(near, 4.0)
    assert math.isclose(far, 8.0)
    assert camera.distance_from_pixel_height(0.0, 1.6) is None


def test_mono_bearing_sign():
    camera = MonoCameraModel(fx=500.0, cx=320.0)
    assert camera.bearing(320.0) == 0.0
    assert camera.bearing(520.0) > 0.0
    assert camera.bearing(120.0) < 0.0


# --------------------------------------------------------- colour detector --
def test_color_detector_finds_coloured_blocks():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[300:390, 60:150] = (230, 120, 30)   # blue
    frame[320:420, 200:300] = (60, 200, 60)   # green

    detector = ColorBlockDetector(min_area=800)
    found = {d.label: d for d in detector.detect(frame)}

    assert BLUE_BLOCK in found
    assert GREEN_BLOCK in found
    assert math.isclose(found[BLUE_BLOCK].width, 90, abs_tol=2)
    assert math.isclose(found[GREEN_BLOCK].height, 100, abs_tol=2)


def test_color_detector_rejects_small_regions():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[10:14, 10:14] = (230, 120, 30)  # 16 px, below min_area
    detector = ColorBlockDetector(min_area=800)
    assert detector.detect(frame) == []


def test_end_to_end_fusion_keeps_block_but_drops_label():
    """Colour detections + YOLO boxes -> final fused list."""
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    frame[250:340, 380:470] = (60, 200, 60)  # a real green block on the floor
    frame[200:300, 100:200] = (60, 200, 60)  # the green label of a bottle

    detector = ColorBlockDetector(min_area=800)
    blocks = detector.detect(frame)
    assert len(blocks) == 2

    # Pretend YOLO saw the bottle around the label.
    bottle = _box(BOTTLE, 90, 190, 210, 400)
    kept, suppressed = suppress_blocks_in_containers(blocks, [bottle])

    assert len(kept) == 1
    assert len(suppressed) == 1
    assert kept[0].x1 > 300  # the floor block survived


# ----------------------------------------------------- detection container --
def test_detection_properties():
    d = _box("a", 10, 20, 30, 60)
    assert d.width == 20.0
    assert d.height == 40.0
    assert d.area == 800.0
    assert d.center == (20.0, 40.0)


def test_clamp_keeps_box_inside_image():
    d = Detection(label="a", score=1.0, x1=-5, y1=100, x2=400, y2=900)
    d.clamp(640, 480)
    assert d.x1 == 0.0
    assert d.y1 == 100.0
    assert d.x2 == 399.0          # width-1 clamp
    assert d.y2 == 479.0
    assert d.width == 399.0 and d.height == 379.0


def test_expand_box_grows_on_all_sides():
    d = _box("a", 10, 10, 20, 20)
    grown = expand_box(d, 0.5)
    assert grown.x1 == 5.0 and grown.x2 == 25.0
    assert grown.y1 == 5.0 and grown.y2 == 25.0


def test_point_inside():
    d = _box("a", 0, 0, 10, 10)
    assert point_inside(5, 5, d)
    assert not point_inside(15, 5, d)


# --------------------------------------------------------- monocular model --
def test_mono_bottom_edge_distance():
    """Z from the gap between the box bottom and a known ground row."""
    camera = MonoCameraModel(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    # box bottom at v=440, ground at v=480 -> 40 px of "height" on a 0.2 m object
    z = camera.distance_from_bottom_edge(440.0, 480.0, 0.2)
    assert math.isclose(z, 2.5)            # 500*0.2/40
    assert camera.distance_from_bottom_edge(440.0, 440.0, 0.2) is None  # zero gap


def test_mono_estimate_prefers_bottom_edge():
    camera = MonoCameraModel(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    det = Detection(label="bottle", score=1.0, x1=0, y1=0, x2=10, y2=40)
    z_pixel = camera.estimate(det, 0.2, use_bottom_edge=False)
    assert math.isclose(z_pixel, 500 * 0.2 / 40.0)
    z_edge = camera.estimate(det, 0.2, use_bottom_edge=True, ground_v=60.0)
    assert math.isclose(z_edge, 500 * 0.2 / 20.0)


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-q"]))
