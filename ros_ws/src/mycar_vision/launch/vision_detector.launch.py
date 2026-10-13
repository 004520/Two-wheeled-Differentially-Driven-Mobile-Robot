"""Launch ``vision_detector`` with optional command-line parameter overrides.

Examples
--------
    ros2 launch mycar_vision vision_detector.launch.py
    ros2 launch mycar_vision vision_detector.launch.py confidence:=0.4
    ros2 launch mycar_vision vision_detector.launch.py \
        image_topic:=/camera/image_raw/compressed image_size:=640
    ros2 launch mycar_vision vision_detector.launch.py \
        params_file:=/path/to/my_vision_detector.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: (launch-argument name, python type) -- "" / unset means "keep the YAML value"
_STRING_OVERRIDES = (
    "model",
    "model_dir",
    "image_topic",
    "detection_topic",
    "annotated_topic",
    "device",
    "frame_id",
)
_INT_OVERRIDES = (
    "image_size",
    "person_class_id",
    "bottle_class_id",
    "color_min_area",
    "annotated_quality",
    "edge_min_area",
    "edge_canny_low",
    "edge_canny_high",
    "edge_blur",
    "edge_close_kernel",
)
_FLOAT_OVERRIDES = (
    "confidence",
    "iou",
    "bottle_containment_ratio",
    "roi_zoom",
    "roi_center_x",
    "roi_center_y",
    "edge_aspect_min",
    "edge_aspect_max",
    "edge_min_circularity",
)
_BOOL_OVERRIDES = (
    "publish_annotated",
    "detect_colored_blocks",
    "roi_zoom_enabled",
    "detect_bottle_edges",
    "edge_fill",
)


def _collect_overrides(context) -> dict:
    overrides = {}

    for name in _STRING_OVERRIDES:
        value = context.perform_substitution(LaunchConfiguration(name)).strip()
        if value:
            overrides[name] = value

    for name in _INT_OVERRIDES:
        value = context.perform_substitution(LaunchConfiguration(name)).strip()
        if value:
            try:
                overrides[name] = int(value)
            except ValueError:
                pass

    for name in _FLOAT_OVERRIDES:
        value = context.perform_substitution(LaunchConfiguration(name)).strip()
        if value:
            try:
                overrides[name] = float(value)
            except ValueError:
                pass

    for name in _BOOL_OVERRIDES:
        value = context.perform_substitution(LaunchConfiguration(name)).strip().lower()
        if value:
            overrides[name] = value in ("1", "true", "yes", "on")

    return overrides


def _launch_setup(context, *args, **kwargs):
    params_file = context.perform_substitution(LaunchConfiguration("params_file"))
    parameters = [params_file]
    overrides = _collect_overrides(context)
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="mycar_vision",
            executable="vision_detector",
            name="vision_detector",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("mycar_vision"), "config", "vision_detector.yaml"
    )

    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file with vision_detector parameters.",
        )
    ]
    descriptions = {
        "model": ("", "YOLO weights file name or absolute path (default yolo11n.pt)."),
        "model_dir": ("", "Directory to look for the weights in."),
        "image_topic": ("", "CompressedImage input topic."),
        "detection_topic": ("", "Detection2DArray output topic."),
        "annotated_topic": ("", "Annotated CompressedImage output topic."),
        "device": ("", "Ultralytics device string, e.g. cpu or 0. Empty = auto."),
        "frame_id": ("", "Force header.frame_id on the outputs."),
        "image_size": ("", "YOLO imgsz (640 by default; empty keeps the YAML value)."),
        "person_class_id": ("", "COCO class id for person (0)."),
        "bottle_class_id": ("", "COCO class id for bottle (39)."),
        "color_min_area": ("", "Minimum colour-block area in pixels (800)."),
        "annotated_quality": ("", "JPEG quality for the annotated stream (70)."),
        "confidence": ("", "YOLO confidence threshold (0.5)."),
        "iou": ("", "YOLO NMS IoU threshold (0.45)."),
        "bottle_containment_ratio": ("", "Overlap ratio that suppresses a green block."),
        "roi_zoom_enabled": ("", "Second YOLO pass on a magnified ROI (false)."),
        "roi_zoom": ("", "ROI magnification factor k (2.0); coverage is 1/k^2."),
        "roi_center_x": ("", "Horizontal centre of the ROI, 0..1 (0.5)."),
        "roi_center_y": ("", "Vertical centre of the ROI, 0..1 (0.75)."),
        "detect_bottle_edges": ("", "Outline-only transparent-bottle layer (false)."),
        "edge_min_area": ("", "Minimum outline contour area in px (200)."),
        "edge_aspect_min": ("", "Minimum outline aspect ratio (1.5)."),
        "edge_aspect_max": ("", "Maximum outline aspect ratio (4.0)."),
        "edge_min_circularity": ("", "Minimum outline circularity (0.3)."),
        "edge_canny_low": ("", "Canny low threshold (50)."),
        "edge_canny_high": ("", "Canny high threshold (150)."),
        "edge_blur": ("", "Gaussian blur kernel before Canny (5)."),
        "edge_close_kernel": ("", "Morphological close kernel on the edges (5)."),
        "edge_fill": ("", "Flood enclosed Canny regions before filtering (true)."),
        "publish_annotated": ("", "Publish the annotated image (true)."),
        "detect_colored_blocks": ("", "Enable the classical colour detector (true)."),
    }
    for name, (default, help_text) in descriptions.items():
        arguments.append(
            DeclareLaunchArgument(name, default_value=default, description=help_text)
        )

    arguments.append(OpaqueFunction(function=_launch_setup))
    return LaunchDescription(arguments)
