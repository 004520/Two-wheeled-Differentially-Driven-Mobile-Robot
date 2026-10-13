"""Bring up the vision stack with the extension nodes switched by flags.

    # perception only (default): detector + radar fusion
    ros2 launch mycar_vision vision_pipeline.launch.py

    # add the Nav2 adapter
    ros2 launch mycar_vision vision_pipeline.launch.py enable_costmap:=true

    # add the reactive avoider (writes /avoidance_cmd, never /cmd_vel)
    ros2 launch mycar_vision vision_pipeline.launch.py enable_avoidance:=true

    # browser view (http://<this host>:8081/) + a lower YOLO threshold
    ros2 launch mycar_vision vision_pipeline.launch.py \
        enable_web_stream:=true confidence:=0.25

Every node gets its own parameter file and can additionally be tuned by editing
the YAML files in ``share/mycar_vision/config``.  ``confidence`` is exposed as
a launch argument so the threshold can be swept without editing YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("mycar_vision")

    def config(name: str) -> str:
        return os.path.join(share, "config", name)

    detector_params = LaunchConfiguration("detector_params")
    fusion_params = LaunchConfiguration("fusion_params")
    avoidance_params = LaunchConfiguration("avoidance_params")
    costmap_params = LaunchConfiguration("costmap_params")
    web_params = LaunchConfiguration("web_params")

    enable_detector = LaunchConfiguration("enable_detector")
    enable_radar_fusion = LaunchConfiguration("enable_radar_fusion")
    enable_avoidance = LaunchConfiguration("enable_avoidance")
    enable_costmap = LaunchConfiguration("enable_costmap")
    enable_web_stream = LaunchConfiguration("enable_web_stream")
    confidence = LaunchConfiguration("confidence")
    detect_bottle_edges = LaunchConfiguration("detect_bottle_edges")
    roi_zoom_enabled = LaunchConfiguration("roi_zoom_enabled")

    detector = Node(
        package="mycar_vision",
        executable="vision_detector",
        name="vision_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            detector_params,
            # node-level override: wins over whatever the YAML says
            {"confidence": ParameterValue(confidence, value_type=float)},
            {
                "detect_bottle_edges": ParameterValue(
                    detect_bottle_edges, value_type=bool
                )
            },
            {
                "roi_zoom_enabled": ParameterValue(
                    roi_zoom_enabled, value_type=bool
                )
            },
        ],
        condition=IfCondition(enable_detector),
    )

    radar_fusion = Node(
        package="mycar_vision",
        executable="vision_radar_fusion",
        name="vision_radar_fusion",
        output="screen",
        emulate_tty=True,
        parameters=[fusion_params],
        condition=IfCondition(enable_radar_fusion),
    )

    avoidance = Node(
        package="mycar_vision",
        executable="avoidance_decider",
        name="avoidance_decider",
        output="screen",
        emulate_tty=True,
        parameters=[avoidance_params],
        condition=IfCondition(enable_avoidance),
    )

    costmap = Node(
        package="mycar_vision",
        executable="vision_to_costmap",
        name="vision_to_costmap",
        output="screen",
        emulate_tty=True,
        parameters=[costmap_params],
        condition=IfCondition(enable_costmap),
    )

    web_stream = Node(
        package="mycar_vision",
        executable="vision_web_stream",
        name="vision_web_stream",
        output="screen",
        emulate_tty=True,
        parameters=[web_params],
        condition=IfCondition(enable_web_stream),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_params",
                default_value=config("vision_detector.yaml"),
                description="Parameter file for vision_detector.",
            ),
            DeclareLaunchArgument(
                "fusion_params",
                default_value=config("vision_radar_fusion.yaml"),
                description="Parameter file for vision_radar_fusion.",
            ),
            DeclareLaunchArgument(
                "avoidance_params",
                default_value=config("avoidance_decider.yaml"),
                description="Parameter file for avoidance_decider.",
            ),
            DeclareLaunchArgument(
                "costmap_params",
                default_value=config("vision_to_costmap.yaml"),
                description="Parameter file for vision_to_costmap.",
            ),
            DeclareLaunchArgument(
                "web_params",
                default_value=config("vision_web_stream.yaml"),
                description="Parameter file for vision_web_stream.",
            ),
            DeclareLaunchArgument(
                "confidence",
                default_value="0.25",
                description=(
                    "YOLO confidence threshold; overrides vision_detector.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "detect_bottle_edges",
                default_value="false",
                description=(
                    "Outline-only bottle fallback for low-contrast / transparent "
                    "bottles; overrides vision_detector.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "roi_zoom_enabled",
                default_value="false",
                description=(
                    "Magnified-ROI second YOLO pass for distant bottles; "
                    "overrides vision_detector.yaml."
                ),
            ),
            DeclareLaunchArgument("enable_detector", default_value="true"),
            DeclareLaunchArgument("enable_radar_fusion", default_value="true"),
            DeclareLaunchArgument(
                "enable_avoidance",
                default_value="false",
                description="Reactive avoider -> /avoidance_cmd (does not touch /cmd_vel).",
            ),
            DeclareLaunchArgument(
                "enable_costmap",
                default_value="false",
                description="Publish LaserScan + PointCloud2 for the Nav2 obstacle layer.",
            ),
            DeclareLaunchArgument(
                "enable_web_stream",
                default_value="false",
                description="Serve the annotated stream in a browser (no Qt).",
            ),
            detector,
            radar_fusion,
            avoidance,
            costmap,
            web_stream,
        ]
    )
