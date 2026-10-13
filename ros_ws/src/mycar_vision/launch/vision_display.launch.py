"""Browser display for the annotated vision stream -- **no Qt involved**.

Two interchangeable back-ends, selected with ``server:=``:

``builtin`` (default)
    :mod:`mycar_vision.vision_web_stream`, shipped with this package.
    Pure Python standard library, no extra apt/pip dependency::

        ros2 launch mycar_vision vision_display.launch.py
        # -> http://<machine-ip>:8081/

``web_video_server``
    The upstream ROS package (``sudo apt install ros-humble-web-video-server``).
    Auto-discovers every image topic and also offers ``?type=ros_compressed``::

        ros2 launch mycar_vision vision_display.launch.py server:=web_video_server
        # -> http://<machine-ip>:8080/

``both``
    Runs them side by side, convenient when comparing.

This launch file only brings up the **display**. The detector is expected to be
running already (``vision_detector.launch.py`` or ``vision_pipeline.launch.py``);
set ``with_detector:=true`` to start it from here as well.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    server = LaunchConfiguration("server")
    port = LaunchConfiguration("port")
    host = LaunchConfiguration("host")
    fps = LaunchConfiguration("fps")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    image_topic = LaunchConfiguration("image_topic")
    video_port = LaunchConfiguration("video_server_port")
    with_detector = LaunchConfiguration("with_detector")

    # NOTE: PythonExpression interpolates the substitution as *raw text*, so the
    # value has to be wrapped in quotes here -- otherwise `server:=builtin`
    # would be evaluated as the bare name `builtin` and raise NameError.
    use_builtin = PythonExpression(["'", server, "' in ('builtin', 'both')"])
    use_upstream = PythonExpression(["'", server, "' in ('web_video_server', 'both')"])

    # 1) the self-written, dependency-free MJPEG server node
    builtin = Node(
        package="mycar_vision",
        executable="vision_web_stream",
        name="vision_web_stream",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_builtin),
        parameters=[
            {
                "image_topic": image_topic,
                "host": host,
                "port": port,
                "fps": fps,
                "jpeg_quality": jpeg_quality,
            }
        ],
    )

    # 2) the upstream C++ server (optional dependency)
    upstream = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_upstream),
        parameters=[
            {
                "port": video_port,
                "address": host,
                "default_stream_type": "ros_compressed",
                "width": 0,
                "height": 0,
                "quality": 80,
            }
        ],
    )

    # 3) optional detector, so the page shows something out of the box
    detector = Node(
        package="mycar_vision",
        executable="vision_detector",
        name="vision_detector",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(with_detector),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "server",
                default_value="builtin",
                choices=["builtin", "web_video_server", "both"],
                description="Which HTTP streaming back-end to run.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8081",
                description="Port of the built-in vision_web_stream server.",
            ),
            DeclareLaunchArgument(
                "video_server_port",
                default_value="8080",
                description="Port of the upstream web_video_server.",
            ),
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="Bind address; 0.0.0.0 exposes the page to the LAN.",
            ),
            DeclareLaunchArgument(
                "fps", default_value="15.0", description="MJPEG push rate cap."
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="75",
                description="Re-encode quality; >=95 forwards the incoming JPEG verbatim.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/vision/image/compressed",
                description="Annotated image topic published by vision_detector.",
            ),
            DeclareLaunchArgument(
                "with_detector",
                default_value="false",
                description="Also start vision_detector from this launch file.",
            ),
            builtin,
            upstream,
            detector,
        ]
    )
