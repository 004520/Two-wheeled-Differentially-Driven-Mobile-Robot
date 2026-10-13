from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share_dir = get_package_share_directory("camsense_lidar")
    parameter_file = LaunchConfiguration("params_file")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "params", "camsense_lidar.yaml"),
        description="Path to the ROS2 parameters file to use.",
    )

    driver_node = Node(
        package="camsense_lidar",
        executable="camsense_lidar_node",
        name="camsense_lidar_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file],
        namespace="/",
    )

    # NOTE: the static transform base_link -> laser_frame is already provided by
    # the URDF (mycar_description/urdf/mycar.urdf, laser_joint: z = +0.075).
    # The static_transform_publisher that used to be created here duplicated that
    # transform (and even used the wrong height z = 0.02), so it was removed to
    # keep exactly one publisher of base_link -> laser_frame.

    return LaunchDescription([
        params_declare,
        driver_node,
    ])
