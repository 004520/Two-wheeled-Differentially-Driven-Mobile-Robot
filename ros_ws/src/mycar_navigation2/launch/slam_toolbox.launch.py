"""Launch slam_toolbox in online-async mapping mode, with an optional RViz view.

Deliberately mirrors upstream ``slam_toolbox/launch/online_async_launch.py``
with four differences:

* ``use_sim_time`` defaults to **false**.  Upstream defaults to true (Gazebo);
  this robot runs on a real clock.  slam_toolbox is started standalone here, so
  it is *not* covered by the ``RewrittenYaml`` rewrite that nav2_bringup
  applies to the Nav2 nodes -- it has to be set explicitly.
* the parameter file defaults to this package's own
  ``config/slam_toolbox.yaml``.
* RViz starts **by default** together with slam_toolbox, loading this package's
  own ``rviz/mapping.rviz`` (Fixed Frame ``map``, ``/map``, ``/scan``, TF,
  RobotModel).  Turn it off with ``rviz:=false``, or load a different file with
  ``rviz_config:=/abs/path/to/other.rviz``.
* RViz and slam_toolbox share the same ``use_sim_time``.

Expected TF tree while this is running (nothing here publishes odom or the
static body frames, those come from ``bringup_all_ekf.launch.py``)::

    map -> odom                    slam_toolbox      (this file)
    odom -> base_footprint         ekf_filter_node   (EKF, publish_tf: true)
    base_footprint -> base_link    robot_state_publisher (URDF)
    base_link -> laser_frame       robot_state_publisher (URDF)

so ``map -> odom`` has exactly one publisher: slam_toolbox.

Usage::

    ros2 launch mycar_navigation2 slam_toolbox.launch.py
    ros2 launch mycar_navigation2 slam_toolbox.launch.py rviz:=false
    ros2 launch mycar_navigation2 slam_toolbox.launch.py rviz_config:=/abs/path/x.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mycar_navigation2")
    default_params = os.path.join(pkg_dir, "config", "slam_toolbox.yaml")
    default_rviz = os.path.join(pkg_dir, "rviz", "mapping.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock (false for the real robot)",
    )
    declare_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_params,
        description="Full path to the slam_toolbox parameter file",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz together with slam_toolbox (default: on)",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to the RViz config used when rviz:=true",
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            # node-level override: wins over whatever the YAML says
            {"use_sim_time": use_sim_time},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_rviz,
        declare_rviz_config,
        start_async_slam_toolbox_node,
        rviz_node,
    ])
