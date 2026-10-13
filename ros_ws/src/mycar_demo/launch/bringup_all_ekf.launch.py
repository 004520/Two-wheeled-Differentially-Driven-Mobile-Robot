import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sudo_password = "mycar"

    ekf_config_path = os.path.join(
        get_package_share_directory('mycar_navigation2'), 'config', 'ekf.yaml')
    urdf_model_path = os.path.join(
        get_package_share_directory('mycar_description'), 'urdf', 'mycar.urdf')

    # 1. Socat 进程 (立即启动)
    socat_cmd = f'echo "{sudo_password}" | sudo -S socat -d -d PTY,link=/dev/lidar,raw,echo=0,mode=666 UDP4-LISTEN:8889,reuseaddr,fork'
    socat_process = ExecuteProcess(cmd=[socat_cmd], shell=True, output='screen')

    # 2. Micro-ROS Agent (立即启动)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '--port', '8888', '-v6'],
        output='screen'
    )

    # 3. Camsense 雷达 (延迟 5 秒启动，等待 /dev/lidar 生成)
    camsense_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('camsense_lidar'), 'launch', 'camsense_lidar.launch.py')
                )
            )
        ]
    )

    # 4. 车体静态 TF + EKF (延迟 8 秒启动，等待 Agent 与雷达就绪)
    #    注意：这里【不启动 mycar_bringup 节点】。
    #    该节点会广播 odom -> base_footprint，与 EKF 的 publish_tf 抢发，造成 TF 双父冲突。
    #    其唯一职责已由 ekf_filter_node 接管；robot_state_publisher / joint_state_publisher
    #    仍在此启动，保证 URDF 的静态 TF（base_footprint -> base_link -> laser_frame / imu_link）不受影响。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        output='screen'
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path],
        output='screen'
    )

    ekf_launch = TimerAction(
        period=8.0,
        actions=[
            joint_state_publisher_node,
            robot_state_publisher_node,
            ekf_filter_node,
        ]
    )

    return LaunchDescription([
        socat_process,
        micro_ros_agent,
        camsense_launch,
        ekf_launch
    ])
