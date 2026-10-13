import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sudo_password = "mycar"
    
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

    # 3. YDLIDAR (延迟 5 秒启动，等待 /dev/lidar 生成)
    ydlidar_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
                )
            )
        ]
    )

    # 4. Mycar (延迟 7 秒启动，确保雷达已经占用了串口)
    mycar_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mycar_bringup'), 'launch', 'mycar_bringup.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        socat_process,
        micro_ros_agent,
        ydlidar_launch,
        mycar_launch
    ])
