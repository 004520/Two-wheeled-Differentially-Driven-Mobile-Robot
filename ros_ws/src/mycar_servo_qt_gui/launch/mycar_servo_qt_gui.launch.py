from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mycar_servo_qt_gui",
                executable="mycar_servo_qt_gui",
                name="mycar_servo_qt_gui",
                output="screen",
                parameters=[{"servo_topic": "/servo_cmd"}],
            )
        ]
    )
