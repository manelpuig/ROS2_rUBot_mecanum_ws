from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_driver',
            executable='rubot_mecanum_driver_node',
            name='rubot_mecanum_driver_node',
            output='screen'
        )
    ])
