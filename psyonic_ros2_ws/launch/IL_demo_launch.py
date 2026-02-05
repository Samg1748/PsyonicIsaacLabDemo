from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_listener',
            namespace='hand_listener',
            executable='hand_listener',
            name='hand'
        ),
        Node(
            package='arm_listener',
            namespace='arm_listener',
            executable='arm_listener',
            name='arm'
        )
    ])