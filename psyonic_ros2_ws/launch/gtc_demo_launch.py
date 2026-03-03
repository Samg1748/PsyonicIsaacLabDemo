from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gtc_bridge',
            executable='gtc_bridge',
            name='bridge'
        ),
        Node(
            package='HIL_feedback',
            executable='HIL_feedback',
            name='feedback'
        ),
        Node(
            package='manus_haply_teleop',
            executable='manus_haply_teleop',
            name='teleop'
        )
    ])
