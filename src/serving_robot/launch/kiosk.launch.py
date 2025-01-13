from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_robot',
            executable='kiosk_ui',
            name='kiosk_ui_node',
            output='screen'
        ),
        Node(
            package='serving_robot',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        ),
    ])

