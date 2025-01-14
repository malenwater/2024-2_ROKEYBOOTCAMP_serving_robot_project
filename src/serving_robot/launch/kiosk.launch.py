from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

     # 기본 로그 레벨을 INFO로 설정하지만, 환경 변수에서 가져올 수 있도록 함
    log_level_ui = os.environ.get('KIOSK_UI_LOG_LEVEL', 'INFO').upper()
    log_level_subscriber = os.environ.get('SUBSCRIBER_LOG_LEVEL', 'INFO').upper()
    
    

    

    return LaunchDescription([
        Node(
            package='serving_robot',
            executable='kiosk_ui',
            name='kiosk_ui_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level_ui]  # 로그 레벨 설정
        ),
        Node(
            package='serving_robot',
            executable='test_client',
            name='client_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level_subscriber]
        ),
       
         Node(
            package='serving_robot',
            executable='test_server',
            name='kitchen_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level_subscriber]
        ),

        Node(
            package='serving_robot',
            executable='robot_sub',
            name='navigation_subscriber',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level_subscriber]
        ),

    ])

