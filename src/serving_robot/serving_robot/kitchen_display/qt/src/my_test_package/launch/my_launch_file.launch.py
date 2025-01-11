from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_test_package',
            executable='pub',  # pub.py 실행
            name='publisher',
            output='screen'
        ),
        Node(
            package='my_test_package',
            executable='kit',  # kit.py 실행
            name='subscriber',
            parameters=[{'labels': ['label_3', 'label_7', 'label_9', 'label_31', 'label_33', 'label_35', 'label_39', 'label_41', 'label_43', 'label_55', 'label_57', 'label_59', 'label_63', 'label_65', 'label_67', 'label_71', 'Label_73', 'label_75', 'label_79', 'label_81', 'label_83', 'label_87', 'label_89', 'label_91', 'label_95', 'label_97', 'label_99']}],
            output='screen'
        ),
    ])
