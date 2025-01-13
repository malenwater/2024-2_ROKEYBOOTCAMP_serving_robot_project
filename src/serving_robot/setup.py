from setuptools import find_packages, setup
import glob
import os

package_name = 'serving_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools', 'PyQt5', 'rclpy', 'mysql-connector-python', 'playsound','dotenv'],
    zip_safe=True,
    maintainer='sunwolee',
    maintainer_email='128200788+malenwater@users.noreply.github.com',
    description='This package is for the serving robot service project using the TurtleBot from the Rocky Bootcamp',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pub = serving_robot.test_node.publish_node_v2:main',
            'test_sub = serving_robot.test_node.subscribe_node_v2:main',
            'test_server = serving_robot.kitchen_display.kit_copy:main',
            'test_client = serving_robot.kitchen_display.pub:main',
            'serving_robot_ui = serving_robot.database.ui_tab:main',
            'kiosk_ui = serving_robot.kiosk.kiosk_ui:main',
            'robot_sub = serving_robot.kit_to_robot.control :main',
            'robot_pub = serving_robot.kit_to_robot.publish :main',
        ],
    },
)
