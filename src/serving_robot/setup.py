from setuptools import find_packages, setup

package_name = 'serving_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunwolee',
    maintainer_email='128200788+malenwater@users.noreply.github.com',
    description='This package is for the serving robot service project using the TurtleBot from the Rocky Bootcamp',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
