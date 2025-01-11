from setuptools import setup

package_name = 'my_test_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # 런치 파일 경로 추가 (중요!)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='your.email@example.com', # 이메일 수정
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = my_test_package.pub:main', # pub.py 실행 스크립트 (중요!)
            'kit = my_test_package.kit:main', # kit.py 실행 스크립트 (중요!)
            'publisher_node = my_test_package.publisher_node:main', # publisher_node.py 실행 스크립트 (필요한 경우)
        ],
    },
)
