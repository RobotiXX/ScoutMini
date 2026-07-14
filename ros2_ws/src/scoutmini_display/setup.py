from setuptools import find_packages, setup

package_name = 'scoutmini_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', [
            'config/status_monitor.yaml',
            'config/50-scoutmini-network.rules',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nle',
    maintainer_email='nhatleminh1997@gmail.com',
    description='Beginner-friendly ROS 2 + PyQt display examples for ScoutMini',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Add new run targets here as: '<command_name> = scoutmini_display.<module_name>:main'
            'example_publisher = scoutmini_display.example_publisher:main',
            'example_subscriber_display = scoutmini_display.example_subscriber_display:main',
            'example_no_ros_hello_world = scoutmini_display.example_no_ros_hello_world:main',
            'example_no_ros_multi_page = scoutmini_display.example_no_ros_multi_page:main',
            'robot_dashboard = scoutmini_display.robot_dashboard:main',
        ],
    },
)
