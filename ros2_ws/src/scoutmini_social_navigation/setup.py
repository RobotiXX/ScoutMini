from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'scoutmini_social_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nhat Le',
    maintainer_email='nhatleminh1997@gmail.com',
    description='LiDAR range fusion and shadow-mode social navigation integration.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_restamper = '
            'scoutmini_social_navigation.odometry_restamp_node:main',
            'odometry_transform_publisher = '
            'scoutmini_social_navigation.odometry_tf_node:main',
            'pointcloud_restamper = '
            'scoutmini_social_navigation.pointcloud_restamp_node:main',
            'render_bag_analysis = '
            'scoutmini_social_navigation.bag_analysis_renderer:main',
            'scan_people_fusion = '
            'scoutmini_social_navigation.scan_people_fusion_node:main',
            'shadow_health_monitor = '
            'scoutmini_social_navigation.shadow_health_monitor_node:main',
            'shadow_path_driver = '
            'scoutmini_social_navigation.shadow_path_driver_node:main',
            'supervised_motion_gate = '
            'scoutmini_social_navigation.motion_gate_node:main',
        ],
    },
)
