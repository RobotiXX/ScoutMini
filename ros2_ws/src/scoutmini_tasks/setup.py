from glob import glob
from setuptools import setup, find_packages

package_name = 'scoutmini_tasks'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nle',
    maintainer_email='noreply@example.com',
    description='Task nodes for ScoutMini',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'route_loop_runner = scoutmini_tasks.route_loop_runner:main',
            'nav_to_pose_runner = scoutmini_tasks.nav_to_pose_runner:main',
            'door_aware_route_runner = scoutmini_tasks.door_aware_route_runner:main',
            'door_scan_filter = scoutmini_tasks.door_scan_filter:main',
        ],
    },
)
