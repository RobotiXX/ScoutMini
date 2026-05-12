import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_tools'


def collect_data_files(source_dir: str, install_base: str):
    data_files = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, source_dir)
        install_dir = install_base if rel_dir == '.' else os.path.join(install_base, rel_dir)
        file_paths = [os.path.join(root, f) for f in sorted(files)]
        data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/waypoints', glob('waypoints/*')),
    ] + collect_data_files('maps', 'share/' + package_name + '/maps')
      + collect_data_files('srv', 'share/' + package_name + '/srv'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhat-m-le',
    maintainer_email='nhatleminh1997@gmail.com',
    description='Tools for collecting named waypoints from RViz clicked points.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_name_publisher = map_tools.map_name_publisher:main',
            'waypoint_server = map_tools.waypoint_server:main',
            'waypoint_collector = map_tools.waypoint_collector:main',
            'apriltag_tag_collector = map_tools.apriltag_tag_collector:main',
        ],
    },
)
