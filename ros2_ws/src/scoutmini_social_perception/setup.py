from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'scoutmini_social_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'deps'), glob('deps/*.repos')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ScoutMini Maintainers',
    maintainer_email='nvidia@todo.todo',
    description='People detection, tracking, projection, and social navigation adapters for ScoutMini.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_people_detector = scoutmini_social_perception.fake_people_detector_node:main',
            'yolo_people_detector = scoutmini_social_perception.yolo_people_detector_node:main',
            'people_tracker = scoutmini_social_perception.people_tracker_node:main',
            'people_projection = scoutmini_social_perception.people_projection_node:main',
            'people_frame_transform = scoutmini_social_perception.people_frame_transform_node:main',
            'adascore_people_adapter = scoutmini_social_perception.adascore_people_adapter_node:main',
            'perception_benchmark = scoutmini_social_perception.perception_benchmark_node:main',
            'adascore_readiness_check = scoutmini_social_perception.adascore_readiness_check:main',
        ],
    },
)
