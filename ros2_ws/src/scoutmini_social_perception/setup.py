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
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nhat Le',
    maintainer_email='nhatleminh1997@gmail.com',
    description='Insta360 person detection and panorama-aware tracking for ScoutMini.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_tracking_corpus = '
            'scoutmini_social_perception.corpus_manifest:main',
            'evaluate_tracking_bag = '
            'scoutmini_social_perception.tracking_evaluator:main',
            'run_tracking_corpus = '
            'scoutmini_social_perception.corpus_runner:main',
            'yolo_people_detector = '
            'scoutmini_social_perception.yolo_people_detector_node:main',
        ],
    },
)
