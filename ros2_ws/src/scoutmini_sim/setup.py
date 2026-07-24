from glob import glob
from os.path import join
from setuptools import find_packages, setup

package_name = 'scoutmini_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (join('share', 'ament_index', 'resource_index', 'packages'),
            [join('resource', package_name)]),
        (join('share', package_name), ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*.py'))),
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
        (join('share', package_name, 'worlds'), glob(join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan-wang',
    maintainer_email='ryan.wang.2022.me@gmail.com',
    description='Simulation launch and world assets for Scout Mini',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'door_slider = scoutmini_sim.door_slider:main',
            'elevator_door_sliders = scoutmini_sim.elevator_door_sliders:main',
        ],
    },
)
