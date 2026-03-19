from setuptools import find_packages, setup
from glob import glob

package_name = 'scoutmini_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/**/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/meshes', glob('meshes/**/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nle',
    maintainer_email='nhatleminh1997@gmail.com',
    description='Robot description package for RobotiXX Scout Mini',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
