from glob import glob
from setuptools import setup

package_name = 'scoutmini_tasks'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
