from glob import glob
from setuptools import find_packages, setup

package_name = 'scoutmini_slack'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/systemd', glob('systemd/*.service')),
    ],
    install_requires=['setuptools', 'slack_sdk>=3,<4'],
    zip_safe=True,
    maintainer='nle',
    maintainer_email='nhatleminh1997@gmail.com',
    description='Slack integration nodes and scripts for ScoutMini',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'slack_gateway = scoutmini_slack.slack_gateway:main',
            'slack_diagnostics = scoutmini_slack.slack_diagnostics:main',
            'slack_status_poster = scoutmini_slack.slack_status_poster:main',
        ],
    },
)
