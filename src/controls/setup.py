import os
from glob import glob

from setuptools import setup

package_name = 'controls'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Esat Sefa Bayhan',
    maintainer_email='esat.bayhan@rub.de',
    description='Controls the turtlebot based on the avai stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = controls.controller:main",
            "heartbeat = controls.heartbeat:main"
        ],
    },
)
