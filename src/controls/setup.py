from setuptools import setup

package_name = 'controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='esat.bayhan@rub.de',
    description='Controls the turtlebot based on the avai stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = controls.controller:main"
        ],
    },
)
