from setuptools import setup

package_name = 'lidar_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Esat Sefa Bayhan',
    maintainer_email='esat.bayhan@rub.de',
    description='Filter LaserScan from lidar to match bounding boxes of cones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "filter = lidar_filter.lidar_filter:main"
        ],
    },
)
