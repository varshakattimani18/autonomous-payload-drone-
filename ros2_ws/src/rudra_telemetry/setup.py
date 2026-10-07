from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rudra_telemetry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Varsha Kattimani',
    maintainer_email='varsha@example.com',
    description='Rudra MAVLink telemetry ROS 2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rudra_telemetry_node = rudra_telemetry.rudra_telemetry_node:main',
        ],
    },
)
