from setuptools import find_packages, setup

package_name = 'rudra_payload'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    description='Rudra payload control ROS 2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rudra_payload_node = rudra_payload.rudra_payload_node:main',
        ],
    },
)
