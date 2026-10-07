from setuptools import find_packages, setup


package_name = 'rudra_survivor_manager'


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

    description=(
        'Rudra survivor identification and '
        'target assignment ROS 2 node'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'rudra_survivor_manager_node = '
            'rudra_survivor_manager.rudra_survivor_manager_node:main',
        ],
    },
)
