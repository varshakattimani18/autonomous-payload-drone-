from setuptools import find_packages, setup


package_name = 'rudra_tracking'


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

    description='Rudra Hungarian ground tracking ROS 2 node',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'rudra_tracking_node = rudra_tracking.rudra_tracking_node:main',
        ],
    },
)
