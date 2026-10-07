from setuptools import find_packages, setup

package_name = 'rudra_log'

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
    maintainer='varsha',
    maintainer_email='varsha@example.com',
    description='Rudra mission logging node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rudra_log_node = rudra_log.rudra_log_node:main',
        ],
    },
)
