from setuptools import setup

package_name = "rudra_camera"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        (
            "share/" + package_name,
            ["package.xml"]
        ),
    ],
    install_requires=[
        "setuptools"
    ],
    zip_safe=True,
    description="Rudra SIYI A8 Mini RTSP camera ROS 2 node",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "rudra_camera_node = "
            "rudra_camera.rudra_camera_node:main",
        ],
    },
)
