from glob import glob
from setuptools import setup

package_name = "xtdrone_px4_gz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="ROS 2 launch integration for PX4 SITL with Gazebo Garden.",
    url="https://github.com/robin-shaun/XTDrone",
    license="MIT",
)
