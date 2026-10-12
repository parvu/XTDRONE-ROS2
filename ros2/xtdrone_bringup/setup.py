from glob import glob
from setuptools import setup

package_name = "xtdrone_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config/scenarios", glob("config/scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="XTDrone launch files for PX4 SITL on Gazebo Harmonic with MAVROS 2.",
    url="https://github.com/robin-shaun/XTDrone",
    license="MIT",
    tests_require=["pytest"],
)
