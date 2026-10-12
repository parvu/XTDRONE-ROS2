from glob import glob

from setuptools import setup

package_name = "xtdrone_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", [f for f in glob("config/*") if not f.endswith("/vins")]),
        ("share/" + package_name + "/config/vins", glob("config/vins/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="XTDrone SLAM chapters on ROS 2: laser SLAM (slam_toolbox, Cartographer) and pose transfer to PX4.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "laser_transfer = xtdrone_slam.laser_transfer:main",
            "odom_relay = xtdrone_slam.odom_relay:main",
        ],
    },
)
