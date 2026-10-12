from glob import glob
from setuptools import setup

package_name = "xtdrone_formation_demo"

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
        ("share/" + package_name + "/config", glob("config/*.txt")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="ROS 2 multi-UAV formation control demo for XTDrone.",
    url="https://github.com/robin-shaun/XTDrone",
    license="MIT",
    entry_points={
        "console_scripts": [
            "leader = xtdrone_formation_demo.leader:main",
            "follower = xtdrone_formation_demo.follower:main",
            "follower_consensus = xtdrone_formation_demo.follower_consensus:main",
            "avoid = xtdrone_formation_demo.avoid:main",
            "formation_comm_controller = xtdrone_formation_demo.comm_model:controller_main",
            "communication_verify = xtdrone_formation_demo.comm_model:verify_main",
            "uav_data_aggregator = xtdrone_formation_demo.comm_model:aggregator_main",
            "formation_comm_all = xtdrone_formation_demo.comm_model:all_main",
        ],
    },
)
