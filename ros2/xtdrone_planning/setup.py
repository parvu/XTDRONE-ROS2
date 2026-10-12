from glob import glob

from setuptools import setup

package_name = "xtdrone_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="XTDrone motion planning chapters on ROS 2 (Nav2 2D planning).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ego_transfer = xtdrone_planning.ego:transfer_main",
            "ego_pos_cmd = xtdrone_planning.ego:pos_cmd_main",
        ],
    },
)
