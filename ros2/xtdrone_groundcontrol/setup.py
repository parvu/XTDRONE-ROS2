from glob import glob

from setuptools import setup

package_name = "xtdrone_groundcontrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["images/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/images", glob(package_name + "/images/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="XTDGroundControl: PyQt5 ground station for XTDrone (ROS 2 port).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "xtdgroundcontrol = xtdrone_groundcontrol.ground_control:main",
        ],
    },
)
