from glob import glob
from setuptools import setup

package_name = "xtdrone_py"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XTDrone contributors",
    maintainer_email="zhuoan@stu.pku.edu.cn",
    description="XTDrone communication, keyboard control and utility nodes for ROS 2.",
    url="https://github.com/robin-shaun/XTDrone",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "multirotor_communication = xtdrone_py.multirotor_communication:main",
            "multirotor_communication_enhanced = xtdrone_py.multirotor_communication:main_enhanced",
            "plane_communication = xtdrone_py.plane_communication:main",
            "vtol_communication = xtdrone_py.vtol_communication:main",
            "rover_communication = xtdrone_py.rover_communication:main",
            "multirotor_keyboard_control = xtdrone_py.keyboard_control:multirotor_main",
            "plane_keyboard_control = xtdrone_py.keyboard_control:plane_main",
            "vtol_keyboard_control = xtdrone_py.keyboard_control:vtol_main",
            "rover_keyboard_control = xtdrone_py.keyboard_control:rover_main",
            "ugv_keyboard_control = xtdrone_py.keyboard_control:ugv_main",
            "ugv_communication = xtdrone_py.ugv_communication:main",
            "lane_detection = xtdrone_py.self_driving:detection_main",
            "self_driving = xtdrone_py.self_driving:driving_main",
            "control_actor = xtdrone_py.control_actor:main",
            "actor_walker = xtdrone_py.actor_walker:main",
            "yolo_detection = xtdrone_py.yolo:detection_main",
            "gimbal_control = xtdrone_py.yolo:gimbal_main",
            "yolo_human_tracking = xtdrone_py.yolo:tracking_main",
            "get_local_pose = xtdrone_py.get_local_pose:main",
            "precision_landing = xtdrone_py.precision_landing:main",
            "attitude_control_demo = xtdrone_py.attitude_control_demo:main",
            "control_gazebo_models = xtdrone_py.gazebo_models:models_main",
            "control_gazebo_vehicles = xtdrone_py.gazebo_models:vehicles_main",
            "contact_detect = xtdrone_py.contact_detect:main",
            "wind_control = xtdrone_py.wind_control:main",
        ],
    },
)
