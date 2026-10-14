#!/usr/bin/env python3
"""Generate the XTDrone UGV (catvehicle) models for Gazebo Harmonic (manual chapter 11).

The source is classic_assets/catvehicle/urdf/catvehicle{1-3,4-6}.xacro (the CAT Vehicle,
a Ford Escape with a SICK 2D lidar, a 3D block lidar, three cameras and an AprilTag
plate on the roof). The xacro is expanded with ROS 2 xacro and converted with
`gz sdf -p`; then the Gazebo Classic plugins are replaced:

  * gazebo_ros_control + the Simulink cmd_vel controller -> gz-sim AckermannSteering
    (gz topic /model/<name>/cmd_vel, linear.x = speed, angular.z = yaw rate; the
    xtdrone_py ugv_communication node converts the XTDrone steering-angle command);
  * gazebo_ros_camera / gazebo_ros_laser / block laser -> gz camera / gpu_lidar sensors,
    bridged to the ROS 1 topic names (xtdrone_bridge.yaml);
  * the car-body collision mesh -> a box (DART), the roof tag mesh -> the apriltag plate
    texture of the xtdrone_gz apriltag models;
  * OdometryPublisher for the ground truth, as on every XTDrone vehicle.

Models: ugv (tags 0-2), ugv_tags3_5 and ugv_nosensor (ugv without cameras and lidars). The meshes are copied from classic_assets at
build time (not committed).

Usage: generate_ugv_models.py [output_models_dir]
"""
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
SRC = Path(__file__).resolve().parents[1] / "classic_assets" / "catvehicle"
SCOPE = "/world/{world}/model/{name}/link/base_link/sensor/{sensor}"

VARIANTS = {  # model name: (xacro, tag plate model, sensors)
    "ugv": ("catvehicle1-3.xacro", "apriltag0-2", True),
    "ugv_tags3_5": ("catvehicle4-6.xacro", "apriltag3-5", True),
    # no cameras and lidars: rendered on the CPU they halve the real-time factor; for scenarios that only drive the
    # UGV (chapter 35)
    "ugv_nosensor": ("catvehicle1-3.xacro", "apriltag0-2", False),
}

WHEEL_BASE = 1.55 + 1.05          # tyre_front_x - tyre_back_x
WHEEL_SEPARATION = 2 * 0.765      # 2 * tyre_y
KINGPIN_WIDTH = 1.29 + 4 * 0.05   # car_width + 4 * str_length
WHEEL_RADIUS = 0.3671951254
STEERING_LIMIT = 0.6              # str_angle

# sensor name in the URDF -> (ROS topic relative to /ugv_N, ROS type, gz type, gz topic suffix)
SENSOR_BRIDGES = {
    "camera_left": [("camera_left/image_raw_left", "sensor_msgs/msg/Image", "gz.msgs.Image", "image"),
                    ("camera_left/camera_left_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
                     "camera_info")],
    "camera_right": [("camera_right/image_raw_right", "sensor_msgs/msg/Image", "gz.msgs.Image", "image"),
                     ("camera_right/camera_right_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
                      "camera_info")],
    "triclops": [("triclops/triclops/left/image", "sensor_msgs/msg/Image", "gz.msgs.Image", "image"),
                 ("triclops/camera_triclops_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo",
                  "camera_info")],
    "sick_lms291": [("front_laser_points", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "scan")],
    "lidar_sensor": [("lidar_points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked",
                      "scan/points")],
}
FRAMES = {"camera_left": "camera_left_link", "camera_right": "camera_right_link", "triclops": "triclops_link",
          "sick_lms291": "front_laser_link", "lidar_sensor": "velodyne_link"}


def xacro_to_sdf(xacro_name):
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for f in (SRC / "urdf").iterdir():
            if f.suffix in (".xacro", ".gazebo"):
                (tmp / f.name).write_text(f.read_text().replace("$(find catvehicle)", str(SRC)))
        urdf = subprocess.run(["xacro", str(tmp / xacro_name), "roboname:=UGV_NAME"], check=True,
                              capture_output=True, text=True).stdout
        urdf = urdf.replace("package://catvehicle/", "model://MODEL_NAME/")
        (tmp / "car.urdf").write_text(urdf)
        return subprocess.run(["gz", "sdf", "-p", str(tmp / "car.urdf")], check=True,
                              capture_output=True, text=True).stdout


def el(text):
    return ET.fromstring(text)


def build(name, xacro_name, tags, sensors, out):
    sdf = xacro_to_sdf(xacro_name).replace("MODEL_NAME", name)
    root = ET.fromstring(sdf)
    model = root.find("model")
    model.set("name", name)
    for p in model.findall("plugin"):
        model.remove(p)
    base = model.find("link[@name='base_link']")

    # car body collision mesh -> box (mesh bounds: x -2.03..2.40, y +-1.0, z -0.61..0.98 at z 1.064)
    for c in base.findall("collision"):
        if c.find("geometry/mesh") is not None and "ford_escape" in c.find("geometry/mesh/uri").text:
            c.find("pose").text = "0.185 0 1.247 0 0 0"
            geom = c.find("geometry")
            geom.remove(geom.find("mesh"))
            geom.append(el("<box><size>4.43 2.03 1.58</size></box>"))

    # roof tag mesh -> flat textured plate (2.5 m x 1.0 m, tags along x, first ID at -x)
    for v in base.findall("visual"):
        mesh = v.find("geometry/mesh")
        if mesh is not None and "apriltag" in mesh.find("uri").text:
            v.find("pose").text = "-0.55 0 2.045 0 0 0"
            geom = v.find("geometry")
            geom.remove(mesh)
            geom.append(el("<box><size>2.5 1.0 0.01</size></box>"))
            for m in v.findall("material"):
                v.remove(m)
            v.append(el(f"""<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
              <specular>0 0 0 1</specular><pbr><metal>
              <albedo_map>model://{tags}/materials/textures/{tags}.png</albedo_map>
              <roughness>1</roughness><metalness>0</metalness></metal></pbr></material>"""))

    bridges = []
    for s in base.findall("sensor"):
        if not sensors:
            base.remove(s)
            continue
        sname = s.get("name")
        for p in s.findall("plugin"):
            s.remove(p)
        if s.get("type") == "ray":
            s.set("type", "gpu_lidar")
            ray = s.find("ray")
            ray.tag = "lidar"
        for old in s.findall("visualize"):
            s.remove(old)
        s.append(el(f"<gz_frame_id>{FRAMES.get(sname, 'base_link')}</gz_frame_id>"))
        s.append(el("<always_on>1</always_on>"))
        for ros_topic, ros_type, gz_type, suffix in SENSOR_BRIDGES.get(sname, []):
            bridges.append({"gz_topic": SCOPE.format(world="{world}", name="{name}", sensor=sname) + "/" + suffix,
                            "ros_topic": ros_topic, "ros_type": ros_type, "gz_type": gz_type})

    # wheel friction (the URDF's <gazebo> mu values are Classic-only)
    for link in model.findall("link"):
        if link.get("name").endswith("wheel_link"):
            for c in link.findall("collision"):
                if c.find("surface") is None:
                    c.append(el("<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"))

    model.append(el(f"""<plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>back_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>back_right_wheel_joint</right_joint>
      <left_steering_joint>front_left_steering_joint</left_steering_joint>
      <right_steering_joint>front_right_steering_joint</right_steering_joint>
      <kingpin_width>{KINGPIN_WIDTH:.3f}</kingpin_width>
      <steering_limit>{STEERING_LIMIT}</steering_limit>
      <wheel_base>{WHEEL_BASE:.3f}</wheel_base>
      <wheel_separation>{WHEEL_SEPARATION:.3f}</wheel_separation>
      <wheel_radius>{WHEEL_RADIUS:.4f}</wheel_radius>
      <min_velocity>-20</min_velocity>
      <max_velocity>20</max_velocity>
      <min_acceleration>-5</min_acceleration>
      <max_acceleration>3</max_acceleration>
      <!-- its wheel odometry would share /model/<name>/odometry with the ground truth of OdometryPublisher -->
      <odom_topic>/xtdrone/ugv_wheel_odometry_unused</odom_topic>
      <tf_topic>/xtdrone/ugv_wheel_odometry_tf_unused</tf_topic>
      <odom_publish_frequency>1</odom_publish_frequency>
    </plugin>"""))
    model.append(el("""<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
      <!-- PX4's gz_bridge reads /model/<name>/odometry_with_covariance (the default topic) as external
           vision. This is ground truth for ROS only (/<name>/ground_truth/odom): keep it away from PX4. -->
      <odom_covariance_topic>/xtdrone/ground_truth_with_covariance_unused</odom_covariance_topic>
    </plugin>"""))
    bridges.append({"gz_topic": "/model/{name}/cmd_vel", "ros_topic": "ackermann_cmd_vel",
                    "ros_type": "geometry_msgs/msg/Twist", "gz_type": "gz.msgs.Twist", "direction": "ROS_TO_GZ"})

    d = out / name
    if d.exists():
        shutil.rmtree(d)
    (d / "meshes").mkdir(parents=True)
    for f in ("ford_escape_no_wheels_2.dae", "ford_escape_wheel.dae", "SickLMS.dae"):
        shutil.copy(SRC / "meshes" / f, d / "meshes" / f)
    for sub in ("ford_escape_no_wheels_2", "ford_escape_wheel"):
        shutil.copytree(SRC / "meshes" / sub, d / "meshes" / sub)
    ET.indent(root)
    text = ET.tostring(root, encoding="unicode")
    text = re.sub(r"UGV_NAME", "ugv", text)
    (d / "model.sdf").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        "<!-- Generated by xtdrone_gz/scripts/generate_ugv_models.py from "
        f"xtdrone_gz/classic_assets/catvehicle/urdf/{xacro_name}. Do not edit. -->\n" + text + "\n")
    (d / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <description>XTDrone UGV: CAT Vehicle (Ford Escape) with Ackermann steering, SICK 2D lidar, 3D lidar,
  three cameras and the {tags} AprilTag plate on the roof (manual chapter 11).</description>
</model>
""")
    (d / "xtdrone_bridge.yaml").write_text("# Generated by generate_ugv_models.py\n" +
                                          yaml.safe_dump({"bridges": bridges}, sort_keys=False))
    return d


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "models"
    for name, (xacro_name, tags, sensors) in VARIANTS.items():
        print("generated", build(name, xacro_name, tags, sensors, out))


if __name__ == "__main__":
    main()
