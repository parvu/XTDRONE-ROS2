#!/usr/bin/env python3
"""Generate the XTDrone vehicle models for Gazebo Harmonic.

Each XTDrone vehicle (iris, iris_stereo_camera, plane, ...) becomes a model that
merges a PX4 gz airframe model and adds the XTDrone sensor kit. Next to each
model.sdf a xtdrone_bridge.yaml lists the gz -> ROS 2 topic pairs; the
xtdrone_bringup launch files read it to configure ros_gz_bridge per vehicle.

Usage: generate_vehicle_models.py [output_models_dir]
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]

SCOPE = "/world/{world}/model/{name}/link/{link}/sensor/{sensor}"

# ---------------------------------------------------------------------------
# Sensor kits. Each returns (sdf_snippet, bridge_entries).
# Bridge entries: (gz_topic_suffix_template, ros_topic_relative, ros_type, gz_type)
# ---------------------------------------------------------------------------


def camera_sensor(name, pose, width, height, hfov, rate, frame, near=0.1, far=300):
    return f"""
      <sensor name="{name}" type="camera">
        <pose>{pose}</pose>
        <always_on>1</always_on>
        <update_rate>{rate}</update_rate>
        <gz_frame_id>{frame}</gz_frame_id>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>{near}</near><far>{far}</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>"""


def image_bridge(link, sensor, ros_prefix):
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor=sensor)
    return [
        (scope + "/image", ros_prefix + "/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (scope + "/camera_info", ros_prefix + "/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
    ]


def sensor_link(link, pose, sensors, mass=0.015):
    return f"""
    <link name="{link}">
      <pose relative_to="base_link">{pose}</pose>
      <inertial><mass>{mass}</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="{link}_visual">
        <geometry><box><size>0.02 0.08 0.02</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>{sensors}
    </link>
    <joint name="{link}_joint" type="fixed">
      <parent>base_link</parent>
      <child>{link}</child>
    </joint>"""


def kit_stereo_camera():
    link = "stereo_camera_link"
    sensors = camera_sensor("left", "0 0.06 0 0 0 0", 752, 480, 1.5708, 30, "stereo_camera_frame")
    sensors += camera_sensor("right", "0 -0.06 0 0 0 0", 752, 480, 1.5708, 30, "stereo_camera_frame")
    sdf = sensor_link(link, "0.12 0 0.03 0 0 0", sensors)
    bridges = image_bridge(link, "left", "stereo_camera/left") + image_bridge(link, "right", "stereo_camera/right")
    return sdf, bridges


def kit_front_stereo():
    """Front stereo camera of rover_with_lidar_stereo (Classic: front_cam, tilted 0.4 rad down on the
    Polaris). PX4's r1_rover is much smaller, so it sits on the front of the chassis."""
    link = "front_cam"
    sensors = camera_sensor("left", "0 0.06 0 0 0 0", 752, 480, 1.5708, 30, "stereo_camera_frame")
    sensors += camera_sensor("right", "0 -0.06 0 0 0 0", 752, 480, 1.5708, 30, "stereo_camera_frame")
    sdf = sensor_link(link, "0.25 0 0.3 0 0.3 0", sensors)
    bridges = image_bridge(link, "left", "front_cam/left") + image_bridge(link, "right", "front_cam/right")
    return sdf, bridges


def kit_rover_3d_lidar():
    link = "laser_3d"
    sensors = lidar_sensor("lidar_3d", "laser_3d", 15, 512, -3.1415927, 3.1415927, 32, -0.2617994, 0.2617994, 0.3, 30)
    sdf = sensor_link(link, "0 0 0.4 0 0 0", sensors)
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="lidar_3d")
    return sdf, [(scope + "/scan/points", "velodyne_points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked")]


def kit_realsense_camera():
    link = "realsense_link"
    sensors = f"""
      <sensor name="depth_camera" type="rgbd_camera">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <gz_frame_id>depth_camera_base</gz_frame_id>
        <camera>
          <horizontal_fov>1.047198</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>20</far></clip>
          <depth_camera><clip><near>0.1</near><far>10</far></clip></depth_camera>
        </camera>
      </sensor>"""
    sdf = sensor_link(link, "0.12 0 0.03 0 0 0", sensors)
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="depth_camera")
    prefix = "realsense/depth_camera"
    bridges = [
        (scope + "/image", prefix + "/color/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (scope + "/camera_info", prefix + "/color/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        (scope + "/depth_image", prefix + "/depth/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (scope + "/points", prefix + "/depth/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
    ]
    return sdf, bridges


def kit_downward_camera():
    link = "camera_link"
    sensors = camera_sensor("camera", "0 0 0 0 0 0", 1280, 720, 2.0944, 30, "camera_link")
    sdf = sensor_link(link, "0 0 -0.08 0 1.5707963 0", sensors)
    return sdf, image_bridge(link, "camera", "camera")


def lidar_sensor(name, frame, rate, hsamples, hmin, hmax, vsamples, vmin, vmax, rmin, rmax):
    return f"""
      <sensor name="{name}" type="gpu_lidar">
        <always_on>1</always_on>
        <update_rate>{rate}</update_rate>
        <gz_frame_id>{frame}</gz_frame_id>
        <lidar>
          <scan>
            <horizontal><samples>{hsamples}</samples><resolution>1</resolution><min_angle>{hmin}</min_angle><max_angle>{hmax}</max_angle></horizontal>
            <vertical><samples>{vsamples}</samples><resolution>1</resolution><min_angle>{vmin}</min_angle><max_angle>{vmax}</max_angle></vertical>
          </scan>
          <range><min>{rmin}</min><max>{rmax}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>"""


def kit_2d_lidar():
    link = "laser_2d"
    sensors = lidar_sensor("laser", "laser_2d", 40, 512, -3.14, 3.14, 1, 0, 0, 0.1, 20)
    sdf = sensor_link(link, "0 0 0.1 0 0 0", sensors)
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="laser")
    # Downward 1D range finder (laser_rangefinder in the Classic model, ROS topic "distance"):
    # the height source of the laser SLAM chapters (laser_transfer).
    # Pitched 1.5 rad (86 deg), not 90 deg: a gpu_lidar looking straight down is a degenerate render
    # camera (view direction parallel to its up vector) and measures wrong ranges; cos(4 deg) = 0.9975.
    sdf += sensor_link("laser_1d", "0 0 -0.08 0 1.5 0",
                       # gpu_lidar renders through a camera: a near-zero field of view (Classic: one ray,
                       # +-0.001 rad) gives garbage, so a narrow fan of 5 rays (+-0.01 rad) is used
                       lidar_sensor("laser_1d", "laser_1d", 50, 5, -0.01, 0.01, 1, 0, 0, 0.1, 20))
    scope_1d = SCOPE.format(world="{world}", name="{name}", link="laser_1d", sensor="laser_1d")
    return sdf, [(scope + "/scan", "scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
                 (scope_1d + "/scan", "distance", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan")]


def kit_3d_lidar():
    link = "laser_3d"
    sensors = lidar_sensor("lidar_3d", "laser_3d", 15, 512, -3.1415927, 3.1415927, 32, -0.2617994, 0.2617994, 0.3, 30)
    sdf = sensor_link(link, "0 0 0.12 0 0 0", sensors)
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="lidar_3d")
    return sdf, [(scope + "/scan/points", "velodyne_points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked")]


def kit_imu_gazebo():
    # Reuse the PX4 airframe IMU on base_link (250 Hz) as XTDrone's imu_gazebo.
    scope = SCOPE.format(world="{world}", name="{name}", link="base_link", sensor="imu_sensor")
    return "", [(scope + "/imu", "imu_gazebo", "sensor_msgs/msg/Imu", "gz.msgs.IMU")]


def kit_airspeed():
    # A gz air_speed sensor where PX4's gz_bridge reads it (link airspeed_link, sensor air_speed). PX4's own
    # airspeed simulation (SENS_EN_ARSPDSIM) derives it from the ground truth that gz_bridge takes from the world's
    # pose/info and stalled with several VTOLs on a loaded machine; the transitions then timed out. Use with
    # SENS_EN_ARSPDSIM=0 (xtdrone_bringup does so for these models).
    sdf = """
    <link name="airspeed_link">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <inertial><mass>0.015</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="air_speed" type="air_speed">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <air_speed><pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure></air_speed>
      </sensor>
    </link>
    <joint name="airspeed_joint" type="fixed">
      <parent>base_link</parent>
      <child>airspeed_link</child>
    </joint>"""
    return sdf, []


def kit_bumper(size="0.65 0.65 0.31", z=-0.072):
    # Like XTDrone's bumper_sensor_iris: a light box around the frame whose
    # contacts are reported (a gz contact sensor only sees its own link).
    # For the x500 frame it spans from the leg bottoms (0.227 m below base_link)
    # to above the body, so a landed vehicle reports a ground contact as before.
    link = "bumper_link"
    sdf = f"""
    <link name="{link}">
      <pose relative_to="base_link">0 0 {z} 0 0 0</pose>
      <inertial><mass>0.01</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="bumper_collision">
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <sensor name="bumper" type="contact">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <contact><collision>bumper_collision</collision></contact>
      </sensor>
    </link>
    <joint name="{link}_joint" type="fixed">
      <parent>base_link</parent>
      <child>{link}</child>
    </joint>"""
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="bumper")
    return sdf, [(scope + "/contact", "benchmarker/collision", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts")]


KITS = {
    "stereo_camera": kit_stereo_camera,
    "realsense_camera": kit_realsense_camera,
    "downward_camera": kit_downward_camera,
    "2d_lidar": kit_2d_lidar,
    "front_stereo": kit_front_stereo,
    "rover_3d_lidar": kit_rover_3d_lidar,
    "3d_lidar": kit_3d_lidar,
    "imu_gazebo": kit_imu_gazebo,
    "bumper": kit_bumper,
    "airspeed": kit_airspeed,
}

# ---------------------------------------------------------------------------
# Vehicles: XTDrone model name -> (PX4 gz base model, sensor kits)
# ---------------------------------------------------------------------------
QUAD = "iris_ardupilot"  # generate_ardupilot_base.py (PX4 branch: "x500")
VEHICLES = {
    "iris": (QUAD, ["imu_gazebo", "bumper"]),
    "iris_stereo_camera": (QUAD, ["stereo_camera", "imu_gazebo", "bumper"]),
    "iris_realsense_camera": (QUAD, ["realsense_camera", "imu_gazebo", "bumper"]),
    "iris_downward_camera": (QUAD, ["downward_camera", "imu_gazebo", "bumper"]),
    "iris_2d_lidar": (QUAD, ["2d_lidar", "imu_gazebo", "bumper"]),
    "iris_3d_lidar": (QUAD, ["3d_lidar", "stereo_camera", "imu_gazebo", "bumper"]),
    "iris_3d_gpu_lidar": (QUAD, ["3d_lidar", "stereo_camera", "imu_gazebo", "bumper"]),
    "solo": (QUAD, ["imu_gazebo", "bumper"]),
    "solo_stereo_camera": (QUAD, ["stereo_camera", "imu_gazebo", "bumper"]),
    "plane": ("rc_cessna", []),
    # Wrappers must not share the name of the PX4 model they include (model:// would resolve to itself).
    "standard_vtol_xtd": ("standard_vtol", ["airspeed"]),
    "rover": ("r1_rover", []),
    "rover_with_lidar_stereo": ("r1_rover", ["front_stereo", "rover_3d_lidar"]),
    # PX4 v1.16 Gazebo VTOLs (PX4's quad tailsitter differs from XTDrone's Classic duo tailsitter).
    "tailsitter": ("quadtailsitter", []),
    "tiltrotor_xtd": ("tiltrotor", []),
}

ODOMETRY = """
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
      <!-- PX4's gz_bridge reads /model/<name>/odometry_with_covariance (the default topic) as external
           vision. This is ground truth for ROS only (/<name>/ground_truth/odom): keep it away from PX4. -->
      <odom_covariance_topic>/xtdrone/ground_truth_with_covariance_unused</odom_covariance_topic>
    </plugin>"""


def ardupilot_copter_plugin():
    """ArduPilot SITL link of a quadrotor (ardupilot_gazebo's iris_with_ardupilot). spawn time sets the FDM port of the
    SITL instance (sim.py): 9002 + 10 * instance."""
    controls = "".join(f"""
      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName>
        <useForce>1</useForce>
        <multiplier>{838 if i < 2 else -838}</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>""" for i in range(4))
    return f"""
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>{controls}
    </plugin>"""


def model_sdf(name, base, kits):
    snippets, bridges = [], []
    for kit in kits:
        sdf, br = KITS[kit]()
        snippets.append(sdf)
        bridges += br
    body = "".join(snippets)
    if base == QUAD:
        body += ardupilot_copter_plugin()
    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by xtdrone_gz/scripts/generate_vehicle_models.py. Do not edit. -->
<sdf version="1.9">
  <model name="{name}">
    <include merge="true">
      <uri>model://{base}</uri>
    </include>{body}{ODOMETRY}
  </model>
</sdf>
"""
    return sdf, bridges


# ---------------------------------------------------------------------------
# Typhoon H480 hexacopter: converted from the XTDrone Gazebo Classic model.
# PX4 v1.16 has no gz hexacopter, so an airframe is generated as well, with the
# control allocation geometry taken from the rotor link poses.
# ---------------------------------------------------------------------------
TYPHOON_AIRFRAME_ID = 4901
CLASSIC_COLORS = {"Gazebo/DarkGrey": "0.175 0.175 0.175 1", "Gazebo/Blue": "0 0 1 1"}

PX4_GZ_MODELS = REPO / "PX4-Autopilot/Tools/simulation/gz/models"


def px4_base_sensors():
    """The sensors PX4's gz_bridge reads (IMU, barometer, magnetometer, NavSat), copied
    from base_link of the pinned PX4's x500_base so they match the PX4 version."""
    src = PX4_GZ_MODELS / "x500_base/model.sdf"
    base = ET.parse(src).getroot().find("model/link[@name='base_link']")
    sensors = base.findall("sensor")
    names = {x.get("name") for x in sensors}
    for required in ("imu_sensor", "air_pressure_sensor", "magnetometer_sensor", "navsat_sensor"):
        if required not in names:
            raise SystemExit(f"{src}: base_link has no sensor '{required}'; PX4 gz_bridge changed?")
    return sensors


def _pose(el):
    p = el.find("pose")
    vals = [float(v) for v in p.text.split()] if p is not None and p.text else []
    return vals + [0.0] * (6 - len(vals))


def generate_typhoon(out, airframe_dir):
    src_dir = Path(__file__).resolve().parents[1] / "classic_assets/models/typhoon_h480"
    root = ET.parse(src_dir / "typhoon_h480.sdf").getroot()
    root.set("version", "1.9")
    model = root.find("model")
    model.set("name", "typhoon_h480")

    motors = []
    for plugin in model.findall("plugin"):
        if plugin.get("filename") == "libgazebo_motor_model.so":
            motors.append({k: (plugin.findtext(k) or "").strip() for k in (
                "jointName", "linkName", "turningDirection", "timeConstantUp", "timeConstantDown",
                "maxRotVelocity", "motorConstant", "momentConstant", "rotorDragCoefficient",
                "rollingMomentCoefficient", "rotorVelocitySlowdownSim")})
        model.remove(plugin)
    # Classic extras that have no gz equivalent here: included sonar/bumper models, IMU link.
    for inc in model.findall("include"):
        model.remove(inc)
    for el in list(model):
        name = el.get("name", "")
        if el.tag == "joint" and name in ("sonar_joint", "bumper_joint", "typhoon_h480/imu_joint"):
            model.remove(el)
        elif el.tag == "link" and name == "typhoon_h480/imu_link":
            model.remove(el)
    for j in model.findall("joint"):
        if j.get("name") in ("left_leg_joint", "right_leg_joint", "cgo3_mount_joint"):
            j.set("type", "fixed")  # no landing gear / mount controller in gz
            for axis in j.findall("axis"):
                j.remove(axis)
    for link in model.iter("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.get("type") == "imu":
                link.remove(sensor)
            elif sensor.get("type") == "camera":
                for pl in sensor.findall("plugin"):
                    sensor.remove(pl)
                ET.SubElement(sensor, "gz_frame_id").text = "cgo3_camera_optical_frame"
    for mat in model.iter("material"):
        script = mat.find("script")
        if script is not None:
            color = CLASSIC_COLORS.get((script.findtext("name") or "").strip(), "0.5 0.5 0.5 1")
            mat.remove(script)
            ET.SubElement(mat, "ambient").text = color
            ET.SubElement(mat, "diffuse").text = color
    base = model.find("link[@name='base_link']")
    for el in px4_base_sensors():
        base.append(el)

    # Motors, numbered in rotor link order; record the geometry for the airframe.
    links = {l.get("name"): l for l in model.findall("link")}
    motors.sort(key=lambda m: m["linkName"])
    rotors = []
    for i, m in enumerate(motors):
        x, y = _pose(links[m["linkName"]])[:2]
        rotors.append((x, -y, 0.05 if m["turningDirection"] == "ccw" else -0.05))
        model.append(ET.fromstring(f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>{m['jointName']}</jointName>
      <linkName>{m['linkName']}</linkName>
      <turningDirection>{m['turningDirection']}</turningDirection>
      <timeConstantUp>{m['timeConstantUp']}</timeConstantUp>
      <timeConstantDown>{m['timeConstantDown']}</timeConstantDown>
      <maxRotVelocity>{m['maxRotVelocity']}</maxRotVelocity>
      <motorConstant>{m['motorConstant']}</motorConstant>
      <momentConstant>{m['momentConstant']}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>{m['rotorDragCoefficient']}</rotorDragCoefficient>
      <rollingMomentCoefficient>{m['rollingMomentCoefficient']}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>{m['rotorVelocitySlowdownSim']}</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""))
    # Gimbal: yaw / roll / pitch joints held by position controllers.
    for joint in ("cgo3_vertical_arm_joint", "cgo3_horizontal_arm_joint", "cgo3_camera_joint"):
        model.append(ET.fromstring(f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>5</p_gain>
      <cmd_max>3</cmd_max>
      <cmd_min>-3</cmd_min>
    </plugin>"""))
    bumper_sdf, bumper_bridges = kit_bumper(size="0.7 0.7 0.35", z=-0.085)
    for el in ET.fromstring(f"<x>{bumper_sdf}</x>"):
        model.append(el)
    model.append(ET.fromstring(ODOMETRY))

    d = out / "typhoon_h480"
    if d.exists():
        shutil.rmtree(d)
    shutil.copytree(src_dir / "meshes", d / "meshes")
    ET.indent(root)
    (d / "model.sdf").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        "<!-- Generated by xtdrone_gz/scripts/generate_vehicle_models.py from "
        "xtdrone_gz/classic_assets/models/typhoon_h480. Do not edit. -->\n" + ET.tostring(root, encoding="unicode") + "\n")
    (d / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>typhoon_h480</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.sdf</sdf>\n  <description>Yuneec Typhoon H480 hexacopter with CGO3 gimbal '
        "camera (XTDrone, converted for Gazebo Harmonic).</description>\n</model>\n")
    link = "cgo3_camera_link"
    scope = SCOPE.format(world="{world}", name="{name}", link=link, sensor="camera")
    bridges = [
        (scope + "/image", "cgo3_camera/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (scope + "/camera_info", "cgo3_camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
    ] + kit_imu_gazebo()[1] + bumper_bridges
    gimbal = [{"gz_topic": "/model/{name}/joint/%s/0/cmd_pos" % j, "ros_topic": "gimbal/%s_cmd" % axis,
               "ros_type": "std_msgs/msg/Float64", "gz_type": "gz.msgs.Double", "direction": "ROS_TO_GZ"}
              for axis, j in (("yaw", "cgo3_vertical_arm_joint"), ("roll", "cgo3_horizontal_arm_joint"),
                              ("pitch", "cgo3_camera_joint"))]
    spec = {"base_model": "typhoon_h480 (converted)",
            "bridges": [{"gz_topic": g, "ros_topic": r, "ros_type": rt, "gz_type": gt} for g, r, rt, gt in bridges]
            + gimbal}
    (d / "xtdrone_bridge.yaml").write_text("# Generated by generate_vehicle_models.py\n"
                                           + yaml.safe_dump(spec, sort_keys=False))
    write_typhoon_airframe(airframe_dir, rotors)
    print(f"generated {d}")


def write_typhoon_airframe(airframe_dir, rotors):
    lines = ["#!/bin/sh", "#", "# @name XTDrone Gazebo Typhoon H480 (hexarotor x)", "#", "# @type Hexarotor x", "#",
             "# Generated by xtdrone_gz/scripts/generate_vehicle_models.py; geometry from the model rotor poses.",
             "", ". ${R}etc/init.d/rc.mc_defaults", "",
             "PX4_SIMULATOR=${PX4_SIMULATOR:=gz}", "PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}",
             "PX4_SIM_MODEL=${PX4_SIM_MODEL:=typhoon_h480}", "",
             "param set-default SIM_GZ_EN 1", "",
             "param set-default MAV_TYPE 13",
             "param set-default MC_PITCHRATE_P 0.08", "param set-default MC_PITCHRATE_I 0.04",
             "param set-default MC_PITCHRATE_D 0.001", "param set-default MC_PITCH_P 9",
             "param set-default MC_ROLLRATE_P 0.08", "param set-default MC_ROLLRATE_I 0.04",
             "param set-default MC_ROLLRATE_D 0.001", "param set-default MC_ROLL_P 9",
             "param set-default RTL_DESCEND_ALT 10", "",
             "param set-default CA_AIRFRAME 0", f"param set-default CA_ROTOR_COUNT {len(rotors)}"]
    for i, (px, py, km) in enumerate(rotors):
        lines += [f"param set-default CA_ROTOR{i}_PX {px:.4f}", f"param set-default CA_ROTOR{i}_PY {py:.4f}",
                  f"param set-default CA_ROTOR{i}_KM {km:.2f}"]
    lines.append("")
    for i in range(len(rotors)):
        lines += [f"param set-default SIM_GZ_EC_FUNC{i + 1} {101 + i}",
                  f"param set-default SIM_GZ_EC_MIN{i + 1} 150", f"param set-default SIM_GZ_EC_MAX{i + 1} 1500"]
    lines += ["", "param set-default MPC_THR_HOVER 0.40", "param set-default NAV_DLL_ACT 2", ""]
    airframe_dir.mkdir(parents=True, exist_ok=True)
    (airframe_dir / f"{TYPHOON_AIRFRAME_ID}_gz_typhoon_h480").write_text("\n".join(lines))


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "models"
    for name, (base, _) in VEHICLES.items():
        if name == base:
            raise SystemExit(f"model '{name}' would include itself (model://{base}); give the wrapper another name")
    for name, (base, kits) in VEHICLES.items():
        if base != QUAD:  # planes, VTOLs and rovers: ported in later steps of the ArduPilot branch
            print(f"skipped {name} (base {base}, not ported to ArduPilot yet)")
            continue
        sdf, bridges = model_sdf(name, base, kits)
        d = out / name
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.sdf").write_text(sdf)
        (d / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>XTDrone</name></author>
  <description>XTDrone {name} for Gazebo Harmonic (PX4 base model: {base}).</description>
</model>
""")
        spec = {
            "base_model": base,
            "bridges": [
                {"gz_topic": gz, "ros_topic": ros, "ros_type": rt, "gz_type": gt}
                for gz, ros, rt, gt in bridges
            ],
        }
        (d / "xtdrone_bridge.yaml").write_text(
            "# Generated by generate_vehicle_models.py\n" + yaml.safe_dump(spec, sort_keys=False)
        )
        print(f"generated {d}")
    # generate_typhoon(...): PX4 airframe; no ArduPilot version yet


if __name__ == "__main__":
    main()
