import pytest

from xtdrone_bringup import fleet


def test_vehicle_type_rules():
    assert fleet.vehicle_type_of("iris_stereo_camera") == "iris"
    assert fleet.vehicle_type_of("typhoon_h480") == "typhoon_h480"
    assert fleet.vehicle_type_of("standard_vtol") == "standard_vtol"
    assert fleet.vehicle_type_of("rover_with_lidar_stereo") == "rover"
    assert fleet.vehicle_type_of("tailsitter") == "tailsitter"
    assert fleet.VEHICLE_TYPES["tiltrotor"]["airframe"] == 4020
    with pytest.raises(ValueError, match="ugv"):
        fleet.vehicle_type_of("ugv_lidar")


def test_spec_numbering_and_ports():
    vs = fleet.parse_spec("iris_stereo_camera*3,plane")
    assert [v.name for v in vs] == ["iris_0", "iris_1", "iris_2", "plane_0"]
    assert [v.instance for v in vs] == [0, 1, 2, 3]
    assert vs[1].fcu_url == "udp://:24541@127.0.0.1:34581"
    assert vs[3].mav_sys_id == 4
    assert vs[3].airframe == 4003
    # vehicles do not overlap
    assert len({tuple(v.pose[:2]) for v in vs}) == 4


def test_more_than_ten_vehicles_get_distinct_ports():
    vs = fleet.parse_spec("iris*20")
    assert len({v.fcu_url for v in vs}) == 20


def test_scenario(tmp_path):
    f = tmp_path / "s.yaml"
    f.write_text("world: indoor1\nvehicles:\n  - {model: iris_stereo_camera, type: iris, id: 0, pose: [0, 7.5, 1]}\n")
    world, vs = fleet.load_scenario(f)
    assert world == "indoor1"
    assert vs[0].name == "iris_0" and vs[0].pose == [0, 7.5, 1, 0, 0, 0]


def test_bridge_config(tmp_path):
    d = tmp_path / "iris_2d_lidar"
    d.mkdir()
    (d / "xtdrone_bridge.yaml").write_text(
        "bridges:\n"
        "- gz_topic: /world/{world}/model/{name}/link/laser_2d/sensor/laser/scan\n"
        "  ros_topic: scan\n  ros_type: sensor_msgs/msg/LaserScan\n  gz_type: gz.msgs.LaserScan\n")
    v = fleet.Vehicle("iris_2d_lidar", "iris", 0)
    entries = fleet.bridge_config(tmp_path, "indoor3", v)
    assert entries[0]["gz_topic_name"] == "/world/indoor3/model/iris_0/link/laser_2d/sensor/laser/scan"
    assert entries[0]["ros_topic_name"] == "/iris_0/scan"
    assert entries[-1]["ros_topic_name"] == "/iris_0/ground_truth/odom"


def test_bridge_direction(tmp_path):
    d = tmp_path / "typhoon_h480"
    d.mkdir()
    (d / "xtdrone_bridge.yaml").write_text(
        "bridges:\n"
        "- gz_topic: /model/{name}/joint/cgo3_camera_joint/0/cmd_pos\n"
        "  ros_topic: gimbal/pitch_cmd\n  ros_type: std_msgs/msg/Float64\n  gz_type: gz.msgs.Double\n"
        "  direction: ROS_TO_GZ\n")
    v = fleet.Vehicle("typhoon_h480", "typhoon_h480", 2)
    e = fleet.bridge_config(tmp_path, "robocup", v)[0]
    assert e["direction"] == "ROS_TO_GZ" and "lazy" not in e
    assert e["gz_topic_name"] == "/model/typhoon_h480_2/joint/cgo3_camera_joint/0/cmd_pos"
    assert v.airframe == 4901
