"""Fixed-wing formation tracking a UGV (chapter 35), the ROS 2 version of multi_uav_sim_ba_3vtol.sh.

Starts, per plane, pack_fw_states (MAVROS <-> FwStates/FwCmd) and task_main (mode state machine and formation
controller), and vir_sim_leader for the leader. The simulation comes first:
  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor2_fw_formation
  ros2 launch xtdrone_fixed_wing_formation fixed_wing_formation.launch.py
Take off, transition to fixed-wing, then start the formation per plane with switch_fw_mode
(formation_type:=1 north-south line, 2 triangle, 3 east-west line).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "xtdrone_fixed_wing_formation"


def launch_setup(context):
    planes = int(LaunchConfiguration("planes").perform(context))
    prefix = LaunchConfiguration("vehicle_prefix").perform(context)
    common = {"vehicle_prefix": prefix, "use_sim_time": True}
    pack = {"spawn_x": float(LaunchConfiguration("leader_x").perform(context)),
            "spawn_y": float(LaunchConfiguration("leader_y").perform(context)),
            "loiter_altitude": float(LaunchConfiguration("loiter_altitude").perform(context)),
            "ugv_odom_topic": LaunchConfiguration("ugv_odom_topic").perform(context)}
    actions = []
    for i in range(planes):
        name = f"{prefix}{i}"
        actions.append(Node(package=PKG, executable="pack_fw_states", name=f"pack_fw_states_{i}", output="screen",
                            parameters=[common, pack, {"plane_id": i}]))
        actions.append(Node(package=PKG, executable="task_main", name=f"task_main_{i}", output="screen",
                            parameters=[common, {"plane_id": i, "leader_id": 0}]))
    actions.append(Node(package=PKG, executable="vir_sim_leader", name="vir_sim_leader", output="screen",
                        parameters=[common, {"leader_id": 0}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planes", default_value="3", description="Number of planes (0 is the leader)."),
        DeclareLaunchArgument("vehicle_prefix", default_value="standard_vtol_"),
        DeclareLaunchArgument("leader_x", default_value="-600", description="Spawn x of the leader."),
        DeclareLaunchArgument("leader_y", default_value="0", description="Spawn y of the leader."),
        DeclareLaunchArgument("loiter_altitude", default_value="50", description="Leader altitude over the UGV."),
        DeclareLaunchArgument("ugv_odom_topic", default_value="/ugv_0/ground_truth/odom"),
        OpaqueFunction(function=launch_setup),
    ])
