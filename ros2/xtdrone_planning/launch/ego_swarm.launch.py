"""EGO-Planner-Swarm for the XTDrone ego_swarm scenarios (chapter 22).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=ego_swarm ekf:=vision communication:=basic
  ros2 run xtdrone_py get_local_pose iris 4        # ground truth as the pose source (one world frame)
  ros2 launch xtdrone_planning ego_swarm.launch.py

One ego_planner.launch.py per vehicle (planner, trajectory server, ego_transfer, ego_pos_cmd). The planners
share their trajectories on /broadcast_bspline and avoid each other, as in EGO-Planner-Swarm (without its
sequential start, see below). The goals are
those of XTDrone's ego_swarm_goal.sh (4 iris) and ego_swarm_goal_8.sh (8 iris): across the obstacle field to
x = 9, at 0.7 m. goals:="x,y,z;x,y,z;..." sets others; each vehicle also takes goals on /<v>/goal_pose
(RViz 2D Goal Pose with that topic) when goal_mode:=rviz.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

GOALS = {
    4: [(9, 5, 0.7), (9, 1.2, 0.7), (9, -1.2, 0.7), (9, -4, 0.7)],
    8: [(9, 7, 0.7), (9, 5, 0.7), (9, 3, 0.7), (9, 1, 0.7), (9, -1, 0.7), (9, -3, 0.7), (9, -5, 0.7), (9, -7, 0.7)],
}


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    count = int(arg("count"))
    goals = ([tuple(float(v) for v in g.split(",")) for g in arg("goals").split(";")] if arg("goals")
             else GOALS[count])
    single = str(Path(get_package_share_directory("xtdrone_planning")) / "launch" / "ego_planner.launch.py")
    out = []
    # EGO-Planner-Swarm's sequential start (each planner waits for the trajectories of drone_id - 1, sent once)
    # is off: on a loaded machine that hand-over was often lost and left vehicles waiting. The planners avoid
    # each other through the trajectories they broadcast on every replan (/broadcast_bspline). They still start
    # a few seconds apart, drone 0 last, which keeps sequential_start:=true usable.
    for i, (x, y, z) in enumerate(goals[:count]):
        out.append(TimerAction(period=float(arg("stagger")) * (count - 1 - i), actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(single), launch_arguments={
            "vehicle_type": arg("vehicle_type"), "id": str(i), "goal": arg("goal_mode"),
            "goal_topic": f"/{arg('vehicle_type')}_{i}/goal_pose", "sequential_start": arg("sequential_start"),
            "target_x": str(x), "target_y": str(y), "target_z": str(z),
            "max_vel": arg("max_vel"), "max_acc": arg("max_acc"), "ceiling": arg("ceiling"),
            "floor": arg("floor"), "inflation": arg("inflation"),
            "min_z": "0.0", "start_delay": "0.0"}.items())]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_type", default_value="iris"),
        DeclareLaunchArgument("count", default_value="4", description="4 (ego_swarm) or 8 (ego_swarm_8)"),
        DeclareLaunchArgument("goals", default_value="", description="x,y,z;x,y,z;... (default: XTDrone's)"),
        DeclareLaunchArgument("goal_mode", default_value="waypoint", description="waypoint or rviz"),
        DeclareLaunchArgument("max_vel", default_value="1.0"), DeclareLaunchArgument("max_acc", default_value="2.0"),
        DeclareLaunchArgument("ceiling", default_value="3.0"), DeclareLaunchArgument("floor", default_value="-1.0"),
        DeclareLaunchArgument("inflation", default_value="0.2"),
        DeclareLaunchArgument("stagger", default_value="4.0", description="seconds between planner starts"),
        DeclareLaunchArgument("sequential_start", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
