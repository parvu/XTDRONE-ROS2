"""EGO-Planner for one XTDrone multirotor with a depth camera (chapter 21).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1_ego ekf:=vision communication:=basic
  ros2 run xtdrone_py get_local_pose iris 1        # ground truth as the pose source (or VIO, chapter 14)
  ros2 launch xtdrone_planning ego_planner.launch.py target_x:=0 target_y:=-4 target_z:=1.2

The EGO-Planner ROS 2 port (submodule ros2/ego-planner-swarm, branch ros2_version) with the
parameters of its advanced_param.launch.py and of XTDrone's run_in_xtdrone.launch, wired to the
XTDrone topics: odometry from MAVROS (ego_transfer, reliable QoS), depth from the RealSense camera of
iris_realsense_camera, camera pose from ego_transfer, and the trajectory server's position command
to /xtdrone/<v>/cmd_pose_enu (ego_pos_cmd). Take off first (keyboard, about 1.5 m, then close it).
The planner and trajectory server run on simulation time; scripts/build_xtdrone_ros2.sh patches the port
for it (xtdrone_planning/scripts/patch_ego_planner.py), as ROS 1 used ros::Time with use_sim_time.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    vtype, vid = arg("vehicle_type"), arg("id")
    v = f"{vtype}_{vid}"
    f = float
    max_vel, max_acc, horizon = f(arg("max_vel")), f(arg("max_acc")), 7.5
    # The planner's map starts empty. With start_delay > 0 (waypoint goals) the planner waits for /traj_start_trigger
    # (its "realworld_experiment" mode) and this launch sends it that many seconds later, so the depth images have
    # built the map before the first trajectory; without it the first one ran straight through the columns in about
    # one run of ten and the vehicle ended against them.
    start_delay = f(arg("start_delay")) if arg("goal") == "waypoint" else 0.0
    waypoints = {"fsm/waypoint_num": 1, "fsm/waypoint0_x": f(arg("target_x")), "fsm/waypoint0_y": f(arg("target_y")),
                 "fsm/waypoint0_z": f(arg("target_z"))}
    for i in range(1, 5):
        waypoints.update({f"fsm/waypoint{i}_x": 0.0, f"fsm/waypoint{i}_y": 0.0, f"fsm/waypoint{i}_z": 1.0})
    params = {
        "fsm/flight_type": 2 if arg("goal") == "waypoint" else 1, "fsm/thresh_replan_time": 1.0,
        "fsm/thresh_no_replan_meter": 1.0, "fsm/planning_horizon": horizon, "fsm/planning_horizen_time": 3.0,
        "fsm/emergency_time": 1.0, "fsm/realworld_experiment": start_delay > 0, "fsm/fail_safe": True, **waypoints,
        "fsm/sequential_start": arg("sequential_start").lower() in ("1", "true", "yes"),
        "grid_map/resolution": 0.1, "grid_map/map_size_x": f(arg("map_size_x")),
        "grid_map/map_size_y": f(arg("map_size_y")), "grid_map/map_size_z": f(arg("map_size_z")),
        "grid_map/local_update_range_x": 5.5, "grid_map/local_update_range_y": 5.5,
        "grid_map/local_update_range_z": 4.5,
        # EGO plans for a point and the inflation stands for the airframe. Inflation, ground height and
        # virtual ceiling are those of XTDrone's ROS 1 advanced_param_xtdrone.xml (upstream: 0.099, -0.01, 2.9)
        "grid_map/obstacles_inflation": f(arg("inflation")),
        "grid_map/local_map_margin": 10,
        # the map starts at the floor height, and the planner treats everything below the map as occupied
        "grid_map/ground_height": f(arg("floor")),
        # iris_realsense_camera depth camera: 640 x 480, horizontal FOV 60 deg
        "grid_map/cx": 320.5, "grid_map/cy": 240.5, "grid_map/fx": 554.254691191187, "grid_map/fy": 554.254691191187,
        "grid_map/use_depth_filter": True, "grid_map/depth_filter_tolerance": 0.15,
        "grid_map/depth_filter_maxdist": 5.0, "grid_map/depth_filter_mindist": 0.2,
        "grid_map/depth_filter_margin": 2, "grid_map/k_depth_scaling_factor": 1000.0, "grid_map/skip_pixel": 2,
        "grid_map/p_hit": 0.65, "grid_map/p_miss": 0.35, "grid_map/p_min": 0.12, "grid_map/p_max": 0.90,
        "grid_map/p_occ": 0.80, "grid_map/min_ray_length": 0.1, "grid_map/max_ray_length": 4.5,
        "grid_map/virtual_ceil_height": f(arg("ceiling")), "grid_map/visualization_truncate_height": 1.8,
        "grid_map/show_occ_time": False, "grid_map/pose_type": 1, "grid_map/frame_id": "world",
        "manager/max_vel": max_vel, "manager/max_acc": max_acc, "manager/max_jerk": 4.0,
        "manager/control_points_distance": 0.4, "manager/feasibility_tolerance": 0.05,
        "manager/planning_horizon": horizon, "manager/use_distinctive_trajs": True, "manager/drone_id": int(vid),
        "optimization/lambda_smooth": 1.0, "optimization/lambda_collision": 0.5,
        "optimization/lambda_feasibility": 0.1, "optimization/lambda_fitness": 1.0, "optimization/dist0": 0.5,
        "optimization/swarm_clearance": 0.5, "optimization/max_vel": max_vel, "optimization/max_acc": max_acc,
        "bspline/limit_vel": max_vel, "bspline/limit_acc": max_acc, "bspline/limit_ratio": 1.1,
        "prediction/obj_num": 1, "prediction/lambda": 1.0, "prediction/predict_rate": 1.0,
    }
    bspline = f"/xtdrone/{v}/planning/bspline"
    return [
        Node(package="ego_planner", executable="ego_planner_node", name=f"{v}_ego_planner_node", output="screen",
             parameters=[params, {"use_sim_time": True}], remappings=[
                 ("odom_world", f"/{v}/ego/odom"), ("planning/bspline", bspline),
                 ("planning/data_display", f"/xtdrone/{v}/planning/data_display"),
                 ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
                 ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
                 ("goal_point", f"/{v}/plan_vis/goal_point"), ("global_list", f"/{v}/plan_vis/global_list"),
                 ("init_list", f"/{v}/plan_vis/init_list"), ("optimal_list", f"/{v}/plan_vis/optimal_list"),
                 ("a_star_list", f"/{v}/plan_vis/a_star_list"),
                 ("grid_map/odom", f"/{v}/ego/odom"), ("grid_map/cloud", f"/{v}/ego/cloud_unused"),
                 ("grid_map/pose", f"/{v}/camera_pose"),
                 ("grid_map/depth", f"/{v}/realsense/depth_camera/depth/image_raw"),
                 ("grid_map/occupancy_inflate", f"/{v}/ego/occupancy_inflate"),
                 ("/move_base_simple/goal", arg("goal_topic"))]),
        Node(package="ego_planner", executable="traj_server", name=f"{v}_traj_server", output="screen",
             parameters=[{"traj_server/time_forward": 1.0, "use_sim_time": True}],
             remappings=[("position_cmd", f"/xtdrone/{v}/planning/pos_cmd"), ("planning/bspline", bspline)]),
        # sim time: the grid map pairs each depth image (stamped with simulation time) with the camera pose
        Node(package="xtdrone_planning", executable="ego_transfer", arguments=[vtype, vid], output="log",
             parameters=[{"use_sim_time": True}]),
        Node(package="xtdrone_planning", executable="ego_pos_cmd", arguments=[vtype, vid], output="log",
             parameters=[{"min_z": f(arg("min_z"))}]),
    ] + ([TimerAction(period=start_delay, actions=[ExecuteProcess(
        cmd=["ros2", "topic", "pub", "--once", "/traj_start_trigger", "geometry_msgs/msg/PoseStamped", "{}"],
        output="log", name=f"{v}_start_trigger")])] if start_delay > 0 else [])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_type", default_value="iris"),
        DeclareLaunchArgument("id", default_value="0"),
        DeclareLaunchArgument("goal", default_value="waypoint", description="waypoint (target_*) or rviz (2D Goal)"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose", description="goals with goal:=rviz"),
        DeclareLaunchArgument("sequential_start", default_value="true",
                              description="swarm: wait for the trajectories of drone id - 1 before the first plan"),
        DeclareLaunchArgument("target_x", default_value="0"), DeclareLaunchArgument("target_y", default_value="-4"),
        DeclareLaunchArgument("target_z", default_value="1.2"),
        DeclareLaunchArgument("map_size_x", default_value="100"), DeclareLaunchArgument("map_size_y", default_value="100"),
        DeclareLaunchArgument("map_size_z", default_value="5"),
        DeclareLaunchArgument("ceiling", default_value="3.0", description="virtual ceiling of the planner (m)"),
        DeclareLaunchArgument("floor", default_value="-1.0", description="bottom of the planner's map (m)"),
        DeclareLaunchArgument("inflation", default_value="0.35", description="obstacle inflation (m)"),
        DeclareLaunchArgument("start_delay", default_value="6.0", description="seconds the planner builds its map "
                              "before the first trajectory (waypoint goals); 0 starts at once"),
        DeclareLaunchArgument("min_z", default_value="1.2", description="lowest position command (m, ENU): keeps the "
                              "vehicle over tables and chairs, and off the floor the depth camera does not always see"),
        DeclareLaunchArgument("max_vel", default_value="1.0"), DeclareLaunchArgument("max_acc", default_value="1.0"),
        OpaqueFunction(function=nodes),
    ])
