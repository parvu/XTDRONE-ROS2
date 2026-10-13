"""Aerial robotic arm (chapter 33): MoveIt 2 and ros2_control for the le_arm under iris_0, plus the vision and
grasping nodes. ROS 1: le_arm/launch/le_arm_gazebo.launch, le_arm_moveit_config/launch/*.launch.

The simulation comes first:
  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=grasping_arm
  ros2 launch xtdrone_robotic_arm robotic_arm.launch.py
Everything runs in the namespace of the vehicle (/iris_0): robot_state_publisher, gz_ros2_control (added to the
running model; it reads the URDF from robot_state_publisher), the controller spawners, move_group,
vision_manager and grasping_demo (demo:=false to leave it out and command the arm from RViz or your own MoveIt
code).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

PKG = "xtdrone_robotic_arm"


def launch_setup(context):
    ns = LaunchConfiguration("namespace").perform(context)
    share = FindPackageShare(PKG).perform(context)
    moveit = (MoveItConfigsBuilder("le_arm", package_name=PKG)
              .robot_description(file_path="urdf/iris_arm.urdf.xacro")
              .robot_description_semantic(file_path="config/le_arm.srdf")
              .robot_description_kinematics(file_path="config/kinematics.yaml")
              .joint_limits(file_path="config/joint_limits.yaml")
              .trajectory_execution(file_path="config/moveit_controllers.yaml")
              .planning_pipelines(pipelines=["ompl"])
              .to_moveit_configs())
    sim = {"use_sim_time": True}
    world = LaunchConfiguration("world").perform(context)
    # gz_ros2_control in the running vehicle model: it reads the URDF (with its <ros2_control> block) from this
    # namespace's robot_state_publisher and starts /<ns>/controller_manager with ros2_controllers.yaml
    inner = (f"<parameters>{share}/config/ros2_controllers.yaml</parameters>"
             f"<ros><namespace>/{ns}</namespace></ros>"
             "<robot_param>robot_description</robot_param><robot_param_node>robot_state_publisher</robot_param_node>")
    # entity/system/add needs the model's entity id (by name it resolved to the world); `gz model -m` prints it
    # (inside the single-quoted --req below: '"$id"' closes the quote to let bash expand the id)
    req = ("entity: {id: '\"$id\"'} plugins: {name: \"gz_ros2_control::GazeboSimROS2ControlPlugin\", "
           "filename: \"gz_ros2_control-system\", innerxml: \"" + inner + "\"}")
    script = (f'for i in $(seq 60); do id=$(gz model -m {ns} 2>/dev/null | sed -n "s/^Model: \\[\\([0-9]*\\)\\]/\\1/p"); '
              f'[ -n "$id" ] && break; sleep 1; done; [ -z "$id" ] && {{ echo "model {ns} not found"; exit 1; }}; '
              f"gz service -s /world/{world}/entity/system/add --reqtype gz.msgs.EntityPlugin_V "
              f"--reptype gz.msgs.Boolean --timeout 10000 --req '{req}'")
    add_control = ExecuteProcess(cmd=["bash", "-c", script], output="screen")
    actions = [
        TimerAction(period=3.0, actions=[add_control]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns, output="log",
             parameters=[moveit.robot_description, sim]),
    ]
    for ctrl in ("joint_state_broadcaster", "le_arm_controller", "gripper_controller"):
        actions.append(Node(package="controller_manager", executable="spawner", namespace=ns, output="log",
                            arguments=[ctrl, "-c", f"/{ns}/controller_manager", "--controller-manager-timeout", "120",
                                       "--param-file", f"{share}/config/ros2_controllers.yaml"]))
    actions.append(Node(package="moveit_ros_move_group", executable="move_group", namespace=ns, output="log",
                        parameters=[moveit.to_dict(), sim,
                                    {"publish_robot_description_semantic": True,
                                     "trajectory_execution.allowed_start_tolerance": 0.05}]))
    actions.append(Node(package=PKG, executable="vision_manager", namespace=ns, output="screen", parameters=[sim]))
    actions.append(Node(package=PKG, executable="grasping_demo", namespace=ns, output="screen",
                        prefix=LaunchConfiguration("demo_prefix").perform(context) or None,
                        condition=IfCondition(LaunchConfiguration("demo")),
                        parameters=[moveit.robot_description, moveit.robot_description_semantic,
                                    moveit.robot_description_kinematics, moveit.joint_limits, sim]))
    actions.append(Node(package="rviz2", executable="rviz2", namespace=ns, output="log",
                        condition=IfCondition(LaunchConfiguration("rviz")),
                        parameters=[moveit.robot_description, moveit.robot_description_semantic,
                                    moveit.robot_description_kinematics, sim]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="iris_0",
                              description="Vehicle namespace = model name of the iris_arm."),
        DeclareLaunchArgument("world", default_value="grasping", description="Gazebo world name."),
        DeclareLaunchArgument("demo", default_value="true", description="Run grasping_demo."),
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz."),
        DeclareLaunchArgument("demo_prefix", default_value="", description="e.g. 'gdb -batch -ex run -ex bt --args'"),
        OpaqueFunction(function=launch_setup),
    ])
