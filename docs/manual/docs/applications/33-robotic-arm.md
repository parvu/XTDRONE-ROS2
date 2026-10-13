# 33. Aerial robotic arm operation and gripping

!!! success "Ported and tested"
    Package `xtdrone_robotic_arm` (URDF, ros2_control, MoveIt 2 configuration, vision and grasping nodes), model
    `iris_arm`, scenario `grasping_arm`. Test: `scripts/tests/run_chapter_tests.sh ch33`: the camera finds the green
    cube, MoveIt moves the arm around it, the gripper takes it and lifts it, and the iris takes off with it (the cube
    stays within 0.5 m of the iris above 1.5 m).

An iris carries the small `le_arm` robotic arm (five joints and a two-finger gripper) under its body. A camera
under the iris looks ahead; the vision node finds the green object on the table and the grasping node moves the
arm with MoveIt and closes the gripper on it.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=grasping_arm      # iris_arm at (0.1, 0), table with the cube
ros2 launch xtdrone_robotic_arm robotic_arm.launch.py                     # controllers, MoveIt, vision, grasping demo
ros2 topic echo /iris_0/le_arm/grasp_state                                # detecting, approaching, grasping, lifting, grasped
```

Then fly the iris as usual (chapter 9, `multirotor_keyboard_control iris 1 vel`): the cube goes with it.

`robotic_arm.launch.py demo:=false rviz:=true` leaves the grasping demo out; plan and move the arm from RViz's
MotionPlanning panel or your own MoveIt code (planning groups `arm` and `gripper`, `move_group` in `/iris_0`).

| Topic / interface (namespace `/iris_0`) | |
|---|---|
| `le_arm/camera/image_raw` | the arm camera (ROS 1: `/iris_0/le_arm/camera/image_raw`) |
| `le_arm/object` | the object in the image: pixel x, y, and its size in pixels as z (`vision_manager`) |
| `le_arm/gripper/attach`, `le_arm/gripper/detach` | `std_msgs/Empty`: fix the cube to the gripper, release it |
| `le_arm/grasp_state` | progress of `grasping_demo` |
| `le_arm_controller/follow_joint_trajectory`, `gripper_controller/follow_joint_trajectory` | ros2_control joint trajectory controllers |
| `move_action` | MoveIt |

## Notes

- **Model.** `iris_arm` is PX4's x500 with the arm converted from the URDF; `ros2/xtdrone_robotic_arm/scripts/
  generate_iris_arm_model.py` regenerates it after a URDF change. The launch adds `gz_ros2_control` to the running
  model (Gazebo's `entity/system/add` service), because it needs the vehicle's namespace and the path of the
  controller configuration.
- **Grasping.** ROS 1 held the object with `gazebo_grasp_fix` (contact forces between fingers and object). Here a
  `DetachableJoint` fixes the cube to the gripper on `le_arm/gripper/attach`; the arm links have no collision
  geometry in Gazebo (the arm hangs under the x500 at ground level), and MoveIt checks the arm against itself.
- **Reach.** The x500 stands lower than the ROS 1 iris, and the arm reaches about 0.2 m ahead of the body at the
  height of the table: the scenario puts the iris at x = 0.1 instead of −0.1. The grasp poses are joint targets,
  as in ROS 1 (`grasping_demo` parameters `pregrasp`, `grasp`, `carry`); the vision node only confirms the object
  is in view.
- MoveIt runs in the vehicle's namespace; `MoveGroupInterface` needs it explicitly (`Options(group,
  "robot_description", ns)`), otherwise it waits for `/move_action` forever.
- The IK solver is KDL (part of MoveIt) with `position_only_ik` for the 5-joint arm, instead of ROS 1's TRAC-IK,
  which saves a dependency; `ros-jazzy-trac-ik-kinematics-plugin` can be set in `config/kinematics.yaml`.
