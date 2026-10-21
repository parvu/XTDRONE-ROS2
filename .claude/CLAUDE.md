# XTDrone-ROS2 (Ardupilot branch)

Port of XTDrone-ROS2 from PX4 to ArduPilot 4.7.2 (SITL + MAVROS + Gazebo). Pushed to origin/Ardupilot.

## Workflow
- Commit and push only when asked.
- Give frequent "Progress" answers; document chapters.
- User-facing differences from PX4: docs/manual/docs/getting-started/03a-ardupilot.md.

## Branches and workspaces
- master (PX4): checkout ~/XTDRONE-ROS2, workspace ~/xtdrone_ros2_ws.
- Ardupilot: git worktree ~/XTDRONE-ROS2-ardupilot, workspace ~/xtdrone_ros2_ws_ardupilot. Use `export XTDRONE_WS=~/xtdrone_ros2_ws_ardupilot` before `source scripts/xtdrone_env.sh`.
- The worktree symlinks the big submodules (ArduPilot, ardupilot_gazebo, PX4-Autopilot, ros2/*, SITL_Models) to the master checkout; they are marked assume-unchanged.
- colcon never deletes old install files. After switching a workspace between branches, stale models (e.g. install/xtdrone_gz/.../models/r1_rover with ArduPilotPlugin) shadow PX4's own and break the tests (PX4 "Gyro STALE", MAVROS not connected). Delete install/ and build/ of xtdrone_gz, then rerun scripts/build_xtdrone_ros2.sh.

## Status (Ardupilot branch)
- Passing: multirotor chapters, ch08 (plane/vtol/rover), ch19, ch21, ch23 (iris_gimbal replaces typhoon_h480), ch28, ch29, ch32, ch33, ch35.
- Relaxed tolerances (flaky): ch13 2.0 m, ch14 1.0 m, ch20 0.7 m.
- Open: ch22 (2 of 4 vehicles stall), ch27, ch07 angular speed, tailsitter (skycat_tvbs NaN at 4 ms step, SIGFPE in SITL), tiltrotor (no model).
- No full-suite run on the branch yet.

## Gotchas
- ArduPlane ignores position/velocity setpoints; only DO_REPOSITION works. VTOLs are flown in QLOITER with RC override and need MAVROS system_id 255.
- Local ENU frame starts at spawn with GPS; world frame with vision EKF.
- DetachableJoint attaches at model load (ch33 cube gets yanked).
- indoor1 floor is raised 0.19 m with an edge near y=7.7 (vehicles slid off and flipped in ch21).
- MAVROS links drop under camera load (ch28 camera at 10 Hz).
