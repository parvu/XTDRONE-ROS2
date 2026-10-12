-- XTDrone 2D laser SLAM (chapter 18), port of sensing/slam/laser_slam/cartographer/configuration_files/xtdrone_2d.lua.
-- Scan matching with the Gazebo IMU (/<v>/imu_gazebo; ROS 1: the MAVROS IMU), no odometry. Cartographer
-- publishes map -> base_link;
-- laser_transfer sends map -> base_link to PX4 as vision pose.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,  -- one transform map -> base_link (no odom), read by laser_transfer
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true,  -- /tracked_pose: what laser_transfer forwards to PX4
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true   -- the 2D default, as in ROS 1
-- ROS 1 accumulated 10 scans. A multirotor tilts while it accelerates; every scan on its own (40 Hz)
-- keeps the motion smear small and gives PX4 fresh poses.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- after gravity alignment keep only points near the scan plane (floor and ceiling hits of a tilted scan out)
TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 0.5
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 350
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.

return options
