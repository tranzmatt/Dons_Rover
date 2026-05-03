-- rover_ld19.lua
-- Cartographer 2D SLAM configuration for UGV Rover with LD19 LIDAR
--
-- LD19 specs:
--   Range:       12m
--   Points/scan: 450+
--   FOV:         360 degrees
--   Scan rate:   10 Hz
--
-- Tuned for:
--   - Indoor environment
--   - Moderate odometry quality (wheel encoders with IMU heading)
--   - Room size up to ~10m x 10m

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frame IDs must match your TF tree
  map_frame = "map",
  tracking_frame = "base_footprint",   -- frame Cartographer tracks
  published_frame = "odom",  -- frame Cartographer publishes pose for
  odom_frame = "odom",

  -- Use odometry from /odom topic
  provide_odom_frame = false,          -- we provide our own odom TF
  publish_frame_projected_to_2d = true,

  -- Odometry and IMU usage
  use_odometry = true,                 -- use /odom topic
  use_nav_sat = false,
  use_landmarks = false,

  -- LIDAR configuration
  num_laser_scans = 1,                 -- one 2D LIDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Timing
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,      -- 200 Hz pose publishing
  trajectory_publish_period_sec = 30e-3,

  -- Sampling ratios (1.0 = use everything)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D SLAM mode
MAP_BUILDER.use_trajectory_builder_2d = true

-- LD19 range limits
-- Min: ignore returns closer than 0.15m (avoids robot body hits)
-- Max: 11m (slightly under LD19's 12m max for reliability)
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 11.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- IMU: set to false since we're using wheel odometry + IMU heading fusion
-- If you want Cartographer to use raw IMU directly, set to true
-- and add imu_topic to the launch file
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Submap size — larger submaps give better loop closure in big rooms
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- ~9 seconds of data per submap at 10Hz

-- Scan matcher settings
-- Higher score thresholds = more confident matches required
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher weights
-- Higher translation_weight = trust odometry more
-- Higher rotation_weight = trust scan rotation more
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- Motion filter — only add new scan data when robot has moved enough
-- Prevents building map from stationary noise
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- Pose graph optimization
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 35  -- optimize more frequently for better real-time quality

-- Loop closure settings tuned for large room (15x25 feet = ~4.5x7.5m)
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.  -- search up to 15m for loop closure

return options