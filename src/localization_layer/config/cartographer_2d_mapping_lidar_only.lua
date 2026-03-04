include "map_builder_mapping.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
-- In repetitive corridor-like maps, online correlative matching can snap to
-- a wrong but similar-looking pose and cause TF jumps.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.08
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(4.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 2.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 3.0
-- Waypoint-lock mapping mode:
-- trust odom trajectory almost entirely and use scan only for map painting.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1e-3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e6
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e6
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80

-- Jump-suppression mode for repetitive corridor-like tracks:
-- prioritize local consistency and disable global loop-closure corrections.
POSE_GRAPH.optimize_every_n_nodes = 0
-- Keep normal sampling ratios so Cartographer does not continuously warn about
-- dropping all data. optimize_every_n_nodes=0 already prevents global corrections.
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.global_sampling_ratio = 1.0
POSE_GRAPH.constraint_builder.min_score = 0.82
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
