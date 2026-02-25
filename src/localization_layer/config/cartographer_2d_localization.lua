include "cartographer_2d_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50.

POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.global_sampling_ratio = 0.0

return options
