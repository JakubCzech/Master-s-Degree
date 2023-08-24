from __future__ import annotations
navfn = """
planner_server:
  ros__parameters:
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner/NavfnPlanner'
      use_astar: True
      allow_unknown: True
      tolerance: 1.0
      use_final_approach_orientation: True
"""
smac_planner = """
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      downsample_costmap: false # whether or not to downsample the map
      downsampling_factor: 1 # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      tolerance: 0.25 # dist-to-goal heuristic cost (distance) for valid tolerance endpoints if exact goal cannot be found.
      allow_unknown: true # allow traveling in unknown space
      max_iterations: 1000000 # maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000 # Maximum number of iterations after within tolerances to continue to try to find exact solution
      max_planning_time: 5.0 # max time in s for planner to plan, smooth
      motion_model_for_search: "REEDS_SHEPP" # Hybrid-A* Dubin, Redds-Shepp
      angle_quantization_bins: 72 # Number of angle bins for search
      analytic_expansion_ratio: 3.5 # The ratio to attempt analytic expansions during search for final approach.
      analytic_expansion_max_length: 3.0 # For Hybrid/Lattice nodes: The maximum length of the analytic expansion to be considered valid to prevent unsafe shortcutting
      minimum_turning_radius: 0.0 # minimum turning radius in m of path / vehicle
      reverse_penalty: 2.0 # Penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.0 # Penalty to apply if motion is changing directions (L to R), must be >= 0
      non_straight_penalty: 1.2 # Penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 2.0 # Penalty to apply to higher cost areas when adding into the obstacle map dynamic programming distance expansion heuristic. This drives the robot more towards the center of passages. A value between 1.3 - 3.5 is reasonable.
      retrospective_penalty: 0.015
      lookup_table_size: 20.0 # Size of the dubin/reeds-sheep distance window to cache, in meters.
      cache_obstacle_heuristic: false # Cache the obstacle map dynamic programming distance expansion heuristic between subsiquent replannings of the same goal location. Dramatically speeds up replanning performance (40x) if costmap is largely static.
      viz_expansions: false # For Hybrid nodes: Whether to publish expansions on the /expansions topic as an array of poses (the orientation has no meaning). WARNING: heavy to compute and to display, for debug only as it degrades the performance.
      smooth_path: True # If true, does a simple and quick smoothing post-processing to the path

      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
"""
theta_star = """
planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_theta_star_planner/ThetaStarPlanner"]
    planner_plugin_ids: ["GridBased"]
    GridBased:
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      use_final_approach_orientation: True
"""
smac_planner_test = """
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerLattice"
      allow_unknown: true                 # Allow traveling in unknown space
      tolerance: 0.25                     # dist-to-goal heuristic cost (distance) for valid tolerance endpoints if exact goal cannot be found.
      max_iterations: 1000000             # Maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000    # Maximum number of iterations after within tolerances to continue to try to find exact solution
      max_planning_time: 5.0              # Max time in s for planner to plan, smooth
      analytic_expansion_ratio: 3.5       # The ratio to attempt analytic expansions during search for final approach.
      analytic_expansion_max_length: 3.0  # For Hybrid/Lattice nodes: The maximum length of the analytic expansion to be considered valid to prevent unsafe shortcutting
      reverse_penalty: 2.0                # Penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.05                # Penalty to apply if motion is changing directions (L to R), must be >= 0
      non_straight_penalty: 1.05          # Penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 2.0                   # Penalty to apply to higher cost areas when adding into the obstacle map dynamic programming distance expansion heuristic. This drives the robot more towards the center of passages. A value between 1.3 - 3.5 is reasonable.
      rotation_penalty: 5.0               # Penalty to apply to in-place rotations, if minimum control set contains them
      retrospective_penalty: 0.015
      lattice_filepath: ""                # The filepath to the state lattice graph
      lookup_table_size: 20.0             # Size of the dubin/reeds-sheep distance window to cache, in meters.
      cache_obstacle_heuristic: false     # Cache the obstacle map dynamic programming distance expansion heuristic between subsiquent replannings of the same goal location. Dramatically speeds up replanning performance (40x) if costmap is largely static.
      allow_reverse_expansion: false      # If true, allows the robot to use the primitives to expand in the mirrored opposite direction of the current robot's orientation (to reverse).
      smooth_path: True                   # If true, does a simple and quick smoothing post-processing to the path
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
"""
smac_planner_2d = """
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.125                      # tolerance for planning if unable to reach exact pose, in meters
      downsample_costmap: false             # whether or not to downsample the map
      downsampling_factor: 1                # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      allow_unknown: true                   # allow traveling in unknown space
      max_iterations: 1000000               # maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000      # maximum number of iterations to attempt to reach goal once in tolerance
      max_planning_time: 2.0                # max time in s for planner to plan, smooth
      cost_travel_multiplier: 2.0           # Cost multiplier to apply to search to steer away from high cost areas. Larger values will place in the center of aisles more exactly (if non-`FREE` cost potential field exists) but take slightly longer to compute. To optimize for speed, a value of 1.0 is reasonable. A reasonable tradeoff value is 2.0. A value of 0.0 effective disables steering away from obstacles and acts like a naive binary search A*.
      use_final_approach_orientation: true # Whether to set the final path pose at the goal's orientation to the requested orientation (false) or in line with the approach angle so the robot doesn't rotate to heading (true)
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1e-10
"""
