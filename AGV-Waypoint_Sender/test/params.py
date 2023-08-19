amcl = """
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::OmniMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.1
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /scan
    set_initial_pose: true
    initial_pose:
      x: -11.110
      y: -18.260
      z: 0.0
      yaw: -0.087
"""
bt_navigator = """
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
"""

bt_navigator_navigate_through_poses_rclcpp_node = """
bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True
"""

bt_navigator_navigate_to_pose_rclcpp_node = """
bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True
"""


local_costmap = """
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.025
      # robot_radius: 0.22
      footprint: "[[-0.4125, 0.275], [0.4125, 0.275], [0.4125, -0.275], [-0.4125, -0.275]]"
      footprint_padding: 0.12
      plugins: ["voxel_layer", "inflation_layer"]
      # plugins: ["voxel_layer","denoise_layer" "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0 # it was 3.0
        inflation_radius: 0.4 # it was 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 0.7 # it was 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 0.7 # was 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      # denoise_layer:
      #   plugin: "nav2_costmap_2d::DenoiseLayer"
      #   enabled: true
      #   minimal_group_size: 2
      #   group_connectivity_type: 8
      static_layer:
        map_subscribe_transient_local: True

      always_send_full_costmap: True
"""
global_costmap = """
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      # robot_radius: 0.41
      footprint: "[[-0.4125, 0.275], [0.4125, 0.275], [0.4125, -0.275], [-0.4125, -0.275]]"
      footprint_padding: 0.05
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 0.7 # was 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0 # it was 3.0
        inflation_radius: 0.8 # it was 0.55
      always_send_full_costmap: True
"""

map_server = """
map_server:
  ros__parameters:
    use_sim_time: True
    # Overridden in launch by the "map" launch configuration or provided default value.
    # To use in yaml, remove the default "map" value in the tb3_simulation_launch.py file & provide full path to map below.
    yaml_filename: ""
"""

map_saver = """
map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 50000
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True
"""

planner_server = """
planner_server:
  ros__parameters:
    expected_planner_frequency: 0.001
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: false
"""
behavior_server = """
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0

    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"

    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: True
    simulate_ahead_time: 1.0
    # simulate_ahead_time: 2.0
    max_rotational_vel: 0.4
    min_rotational_vel: -0.4
    rotational_acc_lim: 2.0
"""

robot_state_publisher = """
robot_state_publisher:
  ros__parameters:
    use_sim_time: True
"""

waypoint_follower = """
waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
"""


velocity_smoother = """
velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: True
    feedback: "OPEN_LOOP"
    max_velocity: [3.0, 3.0, 3.0]
    min_velocity: [-3.0, -3.0, -3.0]
    max_accel: [1.5, 1.5, 1.2]
    max_decel: [-1.5, -1.5, -1.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
"""

controller_server = """
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "agv_controller::AGVController"
      debug_trajectory_details: True
      lookahead_dist: {Lookahead}
      rotate_to_heading_min_angle: {Rotate_to_heading}
      #VFO params
      max_speed: 1.0
      Kp: {Kp}
      Ka: {Ka}
    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.5
"""


def get_params(Ka, Lookahead, Rotate_to_heading):
    params = amcl
    params += bt_navigator
    params += bt_navigator_navigate_through_poses_rclcpp_node
    params += bt_navigator_navigate_to_pose_rclcpp_node
    params += local_costmap
    params += global_costmap
    params += map_server
    params += map_saver
    params += planner_server
    params += behavior_server
    params += robot_state_publisher
    params += waypoint_follower
    params += velocity_smoother
    params += controller_server.format(
        Ka=Ka, Kp=2 * Ka, Lookahead=Lookahead, Rotate_to_heading=Rotate_to_heading
    )
    return params