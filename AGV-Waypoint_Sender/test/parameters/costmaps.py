from __future__ import annotations
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
