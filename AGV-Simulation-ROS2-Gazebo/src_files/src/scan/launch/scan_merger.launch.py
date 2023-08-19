from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    pcl_front = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_front',
        remappings=[
            ('scan_in', '/front_scanner/scan'),
            ('cloud', '/front_scanner/cloud'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.025,
        }],
    )
    pcl_rear = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_rear',
        remappings=[
            ('scan_in', '/rear_scanner/scan'),
            ('cloud', '/rear_scanner/cloud'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.025,
        }],
    )
    pcl_all = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/combined_cloud'), ('scan', '/scan')],
        parameters=[
            {
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -3.141592,  # -M_PI
                'angle_max': 3.141592,  # M_PI
                'angle_increment': 0.0029,
                'scan_time': 0.03,
                'range_min': 0.1,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 5,
            },
        ],
    )
    merger = Node(
        package='scan',
        executable='scan_node',
        name='cloud_merger',
    )
    

    return LaunchDescription(
        [
           
            pcl_front,
            pcl_rear,
            pcl_all,
            merger,
            
        ],
    )
