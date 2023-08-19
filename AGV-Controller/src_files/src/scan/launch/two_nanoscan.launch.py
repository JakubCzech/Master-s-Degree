from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_ip = LaunchConfiguration('controller_ip')
    controller_ip_launch_arg = DeclareLaunchArgument('controller_ip')

    scanner_front = Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name='sick_safetyscanners2_node',
        output='screen',
        namespace = 'front_scanner',
        emulate_tty=True,
        parameters=[
            {
                'frame_id': 'scan_front',
                'sensor_ip': '192.168.0.10',
                'host_ip': controller_ip,
                'interface_ip': '0.0.0.0',
                'host_udp_port': 0,
                'channel': 0,
                'channel_enabled': True,
                'skip': 0,
                'angle_start': -2.34,
                'angle_end': 2.34,
                'time_offset': 0.0,
                'general_system_state': True,
                'derived_settings': True,
                'measurement_data': True,
                'intrusion_data': True,
                'application_io_data': True,
                'use_persistent_config': False,
                'min_intensities': 0.0,
            },
        ],
    )
    scan_rear = Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name='sick_safetyscanners2_node',
        output='screen',
        emulate_tty=True,
        namespace = 'rear_scanner',
        parameters=[
            {
                'frame_id': 'scan_rear',
                'sensor_ip': '192.168.0.11',
                'host_ip': controller_ip,
                'interface_ip': '0.0.0.0',
                'host_udp_port': 0,
                'channel': 0,
                'channel_enabled': True,
                'skip': 0,
                'angle_start': -2.34,
                'angle_end': 2.3,
                'time_offset': 0.0,
                'general_system_state': True,
                'derived_settings': True,
                'measurement_data': True,
                'intrusion_data': True,
                'application_io_data': True,
                'use_persistent_config': False,
                'min_intensities': 0.0,
            },
        ],
    )

    # TODO: sprawdzić czy pakiety są wymagane
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
                'queue_size': 4000,
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
            controller_ip_launch_arg,
            scanner_front,
            scan_rear,
            pcl_front,
            pcl_rear,
            pcl_all,
            merger,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "-0.34",
                    "0.21",
                    "0.0",
                    "2.366194491",
                    "0.0",
                    "3.14159265459",
                    "base_link",
                    "scan_rear",
                ]
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.34",
                    "-0.21",
                    "0.0",
                    "-0.7798163",
                    "0.0",
                    "3.14159265459",
                    "base_link",
                    "scan_front",
                ]
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
            ),
        ],
    )
