from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "-0.34",
                    "0.21",
                    "0.0",
                    "2.366194491",
                    "0.0",
                    "0.0",
                    "base_link",
                    "rear_scanner/scan",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "-0.34",
                    "0.21",
                    "0.0",
                    "2.366194491",
                    "0.0",
                    "0.0",
                    "base_link",
                    "rear_scanner/cloud",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "-0.34",
                    "0.21",
                    "0.0",
                    "2.366194491",
                    "0.0",
                    "0.0",
                    "base_link",
                    "rear_scanner/range",
                ],
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
                    "0.0",
                    "base_link",
                    "front_scanner/scan",
                ],
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
                    "0.0",
                    "base_link",
                    "front_scanner/cloud",
                ],
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
                    "0.0",
                    "base_link",
                    "front_scanner/range",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
            ),
        ]
    )


#  ros2 run tf2_ros static_transform_publisher 0.34 -0.21 0.0 -0.7798163 0.0 0.0 base_link rear_scanner/cloud
#  ros2 run tf2_ros static_transform_publisher 0.34 -0.21 0.0 -0.7798163 0.0 0.0 base_link rear_scanner/scan
#  ros2 run tf2_ros static_transform_publisher 0.34 -0.21 0.0 -0.7798163 0.0 0.0 base_link scan_rear_sensor
#  ros2 run tf2_ros static_transform_publisher -0.34 0.21 0.0 2.366194491 0.0 0.0 base_link rear_scanner/cloud
