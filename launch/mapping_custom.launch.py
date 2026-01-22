#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    # Load parameters - use custom config files
    sensor_config_cmd = os.path.join(config_file_dir, "custom_sensor.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "custom_camera.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2",
    )

    sensor_config_arg = DeclareLaunchArgument(
        'sensor_params_file',
        default_value=sensor_config_cmd,
        description='Full path to the ROS2 parameters file for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file for camera intrinsics',
    )

    sensor_params_file = LaunchConfiguration('sensor_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')

    return LaunchDescription([
        use_rviz_arg,
        sensor_config_arg,
        camera_config_arg,

        # Use parameter_blackboard as global parameters server and load camera params
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[
                camera_params_file,
            ],
            output='screen'
        ),

        # Main FAST-LIVO2 mapping node
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                sensor_params_file,
            ],
            output="screen"
        ),

        # RViz2 visualization
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
