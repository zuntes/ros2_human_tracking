#!/usr/bin/python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

camera_detector_path = get_package_share_directory('camera_detector')

def generate_launch_description():
    ld = LaunchDescription([])

    yolo_node = Node(
            package="camera_detector",
            executable="track_2D_node.py",
            name="track_2D_node",
            output='screen',
    )

    projection_node = Node(
            package="camera_detector",
            executable="projection_3D_node",
            name="projection_3D_node",
            output='screen',
    )

    calculate_human_pos_node = Node(
            package="camera_detector",
            executable="compute_human_pos_node",
            name="compute_human_pos_node",
            output='screen',
    )

    visualization_node = Node(
            package="camera_detector",
            executable="visualize_human",
            name="visualize_human",
            output='screen',
    )

    ld.add_action(yolo_node)
    ld.add_action(projection_node)
    ld.add_action(calculate_human_pos_node)
    ld.add_action(visualization_node)

    return ld 
 
