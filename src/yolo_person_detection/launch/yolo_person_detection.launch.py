import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_person_detection',
            executable='yolo_node',
            name='yolo_person_detector',
            output='screen',
            parameters=[{
                'model_path': 'yolo11n_openvino_model',
                'image_topic': '/image_stitched',
                'device': 'cpu',
                'conf_threshold': 0.5,
                'iou_threshold': 0.45
            }]
        ),
        Node(
            package='yolo_person_detection',
            executable='person_direction_node',
            name='person_direction_node',
            output='screen',
            parameters=[{
                'yaw_offset': 0.0
            }]
        )
    ])
