import os
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
                'model_path': 'yolo11n.pt',
                'image_topic': '/image_stitched',
                'device': 'cpu',
                'conf_threshold': 0.5,
                'iou_threshold': 0.45
            }]
        )
    ])
