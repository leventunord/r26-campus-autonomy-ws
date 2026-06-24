from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pedestrian_trajectory_prediction',
            executable='pedestrian_prediction_node',
            name='pedestrian_prediction_node',
            output='screen',
            parameters=[{
                'detections_topic': '/yolo/detections',
                'lidar_topic': '/lidar_points',
                'output_frame': 'base_link',
                'lidar_frame': 'hesai_lidar',
                'image_width': 3840,
                'yaw_offset': 0.0,

                # Static transform from pointcloud2laserscan.launch.py
                'lidar_to_base_x': 0.34058,
                'lidar_to_base_y': 0.0,
                'lidar_to_base_z': 0.3465,
                'lidar_to_base_yaw': 1.57079632679,

                # Detection-lidar association tuning
                'min_range': 0.4,
                'max_range': 12.0,
                'min_z': -0.8,
                'max_z': 2.2,
                'angular_gate_min': 0.035,
                'angular_gate_padding': 0.030,
                'nearest_cluster_width': 0.8,
                'min_points_per_detection': 3,

                # Tracking and prediction tuning
                'association_distance': 1.2,
                'track_timeout': 1.5,
                'history_length': 10,
                'prediction_horizon': 4.0,
                'prediction_dt': 0.5,
                'velocity_smoothing': 0.35,
                'max_prediction_speed': 2.0,
            }]
        )
    ])
