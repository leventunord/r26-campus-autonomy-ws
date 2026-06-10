# Bringup launch for Hunter SE robot - no IMU, no RViz
# 启动小车底盘（轮式里程计）、URDF、禾赛雷达、点云转 LaserScan、静态 TF
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 启动 hunter_base 底盘驱动，发布 odom->base_link TF（无 IMU，纯轮式里程计）
    hunter_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hunter_base'),
                'launch', 'hunter_base_pub_odom.launch.py'
            )
        )
    )

    # 启动 hunter_se_description，发布 URDF / robot_state_publisher / joint_state_publisher
    hunter_se_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hunter_se_description'),
                'launch', 'display.launch.py'
            )
        )
    )

    # 启动禾赛雷达节点
    hesai_launch = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 启动 pointcloud2laserscan 节点
    pointcloud_to_laserscan_launch = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', ['/lidar_points']),
            ('scan', ['/scan']),
        ],
        parameters=[{
            'target_frame': 'hesai_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.3,
            'max_height': 0.5,
            'angle_min': -3.1416,
            'angle_max': 3.1416,
            'angle_increment': 0.0174,
            'scan_time': 0.01,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        name='pointcloud_to_laserscan'
    )

    # 静态 TF: base_link -> hesai_lidar
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_baselink2hesai',
        arguments=[
            '--x', '0.34058', '--y', '0', '--z', '0.3465',
            '--qx', '0', '--qy', '0', '--qz', '0.70710678', '--qw', '0.70710678',
            '--frame-id', 'base_link', '--child-frame-id', 'hesai_lidar'
        ]
    )

    return LaunchDescription([
        hunter_base_launch,
        hunter_se_description_launch,
        hesai_launch,
        tf_node,
        pointcloud_to_laserscan_launch,
    ])
