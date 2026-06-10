# Cartographer 2D SLAM 建图 launch — 伴随模式
# 需先启动 hunter_bringup (run_bringup.sh)，本 launch 仅额外启动：
#   1. mapping 专用 pointcloud_to_laserscan（发布到 /scan_mapping）
#   2. cartographer_node（订阅 /scan_mapping 和 /odom）
#   3. cartographer_occupancy_grid_node（发布 /map）
#   4. rviz2（可视化）

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='hunter_cartographer').find('hunter_cartographer')

    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(pkg_share, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='hunter_2d.lua'
    )

    # 1. Mapping 专用 pointcloud_to_laserscan（与 bringup 的共存，发布到 /scan_mapping）
    pointcloud_to_laserscan_mapping = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_mapping',
        remappings=[
            ('cloud_in', '/lidar_points'),
            ('scan', '/scan_mapping'),
        ],
        parameters=[{
            'target_frame': 'hesai_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 0.5,
            'angle_min': -3.1416,
            'angle_max': 3.1416,
            'angle_increment': 0.005,
            'scan_time': 0.03333,
            'range_min': 0.2,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen'
    )

    # 2. Cartographer SLAM 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('scan', '/scan_mapping'),
        ]
    )

    # 3. 占据栅格地图发布节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ]
    )

    # 4. RViz 可视化
    rviz_config_dir = os.path.join(pkg_share, 'config', 'cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        pointcloud_to_laserscan_mapping,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
