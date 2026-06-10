# Nav2 导航 launch — 伴随模式
# 需先启动 hunter_bringup (run_bringup.sh)，本 launch 仅额外启动：
#   1. Nav2 导航栈（AMCL 定位 + 规划 + 控制 + 代价地图等）
#   2. rviz2（可视化）
# 不包含：传感器节点、静态 TF、伪造的 map→odom TF（由 AMCL 发布）

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 定位包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    hunter_nav2_dir = get_package_share_directory('hunter_nav2')
    hunter_cartographer_dir = get_package_share_directory('hunter_cartographer')

    # 地图路径：使用 hunter_cartographer 包中的地图
    map_yaml_path = LaunchConfiguration(
        'map',
        default=os.path.join(hunter_cartographer_dir, 'maps', 'hall.yaml')
    )

    # Nav2 参数文件路径
    nav2_param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(hunter_nav2_dir, 'param', 'nav2.yaml')
    )

    # RViz 配置
    rviz_config_dir = os.path.join(hunter_nav2_dir, 'config', 'nav2.rviz')

    # Nav2 导航栈（包含 AMCL、map_server、planner、controller、costmaps 等）
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': 'False',
            'params_file': nav2_param_path,
        }.items(),
    )

    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_launch,
        rviz_node,
    ])
