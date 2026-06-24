import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    bag_path = '/home/agilex03/r26-campus-autonomy-ws/data/amcl_tracking_dataset'
    rviz_config = '/home/agilex03/r26-campus-autonomy-ws/test_pedestrian.rviz'

    return LaunchDescription([
        # 播放 ROS 2 Bag 文件，并发布时钟(--clock)以支持 use_sim_time
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
            output='screen'
        ),

        # 启动行人位置融合节点
        Node(
            package='pedestrian_position_fusion',
            executable='position_fusion_node',
            name='position_fusion_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 启动行人轨迹追踪与预测节点
        Node(
            package='pedestrian_trajectory_prediction',
            executable='pedestrian_prediction_node',
            name='pedestrian_prediction_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 启动 RViz 进行可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
