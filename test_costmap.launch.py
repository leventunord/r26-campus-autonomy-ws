import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bag_path = '/home/agilex03/r26-campus-autonomy-ws/data/amcl_tracking_dataset'
    rviz_config = '/home/agilex03/r26-campus-autonomy-ws/test_costmap.rviz'
    nav2_params_file = '/home/agilex03/r26-campus-autonomy-ws/src/hunter_nav2/param/nav2.yaml'

    # Nav2 Navigation 启动项 (无 TF 发布，安全用于离线 Bag 测试)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        # 播放 ROS 2 Bag 文件，排除包中已录制的 yolo 相关话题，并发布时钟(--clock)以支持 use_sim_time
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '-x', '/yolo/.*'],
            output='screen'
        ),

        # 启动新的 YOLO 节点，用 /image_stitched 进行推理并发布新的 person_directions
        Node(
            package='yolo_person_detection',
            executable='yolo_node',
            name='yolo_person_detector',
            output='screen',
            parameters=[{
                'model_path': '/home/agilex03/r26-campus-autonomy-ws/yolo11n_openvino_model',
                'image_topic': '/image_stitched',
                'device': 'cpu',
                'conf_threshold': 0.5,
                'iou_threshold': 0.45,
                'image_width': 3544,
                'crop_left': 480,
                'crop_right': 480,
                'use_sim_time': True
            }]
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

        # 启动 Nav2 的控制与规划层 (从而启动并可视化 Costmap)
        nav2_launch,
        
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
