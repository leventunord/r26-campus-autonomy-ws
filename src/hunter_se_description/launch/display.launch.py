from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Find the URDF file
    hunter_se_description_path = FindPackageShare(package='hunter_se_description').find('hunter_se_description')
    urdf_file_path = PathJoinSubstitution([hunter_se_description_path, 'urdf', 'hunter_se_description.urdf'])
    # rviz_file_path = PathJoinSubstitution([hunter_se_description_path, 'rviz', 'model.rviz'])
    # Read the URDF file content
    with open(os.path.join(hunter_se_description_path, 'urdf', 'hunter_se_description.urdf'), 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file_path,
            description='URDF model file'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     # arguments=['-d', rviz_file_path]
        # ),
    ])
