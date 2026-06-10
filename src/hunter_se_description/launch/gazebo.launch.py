from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    hunter_description_path = FindPackageShare(package='hunter_se_description').find('hunter_se_description')
    urdf_file = hunter_description_path + '/urdf/hunter_se_description.urdf'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'])
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-file', urdf_file, '-entity', 'hunter_se_description'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/Bool', 'data: true'],
            output='screen'
        ),
    ])
