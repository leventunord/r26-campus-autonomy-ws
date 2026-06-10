# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pointcloud_to_laserscan',
                    plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode)',
                    name='ptcld_2_lsr',
                    remappings=[('cloud_in', ['/lidar_points']),
                        ('scan', ['/scan'])],
                    parameters=[{
                        'target_frame': 'hesai_lidar',
                        'transform_tolerance': 0.01,
                        'min_height': -0.2,
                        'max_height': 1.0,
                        'angle_min': -3.1416,  # -M_PI/2
                        'angle_max': 3.1416,  # M_PI/2
                        'angle_increment': 0.0087,  # M_PI/360.0
                        'scan_time': 0.3333,
                        'range_min': 0.45,
                        'range_max': 4.0,
                        'use_inf': True,
                        'inf_epsilon': 1.0
                    }],
                ),
                ComposableNode(
                    package='hesai_ros_driver',
                    plugin='composition::Listener',
                    name='hesai_ros_driver')
            ],
            output='screen',
    )

return launch.LaunchDescription([container])




