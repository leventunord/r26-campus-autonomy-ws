from setuptools import find_packages, setup

package_name = 'pedestrian_trajectory_prediction'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pedestrian_prediction.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongjin Wang',
    maintainer_email='todo@example.com',
    description='Pedestrian trajectory extraction and prediction from /yolo/detections and /lidar_points',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_dynamic_costmap = pedestrian_trajectory_prediction.dynamic_costmap_node:main',
            'pedestrian_prediction_node = pedestrian_trajectory_prediction.pedestrian_prediction_node:main',
        ],
    },
)
