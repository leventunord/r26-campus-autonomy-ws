# pedestrian_trajectory_prediction


## Inputs

- `/yolo/detections` (`vision_msgs/msg/Detection2DArray`)
- `/lidar_points` (`sensor_msgs/msg/PointCloud2`)

## Outputs

- `/pedestrian/current_poses` (`geometry_msgs/msg/PoseArray`)
- `/pedestrian/predicted_poses` (`geometry_msgs/msg/PoseArray`)
- `/pedestrian/predictions_flat` (`std_msgs/msg/Float32MultiArray`)
  - repeated rows: `[track_id, future_time, x, y, vx, vy]`
- `/pedestrian/trajectory_markers` (`visualization_msgs/msg/MarkerArray`)
  - current pedestrian positions, IDs, history traces, and future prediction traces

## Run

```bash
cd ~/r26-campus-autonomy-ws
colcon build --symlink-install --packages-select pedestrian_trajectory_prediction
source install/setup.bash
ros2 launch pedestrian_trajectory_prediction pedestrian_prediction.launch.py
```

Then play the rosbag and open `direction_image_lidar.rviz` or RViz2 with fixed frame
`base_link`. Add a MarkerArray display for `/pedestrian/trajectory_markers`.
