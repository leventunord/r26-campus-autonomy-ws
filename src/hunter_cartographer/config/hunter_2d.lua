-- Cartographer 2D SLAM 配置 for Hunter SE robot (无 IMU，轮式里程计)
-- 适配自 zzbot_2d.lua，针对同一台机器人硬件

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- published_frame=odom: Cartographer 发布 map->odom 的 TF
  -- hunter_base 已发布 odom->base_link 的 TF，两者不冲突
  published_frame = "odom",
  odom_frame = "odom",
  -- provide_odom_frame=false: 不提供 odom TF（hunter_base 已提供）
  provide_odom_frame = false,
  -- 仅发布 2D 位姿
  publish_frame_projected_to_2d = true,
  -- 使用轮式里程计数据（/odom 话题）
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- 使用一个激光雷达
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  -- 轮式里程计不准，降低采样频率
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 启用 2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.3
-- 限制在激光雷达最大扫描范围内
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- 传感器数据超出有效范围的最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
-- 不使用 IMU 数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 使用在线回环检测进行前端扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 约束构建器：Fast csm 最低分数
POSE_GRAPH.constraint_builder.min_score = 0.65
-- 全局定位最小分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 自适应体素滤波器
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 100

-- 扫描匹配参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- 运动滤波器
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5

-- 优化频率
POSE_GRAPH.optimize_every_n_nodes = 35

-- Huber 尺度
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
