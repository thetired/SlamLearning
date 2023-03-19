-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,    -- trajectory_builder.lua的配置信息
  map_frame = "map",                                                    -- 地图坐标系的名字
  tracking_frame = "base_link",  
  ---tracking_frame = "gyro_link",                      --将所有传感器数据转换到这个坐标系下 imu_link或者base_link
  published_frame = "odom_combined",              -- cartographer与tf树接上  ，tf树最上边的坐标系
  odom_frame = "odom_combined",                       --里程计坐标系的名称
  provide_odom_frame = false,                                   -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
  publish_frame_projected_to_2d = false,            -- 是否将坐标系投影到平面上
  use_pose_extrapolator = false,                                -- 是否使用位姿推测器
  use_odometry = true,                                                  --使用里程计， gps， landmark
  use_nav_sat = false,          
  use_landmarks = false,
  ---------------
  num_laser_scans = 1,                                                   -- laser相关配置参数
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,               -- 1帧数据被分成几次处理 ,cartographer 10
  num_point_clouds = 0,
  -- ----
  lookup_transform_timeout_sec = 0.2,                   --查找tf超时时间
  submap_publish_period_sec = 0.3,                         -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,                              --传感器的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35   --35
TRAJECTORY_BUILDER_2D.min_range = 0.3
-- 雷达最大距离，太小了可能导致扫不到，但是叠图不严重
TRAJECTORY_BUILDER_2D.max_range = 13. 
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
-- 是否使用imu
--TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- 相信先验位姿还是与地图匹配位姿态
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.


POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 70         --35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
