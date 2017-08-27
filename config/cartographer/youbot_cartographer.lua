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
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    --sensor_bridge = {
    --    horizontal_laser_min_range = 0.,
    --    horizontal_laser_max_range = 5.,
    --    horizontal_laser_missing_echo_ray_length = 5.,
    --    constant_odometry_translational_variance = 0.,
    --    constant_odometry_rotational_variance = 0.,
    --},
    map_frame = "odom",
    tracking_frame = "base_link",
    published_frame = "base_link",
    odom_frame = "odom_map",
    provide_odom_frame = true,

    use_odometry = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 0,

    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

SPARSE_POSE_GRAPH.optimize_every_n_scans = 300
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.8


return options
