# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': True,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90',
                'enable_gyro': False,
                'enable_accel': False,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2,
                #'pointcloud.enable': True,
        }]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_image_denoising': True,
                    'rectified_images': True,
                    'enable_planar_mode': True,
                    'enable_imu_fusion': False,
                    'enable_localization_n_mapping': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'image_jitter_threshold_ms': 45.00,
                    'base_frame': 'base_footprint',
                    'imu_frame': 'imu_link',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'camera_optical_frames': [
                        'camera_infra1_optical_frame',
                        'camera_infra2_optical_frame',
                    ],
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                    }],
        remappings=[('visual_slam/image_0', 'camera/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_0', 'camera/infra1/camera_info'),
                    ('visual_slam/image_1', 'camera/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_1', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', '/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container, realsense_camera_node])
