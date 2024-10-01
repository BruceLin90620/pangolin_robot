import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # # Declare launch arguments
        # DeclareLaunchArgument('camera_device', default_value='/dev/video0', description='Path to camera device'),
        # DeclareLaunchArgument('camera_resolution', default_value='1280x720', description='Camera resolution'),
        # DeclareLaunchArgument('camera_frame_rate', default_value='30', description='Camera frame rate'),

        # # Start mjpg-streamer
        # ExecuteProcess(
        #     cmd=['/home/ubuntu/mjpg-streamer/mjpg-streamer-experimental', '-i', 'input_uvc.so', '-d', LaunchConfiguration('camera_device'),
        #          '-r', LaunchConfiguration('camera_resolution'), '-f', LaunchConfiguration('camera_frame_rate'),
        #          '-o', 'output_http.so', '-p', '8080'],
        #     output='screen',
        # ),


        # Node(
        #     package='pangolin_control',
        #     executable='pangolin_imu',
        #     name='pangolin_imu',
        #     output='screen'
        # ),
        Node(
            package='pangolin_control',
            executable='pangolin_control',
            name='pangolin_control',
            output='screen'
        ),
        # Node(
        #     package='pangolin_control',
        #     executable='pangolin_action',
        #     name='pangolin_action',
        #     output='screen'),    
    ])