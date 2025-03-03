from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('/depth', '/camera/depth/image_rect_raw'),
                ('/depth_camera_info', '/camera/depth/camera_info'),
            ],
            parameters=[
                {'range_min': 0.2},           # Minimum range in meters
                {'range_max': 5.0},           # Maximum range in meters
                {'scan_height': 10},          # Number of rows in the depth image to process
                {'output_frame': 'camera_link'}  # Frame ID for the LaserScan
            ]
        )
    ])
