from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pangolin_visual_nav',
            executable='pangolin_visual_nav',
            name='pangolin_visual_nav_node',
            output='screen'
        ),
        Node(
            package='pangolin_visual_nav',
            executable='goal_pose_visualization',
            name='goal_pose_visualization_node',
            output='screen'
        ),
    ])
