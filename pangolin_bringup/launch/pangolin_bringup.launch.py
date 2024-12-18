import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 使用 FindPackageShare 以提高可移植性
    pangolin_control_dir = FindPackageShare('pangolin_control')
    pangolin_description_dir = FindPackageShare('pangolin_description')
    fdlink_ahrs_dir = FindPackageShare('fdlink_ahrs')
    isaac_ros_visual_slam_dir = FindPackageShare('pangolin_isaac_ros_visual_slam')
    isaac_ros_apriltag_dir = FindPackageShare('pangolin_isaac_ros_apriltag')
    apriltag_localize_dir = FindPackageShare('apriltag_localize')
    nav2_vslam_localize_dir = FindPackageShare('nav2_vslam_localize')
    navigation_dir = FindPackageShare('pangolin_navigation')
    pangolin_teleop_dir = FindPackageShare('pangolin_bringup')
    map_yaml_file = LaunchConfiguration('map')

    # 啟動文件
    pangolin_control_launch = PathJoinSubstitution(
        [pangolin_control_dir, 'launch', 'drive_controller.launch.py']
    )
    pangolin_state_publisher_launch = PathJoinSubstitution(
        [pangolin_description_dir, 'launch', 'pangolin.launch.py']
    )
    pangolin_teleop_launch = PathJoinSubstitution(
        [pangolin_teleop_dir, 'launch', 'teleop.launch.py']
    )
    ahrs_driver_launch = PathJoinSubstitution(
        [fdlink_ahrs_dir, 'launch', 'ahrs_driver.launch.py']
    )
    isaac_ros_visual_slam_launch = PathJoinSubstitution(
        [isaac_ros_visual_slam_dir, 'launch', 'isaac_ros_visual_slam_realsense.launch.py']
    )

    isaac_ros_apriltag_launch = PathJoinSubstitution(
        [isaac_ros_apriltag_dir, 'launch', 'isaac_ros_apriltag_realsense.launch.py']
    )

    apriltag_localize_launch = PathJoinSubstitution(
        [apriltag_localize_dir, 'launch', 'apriltag_localize.launch.py']
    )

    nav2_vslam_localize_launch = PathJoinSubstitution(
        [nav2_vslam_localize_dir, 'launch', 'nav2_vslam_localize.launch.py']
    )
    navigation_launch = PathJoinSubstitution(
        [navigation_dir, 'launch', 'bringup.launch.py']
    )

    def launch_setup(context, *args, **kwargs):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pangolin_control_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pangolin_state_publisher_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ahrs_driver_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pangolin_teleop_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(isaac_ros_visual_slam_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(isaac_ros_apriltag_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(apriltag_localize_launch)
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(nav2_vslam_localize_launch)
            # ),
            # Node(
            #     package='teleop_twist_keyboard', 
            #     executable='teleop_twist_keyboard',
            #     name='teleop_twist_keyboard_node', 
            # ),

            # 使用 TimerAction 延遲導航啟動
            # TimerAction(
            #     period=5.0,
            #     actions=[
            #         IncludeLaunchDescription(
            #             PythonLaunchDescriptionSource(navigation_launch)
            #         )
            #     ]
            # ),
            # battery state
            Node(
                package='pangolin_control',
                executable='pangolin_state',
                name='pangolin_state_node',
                output='screen',
                parameters=[]  
            ),
        ]

    # 創建並返回啟動描述
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])