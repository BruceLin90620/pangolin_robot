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
    map_yaml_file = LaunchConfiguration('map')

    # 啟動文件
    pangolin_control_launch = PathJoinSubstitution(
        [pangolin_control_dir, 'launch', 'drive_controller.launch.py']
    )
    pangolin_state_publisher_launch = PathJoinSubstitution(
        [pangolin_description_dir, 'launch', 'pangolin.launch.py']
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
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(ahrs_driver_launch)
            # ),
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
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch)
                    )
                ]
<<<<<<< HEAD
            ),
            # battery state
            Node(
                package='pangolin_control',
                executable='pangolin_state',
                name='pangolin_state_node',
                output='screen',
                parameters=[]  
            ),
=======
            )
>>>>>>> a7124cf98cc1936d07e2465d3109449fe67444c0
        ]

    # 創建並返回啟動描述
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])












# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node, PushRosNamespace
# from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from ament_index_python import get_package_share_directory
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     # Use FindPackageShare for better portability
#     pangolin_control_dir = FindPackageShare('pangolin_control')
#     pangolin_description_dir = FindPackageShare('pangolin_description')
#     fdlink_ahrs_dir = FindPackageShare('fdlink_ahrs')
#     isaac_ros_visual_slam_dir = FindPackageShare('isaac_ros_visual_slam')
#     nav2_vslam_localize_dir = FindPackageShare('nav2_vslam_localize')
#     navigation_dir = FindPackageShare('pangolin_navigation')
#     map_yaml_file = LaunchConfiguration('map')

    

#     # Launch files
#     pangolin_control_launch = PathJoinSubstitution(
#         [pangolin_control_dir, 'launch', 'drive_controller.launch.py']
#     )
#     pangolin_state_publisher_launch = PathJoinSubstitution(
#         [pangolin_description_dir, 'launch', 'pangolin.launch.py']
#     )
#     ahrs_driver_launch = PathJoinSubstitution(
#         [fdlink_ahrs_dir, 'launch', 'ahrs_driver.launch.py']
#     )
#     isaac_ros_visual_slam_launch = PathJoinSubstitution(
#         [isaac_ros_visual_slam_dir, 'launch', 'isaac_ros_visual_slam_realsense.launch.py']
#     )
#     nav2_vslam_localize_launch = PathJoinSubstitution(
#         [nav2_vslam_localize_dir, 'launch', 'nav2_vslam_localize.launch.py']
#     )
#     navigation_launch = PathJoinSubstitution(
#         [navigation_dir, 'launch', 'bringup.launch.py']
#     )

#     # Include launch files
#     pangolin_control_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(pangolin_control_launch)
#     )
#     pangolin_state_publisher_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(pangolin_state_publisher_launch)
#     )
#     ahrs_driver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(ahrs_driver_launch)
#     )
#     isaac_ros_visual_slam_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(isaac_ros_visual_slam_launch)
#     )
#     nav2_vslam_localize_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(nav2_vslam_localize_launch)
#     )
#     navigation_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(navigation_launch),
        
#     )


#     # # Teleop launch
#     teleop_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('teleop_twist_keyboard'), 'launch'),
#             '/teleop.launch.py']),
#         )
    
#     # Node(
#     #     package='teleop_twist_keyboard', 
#     #     executable='teleop_node',
#     #     name='teleop_twist_keyboard_node', 
#     #     ),


#     # Create and return launch description
#     return LaunchDescription([
#         pangolin_control_cmd,
#         # teleop_cmd,
#         pangolin_state_publisher_cmd,
#         # ahrs_driver_cmd,
#         isaac_ros_visual_slam_cmd,
#         nav2_vslam_localize_cmd,
#         navigation_cmd,
#     ])
