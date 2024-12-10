import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

    pkg_share = FindPackageShare(package="pangolin_description").find("pangolin_description")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'pangolin_urdf_20240701.urdf'
    default_rviz_path = os.path.join(pkg_share, "rviz/pangolin_viz.rviz")


    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('pangolin_description'),
        'urdf',
        urdf_file_name)

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            # condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
        ),
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui",
        #     # condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
        # ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d" + default_rviz_path]
            # condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
        ),

    ])