import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Teleop launch
    teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pangolin_bringup'), 'launch'),
            '/teleop.launch.py']),
        )
    # Bring-up motor control
    drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pangolin_control'), 'launch'),
            '/drive_controller.launch.py']),
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(teleop_cmd)
    ld.add_action(drive_controller)
    return ld