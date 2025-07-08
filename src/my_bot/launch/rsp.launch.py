import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch Arguments -----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) time'
    )

    # --- XACRO Processing ----------------------------------------------------
    pkg_share = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc_xml = robot_description_config.toxml()

    # --- Nodes ----------------------------------------------------------------
    # Robot State Publisher: publishes both /tf and /tf_static from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc_xml},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher
    ])
