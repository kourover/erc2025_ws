#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg        = 'my_bot'
    pkg_share  = get_package_share_directory(pkg)
    pkg_prefix = get_package_prefix(pkg)

    # 1) Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2) Gazebo başlatma
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': os.path.join(pkg_share, 'worlds', 'empty.world')
        }.items()
    )

    # 3) Static TF (world → base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        arguments=['0','0','0','0','0','0','world','base_link'],
        output='screen'
    )

    # 4) URDF üzerinden robotu spawn et
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', pkg,
            '-x', '0.0', '-y', '0.0', '-z', '1.5'
        ]
    )

    # 5) Controller Manager
    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )
    joint_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    # 6) GoToPoint düğümü
    go_to_point = ExecuteProcess(
        cmd=[
            os.path.join(pkg_prefix, 'bin', 'go_to_point'),
            '--ros-args',
            '-p', 'target_x:=1.0',
            '-p', 'target_y:=4.0'
        ],
        output='screen'
    )

    # 7) SDF model spawn
    spawn_my_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_my_model_sdf',
        output='screen',
        arguments=[
            '-file', os.path.join(pkg_share, 'models', 'my_model', 'model.sdf'),
            '-entity', 'my_model',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    # 8) SLAM Toolbox (haritalandırma + anlık poz)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
        ]
    )
    zed2=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed2_optical_tf',
            output='screen',
            arguments=[
                '0.06', '0', '0',        # x, y, z offset
                '1.5708', '0', '1.5708',  # roll, pitch, yaw
                'zed2_camera_link',       # parent frame
                'zed2_right_optical_frame',# child frame
                '100'                     # publication rate
            ]
        )
    

  

    return LaunchDescription([
        rsp,
        gazebo,
        static_tf,
        spawn_robot,
        diff_spawner,
        joint_spawner,
        go_to_point,
        spawn_my_model,
        slam_toolbox,   # ← buraya eklendi
        zed2,
    ])
