import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':os.path.join(get_package_share_directory('swarm_bringup'), 'maps', 'warehouse_map.yaml'),
            'params_file':os.path.join(get_package_share_directory('swarm_bringup'), 'config', 'nav2_params.yaml'),
            'use_sim_time':'true',
            'autostart':'true'
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')]
    )

    return LaunchDescription([
        nav2,
        rviz
    ])