import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_swarm_bringup = get_package_share_directory('swarm_bringup')
    
    slam_config_file = os.path.join(pkg_swarm_bringup, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file,
            'use_sim_time': 'true'
        }.items()
    )

    rviz_config_file = os.path.join(pkg_swarm_bringup, 'rviz', 'slam.rviz') 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_launch,
        rviz_node
    ])