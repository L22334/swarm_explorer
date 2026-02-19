import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_swm_bringup = get_package_share_directory('swarm_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_swm_description = get_package_share_directory('swarm_description')

    world_file = os.path.join(pkg_swm_description, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_swm_description, 'urdf', 'robot.urdf.xacro')
    
    gazebo_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', 
        os.path.dirname(pkg_swm_description)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    ) 

    # 4. Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 5. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'swarm_bot',
            '-x', '0.0', '-y', '0.0', '-z', '0.15'
        ]
    )

    # 6. Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_swm_bringup, 'config', 'bridge.yaml'),
            'use_sim_time': True
        }]               
    )




    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge
    ])