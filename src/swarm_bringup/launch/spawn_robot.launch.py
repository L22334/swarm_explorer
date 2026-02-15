import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_swarm_description = get_package_share_directory('swarm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_swarm_bringup = get_package_share_directory('swarm_bringup')
    pkg_salm = get_package_share_directory('rtabmap_launch')
    
    # 2. 定义配置文件路径
    bridge_config_file = os.path.join(pkg_swarm_bringup, 'config', 'bridge.yaml')
    world_file = os.path.join(pkg_swarm_description, 'worlds', 'warehouse.sdf')
    xacro_file = os.path.join(pkg_swarm_description, 'urdf', 'robot.urdf.xacro')

    # 解析 Xacro
    robot_description_content = Command(['xacro ', xacro_file])

    # 设置环境路径
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([os.path.dirname(pkg_swarm_description), ''])
    )

    # 3. 启动 Gazebo 仿真
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. RTAB-Map SLAM 配置 (优化版)
    salm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_salm,'launch','rtabmap.launch.py')),
        launch_arguments={
            'frame_id': 'base_footprint',
            'use_sim_time': 'true',
            'subscribe_depth': 'true',
            'subscribe_scan': 'true',
            # 话题映射 (需与 bridge.yaml 一致)
            'rgb_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth_topic': '/camera/depth/image_raw',
            'scan_topic': '/scan',
            'rtabmap_viz': 'false',
            # 核心参数调整：降低配准所需的最小内点数，启用内存管理
            'args': '--delete_db_on_start --Vis/MinInliers 12 --Vis/InlierDistance 0.1 --RGBD/OptimizeFromGraphEnd true',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'queue_size': '30',
            'qos': '2'
        }.items()
    )

    # 5. 生成机器人模型
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'swarm_bot', '-z', '0.2']
    )

    # 6. 发布机器人状态 (TF 树)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content, 
            'use_sim_time': True,
            'publish_frequency': 50.0 # 提高发布频率确保 TF 连续
        }]
    )

    # 7. Gazebo 与 ROS 2 桥接
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file, 
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        salm
    ])