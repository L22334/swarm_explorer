import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义包路径
    pkg_swarm_description = get_package_share_directory('swarm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_swarm_bringup = get_package_share_directory('swarm_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # 2. 定义文件路径
    xacro_file = os.path.join(pkg_swarm_description, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_swarm_description, 'worlds', 'warehouse.sdf')
    bridge_config_file = os.path.join(pkg_swarm_bringup, 'config', 'bridge.yaml')
    
    # === [关键配置 1] 地图路径 ===
    # 加载你刚才保存好的地图
    map_file = os.path.join(pkg_swarm_bringup, 'maps', 'warehouse_map.yaml')
    
    # === [关键配置 2] Nav2 参数文件 ===
    # 加载刚才拷贝过来的参数文件
    nav2_params_file = os.path.join(pkg_swarm_bringup, 'config', 'nav2_params.yaml')

    # 3. 设置 Gazebo 资源路径
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([os.path.dirname(pkg_swarm_description), ''])
    )

    # 4. 机器人模型发布 (Robot State Publisher)
    robot_description_content = Command(['xacro ', xacro_file])
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 5. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 6. 生成机器人实体
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'swarm_bot',
            '-x', '0.0', '-y', '0.0', '-z', '0.15'
        ]
    )

    # 7. 启动 ROS-Gazebo 桥接
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    # === [核心部分] 启动 Nav2 全家桶 ===
    # 这一步会启动 AMCL(定位), Planner(规划), Controller(控制), MapServer(地图)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,            # 传入地图
            'params_file': nav2_params_file, # 传入参数配置
            'use_sim_time': 'true',     # 使用仿真时间
            'autostart': 'true'         # 自动激活所有节点
        }.items()
    )

    # 8. 启动 RViz2 (加载 Nav2 专用配置，可选)
    # 
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        node_robot_state_publisher,
        node_spawn_entity,
        gazebo,
        node_ros_gz_bridge,
        nav2_launch,
        node_rviz 
    ])