import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup_share = get_package_share_directory('swarm_bringup')

    bt_xml_path = os.path.join(pkg_bringup_share, 'config', 'warehouse_task_tree.xml')

    return LaunchDescription([
        Node(
            package='swarm_bringup',
            executable='bt_dispatcher_node',
            name='bt_dispatcher_node',
            output='screen',
            parameters=[{
                'bt_xml_file': bt_xml_path
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav')
            ]
        )
    ])