from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_bringup',
            executable='bt_dispatcher_node',
            name='bt_dispatcher_node',
            output='screen',
            parameters=[{
                'bt_xml_file': '/home/l/swarm_explorer_ws/src/swarm_bringup/config/warehouse_task_tree.xml'
            }]
        )
    ])