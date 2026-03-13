from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fr3_application',
            executable='vlm_server_node',
            name='vlm_server_node',
            output='screen',
            emulate_tty=True
        )
    ])
