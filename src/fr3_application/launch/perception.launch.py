from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fr3_application',
            executable='object_localization_node',
            name='object_localization_node',
            output='screen',
            emulate_tty=True
        )
    ])
