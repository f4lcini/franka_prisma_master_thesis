from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    
    declare_use_hardware = DeclareLaunchArgument(
        'use_hardware', 
        default_value='false',
        description='If true, remap standard simulation camera topics to RealSense camera topics.'
    )

    reasoning_node = Node(
        package='fr3_application',
        executable='vlm_server_node',
        name='vlm_server_node',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/camera/image_raw', '/camera/color/image_raw')
        ] if use_hardware == 'true' else None
    )

    return LaunchDescription([
        declare_use_hardware,
        reasoning_node
    ])
