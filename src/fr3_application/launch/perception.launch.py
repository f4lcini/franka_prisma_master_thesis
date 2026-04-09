from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    
    declare_use_hardware = DeclareLaunchArgument(
        'use_hardware', 
        default_value='false',
        description='If true, remap standard simulation camera topics to RealSense camera topics.'
    )

    # Hardware mode: remap to RealSense topics
    perception_node_hw = Node(
        package='fr3_application',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/camera/image_raw', '/camera/color/image_raw'),
            ('/camera/depth', '/camera/depth/image_rect_raw'),
            ('/camera/camera_info', '/camera/depth/camera_info')
        ],
        condition=IfCondition(use_hardware)
    )

    # Simulation mode: no remapping needed
    perception_node_sim = Node(
        package='fr3_application',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_hardware)
    )

    return LaunchDescription([
        declare_use_hardware,
        perception_node_hw,
        perception_node_sim,
    ])
