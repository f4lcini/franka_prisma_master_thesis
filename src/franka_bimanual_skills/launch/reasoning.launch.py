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

    # Hardware mode: RealSense topics and Best Effort QoS
    reasoning_node_hw = Node(
        package='franka_bimanual_skills',
        executable='vlm_server_node',
        name='vlm_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'use_sensor_data_qos': True
        }],
        condition=IfCondition(use_hardware)
    )

    # Simulation mode: standard topics and Reliable QoS
    reasoning_node_sim = Node(
        package='franka_bimanual_skills',
        executable='vlm_server_node',
        name='vlm_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/image_raw',
            'use_sensor_data_qos': False
        }],
        condition=UnlessCondition(use_hardware)
    )

    return LaunchDescription([
        declare_use_hardware,
        reasoning_node_hw,
        reasoning_node_sim,
    ])
