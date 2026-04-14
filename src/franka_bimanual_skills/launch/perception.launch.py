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
    perception_node_hw = Node(
        package='franka_bimanual_skills',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'use_sensor_data_qos': True
        }],
        condition=IfCondition(use_hardware)
    )

    # Simulation mode: standard topics and Reliable QoS
    perception_node_sim = Node(
        package='franka_bimanual_skills',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth',
            'camera_info_topic': '/camera/camera_info',
            'use_sensor_data_qos': False
        }],
        condition=UnlessCondition(use_hardware)
    )

    return LaunchDescription([
        declare_use_hardware,
        perception_node_hw,
        perception_node_sim,
    ])
