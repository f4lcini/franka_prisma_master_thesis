import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    
    # Camera Extrinsics parameters
    cam_x = LaunchConfiguration('camera_x')
    cam_y = LaunchConfiguration('camera_y')
    cam_z = LaunchConfiguration('camera_z')
    cam_roll = LaunchConfiguration('camera_roll')
    cam_pitch = LaunchConfiguration('camera_pitch')
    cam_yaw = LaunchConfiguration('camera_yaw')

    declare_use_hardware = DeclareLaunchArgument(
        'use_hardware', 
        default_value='false',
        description='If true, use hardware-specific topics.'
    )

    # Declare Camera Parameters for easy runtime calibration
    declare_cam_x = DeclareLaunchArgument('camera_x', default_value='0.6')
    declare_cam_y = DeclareLaunchArgument('camera_y', default_value='-0.6')
    declare_cam_z = DeclareLaunchArgument('camera_z', default_value='1.3')
    declare_cam_roll = DeclareLaunchArgument('camera_roll', default_value='0.0')
    declare_cam_pitch = DeclareLaunchArgument('camera_pitch', default_value='0.785')
    declare_cam_yaw = DeclareLaunchArgument('camera_yaw', default_value='1.57')

    common_parameters = {
        'camera_x': cam_x,
        'camera_y': cam_y,
        'camera_z': cam_z,
        'camera_roll': cam_roll,
        'camera_pitch': cam_pitch,
        'camera_yaw': cam_yaw,
    }

    perception_node_hw = Node(
        package='franka_bimanual_skills',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[common_parameters, {
            'image_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'use_sensor_data_qos': True
        }],
        condition=IfCondition(use_hardware)
    )

    perception_node_sim = Node(
        package='franka_bimanual_skills',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[common_parameters, {
            'image_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth',
            'camera_info_topic': '/camera/camera_info',
            'use_sensor_data_qos': False
        }],
        condition=UnlessCondition(use_hardware)
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[os.path.join(get_package_share_directory('franka_manipulation_env'), 'config', 'apriltag.yaml')],
        remappings=[
            ('image_rect', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ],
        condition=IfCondition(use_hardware)
    )

    return LaunchDescription([
        declare_use_hardware,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_cam_roll,
        declare_cam_pitch,
        declare_cam_yaw,
        perception_node_hw,
        perception_node_sim,
        apriltag_node,
    ])
