import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_apriltag = LaunchConfiguration('use_apriltag')
    
    # Load robot poses from YAML
    pkg_env = get_package_share_directory('franka_bimanual_config')
    import yaml
    with open(os.path.join(pkg_env, 'config', 'robot_poses.yaml'), 'r') as f:
        robot_poses = yaml.safe_load(f)

    common_parameters = {
        'camera_x': float(robot_poses['camera']['x']),
        'camera_y': float(robot_poses['camera']['y']),
        'camera_z': float(robot_poses['camera']['z']),
        'camera_roll': float(robot_poses['camera']['roll']),
        'camera_pitch': float(robot_poses['camera']['pitch']),
        'camera_yaw': float(robot_poses['camera']['yaw']),
    }

    declare_use_hardware = DeclareLaunchArgument(
        'use_hardware', 
        default_value='false',
        description='If true, use hardware-specific topics.'
    )

    declare_use_apriltag = DeclareLaunchArgument(
        'use_apriltag', 
        default_value='false',
        description='If true, launch apriltag_ros node.'
    )

    perception_node_hw = Node(
        package='franka_bimanual_skills',
        executable='object_localization_node',
        name='object_localization_node',
        output='screen',
        emulate_tty=True,
        parameters=[common_parameters, {
            'image_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
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
        parameters=[os.path.join(get_package_share_directory('franka_bimanual_config'), 'config', 'apriltag.yaml')],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ],
        condition=IfCondition(use_apriltag)
    )

    return LaunchDescription([
        declare_use_hardware,
        declare_use_apriltag,
        perception_node_hw,
        perception_node_sim,
        apriltag_node,
    ])
