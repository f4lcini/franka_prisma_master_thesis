from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    left_ip = LaunchConfiguration('left_ip')
    right_ip = LaunchConfiguration('right_ip')
    
    declare_left_ip = DeclareLaunchArgument(
        'left_ip', 
        default_value='192.168.9.11', 
        description='IP address of the left Franka robot.'
    )
    
    declare_right_ip = DeclareLaunchArgument(
        'right_ip', 
        default_value='192.168.9.12', 
        description='IP address of the right Franka robot.'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='trajectory',
        description='Type of controller to activate: "trajectory" for MoveIt/Pilz or "impedance" for teleop.'
    )

    # --- External Launch Inclusions ---
    
    # 1. Hardware Bringup (using custom bimanual config to avoid modifying library defaults)
    bringup_launch_dir = PathJoinSubstitution([FindPackageShare('franka_bimanual_config'), 'launch'])
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir, 'custom_bimanual.launch.py'])),
        launch_arguments={
            'left_ip': left_ip,
            'right_ip': right_ip,
            'ros2_control': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'controller_type': LaunchConfiguration('controller_type'),
        }.items()
    )


    return LaunchDescription([
        # Declare arguments
        declare_left_ip,
        declare_right_ip,
        declare_use_rviz,
        declare_controller_type,
        
        # Start hardware controllers
        hardware_bringup
    ])
