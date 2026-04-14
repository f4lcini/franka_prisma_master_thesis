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
        default_value='192.168.1.11', # Default lab IP for left robot
        description='IP address of the left Franka robot.'
    )
    
    declare_right_ip = DeclareLaunchArgument(
        'right_ip', 
        default_value='192.168.1.12', # Default lab IP for right robot
        description='IP address of the right Franka robot.'
    )

    # --- External Launch Inclusions ---
    
    # 1. Hardware Bringup (from idra_franka_launch package)
    bringup_launch_dir = PathJoinSubstitution([FindPackageShare('idra_franka_launch'), 'launch'])
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir, 'bimanual.launch.py'])),
        launch_arguments={
            'left_ip': left_ip,
            'right_ip': right_ip,
        }.items()
    )



    return LaunchDescription([
        # Declare arguments
        declare_left_ip,
        declare_right_ip,
        
        # Start hardware controllers
        hardware_bringup
    ])
