import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_manipulation_env')

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    from launch.actions import LogInfo
    # 1. Include the bimanual simulation and control launch
    sim_bimanual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'custom_bimanual.launch.py')),
        launch_arguments={
            'use_gazebo': use_gazebo,
            'use_rviz': 'false'
        }.items(),
    )

    # 2. Include the bimanual MoveIt configuration
    moveit_bimanual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'moveit_bimanual.launch.py')),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    # 3. Delay MoveIt execution
    delayed_moveit_bimanual = TimerAction(
        period=12.0,
        actions=[moveit_bimanual_launch]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true', description='Launch Gazebo'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Visualize in Rviz'),
        LogInfo(msg=["Main launch use_rviz: ", use_rviz]),
        sim_bimanual_launch,
        delayed_moveit_bimanual,
    ])
