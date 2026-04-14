import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
================================================================================
Author: Falco Robotics
Code Description: 
This is the master launch file that coordinates the bringup of the entire simulation 
and control environment. It sequences the start of the Gazebo low-level components 
and strictly delays the execution of MoveIt to ensure ROS 2 Action Servers are ready.

Pipeline: System Bringup & Orchestration

Implementation Steps Summary:
- NODE INITIALIZATION (Step 1): Resolve the package directory to construct paths.
- LOW-LEVEL SPAWN (Step 2): Include the simulation launch file to start Gazebo and Controllers.
- HIGH-LEVEL DELAY (Step 3): Pre-configure the MoveIt launch inclusion but delay its actual execution via TimerAction.
- LAUNCH EXECUTION (Step 4): Return the grouped launch description for ROS 2.
================================================================================
"""

def generate_launch_description():
    # Step 1: Resolve the package directory to construct paths.
    pkg_env = get_package_share_directory('franka_bimanual_config')

    # Step 2: Include the simulation launch file to start Gazebo and Controllers.
    sim_and_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'sim_and_control.launch.py'))
    )

    # Step 3 part 1: Pre-configure the MoveIt launch inclusion...
    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'moveit_core.launch.py'))
    )

    # Step 3 part 2: ...but delay its actual execution via TimerAction (8 seconds).
    delayed_moveit_core = TimerAction(
        period=8.0,
        actions=[moveit_core_launch]
    )

    # Step 4: Return the grouped launch description for ROS 2.
    return LaunchDescription([
        sim_and_control_launch,
        delayed_moveit_core,
    ])