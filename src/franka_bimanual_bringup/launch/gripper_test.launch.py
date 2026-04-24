"""
gripper_test.launch.py

Tests gripper open/close cycles for a single arm.

Usage:
  ros2 launch franka_bimanual_bringup gripper_test.launch.py arm:=left_arm
  ros2 launch franka_bimanual_bringup gripper_test.launch.py arm:=right_arm cycles:=5
  ros2 launch franka_bimanual_bringup gripper_test.launch.py arm:=right_arm open_width:=0.07 close_width:=0.01
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'arm',
            default_value='right_arm',
            description="Which gripper to test: 'left_arm' or 'right_arm'"
        ),
        DeclareLaunchArgument(
            'cycles',
            default_value='3',
            description='Number of open/close cycles'
        ),
        DeclareLaunchArgument(
            'open_width',
            default_value='0.08',
            description='Gripper open width in metres (max = 0.08)'
        ),
        DeclareLaunchArgument(
            'close_width',
            default_value='0.0',
            description='Gripper close width in metres (0.0 = fully closed)'
        ),
        DeclareLaunchArgument(
            'cycle_pause',
            default_value='2.0',
            description='Seconds to hold each position before toggling'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'franka_bimanual_bringup', 'gripper_test.py',
                '--arm',   LaunchConfiguration('arm'),
                '--cycles',LaunchConfiguration('cycles'),
                '--open',  LaunchConfiguration('open_width'),
                '--close', LaunchConfiguration('close_width'),
                '--pause', LaunchConfiguration('cycle_pause'),
            ],
            output='screen',
        ),
    ])
