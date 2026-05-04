import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

"""
================================================================================
Author: Falco Robotics
Code Description: 
This launch file acts as a mid-level wrapper. It takes the specific joint starting 
pose for the FR3 robot and delegates the actual spawning of Gazebo entities to 
`sim_objects.launch.py`. It also injects a static TF publisher to bridge the camera.

Pipeline: Simulation Wrapper

Implementation Steps Summary:
- NODE INITIALIZATION (Step 1): Specify the initial configuration `MY_ROBOT_POSE` as an environment string.
- OBJECT SPAWN DELEGATION (Step 2): Include the hardware simulation launch file passing the initial pose parameter.
- CAMERA TF INJECTION (Step 3): Declare a static TF publisher for the camera_link to correctly anchor depth tracking.
- LAUNCH EXECUTION (Step 4): Return the combined launch description.
================================================================================
"""

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_bimanual_config')
    
    # Step 1: Specify the initial configuration `MY_ROBOT_POSE` as an environment string.
    MY_ROBOT_POSE = "j1:=0.0 j2:=-0.785 j3:=0.0 j4:=-2.356 j5:=0.0 j6:=1.571 j7:=0.785"


    # Step 2: Include the hardware simulation launch file passing the initial pose parameter.
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'sim_objects.launch.py')),
        launch_arguments={'initial_pose': MY_ROBOT_POSE}.items(),
    )

    # Step 3: Declare a static TF publisher for the camera_link to correctly anchor depth tracking.
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'fr3_system/camera_link/camera']
    )

    # Step 4: Return the combined launch description.
    return LaunchDescription([
        sim_launch,
        camera_tf_node,
    ])