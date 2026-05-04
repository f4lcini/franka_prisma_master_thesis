import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

"""
================================================================================
Author: Falco Robotics
Code Description: 
This is the heavy-lifting simulation launch file. It configures Gazebo environment 
variables, spawns the FR3 robot system, injects specific user-defined scene objects 
(like the target cube), and dynamically loads the base ROS 2 controllers.

Pipeline: Low-Level Physics & Control (Gazebo)

Implementation Steps Summary:
- ARGUMENT INGESTION (Step 1): Declare the launch argument to accept the initial joint pose input.
- ENVIRONMENT SETUP (Step 2): Set indispensable Gazebo plugin and resource paths via append variables.
- ROBOT STATE PUBLISHER (Step 3): Parse the XACRO with initial pose and launch the Robot State Publisher to broadcast TFs.
- GAZEBO BRINGUP (Step 4): Include the core Ignition/Gazebo simulator pointing to an empty empty.sdf world.
- ENTITY SPAWNING (Step 5): Use ros_gz_sim create endpoints to inject the fr3_system and the target_cube into physics.
- HARDWARE CONTROLLERS (Step 6): Use controller_manager spawners to load the joint broadcaster, arm impedance, and gripper.
- BRIDGE & LAUNCH EXECUTION (Step 7): Include the GZ-ROS bridge for clock sync and bundle all nodes with a delayed timer.
================================================================================
"""

def generate_launch_description():
    pkg_franka_bimanual_config = get_package_share_directory('franka_bimanual_config')
    pkg_franka_desc = get_package_share_directory('franka_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Step 1: Declare the launch argument to accept the initial joint pose input.
    pose_arg = DeclareLaunchArgument(
        'initial_pose',
        default_value='j1:=0.0 j2:=-0.785 j3:=0.0 j4:=-2.356 j5:=0.0 j6:=1.571 j7:=0.785',
        description='Argomenti Xacro per la posa iniziale'
    )

    # Step 2: Set indispensable Gazebo plugin and resource paths via append variables.
    resources = [os.path.dirname(pkg_franka_desc), ':', os.path.dirname(pkg_franka_bimanual_config)]
    env_vars = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resources),
        AppendEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resources),
        AppendEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib'),
        AppendEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib')
    ]

    # Step 3: Parse the XACRO with initial pose and launch the Robot State Publisher to broadcast TFs.
    xacro_file = os.path.join(pkg_franka_bimanual_config, 'urdf', 'system.urdf.xacro')
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " ", LaunchConfiguration('initial_pose')]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Step 4: Include the core Ignition/Gazebo simulator pointing to an empty empty.sdf world.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Step 5: Use ros_gz_sim create endpoints to inject the fr3_system and the target_cube into physics.
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fr3_system', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.0'],
        output='screen',
    )

    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'target_cube', '-file', os.path.join(pkg_franka_bimanual_config, 'models', 'cube', 'model.sdf'), '-x', '0.6', '-y', '0.0', '-z', '0.775'],
        output='screen',
    )
    
    # Step 6: Use controller_manager spawners to load the joint broadcaster, arm impedance, and gripper.
    load_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    load_jtc = Node(package="controller_manager", executable="spawner", arguments=["fr3_arm_controller"], output="screen")
    load_grip = Node(package="controller_manager", executable="spawner", arguments=["franka_gripper"], output="screen")
    
    # Step 7: Include the GZ-ROS bridge for clock sync and bundle all nodes with a delayed timer.
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen')

    return LaunchDescription(
        [pose_arg] + env_vars + [
        rsp, gazebo, spawn_system, spawn_cube, bridge,
        TimerAction(period=5.0, actions=[load_jsb, load_jtc, load_grip])
    ])