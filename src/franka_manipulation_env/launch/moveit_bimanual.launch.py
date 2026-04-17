import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    try:
        # First try the installation directory
        full_path = os.path.join(package_path, file_path)
        if not os.path.exists(full_path):
            # Fallback to source directory if env variable or standard workspace structure exists
            ws_root = os.getcwd() # Assumes mm_ws
            full_path = os.path.join(ws_root, 'src', package_name, file_path)
            
        with open(full_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None

def generate_launch_description():
    pkg_idra_share = get_package_share_directory('idra_franka_launch')
    pkg_env_share = get_package_share_directory('franka_bimanual_config')
    
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')

    # Bimanual Robot Description (URDF) - CUSTOM VERSION WITH SYMMETRIC GRIPPER
    xacro_file_urdf = os.path.join(pkg_env_share, 'urdf', 'bimanual_custom.urdf.xacro')
    
    # CUSTOM CONTROLLER PATH
    custom_controller_path = os.path.join(pkg_env_share, 'config', 'basic_controllers_custom.yaml')
    
    robot_description_config = Command([
        'xacro ', xacro_file_urdf, 
        ' hand:=true', 
        ' gazebo:=', use_gazebo, 
        ' ros2_control:=true', 
        ' controller_path:=', custom_controller_path
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    # Bimanual SRDF
    xacro_file_srdf = os.path.join(pkg_env_share, 'srdf', 'bimanual_sim_srdf.xacro')
    robot_description_semantic_config = Command([
        'xacro ', xacro_file_srdf, 
        ' name:=idra_bimanual_setup', 
        ' hand:=true',
        ' gazebo:=', use_gazebo
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}

    # Kinematics
    kinematics_yaml = load_yaml('franka_bimanual_config', 'config/kinematics_bimanual.yaml')
    if kinematics_yaml is None:
         kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    
    moveit_controllers_yaml = load_yaml('franka_bimanual_config', 'config/moveit_controllers_bimanual.yaml')
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_override_yaml = load_yaml('franka_bimanual_config', 'config/ompl_planning_bimanual_override.yaml')

    ompl_combined = {}
    if ompl_base_yaml:
        ompl_combined.update(ompl_base_yaml)
    if ompl_override_yaml:
        ompl_combined.update(ompl_override_yaml)

    planning_pipeline_parameters = {
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_combined,
        "pilz_industrial_motion_planner": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "", # No adapters for Pilz to avoid CHOMP interference
            "start_state_max_bounds_error": 0.1,
            "default_acceleration_scaling_factor": 0.1,
            "default_velocity_scaling_factor": 0.1,
        }
    }

    moveit_controller_parameters = {
        "moveit_manage_controllers": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controllers_yaml if moveit_controllers_yaml else {},
    }

    trajectory_execution_parameters = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    moveit_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    joint_limits_yaml = load_yaml('franka_bimanual_config', 'config/joint_limits_bimanual.yaml')

    use_sim_time_arg = LaunchConfiguration('use_sim_time', default='false')

    # Filter out None parameters
    valid_params = [p for p in [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        {
            "robot_description_planning": joint_limits_yaml
        } if joint_limits_yaml else None,
        moveit_capabilities,
        {"use_sim_time": use_sim_time_arg},
        trajectory_execution_parameters,
        moveit_controller_parameters,
        planning_pipeline_parameters,
    ] if p is not None]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace="",
        emulate_tty=True,
        parameters=valid_params,
    )

    # Gazebo Sim Robot spawner
    gazebo_robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([pkg_env_share, 'urdf', 'bimanual_custom.urdf.xacro']),
                   '-name', 'idra_bimanual_setup',
                   '-allow_renaming', 'true',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
    )

    pkg_env = get_package_share_directory('franka_bimanual_config')
    rviz_config_file = os.path.join(pkg_env, 'rviz', 'bimanual_moveit.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit", # Unique name
        namespace="",
        arguments=["-d", rviz_config_file],
        parameters=valid_params, 
        emulate_tty=True,
        # Force start for debugging, ignore condition for now
    )

    # In case of real hardware, we don't want to spawn the controller_manager and robot_state_publisher again
    # as they are already managed by the hardware bringup.
    # However, MoveIt nodes still need the robot_description parameters.
    
    nodes_to_launch = [
        DeclareLaunchArgument('use_rviz', default_value='true', description='Visualize the robot in Rviz'),
        DeclareLaunchArgument('use_gazebo', default_value='false', description='Use Gazebo simulation'),
        move_group_node, 
        rviz_node,
    ]

    # Only add the spawner nodes if we are in Gazebo (Sim)
    # If we are in Lab, these are already running.
    def add_spawners(context):
        if context.perform_substitution(use_gazebo) == 'true':
            return [
                robot_description_dependent_nodes_spawner_opaque_function,
                spawn,
                load_joint_state_broadcaster,
                load_pose_broadcaster,
                load_position_broadcaster
            ]
        return []

    return LaunchDescription(nodes_to_launch + [OpaqueFunction(function=add_spawners)])
