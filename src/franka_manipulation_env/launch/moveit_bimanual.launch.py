import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    try:
        with open(os.path.join(package_path, file_path), 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None

def generate_launch_description():
    pkg_idra_share = get_package_share_directory('idra_franka_launch')
    pkg_env_share = get_package_share_directory('franka_manipulation_env')
    
    use_rviz_arg = LaunchConfiguration('use_rviz', default='true')

    # Bimanual Robot Description (URDF)
    xacro_file_urdf = os.path.join(pkg_idra_share, 'urdf', 'bimanual.urdf.xacro')
    robot_description_config = Command(['xacro', ' ', xacro_file_urdf, ' ', 'hand:=true', ' ', 'gazebo:=true', ' ', 'ros2_control:=false'])
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    # Bimanual SRDF
    xacro_file_srdf = os.path.join(pkg_env_share, 'srdf', 'bimanual_sim_srdf.xacro')
    robot_description_semantic_config = Command(['xacro', ' ', xacro_file_srdf, ' ', 'name:=idra_bimanual_setup', ' ', 'hand:=true'])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}

    # Kinematics
    kinematics_yaml = load_yaml('franka_manipulation_env', 'config/kinematics_bimanual.yaml')
    if kinematics_yaml is None:
         kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    
    moveit_controllers_yaml = load_yaml('franka_manipulation_env', 'config/moveit_controllers_bimanual.yaml')
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_override_yaml = load_yaml('franka_manipulation_env', 'config/ompl_planning_bimanual_override.yaml')

    ompl_combined = {}
    if ompl_base_yaml:
        ompl_combined.update(ompl_base_yaml)
    if ompl_override_yaml:
        ompl_combined.update(ompl_override_yaml)

    planning_pipeline_parameters = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_combined,
    }

    moveit_controller_parameters = {
        "moveit_manage_controllers": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controllers_yaml if moveit_controllers_yaml else {},
    }

    trajectory_execution_parameters = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    moveit_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Filter out None parameters
    valid_params = [p for p in [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        moveit_capabilities,
        {"use_sim_time": True},
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

    pkg_env = get_package_share_directory('franka_manipulation_env')
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

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Visualize the robot in Rviz'),
        move_group_node, 
        rviz_node
    ])
