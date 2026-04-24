import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        full_path = os.path.join(package_path, file_path)
        with open(full_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None

def generate_launch_description():
    pkg_env_share = get_package_share_directory('franka_bimanual_config')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- 1. Load MoveIt Configuration Parameters ---
    
    # URDF (Robot Description)
    xacro_file_urdf = os.path.join(pkg_env_share, 'urdf', 'bimanual_custom.urdf.xacro')
    robot_description_config = Command(['xacro', ' ', xacro_file_urdf, ' ', 'hand:=true', ' ', 'gazebo:=false', ' ', 'ros2_control:=false'])
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    # SRDF (Robot Description Semantic)
    xacro_file_srdf = os.path.join(pkg_env_share, 'srdf', 'bimanual_sim_srdf.xacro')
    robot_description_semantic_config = Command(['xacro', ' ', xacro_file_srdf, ' ', 'name:=idra_bimanual_setup', ' ', 'hand:=true'])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}

    # Kinematics & Joint Limits
    kinematics_yaml = load_yaml('franka_bimanual_config', 'config/kinematics_bimanual.yaml')
    joint_limits_yaml = load_yaml('franka_bimanual_config', 'config/joint_limits_bimanual.yaml')

    # Planning Pipelines (Forcing Pilz)
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_combined = {}
    if ompl_base_yaml: ompl_combined.update(ompl_base_yaml)

    planning_pipeline_parameters = {
        "planning_pipelines": ["pilz_industrial_motion_planner", "ompl"],
        "default_planning_pipeline": "pilz_industrial_motion_planner",
        "ompl": ompl_combined,
        "pilz_industrial_motion_planner": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # --- 2. Define Nodes ---

    # Bimanual C++ Planner (Action Server: /parallel_move)
    planner_node = Node(
        package='franka_bimanual_planner',
        executable='bimanual_planner_node',
        name='bimanual_planner_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"robot_description_planning": joint_limits_yaml} if joint_limits_yaml else {},
            planning_pipeline_parameters,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Python Skill Server (Action Servers: move_home, pick_object, etc.)
    skill_server_node = Node(
        package='franka_bimanual_skills',
        executable='simple_moveit_server',
        name='simple_moveit_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        planner_node,
        skill_server_node
    ])
