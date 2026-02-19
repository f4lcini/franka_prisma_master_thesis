import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    try:
        with open(os.path.join(package_path, file_path), 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_manipulation_env')
    pkg_moveit = get_package_share_directory('franka_fr3_moveit_config')

    # 1. URDF e SRDF
    xacro_file = os.path.join(pkg_env, 'urdf', 'system.urdf.xacro')
    robot_description = {"robot_description": ParameterValue(Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str)}

    srdf_file = os.path.join(pkg_env, 'srdf', 'fr3_sim_srdf.xacro')
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([FindExecutable(name="xacro"), " ", srdf_file, " name:=fr3 hand:=true"]), value_type=str)}

    # 2. Parametri di Cinematica e Traiettoria
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    moveit_controllers_yaml = load_yaml('franka_manipulation_env', 'config/moveit_controllers.yaml')

    
    
    # CARICAMENTO COMBINATO OMPL (Base + Override)
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_override_yaml = load_yaml('franka_manipulation_env', 'config/ompl_planning_override.yaml')

    if ompl_base_yaml is None or ompl_override_yaml is None:
        raise FileNotFoundError("Errore fatale: impossibile localizzare i file di configurazione OMPL base o override.")

    # Unione dei dizionari (l'override sovrascrive o aggiunge chiavi al base)
    ompl_combined = ompl_base_yaml.copy()
    ompl_combined.update(ompl_override_yaml)

    planning_pipeline_parameters = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_combined, # <--- Passiamo il dizionario fuso
    }

    moveit_controller_parameters = {
        "moveit_manage_controllers": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controllers_yaml,
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

    # 3. Nodo Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_capabilities,
            {"use_sim_time": True},
            trajectory_execution_parameters,
            moveit_controller_parameters,
            planning_pipeline_parameters,
        ],
    )

    # 4. Nodo RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, 'rviz', 'moveit.rviz')],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}],
    )

    return LaunchDescription([move_group_node, rviz_node])