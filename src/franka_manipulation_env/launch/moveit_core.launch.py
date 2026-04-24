import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

"""
================================================================================
Author: Falco Robotics
Code Description: 
This file exclusively handles the launch and parameter resolution for the MoveGroup 
node and its companion RViz interface. It dynamically compiles the robot's URDF/SRDF 
files and loads MoveIt kinematics, controllers, and OMPL overrides.

Pipeline: High-Level Path Planning (MoveIt)

Implementation Steps Summary:
- NODE INITIALIZATION (Step 1): Resolve required package directories.
- ROBOT DESCRIPTION (Step 2): Execute xacro to generate URDF and SRDF semantic representations dynamically.
- CONFIGURATION INGESTION (Step 3): Load core kinematics and basic controller YAML parameters.
- OMPL OVERRIDE MERGE (Step 4): Combine base OMPL parameters with custom environment overrides via dictionary updates.
- MoveGroup METADATA (Step 5): Package trajectory execution, planning pipelines, and MoveIt capabilities into objects.
- NODE DEFINITIONS (Step 6): Declare the MoveGroup node and the RViz2 UI node with their mapped arguments.
- LAUNCH EXECUTION (Step 7): Return the bundled node list to the ROS 2 Launch process.
================================================================================
"""

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    try:
        with open(os.path.join(package_path, file_path), 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Step 1: Resolve required package directories.
    pkg_env = get_package_share_directory('franka_bimanual_config')
    pkg_moveit = get_package_share_directory('franka_fr3_moveit_config')

    # Step 2: Execute xacro to generate URDF and SRDF semantic representations dynamically.
    xacro_file = os.path.join(pkg_env, 'urdf', 'system.urdf.xacro')
    robot_description = {"robot_description": ParameterValue(Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str)}

    srdf_file = os.path.join(pkg_env, 'srdf', 'fr3_sim_srdf.xacro')
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([FindExecutable(name="xacro"), " ", srdf_file, " name:=fr3 hand:=true"]), value_type=str)}

    # Step 3: Load core kinematics and basic controller YAML parameters.
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    moveit_controllers_yaml = load_yaml('franka_bimanual_config', 'config/moveit_controllers.yaml')

    # Step 4: Combine base OMPL parameters with custom environment overrides via dictionary updates.
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_override_yaml = load_yaml('franka_bimanual_config', 'config/ompl_planning_override.yaml')

    if ompl_base_yaml is None or ompl_override_yaml is None:
        raise FileNotFoundError("Errore fatale: impossibile localizzare i file di configurazione OMPL base o override.")

    ompl_combined = ompl_base_yaml.copy()
    ompl_combined.update(ompl_override_yaml)

    # Step 5: Package trajectory execution, planning pipelines, and MoveIt capabilities into objects.
    planning_pipeline_parameters = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_combined,
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

    # Step 6: Declare the MoveGroup node and the RViz2 UI node with their mapped arguments.
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_moveit, 'rviz', 'moveit.rviz')],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}],
    )

    # Step 7: Return the bundled node list to the ROS 2 Launch process.
    return LaunchDescription([move_group_node, rviz_node])