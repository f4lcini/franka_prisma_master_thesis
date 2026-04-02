import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

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
    
    # OMPL Planning Pipeline
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

    # Controllers
    moveit_controllers_yaml = load_yaml('franka_manipulation_env', 'config/moveit_controllers_bimanual.yaml')
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

    # Joint Limits
    joint_limits_yaml = load_yaml('franka_description', 'robots/fr3/joint_limits.yaml')

    valid_params = [p for p in [
        robot_description,
        robot_description_semantic,
        {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else None,
        {"robot_description_planning": joint_limits_yaml} if joint_limits_yaml else None,
        {"use_sim_time": True}, # Essential for Gazebo Execution Acceptance
        trajectory_execution_parameters,
        moveit_controller_parameters,
        planning_pipeline_parameters,
    ] if p is not None]

    mtc_node = Node(
        package="franka_task_orchestrator",
        executable="simple_moveit_server",
        name="python_moveit_server",
        output="screen",
        parameters=valid_params
    )

    # ── Static Transform Publishers (RViz visualization only) ──────────────────
    # Chain: world → camera/link (SDF body pose)
    #               → camera/link/rgb_camera (ROS optical rotation Rx(-π/2)Rz(-π/2))
    # ────────────────────────────────────────────────────────────────────────────

    # 1) world → camera/link  (body pose from SDF)
    static_tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_body_broadcaster',
        arguments=[
            '--x', '0.6', '--y', '1.0', '--z', '1.3',
            '--roll', '0', '--pitch', '0.785', '--yaw', '-1.57',
            '--frame-id', 'world',
            '--child-frame-id', 'camera/link'
        ]
    )

    # 2) camera/link → camera/link/rgb_camera  (ROS optical frame rotation)
    static_tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'camera/link',
            '--child-frame-id', 'camera/link/rgb_camera'
        ]
    )

    return LaunchDescription([
        mtc_node, 
        static_tf_body, 
        static_tf_optical
    ])

