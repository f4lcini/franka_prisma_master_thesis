import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock. Set true for Gazebo, false for real hardware.'
    )

    pkg_idra_share = get_package_share_directory('idra_franka_launch')
    pkg_env_share = get_package_share_directory('franka_bimanual_config')

    # Load Robot Poses from YAML
    robot_poses_file = os.path.join(pkg_env_share, 'config', 'robot_poses.yaml')
    with open(robot_poses_file, 'r') as f:
        robot_poses = yaml.safe_load(f)

    # Bimanual Robot Description (URDF)
    xacro_file_urdf = os.path.join(pkg_env_share, 'urdf', 'bimanual_custom.urdf.xacro')
    
    # Construct xacro command with all arguments
    xacro_args = [
        'xacro', xacro_file_urdf,
        'hand:=true', 'gazebo:=true', 'ros2_control:=false',
        f"table_x:={robot_poses['table']['x']}",
        f"table_y:={robot_poses['table']['y']}",
        f"table_z:={robot_poses['table']['z']}",
        f"table_roll:={robot_poses['table']['roll']}",
        f"table_pitch:={robot_poses['table']['pitch']}",
        f"table_yaw:={robot_poses['table']['yaw']}",
        f"franka1_x:={robot_poses['franka1']['x']}",
        f"franka1_y:={robot_poses['franka1']['y']}",
        f"franka1_z:={robot_poses['franka1']['z']}",
        f"franka1_yaw:={robot_poses['franka1']['yaw']}",
        f"franka2_x:={robot_poses['franka2']['x']}",
        f"franka2_y:={robot_poses['franka2']['y']}",
        f"franka2_z:={robot_poses['franka2']['z']}",
        f"franka2_yaw:={robot_poses['franka2']['yaw']}",
        f"shared_x:={robot_poses['shared_zone']['x']}",
        f"shared_y:={robot_poses['shared_zone']['y']}",
        f"shared_z:={robot_poses['shared_zone']['z']}"
    ]
    
    robot_description_config = Command(xacro_args)
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    # Bimanual SRDF
    xacro_file_srdf = os.path.join(pkg_env_share, 'srdf', 'bimanual_sim_srdf.xacro')
    robot_description_semantic_config = Command(['xacro', ' ', xacro_file_srdf, ' ', 'name:=idra_bimanual_setup', ' ', 'hand:=true'])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}

    # Kinematics
    kinematics_yaml = load_yaml('franka_bimanual_config', 'config/kinematics_bimanual.yaml')
    if kinematics_yaml is None:
         kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    
    # OMPL Planning Pipeline
    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_override_yaml = load_yaml('franka_bimanual_config', 'config/ompl_planning_bimanual_override.yaml')

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
    moveit_controllers_yaml = load_yaml('franka_bimanual_config', 'config/moveit_controllers_bimanual.yaml')
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
        {"use_sim_time": use_sim_time},
        trajectory_execution_parameters,
        moveit_controller_parameters,
        planning_pipeline_parameters,
    ] if p is not None]

    # ── MoveIt Python Action Server ──────────────────────────────────────────
    # Serves: /pick, /place, /move_home, /give_object, /take_object actions
    moveit_server_node = Node(
        package="franka_bimanual_skills",
        executable="simple_moveit_server",
        name="python_moveit_server",
        output="screen",
        parameters=valid_params
    )

    # ── Handover Coordinator ─────────────────────────────────────────────────
    # Services: /donor_ready + /recipient_ready  (rendezvous sync for handover)
    handover_coordinator_node = Node(
        package="franka_bimanual_skills",
        executable="handover_coordinator",
        name="handover_coordinator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # ── Static Transform Publishers (camera TF for RViz) ─────────────────────
    # Exposed as arguments for calibration alignment
    cam_x = LaunchConfiguration('camera_x', default='0.6')
    cam_y = LaunchConfiguration('camera_y', default='1.0')
    cam_z = LaunchConfiguration('camera_z', default='1.3')
    cam_roll = LaunchConfiguration('camera_roll', default='0.0')
    cam_pitch = LaunchConfiguration('camera_pitch', default='0.785')
    cam_yaw = LaunchConfiguration('camera_yaw', default='-1.57')

    declare_cam_x = DeclareLaunchArgument('camera_x', default_value='0.6')
    declare_cam_y = DeclareLaunchArgument('camera_y', default_value='1.0')
    declare_cam_z = DeclareLaunchArgument('camera_z', default_value='1.3')
    declare_cam_roll = DeclareLaunchArgument('camera_roll', default_value='0.0')
    declare_cam_pitch = DeclareLaunchArgument('camera_pitch', default_value='0.785')
    declare_cam_yaw = DeclareLaunchArgument('camera_yaw', default_value='-1.57')

    static_tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_body_broadcaster',
        arguments=[
            '--x', cam_x, '--y', cam_y, '--z', cam_z,
            '--roll', cam_roll, '--pitch', cam_pitch, '--yaw', cam_yaw,
            '--frame-id', 'world',
            '--child-frame-id', 'camera/link'
        ]
    )

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
        declare_use_sim_time,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_cam_roll,
        declare_cam_pitch,
        declare_cam_yaw,
        moveit_server_node,
        handover_coordinator_node,
        static_tf_body,
        static_tf_optical
    ])
