import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    Shutdown
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        use_fake_hardware,
        load_gripper,
        left_ip,
        right_ip
    ):

    pkg_config = get_package_share_directory('franka_bimanual_config')
    
    load_gripper_str = context.perform_substitution(load_gripper)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    left_ip_str = context.perform_substitution(left_ip)
    right_ip_str = context.perform_substitution(right_ip)

    # 1. URDF Processing
    franka_xacro_filepath = os.path.join(pkg_config, 'urdf', 'bimanual_custom.urdf.xacro')
    robot_poses_file = os.path.join(pkg_config, 'config', 'robot_poses.yaml')
    with open(robot_poses_file, 'r') as f:
        robot_poses = yaml.safe_load(f)

    franka_controllers = os.path.join(pkg_config, 'config', 'basic_controllers_custom.yaml')

    robot_description_config = xacro.process_file(
        franka_xacro_filepath,
        mappings = {      
            'hand': load_gripper_str,
            'gazebo': 'false',
            'ros2_control': 'false',
            'use_fake_hardware': use_fake_hardware_str,
            'franka1_ip': right_ip_str,
            'franka2_ip': left_ip_str,
            'limit_override': 'true',
            'controller_path': franka_controllers,
            'franka1_x': str(robot_poses['franka1']['x']),
            'franka1_y': str(robot_poses['franka1']['y']),
            'franka1_z': str(robot_poses['franka1']['z']),
            'franka1_yaw': str(robot_poses['franka1']['yaw']),
            'franka2_x': str(robot_poses['franka2']['x']),
            'franka2_y': str(robot_poses['franka2']['y']),
            'franka2_z': str(robot_poses['franka2']['z']),
            'franka2_yaw': str(robot_poses['franka2']['yaw']),
            'table_size_x': str(robot_poses['table']['size_x']),
            'table_size_y': str(robot_poses['table']['size_y']),
            'table_size_z': str(robot_poses['table']['size_z']),
            'table_z': str(robot_poses['table']['z']),
            'camera_x': str(robot_poses['camera']['x']),
            'camera_y': str(robot_poses['camera']['y']),
            'camera_z': str(robot_poses['camera']['z']),
            'camera_roll': str(robot_poses['camera']['roll']),
            'camera_pitch': str(robot_poses['camera']['pitch']),
            'camera_yaw': str(robot_poses['camera']['yaw']),
        }
    ).toprettyxml('\t')
    robot_description = {'robot_description': robot_description_config}

    # 2. SRDF Processing
    xacro_file_srdf = os.path.join(pkg_config, 'srdf', 'bimanual_sim_srdf.xacro')
    robot_description_semantic_config = xacro.process_file(
        xacro_file_srdf,
        mappings={'name': 'idra_bimanual_setup', 'hand': load_gripper_str, 'gazebo': 'false'}
    ).toprettyxml('\t')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # 3. MoveIt Parameters
    kinematics_yaml = os.path.join(pkg_config, 'config', 'kinematics_bimanual.yaml')
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    joint_limits_yaml = os.path.join(pkg_config, 'config', 'joint_limits_bimanual.yaml')
    with open(joint_limits_yaml, 'r') as f:
        joint_limits_data = yaml.safe_load(f)
    
    # MoveIt 2 Humble expects joint limits under 'robot_description_planning'
    joint_limits_config = {'robot_description_planning': joint_limits_data}

    # 3. MoveIt Planning Pipeline (Forcing OMPL only)
    ompl_planning_yaml = os.path.join(pkg_config, 'config', 'ompl_planning_bimanual_override.yaml')
    with open(ompl_planning_yaml, 'r') as f:
        ompl_config_data = yaml.safe_load(f)
    
    # Add planner definitions
    ompl_config_data['planner_configs'] = {
        'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0},
        'RRTstarkConfigDefault': {'type': 'geometric::RRTstar', 'range': 0.0}
    }

    # Planning adapters for smoothing (Ruckig provides C2 continuity, preventing acceleration reflexes)
    adapters = [
        'default_planner_request_adapters/AddTimeOptimalParameterization',
        'default_planner_request_adapters/AddRuckigTrajectorySmoothing',
        'default_planner_request_adapters/ResolveConstraintFrames',
        'default_planner_request_adapters/FixWorkspaceBounds',
        'default_planner_request_adapters/FixStartStateBounds',
        'default_planner_request_adapters/FixStartStateCollision',
        'default_planner_request_adapters/FixStartStatePathConstraints',
    ]

    # Force OMPL as the ONLY and DEFAULT pipeline
    planning_pipeline_config = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            **ompl_config_data,
            'request_adapters': ' '.join(adapters),
        },
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': ' '.join(adapters),
            'start_state_max_bounds_error': 0.1,
            'default_acceleration_scaling_factor': 1.0,
            'default_velocity_scaling_factor': 1.0,
        },
        'moveit_manage_controllers': False,
    }

    # 4. Controllers (MoveIt side)
    moveit_controllers_yaml = os.path.join(pkg_config, 'config', 'moveit_controllers_bimanual.yaml')
    with open(moveit_controllers_yaml, 'r') as f:
        moveit_controllers_data = yaml.safe_load(f)
    
    moveit_controllers_config = {
        'moveit_simple_controller_manager': moveit_controllers_data,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 10.0,
        'trajectory_execution.allowed_goal_duration_margin': 5.0,
        'trajectory_execution.allowed_start_tolerance': 0.1,
        'default_velocity_scaling_factor': 1.0,
        'default_acceleration_scaling_factor': 1.0,
    }

    return [
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name='franka1_gripper',
            parameters=[{
                'robot_ip': right_ip_str,
                'limit_override': 'true',
                'joint_names': ['franka1_fr3_finger_joint1', 'franka1_fr3_finger_joint2']
            }],
        ),
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name='franka2_gripper',
            parameters=[{
                'robot_ip': left_ip_str,
                'joint_names': ['franka2_fr3_finger_joint1', 'franka2_fr3_finger_joint2']
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),        
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description, # Crucial: the node needs the URDF to know it's NOT fake
                franka_controllers,
                {'load_gripper': load_gripper_str}
            ],
            remappings=[
                ('~/robot_description', '/robot_description'),
                ('joint_states', '/joint_states')
            ],
            output='screen',
            on_exit=Shutdown(),
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                planning_pipeline_config,
                moveit_controllers_config,
                trajectory_execution,
            ],
        ),
        

        # RViz with MoveIt parameters
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', os.path.join(pkg_config, 'rviz', 'bimanual_moveit.rviz')],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                planning_pipeline_config,
            ],
        ),
    ]

def generate_launch_description():
    pkg_config = get_package_share_directory('franka_bimanual_config')

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    left_ip = LaunchConfiguration('left_ip', default='192.168.9.11')
    right_ip = LaunchConfiguration('right_ip', default='192.168.9.12')
    load_gripper = LaunchConfiguration('load_gripper', default='true')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')

    rviz_file = os.path.join(pkg_config, 'rviz', 'bimanual_moveit.rviz')

    # Spawner Function
    nodes_spawner = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[use_fake_hardware, load_gripper, left_ip, right_ip]
    )

    # Controller Spawners
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_franka1_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka1_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_franka2_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka2_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('left_ip', default_value='192.168.9.11'),
        DeclareLaunchArgument('right_ip', default_value='192.168.9.12'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        nodes_spawner,

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ["/joint_states", "/franka1_gripper/joint_states", "/franka2_gripper/joint_states"],
                'rate': 1000.0,
                'use_robot_description': False,
            }],
            output='screen',
        ),

        # Sequence controllers
        TimerAction(period=1.0, actions=[load_jsb]),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[
                    load_franka1_controller, 
                    load_franka2_controller
                ],
            )
        ),
    ])
