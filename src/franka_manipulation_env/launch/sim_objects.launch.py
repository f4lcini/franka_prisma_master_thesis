import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_franka_manipulation_env = get_package_share_directory('franka_manipulation_env')
    pkg_franka_desc = get_package_share_directory('franka_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. ARGOMENTO POSA (Input dallo script Master)
    pose_arg = DeclareLaunchArgument(
        'initial_pose',
        default_value='j1:=0.0 j2:=-0.785 j3:=0.0 j4:=-2.356 j5:=0.0 j6:=1.571 j7:=0.785',
        description='Argomenti Xacro per la posa iniziale'
    )

    # 2. CONFIGURAZIONE AMBIENTE
    resources = [os.path.dirname(pkg_franka_desc), ':', os.path.dirname(pkg_franka_manipulation_env)]
    env_vars = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resources),
        AppendEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resources),
        AppendEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib'),
        AppendEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib')
    ]

    # 3. URDF (Usa la posa passata come argomento)
    xacro_file = os.path.join(pkg_franka_manipulation_env, 'urdf', 'system.urdf.xacro')
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " ", LaunchConfiguration('initial_pose')]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 4. NODI
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fr3_system', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.0'],
        output='screen',
    )

    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'target_cube', '-file', os.path.join(pkg_franka_manipulation_env, 'models', 'cube', 'model.sdf'), '-x', '0.6', '-y', '0.0', '-z', '0.775'],
        output='screen',
    )
    
    # Controllers Spawner (Incluso il gripper)
    load_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    load_jtc = Node(package="controller_manager", executable="spawner", arguments=["fr3_arm_controller"], output="screen")
    load_grip = Node(package="controller_manager", executable="spawner", arguments=["franka_gripper"], output="screen")
    
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen')

    return LaunchDescription(
        [pose_arg] + env_vars + [
        rsp, gazebo, spawn_system, spawn_cube, bridge,
        TimerAction(period=5.0, actions=[load_jsb, load_jtc, load_grip])
    ])