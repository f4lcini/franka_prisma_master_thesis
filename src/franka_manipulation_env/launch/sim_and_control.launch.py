import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_manipulation_env')
    MY_ROBOT_POSE = "j1:=0.0 j2:=-0.785 j3:=0.0 j4:=-2.356 j5:=0.0 j6:=1.571 j7:=0.785"

    # 1. Elaborazione URDF
    xacro_file = os.path.join(pkg_env, 'urdf', 'system.urdf.xacro')
    robot_description_content = Command([FindExecutable(name="xacro"), " ", xacro_file, " ", MY_ROBOT_POSE])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 2. Robot State Publisher (TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # 3. Inclusione Simulatore Gazebo e Spawner Interni
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'sim_objects.launch.py')),
        launch_arguments={'initial_pose': MY_ROBOT_POSE}.items(),
    )

    camera_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_tf_bridge',
    arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'fr3_system/camera_link/camera']
    )

    return LaunchDescription([
        rsp_node,
        sim_launch,
        camera_tf_node,
    ])