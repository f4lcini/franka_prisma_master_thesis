import os
from xml.dom.minidom import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_manipulation_env')

    # 1. Inclusione Sottosistema Livello Basso (Gazebo + TF + Controllers)
    sim_and_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'sim_and_control.launch.py'))
    )

    # 2. Inclusione Sottosistema Livello Alto (MoveIt + RViz)
    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_env, 'launch', 'moveit_core.launch.py'))
    )

    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    #         '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
    #         '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
    #     ],
    #     output='screen'
    # )

    # 3. Orchestrazione Temporale
    # Avvia MoveIt e RViz 8.0 secondi DOPO l'avvio della simulazione, 
    # garantendo che l'Action Server /fr3_arm_controller/follow_joint_trajectory sia attivo.
    delayed_moveit_core = TimerAction(
        period=8.0,
        actions=[moveit_core_launch]
    )

    return LaunchDescription([
        sim_and_control_launch,
        delayed_moveit_core,
        #bridge
    ])