from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock. Set true for Gazebo, false for real hardware.'
    )

    # Instantiation of the Python Client (Agnostic to URDF/SRDF)
    fr3_pnp_client_node = Node(
        package="fr3_application",
        executable="fr3_pnp_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([declare_use_sim_time, fr3_pnp_client_node])