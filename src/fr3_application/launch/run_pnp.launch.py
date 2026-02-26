from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Instantiation of the Python Client (Agnostic to URDF/SRDF)
    fr3_pnp_client_node = Node(
        package="fr3_application",
        executable="fr3_pnp_node",
        output="screen",
        parameters=[
            {"use_sim_time": True} # Strict directive for time sync with Gazebo
        ],
    )

    return LaunchDescription([fr3_pnp_client_node])