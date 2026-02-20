from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Istanziazione del Client Python (Agnostico rispetto a URDF/SRDF)
    fr3_pnp_client_node = Node(
        package="fr3_application",
        executable="fr3_pnp_node",
        output="screen",
        parameters=[
            {"use_sim_time": True} # Direttiva tassativa per il sync temporale con Gazebo
        ],
    )

    return LaunchDescription([fr3_pnp_client_node])