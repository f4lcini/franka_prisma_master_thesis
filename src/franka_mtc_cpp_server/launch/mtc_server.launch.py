import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    """Funzione di estrazione deterministica dei parametri YAML."""
    package_path = get_package_share_directory(package_name)
    try:
        with open(os.path.join(package_path, file_path), 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_env = get_package_share_directory('franka_manipulation_env')
    
    # 1. Compilazione deterministica URDF via Xacro
    xacro_file = os.path.join(pkg_env, 'urdf', 'system.urdf.xacro')
    robot_description = {"robot_description": ParameterValue(Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str)}

    # 2. Compilazione deterministica SRDF via Xacro (con i flag richiesti dal tuo ecosistema)
    srdf_file = os.path.join(pkg_env, 'srdf', 'fr3_sim_srdf.xacro')
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([FindExecutable(name="xacro"), " ", srdf_file, " name:=fr3 hand:=true"]), value_type=str)}

    # 3. Estrazione dei Parametri di Cinematica
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_base_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    planning_pipeline_parameters = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_base_yaml,
    }

    # 4. Istanziazione del Microservizio C++ MTC
    mtc_server_node = Node(
        package='franka_mtc_cpp_server',
        executable='mtc_pick_server_cpp',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipeline_parameters,
            {"use_sim_time": True}  # Sincronizzazione dell'orologio con Gazebo
        ],
    )

    return LaunchDescription([mtc_server_node])