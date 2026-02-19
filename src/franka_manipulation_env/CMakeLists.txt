cmake_minimum_required(VERSION 3.8)
project(franka_manipulation_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ... (altre dipendenze se presenti)

# --- QUESTA È LA PARTE CRITICA DA AGGIUNGERE/MODIFICARE ---
install(DIRECTORY
  launch
  models
  srdf
  config  # <--- Fondamentale per il .yaml dei controller
  urdf    # <--- Fondamentale per il system.urdf.xacro
  worlds  # (Se hai cartelle worlds)
  DESTINATION share/${PROJECT_NAME}
)


ament_package() # Deve sempre essere l'ultima riga
