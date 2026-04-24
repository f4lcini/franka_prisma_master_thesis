cmake_minimum_required(VERSION 3.8)
project(franka_bimanual_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/automate_scenarios/atom_bimanual_handshake.py
  scripts/automate_scenarios/atom_heterogeneous_stress.py
  scripts/automate_scenarios/atom_synch_home.py
  scripts/automate_scenarios/handover_optimizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
