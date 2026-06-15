# Franka Prisma Master Thesis

> ⚠️ **Notice / Avviso**
> **This project is currently under active development as part of a Master's Thesis work.**
> *Questo progetto è un lavoro di tesi magistrale ed è attualmente in via di sviluppo.*

## Overview

This repository contains the ROS 2 workspace (`mm_ws`) developed for my Master's Thesis. The project involves the control and orchestration of a bimanual Franka Research 3 (FR3) setup, incorporating advanced perception pipelines (YOLO), Vision-Language Models (VLM) for reasoning, and Behavior Trees for executing complex manipulation tasks.

## Workspace Packages Overview

The `src/` directory contains various ROS 2 packages, each dedicated to a specific aspect of the framework:

- **`franka_bimanual_bringup`**: Provides centralized launch files, system bringup scripts, and automated scenarios to start both hardware and software backends.
- **`franka_bimanual_config`**: Overrides all necessary YAML files and the real dual-arm setup (e.g., referring to `src/franka_bimanual_config/config/robot_poses.yaml`). It contains hardware interface settings, MoveIt configurations, controller parameters, and AprilTag setups for the bimanual environment.
- **`franka_bimanual_orchestrator`**: The core execution engine utilizing Behavior Trees to coordinate perception, reasoning, and manipulation skills based on Natural Language or predefined JSON plans.
- **`franka_bimanual_planner`**: Contains the `bimanual_planner_node`, a custom action server (`parallel_move`) that orchestrates dual-arm motion. It interfaces with MoveIt 2 for planning (supporting OMPL and Pilz) but executes paths directly via `FollowJointTrajectory` controllers to bypass MoveIt execution bottlenecks. It also implements a custom "hold phase" trajectory adjustment to prevent `libfranka` acceleration discontinuities on startup.
- **`franka_bimanual_skills`**: Implements atomic robotic skills through ROS 2 actions and services, including perception (YOLO integration) and reasoning (VLM integration).
- **`franka_custom_interfaces`**: Defines custom messages (`msg`), services (`srv`), and actions (`action`) used for communication across the framework.
- **`franka_description`**: Contains the URDF, Xacro, and 3D mesh files that represent the physical layout and kinematics of the robots.
- **`franka_manipulation_env`**: Defines the surrounding workspace, laboratory setup, and static collision objects via URDF configurations(USED FOR GAZEBO SIMULATION ONLY!)
- **`franka_ros2` & `franka_ros2_multimanual`**: Core drivers and hardware interfaces for communicating directly with the real Franka controllers in concurrent bimanual setups.


## 💻 Execution Guide: Simulation Pipeline (Gazebo)

> ⚠️ **IMPORTANT**: To launch the simulation using the bimanual setup in Gazebo, you **MUST** switch to the `Parallel_Coordinated` branch!

To run the framework in simulation, use the following commands across different terminals:

### Terminal 1: Simulation (Gazebo + MoveGroup + RViz)
*Note: `demo_moveit_bimanual` is a wrapper that launches Gazebo AND MoveGroup. Wait ~12 seconds after launch, as MoveGroup starts with a delay to give Gazebo time to load.*
```bash
ros2 launch franka_bimanual_config demo_moveit_bimanual.launch.py use_gazebo:=true
```

### Terminal 2: Python MoveIt Server & Skills (The Muscles)
*Launches MoveIt server, Cartesian Bridge, and Sync Barrier Coordinator.*
```bash
ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py 
```

---

## 🚀 Execution Guide: Hardware Pipeline (Modular Bimanual)

To run the framework on the physical hardware, you need to launch several independent modules across different terminals. This modular approach ensures clean logging and isolated debugging.

### Terminal 1: Hardware Bringup
Starts the physical controllers and MoveIt for the dual arm setup.
```bash
cd /mm_ws && source install/setup.bash
ros2 launch franka_bimanual_config lab_bringup_final.launch.py left_ip:=192.168.9.11 right_ip:=192.168.9.12
```

### Terminal 2: Python MoveIt Server & Skills
Launches the MoveIt server, Cartesian Bridge, Sync Barrier Coordinator, and RealSense camera.
```bash
# Start MoveIt Servers and Sync Barrier
ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py

# Start RealSense Camera
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
```

### Terminal 3: YOLO Perception (The Eye)
```bash
cd /mm_ws && source install/setup.bash
ros2 launch franka_bimanual_skills perception.launch.py use_hardware:=true
```

### Terminal 4: VLM Reasoning (The Brain)
```bash
cd /mm_ws && source install/setup.bash
ros2 launch franka_bimanual_skills reasoning.launch.py use_hardware:=true
```

### Terminal 5: Behavior Tree Visualizer
For live viewing of the Behavior Tree execution.
```bash
QTWEBENGINE_DISABLE_SANDBOX=1 /opt/ros/humble/bin/py-trees-tree-viewer
```

### Terminal 6: Natural Language Command
Used to send tasks to the Orchestrator.
```bash
cd /mm_ws && source install/setup.bash
ros2 run franka_bimanual_orchestrator main_engine "Place the green ball in the box"
```

---

## 📷 Camera & AprilTag Calibration

Use the following commands to initialize and verify the vision setup:

```bash
# Fix Camera Extrinsics
# The `fix_camera.py` script calculates the camera pose relative to the table using a Camera-to-Tag transformation matrix. 
# It then automatically updates the `robot_poses.yaml` file located in `franka_bimanual_config` with the calibrated camera coordinates (x, y, z, roll, pitch, yaw).
python3 src/franka_bimanual_config/fix_camera.py

# Start AprilTag Detection
ros2 run apriltag_ros apriltag_node --ros-args --params-file src/franka_bimanual_config/config/apriltag.yaml -r image_rect:=/camera/camera/color/image_raw -r camera_info:=/camera/camera/color/camera_info

# Verify Tag Detections & TF (Run in separate terminals as needed)
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /detections
ros2 run tf2_ros tf2_echo camera_link tag_0
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag_0  # Currently using this frame

# Optional Action verification
ros2 action send_goal /franka2_gripper/homing franka_msgs/action/Homing "{}"
```

> **Note**: To ensure you have the required ROS 2 dependencies for AprilTags, run:
> ```bash
> sudo apt update
> sudo apt install ros-humble-apriltag-ros
> ```

To move the robots to their home position synchronously (ensure `ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py` is running first):
```bash
python3 src/franka_bimanual_bringup/scripts/automate_scenarios/atom_synch_home.py
```

*(Permissions fix if needed: `sudo chown -R $USER:$USER /home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis`)*

---

## 🧪 Thesis Experiments Prompts

You can launch automated experiments either by sending a Natural Language prompt or by providing a pre-compiled JSON plan. 
*Note: You can also use the `scan table` service in static experiments via a JSON file.*

### EXP1: Sort Items
**Prompt:**
```bash
ros2 run franka_bimanual_orchestrator main_engine "Sort all the items inside the designated boxes"
```
**JSON Plan:**
```bash
ros2 run franka_bimanual_orchestrator main_engine --plan src/franka_bimanual_bringup/scripts/automate_scenarios/json_plans/EXP1_sort_items.json
```

### EXP2: Transfer Object
**Prompt:**
```bash
ros2 run franka_bimanual_orchestrator main_engine "Transfer the object on the table to the box"
```
**JSON Plan:**
```bash
ros2 run franka_bimanual_orchestrator main_engine --plan src/franka_bimanual_bringup/scripts/automate_scenarios/json_plans/EXP2_transfer_object.json
```

### EXP3: Clear Workspace
**Prompt:**
```bash
ros2 run franka_bimanual_orchestrator main_engine "Clear the entire workspace"
```
**JSON Plan:**
```bash
ros2 run franka_bimanual_orchestrator main_engine --plan src/franka_bimanual_bringup/scripts/automate_scenarios/json_plans/EXP3_clear_workspace.json
```
