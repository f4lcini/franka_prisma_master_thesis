# Bimanual Franka FR3 Framework: Exhaustive Technical Architecture (V17.0)

This document represents the definitive, exhaustive technical specification of the bimanual framework. It is intended to serve as the direct foundation for Chapter 4 (System Architecture) of the robotics engineering thesis, documenting every millimetric constant, logical branching, thread management strategy, hardware-software interaction, and the complete ROS 2 workspace topology. 

---

## 1. Perception System: High-Fidelity Spatial Intelligence

The perception pipeline is responsible for transforming raw multimodal sensor telemetry into a semantically-anchored 6DoF World-Frame representation.

### 1.1 Hardware and Driver Infrastructure
- **Sensor**: Intel RealSense D435i (Stereo Depth + RGB + IMU).
- **Communication Protocol**: ROS 2 Humble over **CycloneDDS**.
- **QoS (Quality of Service)**:
    - **Policy**: `SensorData` (Reliability: Best Effort, Durability: Volatile).
    - **Rationale**: Discards stale frames to ensure that the AI reasoning layer always operates on the most current world state (latency $< 33\text{ms}$).
- **Depth-RGB Registration**: Hardware-accelerated alignment (`align_depth:=true`). For any pixel $P(u, v)$ in the $640 \times 480$ RGB stream, the depth $Z$ is retrieved from the synchronized depth map at the identical $(u, v)$ coordinates, eliminating parallax error.

### 1.2 Object Detection & Semantic Filtering (YOLOv8)
- **Model Architecture**: YOLOv8n (Nano), optimized for real-time edge inference via PyTorch/TensorRT.
- **Confidence Threshold**: $0.65$ minimum confidence score to filter spurious bounding boxes.
- **HSV Heuristic Filter (Target Payload Isolation)**:
    - **Logic**: Used to distinguish the specific target payload (a red cube) from ambient red noise or clothing.
    - **Lower Filter Mask**: $H \in [0, 10], S \in [100, 255], V \in [100, 255]$.
    - **Upper Filter Mask**: $H \in [160, 180], S \in [100, 255], V \in [100, 255]$.
    - **Validation Threshold**: The system requires a $>15\%$ red-pixel density within the YOLO bounding box to validate and lock onto the "red_cube" class.

### 1.3 3D Spatial Grounding (Inverse Pinhole)
The conversion from image pixels $(u, v)$ to metric world coordinates $(X, Y, Z)$ follows the **Inverse Pinhole Camera Model**:
1.  **Metric Depth Acquisition**: $Z = d / 1000.0$ (sampled exactly at the bounding box centroid).
2.  **Robustness (Spiral Search)**: If $d=0$ (due to infrared reflection or occlusion), the algorithm performs a 20-pixel radius spiral search to find the nearest valid depth neighbor.
3.  **Deprojection Equations**:
    - $X = (u - c_x) \cdot Z / f_x$
    - $Y = (v - c_y) \cdot Z / f_y$
    - *Intrinsics*: Focal lengths ($f_x, f_y$) and optical centers ($c_x, c_y$) are retrieved dynamically from the `/camera/color/camera_info` topic to account for factory calibration.

### 1.4 Orientation Estimation (6DoF PCA)
Orientation is computed via **Principal Component Analysis** on the localized 3D point cloud $P$:
1.  **Mean Centering**: $P_{norm} = P - \text{mean}(P)$.
2.  **Covariance Matrix**: $C = \text{cov}(P_{norm}) \in \mathbb{R}^{3 \times 3}$.
3.  **Eigen-Decomposition**: Extracted via `numpy.linalg.eigh`. The primary eigenvector ($v_{major}$) defines the longitudinal axis of the object.
4.  **Quaternion Mapping**: The rotation matrix $R = [v_{major} | v_{minor} | v_{major} \times v_{minor}]$ is mathematically converted to a quaternion $(x, y, z, w)$ using the SciPy spatial rotation library.

### 1.5 Eye-to-Hand Extrinsic Calibration
The transformation from the Camera Optical Frame to the World Frame is statically defined as a baseline:
- **Translation**: $X=0.6, Y=-0.6, Z=1.3$ meters.
- **Rotation (Euler RPY)**: $Roll=0.0, Pitch=0.785, Yaw=1.57$ radians ($45^\circ$ downward tilt).

### 1.6 Camera Calibration & Extrinsic Refinement (AprilTag)
To overcome the physical inaccuracies of a purely static hardcoded transform, the framework incorporates an active calibration layer using fiducial markers.
- **Node Integration**: Uses the `apriltag_ros` package, dynamically injected into the perception pipeline via the `use_apriltag:=true` launch argument.
- **Fiducial Specifications**: 
    - **Family**: `Tag36h11` (high robustness against false positives).
    - **ID**: `0` (Reference Tag).
    - **Physical Size**: $0.074\text{m}$ ($7.4\text{cm}$ edge length).
- **Sub-Pixel Refinement**: The `apriltag_node` operates on the raw grayscale stream, utilizing corner refinement (`refine: true`) and decimation (`decimate: 1.0`) to extract the exact 6DoF pose of the marker relative to the optical center.
- **Dynamic TF Publishing**: When active (`publish_tfs: true`), the node continuously updates the `tf2` tree, establishing a dynamic mathematical link (`tag_0`) between the World Frame (where the physical tag is anchored on the table) and the Camera Link. This allows for real-time drift compensation and robust manipulation even if the camera tripod is accidentally bumped.

---

## 2. The Architectural Split: Python (Cognition) vs C++ (Control)

The framework strictly enforces a dichotomy between the cognitive orchestrator layer and the physical motion layer. This architectural decision guarantees system robustness and deterministic execution.

### 2.1 The Python Cognitive Layer ("The Brain")
Python was selected for the high-level orchestration (`franka_bimanual_orchestrator`) and Vision-Language Model integration (`franka_bimanual_skills` / `vlm_server_node.py`).
- **Motivation**: Python excels in asynchronous network operations (interacting with the Google Gemini REST APIs), complex string manipulation, and rapid dynamic parsing (using `Pydantic` for schema validation).
- **Behavior Tree Integration**: The `py_trees` library allows for highly flexible, stateful logic modeling without complex memory management.
- **The GIL Limitation**: Python's Global Interpreter Lock (GIL) prevents true multi-core parallel execution. Thus, Python acts purely as a "Dispatcher" (sending asynchronous Action goals), never blocking the thread with heavy calculations.

### 2.2 The C++ Motion Layer ("The Muscle")
C++ was chosen for the Motion Planning (`franka_bimanual_planner`) and Hardware Interface (`franka_ros2_multimanual`).
- **Motivation**: Heavy mathematical operations (Inverse Kinematics, RRT* trajectory generation, Polynomial interpolation) require maximum computational efficiency.
- **Deterministic Loop Requirements**: `libfranka` dictates a strict 1kHz ($1\text{ms}$) real-time control loop for torque generation. Python's Garbage Collector would introduce unpredictable latency spikes, resulting in hardware-level communication drops and "Communication Errors". C++ allows memory-safe, hard real-time execution.

---

## 3. Cognitive Layer: Neuro-Symbolic Reasoning

### 3.1 The LMM Brain: Google Gemini 2.5 Flash
- **Multimodal Integration**: Receives the operator's text prompt concatenated with the `cv_bridge` processed PIL Image.
- **Topology Awareness**: The System Prompt explicitly informs the model of the workspace layout.

### 3.2 Pydantic Data Contracts (The Safety Filter)
To prevent "AI Hallucinations", the LMM output is strictly parsed through a Pydantic schema:

```python
class RobotSkill(BaseModel):
    action: Literal["PICK", "PLACE", "WAIT", "MOVE_HOME", "FIND_OBJECT", "GIVE", "TAKE"]
    arm: Literal["left_arm", "right_arm"]
    target_name: Optional[str] = Field(None, description="Object label for YOLO")
    target_location: Optional[str] = Field(None, description="Workspace target: 'shared', 'box', 'ready'")
```
If validation fails, the system triggers an internal retry loop.

---

## 4. Behavior Tree Orchestrator: The Fundamental Core

The `franka_bimanual_orchestrator` package is the absolute brain and central nervous system of the entire architecture. Without it, the hardware and perception pipelines are disconnected entities. It bridges the neuro-symbolic reasoning of the VLM with the physical execution layers using an asynchronous Behavior Tree executing at a $1Hz$ tick rate.

### 4.1 Deep Dive: `main_engine.py` (The Heartbeat)
This script is the master conductor. It programmatically constructs the logic tree and manages the shared memory (Blackboard). 

**A. Root Guard & Global Memory Initialization**
The engine initializes a global `py_trees.blackboard.Client` and explicitly registers the global state keys: `mission_completed`, `vlm_plan`, `handover_ready`, and `handover_starting`. The absolute root of the tree is a `Selector` named `Root_Guard`, which immediately halts execution if `mission_completed == True`.

**B. The Planning Gate**
Execution flows into a `Sequence` that starts with the `Planning_Gate`. This node uses a `CheckBlackboardVariableValue` behavior to verify if `vlm_plan` is populated. If not, it triggers the `VlmActionClient`. Crucially, `main_engine.py` accepts a `--plan` terminal argument to bypass the Gemini API and inject a deterministic `simple_test_plan.json` directly into the Blackboard for controlled hardware testing.

**C. Parallel Lane Generation (The Core Masterpiece)**
To achieve "Parallelismo Spinto" (Extreme Parallelism), the engine uses `py_trees.composites.Parallel(policy=SuccessOnAll)`. It spawns two independent lanes via the `create_arm_lane(arm_name)` function. Each lane is an infinite `Repeat` loop containing a 4-step sequence:
1.  **Guard Condition**: `CheckBlackboardVariableValue` verifies that `arm_name_arm_plan != []`. If the queue is empty, this fails.
2.  **Iterator**: The `DynamicActionIterator` peeks at the top of the queue and writes the parameters to the Blackboard (e.g., `left_active_action = "PICK"`).
3.  **The Dispatcher**: A massive `Selector` node containing every available physical skill. Each skill is protected by a sub-Sequence. For example, the `Pick_left` sequence first checks if `left_active_action == "PICK"`. If true, it executes the `PickActionClient`. If false, the Selector tries the next skill.
4.  **The Popper**: If the Dispatcher returns `SUCCESS` (meaning the physical move succeeded), the `PlanPopper` finally removes the action from the Blackboard queue.

*Exit Strategy*: When an arm's plan is finally empty, the Guard Condition (Step 1) returns `FAILURE`. This cascades up to the `Repeat` loop, breaking it. A `FailureIsSuccess` decorator then catches this intended failure and returns a clean `SUCCESS` to the parent `Parallel` node.

### 4.2 Deep Dive: `behaviors/planner_utils.py` (The Instruction Decoder)
While `main_engine.py` sets the structure, `planner_utils.py` is the fundamental logic processor. It is responsible for parsing the VLM JSON and injecting data safely into the Behavior Tree loop. It completely isolates the two arms, preventing race conditions.

**A. The `PlanSplitter` (Isolation & Metadata Extraction)**
- *Mechanism*: This node operates immediately after the VLM phase. It reads the monolithic `vlm_plan`. First, it parses the `mission_metadata` to register the `mission_type` (e.g., "SIMPLE"). Then, it uses `copy.deepcopy()` to completely isolate the `left_arm_sequence` and `right_arm_sequence` into two independent arrays on the Blackboard. 
- *Importance*: Deep copying is vital; it guarantees that the parallel execution threads (lanes) do not encounter race conditions when reading from the same shared memory object.
- *Memory Tracing*:
    - *READS*: `vlm_plan`
    - *WRITES*: `left_arm_plan`, `right_arm_plan`, `mission_type`, `mission_metadata`.

**B. The `DynamicActionIterator` (The VLM-to-BT Translator)**
- *Mechanism*: Operating at the start of every BT loop tick, this node peeks at the 0-th index of the arm's plan. It acts as an instruction decoder, extracting JSON strings into dynamic BT Blackboard variables. It handles data edge cases: if Gemini returns a nested dictionary for the `arm` instead of a flat string, the Iterator safely extracts the value. It also injects safe defaults (e.g., `target_name="none"`, `grasp_type="top"`) to prevent Python `KeyErrors` from crashing the execution lane.
- *Memory Tracing*:
    - *READS*: `prefix_arm_plan` (e.g., `left_arm_plan`)
    - *WRITES*: `prefix_active_action`, `prefix_target_name`, `prefix_active_arm`, `prefix_grasp_type`, `prefix_target_location`.

**C. The `PlanPopper` (Implicit Fault Tolerance)**
- *Mechanism*: This node deletes the 0-th index of the arm's plan using `plan.pop(0)`. 
- *Importance*: Because of the structural arrangement in `main_engine.py`, the `PlanPopper` is *only* reached if the upstream Dispatcher (and thus the physical hardware action) returned a clean `py_trees.common.Status.SUCCESS`. If an action fails (e.g., IK timeout), the Sequence breaks before reaching the Popper. This creates implicit fault tolerance: on the next tick, the Iterator will simply re-attempt the exact same instruction without skipping steps.
- *Memory Tracing*:
    - *READS/WRITES*: Mutates `prefix_arm_plan`.

### 4.3 The Physical Clients & Blackboard Tracing
The rest of the `behaviors` directory houses the leaf nodes that translate BT statuses into ROS 2 Action Server calls.
- **`vlm_client.py`**:
    - *WRITES*: `vlm_plan` (Populates the global plan from Gemini).
- **`pick_client.py`**: 
    - *READS*: `prefix_target_name`, `prefix_active_arm`. It dynamically reads `prefix_target_pose` (which was generated by YOLO) unless the target is a hardcoded string like "shared" or "box".
    - *WRITES*: `handover_ready = False`. (If the arm picks from the "shared" zone, it explicitly resets the global handover flag).
    - *Execution*: Commands a rigorous `approach_distance` of exactly $0.15\text{m}$.
- **`place_client.py`**:
    - *READS*: `prefix_target_location`, `prefix_active_arm`.
    - *Execution*: Evaluates the target location and commands the arm to transition there, utilizing the safe $0.15\text{m}$ approach clearance before opening the gripper.
- **`object_localization_client.py`**:
    - *READS*: `prefix_target_name`.
    - *WRITES*: `prefix_target_pose`. (Triggers YOLO and writes the resulting 6DoF PoseStamped back to the blackboard so the `pick_client` can read it in the next tick).
- **`give_client.py` / `take_client.py`**: 
    - *READS*: `prefix_active_arm`.
    - *Execution*: The dual nodes for handing over objects. `give` commands the Donor to the spot; `take` commands the Recipient.
- **`sync_barrier_client.py`**: 
    - *READS*: None directly from Blackboard. It uses its initialized `role` ("donor" or "recipient") to call the appropriate `/donor_ready` or `/recipient_ready` ROS 2 Service on the `HandoverCoordinator`.
- **`wait_client.py`**: 
    - *READS*: `handover_ready` or `handover_starting` (depending on the logic). Pauses an arm's specific lane without blocking the parallel execution of the other arm.
- **`move_home_client.py`**: 
    - *READS*: `prefix_active_arm`, `prefix_target_pose_name`.
    - *Execution*: Commands the arm to a predefined named joint configuration (e.g., 'ready').

---

## 5. Bimanual Motion Planner (C++): The Execution Middleware

The `franka_bimanual_planner` package contains the absolute core of the physical execution layer: the `bimanual_planner_node.cpp`. This node is a custom C++ middleware specifically designed to circumvent the execution bottlenecks and thread-blocking limitations inherent to standard MoveIt 2 implementations.

### 5.1 Real-Time Multithreading Architecture
Standard MoveIt instances (`MoveGroupInterface`) are blocking; requesting a plan halts the execution thread. To achieve "Parallelismo Spinto", the C++ node implements an asynchronous, detached threading model:
1.  **Dual-Node Executor**: The executable actually instantiates two ROS 2 nodes (`bimanual_mg_interface` and `bimanual_planner`). Both are injected into a single `rclcpp::executors::MultiThreadedExecutor`, enabling concurrent execution of callbacks.
2.  **Thread Detachment**: The node serves a custom `/parallel_move` Action. When a goal is accepted, the `handle_accepted` callback immediately spawns a detached thread:
    ```cpp
    std::thread{ [this, goal_handle]() { this->execute(goal_handle); } }.detach();
    ```
    This instantly frees the ROS 2 executor to accept new goals, allowing the right and left arms to compute kinematics simultaneously.
3.  **Hardware-Level Mutexing**: Because `MoveGroupInterface` is not intrinsically thread-safe when accessing identical memory structures, the node utilizes two distinct `std::mutex` locks (`mutex_right_`, `mutex_left_`). The execution thread applies a `std::lock_guard` based on the requested arm. This mathematically guarantees that no two threads can attempt to command the *same* physical arm simultaneously, while completely allowing concurrent access to different arms.

### 5.2 Dynamic Planner Routing
The node acts as a bridge between the high-level Python commands and the low-level math. It evaluates the `goal->planner_id` string and hot-swaps the MoveIt planning pipeline on-the-fly:
- If `"ompl"` or `"RRTConnect"` is requested, it binds to the standard probabilistic solver, explicitly forcing the `RRTConnectkConfigDefault` planner for fast, randomized obstacle avoidance.
- If `"PTP"` (Point-To-Point) is requested, it swaps the pipeline to `pilz_industrial_motion_planner`, allowing for deterministic, industrial-grade linear path generation.

### 5.3 Direct JTC Bypass (The Latency Hack)
Standard MoveIt execution relies on a `trajectory_execution_manager` that publishes to a topic, waits for controllers, and monitors execution—introducing massive latency and potential synchronization drops.
- **The Solution**: The `bimanual_planner_node` bypasses the MoveIt execution layer entirely. It only uses MoveIt to *plan* the mathematical path (`move_group_ptr->plan(plan)`). 
- **Direct Injection**: It extracts the `joint_trajectory` from the resulting plan, constructs a native `control_msgs::action::FollowJointTrajectory` goal, and sends it *directly* to the physical `ros2_control` hardware action server (`/frankaX_arm_controller/follow_joint_trajectory`). This shaves off critical milliseconds of latency.

### 5.4 The 0.5s "Hold Phase" Discontinuity Fix
A critical algorithmic patch was introduced to prevent violent acceleration spikes (and subsequent Reflex Errors) triggered by the `libfranka` motion generator upon takeoff:
1.  **Timestamp Shifting**: The algorithm iterates through every `trajectory_msgs::msg::JointTrajectoryPoint` in the generated plan, adding exactly $+0.5$ seconds to their `time_from_start`.
2.  **Zero-Velocity Injection**: It inserts a new artificial point at the absolute beginning of the trajectory ($t=0.0$). This point contains the current hardware positions, but with `velocities` and `accelerations` strictly zeroed out.
3.  **Smooth Interpolation**: It zeros the velocities of the second point as well. This 500ms "stationary phase" acts as a physical buffer, granting the Franka Control Interface (FCI) time to initialize its internal interpolation engines before commanding physical torque.

### 5.5 Dynamic Midline Constraints (The Virtual Fence)
To mathematically guarantee that the arms do not collide near their structurally vulnerable bases, the node injects a `moveit_msgs::msg::PositionConstraint` prior to solving IK.
- **The Geometry**: It generates a `shape_msgs::msg::SolidPrimitive::BOX` of dimensions $2.2 \times 2.0 \times 2.2$ meters.
- **The Offsets**: 
  - If `arm == "right"`, the box is clamped at $X=1.25\text{m}$.
  - If `arm == "left"`, the box is clamped at $X=-0.05\text{m}$.
- **The Exception**: This creates an invisible, impassable mathematical wall running exactly down the centerline of the table. The only way to breach this wall is if the Behavior Tree passes the `goal->is_handover = true` flag, which triggers `clearPathConstraints()`, allowing the arms to merge in the shared spatial zone.

---

## 6. Low-Level Control & Hardware Interface

The `franka_ros2_multimanual` package provides direct 1kHz interaction with the FR3 manipulators.

### 6.1 Real-Time Threading Architecture
- **Control Loops**: Each Franka arm is assigned a dedicated `std::thread` executing the control callback at exactly $1000\text{Hz}$.
- **Kernel Requirement**: Requires `PREEMPT_RT` Linux kernel. Thread Priority is elevated to `98` to prevent OS preemption.
- **Service Segregation**: A separate `SingleThreadedExecutor` running in an isolated thread handles the `FrankaParamServiceServer`.

### 6.2 Millimetric Kinematic Constants
- **Joint Limits (1-7)**: Max Vel: $2.5\text{ rad/s}$, Max Accel: $5.0\text{ rad/s}^2$, Max Jerk: $1000.0\text{ rad/s}^3$.
- **Initial Hardware Impedance**: Joint $[3000, 3000, 3000, 2500, 2500, 2000, 2000]\text{ Nm/rad}$.
- **Active Compliance (ros2_control PD Gains)**:
    - Base Joints (1-4): $P=600, D=30$
    - Wrist Joints (5-6): $P=150, D=10$
    - End-Effector Joint (7): $P=50, D=5$

---

## 7. Asynchronous Handover Protocol

The framework manages "Loose Cooperation" via an event-driven sync barrier protocol.

### 7.1 Dual-Service Sync Barrier Protocol
The `HandoverCoordinator` node exposes two separate ROS 2 services (`/donor_ready` and `/recipient_ready`) to overcome single-service blocking limitations.
1.  **Thread Blocking**: The `HandoverCoordinator` relies on Python `threading.Event().wait()` primitives, configured with a default `timeout_sec` of $120.0$ seconds to ensure safety during complex kinematic calculations.
2.  **OS Level Suspension**: Calling `.wait()` suspends the thread at the OS kernel level, dropping CPU usage to near-zero while waiting for the partner arm.
3.  **Simultaneous Release**: Once both the Donor and Recipient register at their respective pre-handover poses, the Coordinator simultaneously releases both events.

---

## 8. Deployment & Configuration: The `franka_bimanual_bringup` Package

The `franka_bimanual_bringup` package acts as the centralized launch and configuration repository. It is the entry point for the entire architecture, orchestrating the startup of simulated environments, hardware drivers, and testing sequences without scattering configuration logic.

### 8.1 The Core Launcher: `executor.launch.py`
This is the fundamental bringup script that mounts the MoveIt environment and bridges the ROS 2 ecosystem. It is responsible for:
- **URDF & SRDF Processing**: Dynamically compiles the `bimanual_custom.urdf.xacro` and `bimanual_sim_srdf.xacro` files via the `Command` interface, injecting them into the parameter server for collision checking.
- **Kinematics overriding**: Loads the custom `kinematics_bimanual.yaml` to configure the precise KDL (Kinematics and Dynamics Library) solvers for both the left and right manipulators.
- **Planning Pipeline Configuration**: Merges the base OMPL parameters with the custom `ompl_planning_bimanual_override.yaml`, establishing the `RRTConnectkConfigDefault` as the primary obstacle-avoidance planner when Pilz is not requested.
- **Controller Management**: Mounts the `moveit_controllers_bimanual.yaml` to bind MoveIt's trajectory execution to the specific `franka1_arm_controller` and `franka2_arm_controller` instances.
- **Static TF Broadcasting**: Spawns `tf2_ros` `static_transform_publisher` nodes to establish the static baseline camera extrinsics (e.g., `world` $\rightarrow$ `camera/link` $\rightarrow$ `camera/link/rgb_camera`).

### 8.2 The Alternative Execution Launcher: `parallel_test_backends.launch.py`
This specialized launch script is designed to test an alternative hardware execution backend, bypassing the standard `bimanual_planner_node.cpp` (Joint Trajectory Control path) in favor of direct Cartesian control.
- **Planner Forcing**: Unlike the main executor, this script explicitly overrides the planning pipeline parameters to force `pilz_industrial_motion_planner` as the absolute default, utilizing the `pilz_industrial_motion_planner/CommandPlanner` plugin.
- **Cartesian Bridge Node Integration**: It replaces the standard C++ planner node with the `cartesian_bridge_node` (from the `franka_bimanual_skills` package). 
    - *Purpose*: The `cartesian_bridge_node` exposes the identical `/parallel_move` ROS 2 action interface to the Behavior Tree, but translates the MoveIt trajectories into low-level **Cartesian Impedance** commands instead of standard Joint Trajectories. This allows testing highly compliant manipulation strategies where the arm behaves like a virtual spring-mass-damper system, rather than a rigid position-controlled actuator.
- **Node Orchestration**: Alongside the bridge node, it spins up the standard `simple_moveit_server` (for Pick/Place skills) and the `sync_barrier_coordinator` (for Sync Barrier), creating a complete, isolated testing sandbox for compliant control.

### 8.3 Testing and Automation Scripts (`/scripts`)
The package houses isolated Python executables designed to independently validate specific layers of the architecture before full BT orchestration:
- **`automate_scenarios.py`**: The primary integration testing script. It runs automated, end-to-end task sequences to validate the complete pipeline from YOLO detection to physical handover.
- **`test_handover.py`**: An isolated script to rigorously stress-test the Loose Cooperation handover protocol, validating the `HandoverCoordinator` synchronization independently of the Gemini VLM.
- **`test_perception_logic.py`**: Validates the visual pipeline, testing the RGB-D alignment, HSV filtering, and PCA orientation estimation without moving the physical hardware.
- **`test_skills.py`**: Tests individual Action Servers (e.g., sending a pure `PICK` or `MOVE_HOME` command) to calibrate PD gains and check hardware compliance limits.
- **`validate_tamp.py`**: Tests the Task and Motion Planning (TAMP) layer, verifying that generated `TaskPlan` sequences result in mathematically solvable IK trajectories.

---

## 9. Configuration and Physics Sandbox: The `franka_bimanual_config` Package

While `franka_bimanual_bringup` acts as the execution trigger, the `franka_bimanual_config` package holds the foundational blueprint of the robotic system. It defines the physical constraints, the kinematic topologies, and the simulated environments that dictate *how* the robots exist and move within the workspace.

### 9.1 The Structural Blueprint (`/urdf`)
The Universal Robot Description Format (URDF) defines the exact spatial relationship and physical properties of every link, joint, and hardware interface. 
- **`bimanual_custom.urdf.xacro`**: The core structural macro governing the entire physical layout. 
    - **Anchoring**: It instantiates two distinct Franka FR3 manipulators (`franka1` and `franka2`) and mathematically anchors them to a common `world` link (via the table). 
    - **Exact Base Offsets**: The right arm (`franka1`) is anchored at `[X=0.4375, Y=0.0]` while the left arm (`franka2`) is anchored symmetrically at `[X=-0.4175, Y=0.0]`. 
    - **Environment Definition**: It defines the `shared_link` (a visual green sphere marking the conceptual center of the handover zone at `X=0.4, Z=0.05`) and the `tag_0_fixed` link, which represents the mathematical expectation of the physical AprilTag location on the table.
    - **Hardware Interface Instantiation**: Crucially, this file dictates the `ros2_control` backend. It defines a massive `<ros2_control>` block named `FrankaMultimanualHardwareInterface`. Depending on launch arguments, it conditionally loads either fake hardware plugins (`fake_components/GenericSystem`), the Ignition Gazebo physical simulator, or the custom `franka_mm_hardware_interface` for the real physical robots (targeting IP addresses `192.160.100.11` and `192.160.100.12`). Within this block, every single joint is assigned its required state/command interfaces (Position, Velocity, Effort), alongside custom GPIO arrays for low-level Cartesian Impedance overrides.

### 9.2 The Extrinsic Calibration Utility: `fix_camera.py`
Located in the root of the configuration package, this Python utility script solves a critical mathematical problem regarding the physical camera mounting.
- **Purpose**: It calculates the exact, static $4 \times 4$ transformation matrix between the physical table center and the camera's optical frame.
- **Mechanism**: The script ingests raw translational offsets (`t_opt_tag`) and Euler orientations (`r_opt_tag_rpy`) retrieved from a live `tf2_echo` command targeting the AprilTag. It combines this empirical data with the standard ROS camera optical rotation conventions (`-pi/2, 0, -pi/2`).
- **Matrix Inversion**: Using `numpy` and `scipy.spatial.transform.Rotation`, it computes the inverse spatial transform: `T_table_link = inv(T_link_opt * T_opt_tag)`.
- **Output generation**: The script outputs the finalized `X, Y, Z, Roll, Pitch, Yaw` values, formatted identically to YAML syntax, ready to be hardcoded directly into the system's `robot_poses.yaml` to ensure the perception pipeline has a physically accurate baseline before dynamic AprilTag tracking even begins.

### 9.3 The Semantic Blueprint (`/srdf`)
The Semantic Robot Description Format (SRDF) provides metadata for MoveIt, informing the planner about safe operational zones.
- **`bimanual_sim_srdf.xacro`**: Defines the "Planning Groups". 
    - **Groups**: `franka1_manipulator`, `franka2_manipulator`, and the combined `bimanual_manipulator`.
    - **Collision Matrix (ACM)**: This is vital for performance. The SRDF explicitly disables self-collision checking between structurally adjacent links (e.g., `link1` and `link2`) and statically disables collisions between the bases of the two robots, exponentially speeding up the OMPL collision-checking algorithms.

### 9.4 The Core Yaml Constraints (`/config`)
These parameter files dictate the strict limits enforced during both trajectory planning and hardware execution:
- **`joint_limits_bimanual.yaml`**: Imposes hard limits on positions (min/max radians), max velocities ($2.5\text{ rad/s}$), and max accelerations ($5.0\text{ rad/s}^2$) specifically tuned for the dual-arm setup to prevent dangerous torque overloads.
- **`kinematics_bimanual.yaml`**: Configures the Inverse Kinematics solver. It assigns `kdl_kinematics_plugin/KDLKinematicsPlugin` to each arm, dictating the search resolution and maximum IK solver timeouts.
- **`ompl_planning_bimanual_override.yaml`**: Finetunes the Open Motion Planning Library parameters, enforcing `RRTConnect` and optimizing path smoothing constraints.
- **`apriltag.yaml`**: Holds the configurations for the `apriltag_ros` node, defining the `36h11` tag family and the $7.4\text{cm}$ physical size for active calibration.

### 9.5 The Simulation Sandbox (`/worlds` & `/models`)
To safely validate neuro-symbolic reasoning without risking physical hardware, the package contains the simulated counterparts:
- Defines the Gazebo/Ignition XML world geometries, including the central handover table, the target payload (red cube), and the destination container, ensuring the simulated optical sensors perceive an environment structurally identical to the real-world laboratory.

---

## 10. ROS 2 System Infrastructure & Topology

The bimanual framework is strictly modularized into 6 custom ROS 2 packages.

### 10.1 Workspace Hierarchy
- `franka_bimanual_orchestrator/`: Frontend (BT and VLM integration).
- `franka_bimanual_skills/`: Backend (Action Servers and Python APIs).
- `franka_bimanual_planner/`: C++ Middleware (MoveIt/Pilz execution).
- `franka_ros2_multimanual/`: Low-level Hardware Interface (libfranka wrappers).
- `franka_custom_interfaces/`: Global communication definitions.
- `franka_bimanual_bringup/`: Centralized launch and script execution repository (Section 8).
- `franka_bimanual_config/`: Physical blueprints and kinematic constraints (Section 9).

### 10.2 Communication Topology (Contracts)
Data flows through the system strictly via custom ROS 2 Actions and Services, defined in `franka_custom_interfaces`:

- **`VlmQuery.action`**: Goal: `string task_description`. Result: JSON Plan.
- **`ParallelMove.action`**: Goal: `arm`, `target_pose`, `is_handover`. Evaluated by the C++ Multithreaded planner.
- **`HandoverReady.srv`**: Called by both arms independently to coordinate the physical sync barrier phase without deadlocking the ROS 2 executor.
- **Blackboard**: Acts as the shared memory for the entire `py_trees` execution tree.

---

*Document Version 17.0 - Authored by Falco Robotics.*
