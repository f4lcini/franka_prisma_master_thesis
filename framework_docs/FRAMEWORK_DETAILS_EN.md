# Bimanual Franka FR3 Framework: Technical Overview

This document provides a comprehensive technical breakdown of the bimanual framework architecture, detailing the logic flow from high-level reasoning to low-level physical execution.

---

## 1. Perception System: Spatial Intelligence
The perception pipeline converts raw sensor telemetry into actionable 3D spatial information.

### 1.1 Hardware and Drivers
- **Sensor**: Intel RealSense D435 depth camera.
- **Middleware**: `realsense2_camera` ROS 2 driver.
- **QoS Requirements**: The driver publishes color and depth streams using the **Best Effort** Reliability Policy. Subscribers must match this policy to avoid dropped frames.

### 1.2 Object Detection (YOLOv8)
The system utilizes the **YOLOv8n (Nano)** model for real-time inference on the RGB stream (640x480).
- **Inference**: Orchestrated via an Action Server (`/detect_object`).
- **Processing**: The model extracts bounding boxes, class labels (80 COCO classes), and confidence scores.
- **Selection**: The detection with the highest confidence matching the requested semantic label is selected for further processing.

### 1.3 3D Localization (Pinhole Deprojection)
To map 2D pixel coordinates $(u, v)$ to 3D metric space $(X, Y, Z)$, the system employs the **Inverse Pinhole Camera Model**:
1. **Depth Acquisition**: The depth value $d$ (in mm) is sampled at the bounding box center $(u, v)$ from the aligned depth image.
2. **Coordinate Transform**:
   $$Z = d / 1000.0$$
   $$X = (u - c_x) \cdot Z / f_x$$
   $$Y = (v - c_y) \cdot Z / f_y$$
   *Where $f_x, f_y$ are focal lengths and $c_x, c_y$ are principal point coordinates from the `CameraInfo` intrinsics matrix.*

### 1.4 Orientation Estimation (PCA)
The system estimates the object's 3D orientation to ensure precise grasping:
1. **Local Point Cloud**: A subset of depth pixels within the bounding box is deprojected into a local 3D point cloud.
2. **Covariance Analysis**: The 3x3 covariance matrix of the centered point cloud is computed.
3. **Eigen-Decomposition**: Using `np.linalg.eigh`, the system extracts eigenvalues and eigenvectors.
4. **Principal Axis**: The 1st eigenvector represents the principal axis of the object.
5. **Quaternion Extraction**: A rotation matrix is constructed from the eigenvectors and converted into a $[x, y, z, w]$ quaternion using **SciPy's `Rotation`** class.

---

## 2. Reasoning Layer: Cognitive Decision Making
The reasoning pipeline acts as the "Action Dispatcher," interpreting natural language and scene context through a Vision-Language Model (VLM).

### 2.1 Engine: Gemini 2.5 Flash
The framework leverages **Google Gemini 2.5 Flash** for its multimodal capabilities and low latency.
- **Prompt Engineering**: The **System Prompt** embeds hard-coded kinematic constraints.
- **Chain-of-Thought (CoT)**: The model is forced to output its internal logic in a `reasoning` field before finalizing categorical decisions, significantly increasing accuracy.

### 2.2 Kinematic Constraint Injection
Gemini is instructed to obey physical workspace rules to prevent singularities and collisions:
- **Midline Division**: Objects with $X < 0$ (left side) are assigned to the Left arm; $X > 0$ to the Right arm.
- **Singularity Avoidance**: Reaching across the workspace midline is prohibited.
- **Handover Estimation**: For bimanual transfers, Gemini estimates a safe $Z$ rendezvous height centered between the arms.

### 2.3 The Role of Pydantic in Structured Output
The framework enforces a rigorous boundary between the "fuzzy" reasoning of the LLM and the "rigid" logic of ROS 2 using **Pydantic**.
- **Schema Enforcement**: The `TaskPlan` and its constituent skills (e.g., `PickSkill`, `PlaceSkill`) are defined as Pydantic `BaseModel` classes. 
- **Type Safety**: By using `Literal` and `Union` types for action names and arm selections, the system guarantees that the VLM cannot hallucinate non-existent operations.
- **Validation**: Every response from Gemini is validated at the source. If the VLM generates a malformed plan, it is caught as a Python exception before it can reach the planners, preventing potential physical collisions.

---

## 3. Behavior Tree Orchestration
The high-level logic and mission flow are managed by a **Behavior Tree (BT)** using `py_trees`.

### 3.1 Architecture Overview
The core of the tree is a `Parallel` node with a `SuccessOnAll` policy. This allows the orchestrator to maintain two active execution "threads" (lanes) simultaneously, dedicated to the Left and Right Franka arms.

### 3.2 Non-Blocking Behaviors
Each arm skill (e.g., `PickActionClient`) is implemented using asynchronous ROS 2 Action Clients (`send_goal_async`). When a skill is triggered, it returns `RUNNING` to the BT, allowing the tree to continue ticking and processing the other arm's state in the same cycle.

### 3.3 Dynamic Dispatching
The tree utilizes a `DynamicActionIterator` to "peek" into the arm-specific plan queues stored on the blackboard. This enables the system to reconfigure parameters (target_pose, grasp_offsets) on-the-fly without rebuilding the tree structure.

---

## 4. Control Architecture: MoveGroup & Custom Hardware Interface
The framework overcomes the limitations of standard drivers through a custom architecture that integrates `ros2_control` and `libfranka`.

### 4.1 Pilz Industrial Motion Planner: Determinism and Safety
The system utilizes the **Pilz Industrial Motion Planner** to generate trajectories with industrial-grade characteristics:
- **Mathematical Formulation**: It uses **5th-degree (quintic) polynomials** to ensure **C2 continuity** (continuous position, velocity, and acceleration). This prevents jerk spikes and protects the FR3 motors.
- **PTP (Point-To-Point) Trajectories**: Synchronized velocity profiles across joints for fast spatial transitions between distant waypoints.
- **LIN (Linear) Trajectories**: Linear interpolation in task space (TCP), essential for object approach and handover rendezvous phases.

**Applied Kinematic Constraints:**
- **Joint Velocity**: Max 2.62 rad/s (joints 1-4), 5.26 rad/s (joints 5-7).
- **Joint Acceleration**: Max 5.0 rad/s² (joints 1-4), 10.0 rad/s² (wrists 5-7).
- **Cartesian Velocity (LIN)**: Max 1.0 m/s linear, 1.57 rad/s rotational.

### 4.2 Franka Multi-Manual (MM) Hardware Interface
Hardware integration is managed by the custom `franka_mm_hardware_interface` package, which extends `libfranka` capabilities:

1.  **FrankaRobotWrapper**: Each arm is managed by a dedicated C++ wrapper that encapsulates the `franka::Robot` instance and manages a **dedicated control thread** at 1kHz.
2.  **Mode Switching**: The framework implements an advanced `ModeSwitchPlan`. This allows for deterministic transitions between different controllers (e.g., from **Joint Position** to **Cartesian Impedance**) during mission phases.
3.  **Dynamic Parameter Server**: Via `FrankaParamServiceServer`, it is possible to update collision thresholds and load parameters (`setLoad`) on-the-fly without restarting the driver, allowing the VLM to optimize dynamics based on the grasped object.

### 4.3 State Hub: Custom Readers
To optimize bimanual telemetry, two specialized controllers are used:
- **`PositionReader`**: Aggregates joint positions from both robots for unified monitoring.
- **`PoseReader`**: Monitors Cartesian TCP poses in real-time for rendezvous synchronization.

---

## 5. Advanced Concurrency & Synchronization
Achieving robust bimanual operation in a Python environment requires solving three critical technical challenges.

### 5.1 Bypassing the Python GIL
Despite the Python **Global Interpreter Lock (GIL)**, the framework achieves true physical concurrency through **I/O-Bound Context Switching**:
- When a Python thread waits for a response from a ROS 2 Action Server or a TF transform, it **releases the GIL**.
- This allows the actual "heavy lifting" (IK, Path Planning) to be executed by **MoveIt 2 and `libfranka` backends**, which are multi-threaded C++ processes running concurrently on separate CPU cores.

### 5.2 ROS 2 Multi-Threading Strategy
- **MultiThreadedExecutor**: The engine uses a custom executor with a high thread count (e.g., 12 threads) to process Action feedback and perception data in parallel.
- **Reentrant Callback Groups**: All custom nodes utilize `ReentrantCallbackGroup`. This ensures that a long-running action (like a move) does not block the processing of other important telemetry or status callbacks.

### 5.3 Shared Memory & Thread Safety
- **Blackboard Locking**: Shared mission data is managed by `py_trees`' internal **Recursive Read-Write Lock (RWMutex)**, preventing race conditions between the two arm lanes.
- **Namespaced Access**: Parameters are partitioned into `left_` and `right_` namespaces, minimizing lock contention.

### 5.3.3 Deterministic Synchronization: The Rendezvous Handshake
Direct coordination between arms (e.g., for mid-air handovers) is achieved via **Thread-Safe Selection Primitives**:
- **Threading Events**: The framework utilizes `threading.Event()` objects (e.g., `_donor_ready`, `_recipient_ready`). These are primitive synchronization objects that allow one thread to signal another. 
- **Non-Polling Wait**: Unlike "busy-wait" loops, the `.wait()` method blocks the thread in a low-power state and is woken up by the OS kernel as soon as the other thread calls `.set()`. 
- **Design Note on Wrench Feedback**: While the framework architecture is designed to support closed-loop **Wrench Feedback** ($O\_F\_ext\_hat\_K$ force spike detection) for reflex-based release, the current implementation utilizes deterministic logic-gate synchronization to ensure safety in common industrial handover scenarios.

---

## 6. The Behavior Tree Lifecycle: A Granular Walkthrough
The execution follows a deterministic three-phase lifecycle.

### 6.1 Phase 1: Cognitive Planning
1. **Inference**: The `VlmActionClient` sends scene metadata to Gemini 2.5 Flash.
2. **Validation**: The Pydantic-validated plan is projected into `blackboard.vlm_plan`.
3. **Partitioning**: `PlanSplitter` performs a spatial partition, populating the arm-specific queues (`left_arm_plan` / `right_arm_plan`).

### 6.2 Phase 2: Parallel Synchronized Execution
The `Parallel` node bifurcation drives the arm lanes through a **Tick-Tock Cycle**:
1. **Parameter Injection**: `DynamicActionIterator` updates namespaced keys on the blackboard.
2. **Dispatching**: The `Dispatcher` selector triggers the corresponding Action Client.
3. **Asynchronous Monitoring**: The tree monitors ROS 2 feedback strings and completion statuses non-blockingly.

### 6.3 Phase 3: Queue Maintenance & Fault Recovery
1. **Popping**: `PlanPopper` removes the action from the queue only after a `SUCCESS` status.
2. **Fail-Stop**: If one arm encounter an IK failure or timeout, the specific lane halts. The other arm, if in a synchronized state, will **timeout** its wait events, preventing mid-air collisions.

---

## 7. Development Delta: Workspace Contributions
The framework represents a significant engineering effort built upon the Franka Research 3 foundation:
- **Custom Development**: >70% of the workspace code represents original development, including the perception-to-reasoning bridge, the bimanual BT orchestrator, and the custom rendezvous protocol.
- **Package Architecture**: Orchestrates 6 custom ROS 2 packages (`franka_bimanual_skills`, `franka_bimanual_orchestrator`, `franka_bimanual_bringup`, `franka_custom_interfaces`, `franka_bimanual_config`, `franka_action_server`) into a cohesive robotic system.

---

## 8. Analysis: Assumptions and Architectural Gaps

A rigorous technical framework must be evaluated within the context of its design constraints and potential vulnerabilities for academic and industrial transparency.

### 8.1 Design Assumptions
The framework is built upon specific constraints and environmental simplifications necessary to isolate and validate the high-level orchestration architecture.

- **Top-Down Grasping Constraint**: The system assumes objects are manipulable via a top-down grasp along the Z-axis. This assumption prioritizes grasp stability and collision avoidance with the tabletop over the full 6DoF kinematics provided by the PCA pipeline.
- **2D Centroid Representativeness**: It is assumed that the geometric center of the YOLOv8 bounding box corresponds to the physical solid surface of the object. This allows for single-pixel depth sampling ($d$) via the Inverse Pinhole model, bypassing computationally expensive instance segmentation.
- **Static Spatial Partitioning (Midline Division)**: The workspace is assumed to be divisible into two independent hemispheres. This prevents arm-to-arm base collisions by design, offloading the burden from the motion planner regarding complex bimanual crossovers.
- **Ideal Kinematics**: The extrinsic transformation between the Intel RealSense D435 and the robot base frames is assumed to be static, rigid, and known with millimeter precision.
- **Logic-Gate Synchronization**: In the absence of live wrench feedback integration, the system assumes that temporal coordination via `threading.Event()` and LIN Cartesian moves is sufficient for safe object transfer, provided there is no mechanical slippage.

### 8.2 Architectural Gaps & Vulnerabilities
The following technical limitations represent areas for future refinement and critical academic discussion.

- **Topological Sensitivity of 3D Deprojection**: Sampling depth at the bounding box center fails for concave or annulus-shaped objects (e.g., a tape roll or a mug seen from above). In such cases, the center pixel represents the table depth, leading to vertical collisions or "air-grasps."
- **Open-Loop Handover Risks**: The current handover protocol relies on logical state-machine transitions rather than closed-loop force feedback. If a recipient arm fails a grasp physically but its Action Server returns SUCCESS, the donor will release the object, resulting in a dropped item.
- **Lack of Active Recovery**: The "Fail-Stop" policy ensures physical safety but limited autonomy. The Behavior Tree currently lacks "Retry" or "Re-plan" behaviors to handle unreachable IK poses or dynamic obstacles autonomously.
- **Cloud Dependency**: The VLM acts as the primary cognitive engine. Network latency, API quota limits, or server downtime for Gemini 2.5 Flash can paralyze the mission flow, making the framework unsuitable for isolated (air-gapped) industrial environments in its current state.

---

## 11. Workspace Refactoring: Naming Strategy
To eliminate technical debt and clarify component roles, the following package renames have been implemented in the bimanual framework:

| Previous Name | New Refactored Name | Functional Role |
| :--- | :--- | :--- |
| `franka_bimanual_bringup` | `franka_bimanual_bringup` | Entry point for mission-level testing and handover validation scripts. |
| `franka_bimanual_skills` | `franka_bimanual_skills` | Backend hosting the ROS 2 Skill Servers, repertoire, and handover coordinator. |
| `franka_bimanual_orchestrator` | `franka_bimanual_orchestrator` | Frontend housing the Behavior Tree and Gemini VLM logic. |


---

## 12. Behavior Tree: Frontend vs Backend Architecture
The system enforces a clean separation of concerns between mission orchestration and physical execution.

### 12.1 The Frontend (The "Brain")
Located in `franka_bimanual_orchestrator`, the frontend manages the cognitive flow.
- **Mission Execution**: Uses `py_trees` to maintain a 10Hz tick rate.
- **VLM Interface**: Invokes the `vlm_server_node` to convert high-level instructions into categorized `TaskPlan` models.
- **ROS 2 Interface (Action Clients)**: Sends goals for `PickObject`, `PlaceObject`, `MoveHome`, `GiveObject`, and `TakeObject`.
- **ROS 2 Interface (Service Clients)**: Interrogates `HandoverReady` for synchronization and `GeminiPlan` for reasoning.
- **Blackboard**: Serves as the shared memory for synchronized bimanual variables.

### 12.2 The Backend (The "Engine Room")
Located in `franka_bimanual_skills`, the backend handles the "heavy lifting" of robotics.
- **Action Servers**: Implements the `Pick`, `Place`, `Handover`, and `Home` servers in Python. These report feedback and status.
- **Skill Repertoire**: A modular library of motion primitives (e.g., `execute_pick`, `execute_take`) that utilize MoveIt.
- **Handover Coordinator**: Manages the `threading.Event` handshake and rendezvous logic.
- **Motion Planning**: Integrates MoveIt 2 and the **Pilz Industrial Motion Planner**.
- **Real-Time Bridge**: Communicates with the `franka_bimanual_driver` (C++) to send trajectory setpoints at 1kHz.
- **Transform Listeners**: Continuously polls `tf2` to maintain the relationship between camera frames, robot bases, and detected objects.

---

## 13. Deep-Dive Design Strengths

### 13.1 Concurrency and the GIL
While the orchestrator is Python-based, the framework achieves true bimanual concurrency. By offloading Inverse Kinematics (IK) and path planning to C++ backends (MoveIt/libfranka), the Python threads release the Global Interpreter Lock (GIL) during I/O-bound operations, allowing both arms to plan and execute paths simultaneously on separate CPU cores.

### 13.2 Industrial Safety via Quintic Polynomials
The use of the Pilz planner ensures that all trajectories follow **5th-degree polynomials**. This guarantees **C2 continuity**, meaning that velocity and acceleration are always smooth and continuous. This prevents high-frequency vibrations in the robot structure and ensures that torque limits are never instantaneously exceeded during high-speed bimanual coordination.

---

---

## 9. Future Work: Elevating the State-of-the-Art

To evolve the framework toward industrial-grade autonomy and high-fidelity physics, four strategic technical paths are proposed.

### 9.1 Dynamic Kinematic Load Balancing (Yoshikawa Index)
The current `PlanSplitter` utilizes a binary spatial split. A superior approach involves dynamic optimization:
- **Manipulability Measure**: Calculating the **Yoshikawa Index** ($\sqrt{det(J \cdot J^T)}$) for both manipulators in real-time.
- **Singularity Avoidance**: The Task Orchestrator will assign objects to the arm that can perform the pick/place with the highest kinematic transparency, automatically handling "edge-case" objects near the workspace midline.

### 9.2 Closed-Loop Semantic Recovery
To move beyond the "Fail-Stop" policy, the framework will implement autonomous cognitive recovery:
- **Visual Re-Interrogation**: Upon a BT `FAILURE` (e.g., dropped object), the system triggers a new VLM inference cycle.
- **Autonomous Feedback**: The VLM analyzes the failure scene ("Object slipped Z-axis") and generates a corrective task plan (e.g., "Re-Pick with +20mm depth offset") without human intervention.

### 9.3 Migration to Edge-VLM (Local AI)
To eliminate latency and dependency on external services, the reasoning layer will migrate to **Edge AI**:
- **On-Device Inference**: Utilizing quantized VLMs (e.g., **LLaVA**, **Qwen-VL**) running on local GPUs/NPUs.
- **Reliability**: This ensures 100% operational uptime in air-gapped industrial facilities and allows for higher-frequency visual polling.

### 9.4 Native Cartesian Impedance Control
While the current protocol utilizes Moveit LIN moves, physical handover robustness can be significantly increased via **Active Compliance**:
- **Stiffness Modulation**: Transitioning to a `cartesian_impedance_controller` during the exact moment of object contact.
- **Spring-Damper Dynamics**: Mimicking human "yielding" behavior, allowing the arms to act as virtual springs to absorb internal forces caused by sub-millimeter calibration misalignments.

---

## 10. Experimental Validation and Metrics

A rigorous experimental plan is established to validate the framework's cognitive scalability, software robustness, and physical fidelity across two distinct phases.

### 10.1 Phase 1: Simulation (Gazebo / Ignition)
Simulation is utilized to isolate the cognitive architecture from physical sensor noise and validate multi-threaded efficiency.

#### 10.1.1 Zero-Shot Cognitive Scalability (VLM vs. Scene Complexity)
- **Goal**: Demonstrate that the system generalizes to novel tasks without retraining (unlike RL-based approaches).
- **Setup**: Execute 50 prompts of increasing complexity (e.g., Level 1: "Move the cube right" -> Level 3: "Swap the red cylinder and blue box").
- **Metrics**:
    - **Reasoning Success Rate (RSR)**: % of syntactically correct and logical task plans.
    - **Pydantic Rejection Rate**: Frequency of "hallucination blockages" by the Pydantic schema (validating the safety filter).
    - **VLM Latency**: Mean duration (ms) for plan generation.

#### 10.1.2 Multithreading Validation (The GIL Test)
- **Goal**: Prove that the ROS 2 Python architecture achieves true physical concurrency despite the CPython Global Interpreter Lock.
- **Setup**: Execute independent asynchronous tasks on both arms simultaneously (e.g., Left: Long Pick & Place; Right: Small repetitive moves).
- **Metrics**:
    - **Behavior Tree Tick Jitter**: Frequency stability of the BT (Target: 10Hz). Stability during intensive MoveIt motion planning confirms successful I/O-Bound Context Switching.
    - **Parallel Execution Overlap**: Gantt-style metrics showing overlapping `RUNNING` states of both Action Clients.

#### 10.1.3 Determinism and Fail-Stop Safety
- **Goal**: Confirm the system is "Safe-by-Design," preventing collisions through architectural constraints.
- **Setup**: Dynamically introduce obstacles during trajectory execution or force VLM prompts to violate the midline boundary.
- **Metrics**:
    - **Collision Rate**: Must be strictly 0%.
    - **Fail-Stop Latency**: Time elapsed between a planner failure (IK not found) and the halting of the specific BT lane.

### 10.2 Phase 2: Physical Robot (Franka FR3)
Hardware validation introduces real-world uncertainty, including sensor noise, calibration errors, and contact physics.

#### 10.2.1 Perception Pipeline Robustness (YOLO + Pinhole)
- **Goal**: Validate that the simplified Inverse Pinhole + 2D Centroid approach is sufficient for unstructured tabletop manipulation.
- **Setup**: Position 3 distinct objects across 10 randomized coordinates under varying lighting conditions.
- **Metrics**:
    - **Localization Error (RMSE)**: Root Mean Square Error between computed $(X,Y,Z)$ and physical ground truth.
    - **Grasp Success Rate (GSR)**: % of successful captures on the first attempt, demonstrating metric error tolerance.

#### 10.2.2 Handover Synchronization (The Rendezvous Handshake)
- **Goal**: Verify the reliability of the `threading.Event`-based protocol and linear (LIN) Cartesian execution.
- **Setup**: Perform 20 consecutive mid-air bimanual handovers.
- **Metrics**:
    - **Handover Success Rate (HSR)**: % of transfers completed without item drops.
    - **Synchronization Wait Time**: Idle duration for the donor/recipient at the rendezvous point (demonstrates temporal coordination efficiency).
    - **TCP Linear Deviation**: Millimetric deviation from the ideal straight line during the LIN phase ($O\_F\_ext\_hat\_K$).

#### 10.2.3 Dynamic Obstacle Reactivity
- **Goal**: Validate that Planning Scene updates provide real-time physical protection.
- **Setup**: Manually insert an obstacle (e.g., a shipping box) into one arm's trajectory.
- **Metrics**:
    - **Obstacle Detection Time**: Latency between RGB appearance and MoveIt Collision Object injection.
    - **Fail-Stop Response**: Qualitative confirmation of trajectory halting or autonomous re-planning.

---

### Logic Flow Summary
- **Command**: User input -> Gemini (Reasoning).
- **Execution**: Behavior Tree splits task and drives parallel lanes.
- **Localization**: YOLO + PCA converts pixels into 6DoF PoseStamped.
- **Planning**: MoveGroup + Pilz ensures deterministic, linear/PTP trajectories.
- **Sync**: Handover logic uses Threading Events and **Wrench Feedback** (as planned) for robust bimanual coordination.
- **Orientation Note**: While the perception pipeline is capable of 6DoF orientation estimation via PCA, the current manipulation layer implements a **top-down grasp constraint** for increased stability in unstructured tabletop scenarios.

---

## 15. Laboratory Deployment Readiness Checklist

To transition from simulation to the physical Franka Research 3 hardware, the following environmental and system prerequisites must be satisfied.

### 15.1 Host Machine Requirements (Physical PC)
- [ ] **Kernel**: `PREEMPT_RT` modified kernel is installed and active (`uname -a` should show `PREEMPT_RT`).
- [ ] **Networking**:
    - [ ] Static IP configured on the dedicated Franka NIC (e.g., `192.168.1.1/24`).
    - [ ] Communication verified: `ping 192.168.1.11` (Left) and `ping 192.168.1.12` (Right).
- [ ] **Hardware Drivers**:
    - [ ] NVIDIA Drivers + `nvidia-container-toolkit` must be installed on the host for GPU-accelerated YOLO inference.
    - [ ] RealSense `udev` rules installed on the host (`liblibrealsense2-udev-rules`).

### 15.2 Software & Middleware Configuration
- [ ] **DDS Implementation**: Confirm `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set in the environment to ensure control loop stability.
- [ ] **API Access**: `GEMINI_API_KEY` must be configured in the `.env` file at the workspace root to enable VLM reasoning.
- [ ] **Docker Privileges**: The `docker-compose.yml` must retain `privileged: true` and `network_mode: host` to allow low-level socket access for `libfranka`.

### 15.3 Deployment Hardware Verification
1. **Connectivity Sync**: Run `ros2 launch franka_bimanual_skills lab_bringup.launch.py` and verify `JointState` updates in RViz.
2. **Perception Feed**: Run `ros2 launch franka_bimanual_skills perception.launch.py use_hardware:=true` and verify RGB-D alignment.
3. **Safety Stop**: Verify the physical E-Stop functionality before triggering any `MoveHome` or `Pick` behaviors.

## 16. Dynamic Lab Calibration (Fly-by-Wire)
The framework now supports real-time parameter tuning for the laboratory environment. All critical workspace targets and perception topics have been decoupled from the source code and exposed as ROS 2 parameters.

### 16.1 Perception & VLM Tuning
These parameters are hosted by the `vlm_server_node` and `object_localization_node`.
- `image_topic`: (e.g., `/camera/color/image_raw`).
- `depth_topic`: (e.g., `/camera/depth/image_rect_raw`).
- `use_sensor_data_qos`: **Set to `True`** for RealSense hardware. This uses `Best Effort` (SensorData) reliability to minimize frame lag on local networks.

### 16.2 Workspace Target Tuning
These parameters are hosted by the `fr3_application` node and allow for "one-click" calibration of the physical work area.
- `targets.shared`: The coordinates `[x, y, z]` where arms rendezvous for handover.
- `targets.mid_air`: Safe coordination point.
- `targets.box`: The precise location of the target container in the lab.
- `offsets.pick_z_offset`: Calibrate this for the specific height of the table vs. the robot base.
- `offsets.gripper_open_width`: Set this based on the largest object currently in the lab.

> [!TIP]
> Launching with `use_hardware:=true` automatically injects these lab-standard defaults. For fine-tuning, use:
> `ros2 param set /fr3_application offsets.pick_z_offset 0.105`
