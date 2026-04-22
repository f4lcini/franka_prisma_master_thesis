# Franka Bimanual Framework Status

**Last Updated:** April 2049

## 5. Safety & Collision Avoidance
La sicurezza del framework bimanuale è garantita su più livelli, anche con l'esecuzione diretta:

*   **Pianificazione Collision-Aware**: Ogni comando (anche in parallelo) passa attraverso `move_group.plan(plan)`. Questo processo verifica la validità del percorso contro il `PlanningScene`, evitando ostacoli statici (tavolo, box) e il corpo stesso dei robot.
*   **Lane Constraints (Vincoli di Corsia)**: Il planner C++ applica dinamicamente un `PositionConstraint` che confina ogni braccio nella propria metà del tavolo. Questo impedisce ai bracci di incrociarsi e collidere tra loro durante i movimenti indipendenti.
*   **Aggiornamento Continuo del World**: L'API Python pubblica costantemente lo stato attuale del robot nel `PlanningScene`. Quando un braccio pianifica, "vede" dove si trova l'altro braccio in quel momento.
*   **Bypass dell'Esecuzione**: L'invio diretto ai controller (`FollowJointTrajectory`) è sicuro perché la traiettoria inviata è **già stata validata** come sicura e senza collisioni da MoveIt pochi millisecondi prima.

**Workspace:** `/home/falco_robotics/vf_projects_portfolio/mm_ws`

## 4. Behavior Tree & Orchestration Design
The framework's Behavior Tree (BT) is designed to be **plan-agnostic**. It operates as a "Universal Dispatcher":

*   **Dynamic Dispatching**: The tree contains all possible skill branches (PICK, PLACE, MOVE_HOME, etc.). At each tick, the `DynamicActionIterator` reads the current step from the Pydantic JSON plan (residing on the Blackboard) and illuminates only the required branch.
*   **Robust Parallelism**: The `PlanSplitter` automatically divides a unified bimanual plan into two independent arm-specific lanes. This ensures that the framework handles arbitrary parallel sequences without manual BT reconfiguration.
*   **Pydantic Integration**: The root `VlmActionClient` is responsible for fetching and validating the JSON plan. The BT structure itself is a permanent architecture that executes whatever the JSON dictates.
*   **Test Scenarios**: Scripts like `atom_synch_home.py` are diagnostic tools to bypass the VLM during physical validation, but the core `main_engine.py` remains the production orchestrator for dynamic tasks.

## 1. Overview of the Framework Architecture
The framework is a complex and highly structured ROS 2 workspace designed for bimanual manipulation using Franka Emika (FR3) robots. It leverages both Python (high-level skills/API) and C++ (low-level planning). 

### Core Components:
*   **`franka_bimanual_skills`**: Contains the higher-level logic in Python.
    *   `RobotControlAPI`: Provides a synchronous interface to avoid ROS 2 `asyncio` deadlocks (e.g., `wait_for_future` wrapper). Wraps common commands for both joint goals, Cartesian goals, and parallel goals.
    *   `SkillBehaviors`: Defines complex manipulation sequences like `execute_pick`, `execute_place`, `execute_give`, `execute_take`, and `execute_home`.
*   **`franka_bimanual_planner`**: Contains `bimanual_planner_node.cpp`, a powerful Action Server (`franka_custom_interfaces/action/ParallelMove`). 
    *   It handles both right and left arms (`franka1_manipulator`, `franka2_manipulator`).
    *   It explicitly uses the **Pilz Industrial Motion Planner** for smooth, deterministic trajectories (PTP) instead of default MoveIt! planners.
*   **`franka_manipulation_env`**: Environment and simulation definitions.
    *   `sim_and_control.launch.py`: Wraps standard simulation spawns, handles specifying starting joint configurations, and automatically bridges the static TF for `camera_link`.
*   **`moveit_task_constructor` (MTC)**: Submodules/packages used for multi-stage planning logic.

## 2. Current Status of the Simulations
The environment is thoroughly set up for Gazebo-based simulation and MoveIt 2 integration.
*   **Ready-to-Use Launch Files**: Files like `demo_moveit_bimanual.launch.py` correctly orchestrate Gazebo and MoveIt. 
*   **Robust Startup Sequencing**: The framework incorporates a timer-based delay (`period=12.0s`) to ensure Gazebo is fully loaded before the MoveIt Nodes execute, preventing crash-on-startup issues.
*   **Sensory Integration**: The camera tracking frame is properly injected via `static_transform_publisher` (`fr3_system/camera_link/camera`), allowing manipulation tasks based on vision.
*   **Current State**: Simulations are functional and capable of resolving kinematic goals alongside constraint-aware planning (e.g., table constraints are dynamically applied in the C++ planner when not doing handovers).

## 3. Main Challenges and Identified Bottlenecks
Through code inspection, several key challenges and implementation hurdles that were tackled (or are actively pending) have been identified:

1.  **ROS 2 Python Async/Futures Deadlocks**:
    *   In `RobotControlAPI` there is deliberate engineering to create *synchronous paradigms* out of the asynchronous ActionClient futures. Without this, standard execution loops block each other.
2.  **MoveIt Frame References**:
    *   There is a known quirk between Python action requests and MoveIt headers, specifically patched in the `bimanual_planner_node`: `if (target_pose_msg.header.frame_id == "base_pose") target_pose_msg.header.frame_id = "world";`. Frame mismatches without this fix lead to sudden planning failures.
3.  **MoveIt Default Trajectories vs. Pilz**:
    *   MoveIt's standard probabilistic planners cause jerky motion; the framework actively forces `setPlanningPipelineId("pilz_industrial_motion_planner")` for predictability. 
4.  **Executor Conflicts (C++)**:
    *   In `bimanual_planner_node.cpp`, MoveIt and the Action server execute in isolated, distinct Nodes added to a `MultiThreadedExecutor`. Mixing them into a single Node blocks execution.
5.  **MoveIt Task Constructor (MTC) Edge Cases**:
    *   There remain deep `TODO/FIXME` issues inside the Python bindings (`pybind11`) for the MTC (e.g., `test_gil_scoped.py` and `test_exceptions.py`), signaling that extending MTC stages purely in Python might lead to GIL (Global Interpreter Lock) conflicts.
6.  **Permission/Build Overhead**:
    *   (Observed from running processes): File permission mismatch between Docker environments and local Linux users requiring long `sudo chown` operations on `build/`, `install/`, and `log/`.
