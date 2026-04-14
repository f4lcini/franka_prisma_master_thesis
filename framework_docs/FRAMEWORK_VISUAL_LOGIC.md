# Bimanual Franka FR3 Framework: Visual Logic & Workflows

This document centralizes the visual representations of the framework's internal logic, synchronization protocols, and behavior tree structures.

---

## 1. Functional Architecture: Custom Framework & Repertoire
This diagram illustrates the relationship between custom ROS 2 packages and the modular **Atomic Skill Repertoire**.

```mermaid
mindmap
    root((FUNCTIONAL ARCHITECTURE))
        Custom ROS 2 Packages
            franka_bimanual_skills
            franka_bimanual_orchestrator
            franka_bimanual_bringup
            franka_custom_interfaces
        Atomic Skill Repertoire
            Perception
                FindObject Detect YOLO 3D-Localization
            Manipulation
                PickSkill LIN-Approach Grasp
                PlaceSkill LIN-Position Release
            Bimanual Handover
                GiveSkill Donor-Sync Wait-Recipient
                TakeSkill Recipient-Sync Wait-Donor
            Utilities
                Home PTP-Sync
                Wait Stationary
```

---

## 2. Perception Pipeline: From Pixels to 6DoF
The transformation of raw sensor telemetry into high-fidelity spatial telemetry.

```mermaid
graph TD
    RGB[RGB Stream 640x480] --> YOLO[YOLOv8 Object Detection]
    Depth[Depth Stream Aligned] --> ROI[ROI Extraction / BBox]
    
    YOLO --> ROI
    
    ROI --> Deprojection[Inverse Pinhole Deprojection]
    Deprojection --> Centroid[3D Centroid Positioning]
    
    ROI --> PCA[PCA Eigen-Decomposition]
    PCA --> Orientation[3D Orientation Quaternion]
    
    Centroid --> Tworld[TF-Free World Frame Transform]
    Orientation --> Tworld
    
    Tworld --> Result([6DoF PoseStamped in World Frame])
```

---

## 3. Reasoning Layer: Cognitive Task Planning
The transition from natural language to type-safe action sequences.

```mermaid
graph TD
    CMD[Natural Language Command] --> Gemini[Gemini 2.5 Flash Engine]
    Gemini --> CoT[Chain-of-Thought Reasoning]
    CoT --> JSON[Raw JSON Output]
    JSON --> Pydantic[Pydantic Schema Validation]
    Pydantic --> TaskPlan[Validated Bimanual TaskPlan]
    
    TaskPlan --> Split[PlanSplitter: Workspace Partitioning]
    Split --> Plans([Left Arm Plan / Right Arm Plan])
```

---

## 4. Behavior Tree Architecture: Mission Orchestration
The hierarchical logic that drives the parallel mission execution.

```mermaid
graph TD
    BT[Behavior Tree Root] --> Selector[Selector: Root_Guard]
    
    Selector --> Done[Condition: Mission_Done?]
    Selector --> Main[Sequence: Bimanual_Orchestrator]
    
    Main --> Gate[Selector: Planning_Gate]
    Gate --> Exists[Condition: Plan_Exists?]
    Gate --> VLM[Action: Gemini_Planner]
    
    Main --> Splitter[Action: Plan_Splitter]
    
    Main --> Parallel[Parallel: Side_By_Side_Execution]
    
    Parallel --> LeftL[Lane: LEFT_ARM]
    Parallel --> RightL[Lane: RIGHT_ARM]
    
    subgraph "Arm Execution Lane (Iterative)"
        Lane[Decorator: Repeat] --> Step[Sequence: Step_Arm]
        Step --> HasPlan[Condition: Has_Arm_Plan?]
        Step --> Iterator[Action: Dynamic_Iterator]
        Step --> Dispatcher[Selector: Dispatcher_Arm]
        Step --> Popper[Action: Plan_Popper]
    end
    
    LeftL --- Lane
    RightL --- Lane

### 4.1 Arm Lane Internal Logic: The Pipeline Pattern
Each arm execution lane operates as a deterministic consumer of the plan queue. The diagram below illustrates the state-machine logic within a single tick sequence.

```mermaid
graph TD
    Start[Tick Lane] --> Iterator[Action: Dynamic_Iterator]
    
    subgraph "Iterator Logic"
        Iterator --> Peek["Peek: action = arm_plan[0]"]
        Peek --> Inject["Inject: Update BB(target, skill, mode)"]
        Inject --> IterSuccess([Return: SUCCESS])
    end

    IterSuccess --> Dispatcher[Selector: Dispatcher_Arm]

    subgraph "Dispatcher Logic (Asynchronous)"
        Dispatcher --> CheckPick{"If skill == 'PICK'"}
        CheckPick -- Yes --> Pick[Trigger PickActionClient]
        
        Dispatcher --> CheckPlace{"If skill == 'PLACE'"}
        CheckPlace -- Yes --> Place[Trigger PlaceActionClient]
        
        Pick -- Running --> RetWait([Return: RUNNING])
        Place -- Running --> RetWait
        
        Pick -- Done --> DispSuccess([Return: SUCCESS])
        Place -- Done --> DispSuccess
    end

    DispSuccess --> Popper[Action: Plan_Popper]

    subgraph "Popper Logic"
        Popper --> Pop["Execute: arm_plan.pop(0)"]
        Pop --> PopSuccess([Return: SUCCESS])
    end

    PopSuccess --> End[Cycle Complete: Wait for Next Tick]
```
```

---

## 5. Motion Planning & Execution Stack
How high-level goals are transformed into physical joint trajectories.

```mermaid
graph TD
    Skill[Atomic Skill Client] --> MoveGroup[MoveGroup Action Client]
    MoveGroup --> Planner[Pilz Industrial Motion Planner]
    
    Planner --> Constraints[Path Constraints: Constraint Planes]
    Planner --> Scene[MoveIt Planning Scene: Dynamic Obstacles]
    
    Constraints --> Trajectory[Trajectory Generation: PTP/LIN]
    Scene --> Trajectory
    
    Trajectory --> Controller[franka_ros2_control: Joint Trajectory Controller]
    Controller --> Hardware([Franka Research 3 Actuators])
```

---

## 6. Bimanual Handover Workflow (Mid-Air Rendezvous)
This diagram maps the low-level Python functions and threading events used to synchronize the donor and recipient arms during a physical object transfer.

```mermaid
graph TD
    Root[Parallel: Side-By-Side Execution]
    
    subgraph "Recipient Lane (execute_take)"
        L1["send_pose_goal pre_take_pose"] --> L2["_recipient_pre_pos_ready.set"]
        L2 --> L3["_donor_pre_pos_ready.wait"]
        L3 --> L4["_donor_ready.wait"]
        L4 --> L5["send_pose_goal_custom take_pose LIN"]
        L5 --> L6["_recipient_ready.set"]
        L6 --> L7["send_gripper_goal grasp_width"]
        L7 --> L8["_recipient_grasped.set"]
        L8 --> L9["send_pose_goal_custom pre_take_pose LIN"]
    end

    subgraph "Donor Lane (execute_give)"
        R1["send_pose_goal pre_give_pose"] --> R2["_donor_pre_pos_ready.set"]
        R2 --> R3["_recipient_pre_pos_ready.wait"]
        R3 --> R4["send_pose_goal_custom give_pose LIN"]
        R4 --> R5["_donor_ready.set"]
        R5 --> R6["_recipient_ready.wait"]
        R6 --> R7["_recipient_grasped.wait"]
        R7 --> R8["send_gripper_goal open_width"]
        R8 --> R9["send_pose_goal_custom pre_give_pose LIN"]
    end

    Root --> L1
    Root --> R1

    %% Handshake connections (In-process Events)
    L2 -. set .-> R3
    R2 -. set .-> L3
    R5 -. set .-> L4
    L6 -. set .-> R6
    L8 -. set .-> R7
```

---

## 7. Technical Stack: Libraries & Middleware
Detailed overview of the third-party libraries and ROS 2 dependencies powering the framework.

```mermaid
mindmap
    root((TECHNICAL STACK))
        ROS 2 Core
            rclpy / rclcpp
            tf2_ros
            cv_bridge
            sensor_msgs
            geometry_msgs
            action_msgs
        Motion Planning
            MoveIt 2 / MoveGroup
            Pilz Industrial Planner
            moveit_msgs
            franka_msgs
            Trajectory Constraints
        Cognitive Layer
            Google Gemini 2.5 Flash
            Pydantic 
            google-generativeai
            Action Dispatcher logic
        Perception & Vision
            YOLOv8 Ultralytics
            OpenCV cv2
            NumPy / SciPy
            PCA via NumPy Linear Algebra
        Orchestration
            py_trees
            py_trees_ros
            threading Events
```
---

## 8. Atomic Skill Repertoire: Modular Execution
The framework's library of primitive behaviors, categorized by functional domain.

```mermaid
mindmap
    root((SKILL REPERTOIRE))
        Perception
            FindObject
            3D Localization
            PCA Orientation
        Manipulation
            Pick Object
            Place Object
            Move To Pose
        Bimanual
            Execute Give
            Execute Take
            Sync Handshake
        Utility
            Home Robot
            Stationary Wait
            Clear Scene
```

---

## 9. Parallelism Model: Logical vs. Physical Concurrency
The framework employs a two-layer parallelism strategy to ensure deterministic control and non-blocking bimanual motion.

### 9.1 Logical Concurrency (Behavior Tree Ticking)
Within the BT, the `Parallel` node manages two execution contexts. This is **logical concurrency**: the tree "taps" both branches in every tick cycle, monitoring states without blocking.

### 9.2 Physical Concurrency (ROS 2 MultiThreadedExecutor)
Underneath the BT, the ROS 2 middleware handles the actual threading. This is **physical concurrency**: callbacks from different robots are processed in parallel on separate CPU cores.

> [!IMPORTANT]
> **GIL Optimization**: The system exploits I/O-bound context switching. When an action goal is sent, the GIL is released, allowing concurrent execution of C++ backends (MoveIt/Pilz) and other Python threads.
> **Deadlock Prevention**: All custom nodes utilize an explicit **ReentrantCallbackGroup** policy. This allows the executor to process overlapping feedback/status callbacks from dual arms simultaneously, even when nested action calls are active.
> **Blackboard Safety**: The shared state is protected by an internal **Recursive Read-Write Lock (RWMutex)** within `py_trees`, ensuring atomic access for dual-arm lanes.

```mermaid
graph TD
    subgraph "Thread 1: BT Management (1Hz)"
        BT[Behavior Tree Tick] --> Parallel[Parallel Node]
        Parallel --> LaneL[Lane Left: RUNNING]
        Parallel --> LaneR[Lane Right: RUNNING]
    end

    subgraph "Thread Pool: ROS 2 MultiThreadedExecutor"
        Core1[Thread Alpha] --> CallL[Callback: Left Arm Feedback]
        Core2[Thread Beta] --> CallR[Callback: Right Arm Feedback]
        Core3[Thread Gamma] --> Perception[Callback: YOLO Stream]
    end

    subgraph "Shared Space (Inter-process)"
        BB[(Blackboard: Shared State)]
        EventL{Threading.Event: Left}
        EventR{Threading.Event: Right}
    end

    LaneL -. status .-> BT
    LaneR -. status .-> BT
    
    CallL -. data .-> BB
    CallR -. data .-> BB
    
    LaneL -. wait/set .-> EventL
    LaneR -. wait/set .-> EventR
```

---

## 10. Design Analysis: Assumptions & Architectural Gaps
Visual overview of the system's design constraints and current technical vulnerabilities.

### 10.1 Project Design Assumptions
```mermaid
mindmap
    root((DESIGN ASSUMPTIONS))
        Kinematics
            Top-Down Grasping Z-Axis
            Midline Division Static Split
        Perception
            Bounding Box Center Rep
            Single-Pixel Depth Sample
        Environment
            Static Camera Extrinsics
            Unstructured Tabletop
        Handover
            Logic-Gate Sync
            No Mechanical Slippage
```

### 10.2 Architectural Gaps & Vulnerabilities
```mermaid
graph TD
    subgraph "Perception Gaps"
        Gap1[Single-Pixel Sampling] --> Fail1[Concave Objects / U-Shape]
        Fail1 --> Result1([Depth Collision / Air Grasps])
    end

    subgraph "Synchronization Gaps"
        Gap2[Open-Loop Handover] --> Fail2[Grasp Failure + Logic Success]
        Fail2 --> Result2([Object Dropped])
    end

    subgraph "System Gaps"
        Gap3[VLM Cloud Dependency] --> Fail3[Network Latency / API Outage]
        Gap3 --> Fail4[Air-Gapped Environments]
        
        Gap4[Fail-Stop Policy] --> Fail5[No Autonomous Recovery]
    end
```

---

## 11. Future Roadmap: Advanced Framework Evolutions
Strategic technical paths for industrial-grade autonomy and physics.

### 11.1 Evolution Pillars

```mermaid
mindmap
    root((FUTURE ROADMAP))
        Kinematics
            Dynamic Load Balancing
            Yoshikawa Manipulability
        Cognitive
            Closed-Loop Recovery
            VLM Failure Analysis
        Infrastructure
            Edge-VLM Local AI
            Zero-Latency Inference
        Control
            Cartesian Impedance
            Active Compliance
```

---

## 12. Experimental Metrics & Competitive Analysis
Templates for performance reporting and comparative evaluation.

### 12.1 Parallel Execution Gantt Chart (Sample Template)
This chart visualizes the bimanual concurrency enabled by the **ReentrantCallbackGroup** and the asynchronous **Behavior Tree**.


### 12.2 Comparative Analysis: White-Box Advantage
```mermaid
mindmap
    root((COMPARISON))
        SOTA Models RT-1/X
            Training: Months/100k Demos
            Nature: Black-Box Weights
            Safety: Statistical
        This Framework
            Training: Zero-Shot Reasoning
            Nature: White-Box / Interpretable
            Safety: Architectural / Hard Limits
```

---

### 12.3 Summary of Evaluation Metrics
```mermaid
mindmap
    root((METRICS))
        Simulation Logic
            Reasoning Success RSR
            Pydantic Rejection Rate
            VLM Latency ms
            BT Jitter Frequency
        Physical Performance
            Localization RMSE
            Grasp Success GSR
            Handover Success HSR
            TCP Linear Deviation
        Safety & Reactivity
            Collision Rate strictly 0
            Fail-Stop Latency
            Obstacle Detect Time
```

---

## 13. System Summary: The End-to-End Pipeline
A high-level view of the framework's operational flow.

```mermaid
graph LR
    Input[Natural Language Command] --> GN[Gemini 2.5: Reasoning]
    GN --> BT[Behavior Tree: Orchestration]
    BT --> LOC[Perception: 3D Localization]
    LOC --> MG[MoveGroup: Pilz Planning]
    MG --> FR3([Franka FR3: Execution])
    FR3 --> SYNC{Handover Sync}
```

---

## 14. Simulation Progress & Deployment Milestones

This diagram summarizes the current validation status of the bimanual framework within the Gazebo/MoveIt 2 simulation environment.

```mermaid
graph LR
    subgraph "Phase 1: Validated (SUCCESS)"
        P1[Single Arm PNP] --- P2[Dual Arm Parallel PNP]
        P2 --- P3[Shared Workspace Coordination]
        P3 --- P4[YOLO + 3D localization]
    end

    subgraph "Phase 2: In Development (STALLED/HARDCODED)"
        D1[Linear Rendezvous Logic] -.-> D2[Mid-Air Handover]
        D2 -.-> D3[Wrench Feedback Integration]
        
        style D2 fill:#f96,stroke:#333,stroke-width:2px,stroke-dasharray: 5 5
    end

    P4 --> D1
```

### 14.1 Status Breakdown
- **Success**: The system correctly handles **Bimanual Pick and Place** in a shared tabletop workspace. Collision avoidance is ensured by the midline spatial partitioning and MoveIt planning scene updates.
- **In-Progress**: The **Mid-Air Handover** is currently utilizing **hardcoded joint positions** for experimental validation. Continuous linear (LIN) Cartesian execution and robust threading-event handshakes are currently being refined to transition from hardcoded rendezvous to dynamic, perception-driven transfers.

