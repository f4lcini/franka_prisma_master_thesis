# Bimanual Mid-Air Handover: Technical Report (2026-04-04)

## 1. Objective
Enable a robust, autonomous, and collision-free mid-air handover between two Franka Research 3 robots without centralized 14-DOF planning. The handover must accommodate 90-degree relative gripper orientations (Horizontal vs. Vertical) to ensure a stable cube exchange.

---

## 2. "Safe-Decentralized" Architecture
Instead of a single complex planning group, we implemented a **Decentralized Coordination** system with a multi-layered safety framework.

### 2.1 Virtual Fences (Path Constraints)
Each robot is restricted to its half of the workspace during routine pick/place operations to prevent inter-arm interference:
- **X-Axis Split**: Located at `X=0.6` in the `world` frame.
- **Robot 1 (Right)**: Restricted to `X > 0.58`.
- **Robot 2 (Left)**: Restricted to `X < 0.62`.
- **Buffer Zone**: A ±2cm overlapping buffer allows for small elbow swings while maintaining strict separation.
- **Dynamic Bypass**: The `is_handover` flag automatically disables these constraints only during the synchronized exchange phase.

### 2.2 Parallel Execution Skillset
Leveraging ROS 2 Action Servers and Behavior Tree parallel nodes, we defined specialized skills:
- **`GIVE` (Donor)**: Approaches the meeting point, waits for the recipient's ready signal, synchronizes the final linear approach, and releases.
- **`TAKE` (Recipient)**: Positions itself in pre-handover, waits for donor arrival, performs the grasp, and signals completion.

---

## 3. Kinematic Optimizations
To resolve `NO_IK_SOLUTION` issues and singularity rejections, the following adjustments were implemented:

### 3.1 Handover Orientation (Opposed Horizontal)
- **Donor**: Gripper pitch -90° (Facing -X), Fingers Top/Bottom.
- **Recipient**: Gripper pitch +90° (Facing +X), Fingers Side-to-Side.
- This creates a face-to-face exchange where fingers do not collide.

### 3.2 Workspace Relocation
- **Front-Edge Meeting Point**: Shifted the handover coordinates to `(0.6, 0.3, 0.5)`.
- **Rationale**: Moving the hand-off further from the base column (`Y=0.3` instead of `0.5`) significantly increases the elbow workspace and IK stability for both arms, as they reach "forward" and away from singularities.

### 3.3 Linear Approach (LIN)
- **Safety Gap**: Initial pre-handover points are separated by **50cm** (`offset = 0.25m` per arm).
- **Execution**: Both arms perform a Pilz-based `LIN` (Linear) move towards `X=0.6` once synchronized.

---

## 4. Verification Framework
Due to Gemini API quota limits (429 Resource Exhausted), a dedicated **Manual Orchestrator** was developed:
- **`test_handover.py`**: A threaded Python script that bypasses the VLM and directly calls the action sequence: `PICK` -> `GIVE/TAKE` -> `PLACE`.
- **Threaded Spinning**: Implemented a background executor to reliably capture action feedback and result statuses in an asynchronous environment.

---

## 5. Summary of Key Files
- `/src/franka_task_orchestrator/franka_task_orchestrator/simple_moveit_server.py`: Core logic for Action Servers and Virtual Walls.
- `/src/fr3_mtc_executor/scripts/test_handover.py`: Manual orchestration and verification script.
- `/src/fr3_application/fr3_application/skills_repertoire.py`: VLM-level skill definitions updated for parallel logic.

---
**Status**: Architecture verified and functional in simulation. Ready for full integration with the VLM reasoning pipeline once API quota resets.
