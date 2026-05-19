from typing import List, Literal, Union, Optional
from pydantic import BaseModel, Field

# ==========================================
# 1. ATOMIC SKILL DEFINITIONS
# ==========================================

class FindObjectSkill(BaseModel):
    """Triggers YOLO detection for a specific object."""
    action: Literal["FIND_OBJECT"] = "FIND_OBJECT"
    target_name: str = Field(description="Name of the object to locate (e.g., 'sports').")
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm's camera detects the object.")

class WaitSkill(BaseModel):
    """Synchronizes bimanual tasks by waiting for a specified duration."""
    action: Literal["WAIT"] = "WAIT"
    seconds: float = Field(default=5.0, description="Duration of wait in seconds.")
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm is waiting.")
    message: str = Field(description="Explanation of what this arm is waiting for.")

class PickSkill(BaseModel):
    """Grasps an object or predefined target."""
    action: Literal["PICK"] = "PICK"
    target_name: str = Field(description="Target name (e.g. 'sports', 'shared').")
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm to use.")
    grasp_type: Literal["top", "side"] = "top"

class PlaceSkill(BaseModel):
    """Releases the object at a target location."""
    action: Literal["PLACE"] = "PLACE"
    target_name: str = Field(description="Target destination name. Use 'shared' for handover or 'box_ws_sx'/'box_ws_dx' for final placement.")
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm is releasing.")

class MoveHomeSkill(BaseModel):
    """Moves an arm back to its safe resting position."""
    action: Literal["MOVE_HOME"] = "MOVE_HOME"
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm moves home.")
    pose_name: Literal["ready", "midway"] = "ready"

class RendezvousSkill(BaseModel):
    """Explicitly synchronizes both arms at a specific waypoint.
    Both arms must hit the RENDEZVOUS point; the orchestrator coordinates them safely."""
    action: Literal["RENDEZVOUS"] = "RENDEZVOUS"
    arm: Literal["left_arm", "right_arm"] = Field(description="Which arm enters rendezvous.")

# ==========================================
# 2. GLOBAL REPERTOIRE DEFINITION
# ==========================================
RobotSkill = Union[FindObjectSkill, WaitSkill, PickSkill, PlaceSkill, MoveHomeSkill, RendezvousSkill]

class TaskPlan(BaseModel):
    """Logical plan for Dual-Arm execution. 
    The plan consists of two parallel lanes of actions."""
    task_name: str = Field(description="Brief title of the task.")
    left_arm_sequence: List[RobotSkill] = Field(default_factory=list, description="Sequence of actions for the left arm.")
    right_arm_sequence: List[RobotSkill] = Field(default_factory=list, description="Sequence of actions for the right arm.")
    coordination_notes: Optional[str] = Field(None, description="Reasoning for parallel choices.")