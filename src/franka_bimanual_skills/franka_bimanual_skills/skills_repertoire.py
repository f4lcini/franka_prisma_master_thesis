from typing import List, Literal, Union, Optional
from pydantic import BaseModel, Field

# ==========================================
# 1. ATOMIC SKILL DEFINITIONS
# ==========================================

class ArmSelection(BaseModel):
    arm: Literal["left_arm", "right_arm"]

class FindObjectSkill(BaseModel):
    """Triggers YOLO detection for a specific object."""
    action: Literal["FIND_OBJECT"] = "FIND_OBJECT"
    object_name: str = Field(description="Name of the object to locate (e.g., 'red_cube').")

class WaitSkill(BaseModel):
    """Synchronizes bimanual tasks by waiting for the other arm to finish an action.
    Example: Arm 2 WAIT for Arm 1 to finish PLACE in 'shared' before starting PICK."""
    action: Literal["WAIT"] = "WAIT"
    arm: ArmSelection = Field(description="Which arm is waiting.")
    message: str = Field(description="Explanation of what this arm is waiting for (e.g., 'Wait for left_arm to clear shared zone').")

class PickSkill(BaseModel):
    """Grasps an object or predefined target.
    If target_name is 'base_pose' or 'shared', it uses calibrated coordinates."""
    action: Literal["PICK"] = "PICK"
    target_name: str = Field(description="Target name. Use 'shared' for a table handover, or the YOLO object name.")
    arm: ArmSelection = Field(description="Which arm to use.")
    grasp_type: Literal["top", "side"] = "top"

class PlaceSkill(BaseModel):
    """Releases the object at a target location.
    Predefined locations: 'shared' (middle), 'box' (destination)."""
    action: Literal["PLACE"] = "PLACE"
    target_location: str = Field(description="Location name. Use 'shared' for table handover or 'box' for final placement.")
    arm: ArmSelection = Field(description="Which arm is releasing.")

class MoveHomeSkill(BaseModel):
    """Moves an arm back to its safe resting position."""
    action: Literal["MOVE_HOME"] = "MOVE_HOME"
    arm: ArmSelection
    target_pose_name: Literal["ready", "midway"] = "ready"

# --- HANDOVER SKILLS (Legacy support if needed, but VLM uses Table Handover now) ---
class GiveObjectSkill(BaseModel):
    action: Literal["GIVE"] = "GIVE"
    arm: ArmSelection
    target_name: str = Field(default="mid_air")

class TakeObjectSkill(BaseModel):
    action: Literal["TAKE"] = "TAKE"
    arm: ArmSelection
    target_name: str = Field(default="mid_air")

# ==========================================
# 2. GLOBAL REPERTOIRE DEFINITION
# ==========================================
RobotSkill = Union[FindObjectSkill, WaitSkill, PickSkill, PlaceSkill, MoveHomeSkill, GiveObjectSkill, TakeObjectSkill]

class TaskPlan(BaseModel):
    """Logical plan for Dual-Arm execution. 
    The plan consists of two parallel lanes of actions."""
    task_name: str = Field(description="Brief title of the task.")
    left_arm_sequence: List[RobotSkill] = Field(default_factory=list)
    right_arm_sequence: List[RobotSkill] = Field(default_factory=list)
    coordination_notes: Optional[str] = Field(None, description="Reasoning for parallel choices.")