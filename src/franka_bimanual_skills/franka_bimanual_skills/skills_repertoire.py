from pydantic import BaseModel, Field
from typing import Literal, List, Union

ArmSelection = Literal["left", "right", "both", "left_arm", "right_arm"]

# ==========================================
# 1. ATOMIC ACTIONS DEFINITION (SKILLS)
# ==========================================

class FindObjectSkill(BaseModel):
    """Locates the 3D coordinates of an object using YOLO in real-time."""
    action: Literal["FIND_OBJECT"] = "FIND_OBJECT"
    target_name: str = Field(description="The name of the object to search for (e.g. 'red_cube').")
    arm: ArmSelection = Field(description="Which arm requires finding the object before picking.")

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

# --- MID-AIR HANDOVER SKILLS (TEMPORARILY DISABLED) ---
# class GiveObjectSkill(BaseModel):
#     """(Donor Arm) High-level skill for bimanual exchange. 
#     The arm moves to the mid-air point and holds the object horizontally (pointing toward recipient).
#     Wait for TAKE to be called in parallel."""
#     action: Literal["GIVE"] = "GIVE"
#     arm: ArmSelection
#     target_name: str = Field(default="mid_air", description="Handover point 'mid_air'.")
#
# class TakeObjectSkill(BaseModel):
#     """(Recipient Arm) High-level skill for bimanual exchange.
#     The arm moves to the mid-air point and takes the object horizontally (pointing toward donor).
#     Execute in parallel with GIVE for maximum fluidity."""
#     action: Literal["TAKE"] = "TAKE"
#     arm: ArmSelection
#     target_name: str = Field(default="mid_air", description="Handover point 'mid_air'.")

# ==========================================
# 2. GLOBAL REPERTOIRE DEFINITION
# ==========================================
RobotSkill = Union[FindObjectSkill, WaitSkill, PickSkill, PlaceSkill, MoveHomeSkill]

class TaskPlan(BaseModel):
    """Logical plan for Dual-Arm execution. 
    Coordinate both arms in parallel for maximum efficiency. 
    If a target is out of reach, use a handover via 'shared' zone."""
    reasoning: str = Field(description="Explain why you chose this arm and if a handover is needed.")
    sequence: List[RobotSkill] = Field(description="Ordered list of actions.")