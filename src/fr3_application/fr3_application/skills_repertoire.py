from pydantic import BaseModel, Field
from typing import Literal, List, Union

ArmSelection = Literal["left_arm", "right_arm"]

# ==========================================
# 1. ATOMIC ACTIONS DEFINITION (SKILLS)
# ==========================================

class FindObjectSkill(BaseModel):
    """Locates the 3D coordinates of an object in space using the shared camera (YOLO).
    This action detects the global scene and does not depend on a specific arm."""
    action: Literal["FIND_OBJECT"] = "FIND_OBJECT"
    target_name: str = Field(
        description="The name of the object to search for (e.g., 'red cube', 'blue apple'). "
                    "Always include the color if specified by the user to avoid ambiguity."
    )

class PickSkill(BaseModel):
    """Grasps an object using one of the two robotic arms.
    WARNING: This action MUST ALWAYS be preceded by a FIND_OBJECT action for the same target."""
    action: Literal["PICK"] = "PICK"
    target_name: str = Field(
        description="The name of the object to grasp. This must exactly match the target_name of the preceding FIND_OBJECT action."
    )
    arm: ArmSelection = Field(
        description="Which arm to use ('left_arm' or 'right_arm'). Select the most suitable arm based on reachability and position."
    )
    grasp_type: Literal["top", "side"] = Field(
        description="Use 'top' for objects accessible from above (e.g., flat cylinders, boxes), and 'side' for vertical objects (e.g., bottles).",
        default="top"
    )

class PlaceSkill(BaseModel):
    """Releases the currently held object at a specified target location.
    WARNING: If the target_location is a physical object (e.g., 'blue box', 'tray'), this action 
    MUST be immediately preceded by a FIND_OBJECT action for that specific container."""
    action: Literal["PLACE"] = "PLACE"
    target_location: str = Field(
        description="The name of the container (e.g., 'blue box') or static area (e.g., 'shared_workspace', 'home') where the object should be released."
    )
    arm: ArmSelection = Field(
        description="Which arm is releasing the object. THIS MUST BE THE SAME ARM that performed the corresponding PICK action."
    )

class MoveHomeSkill(BaseModel):
    """Moves a specific robotic arm back to its safe resting position (Home).
    Usually performed at the end of a task or to clear the workspace for the other arm."""
    action: Literal["MOVE_HOME"] = "MOVE_HOME"
    arm: ArmSelection = Field(
        description="Which arm to move back to the rest position."
    )

# ==========================================
# 2. GLOBAL REPERTOIRE DEFINITION
# ==========================================
# The union of all possible actions the robot can perform.
RobotSkill = Union[FindObjectSkill, PickSkill, PlaceSkill, MoveHomeSkill]

# ==========================================
# 3. FINAL LOGICAL PLAN SCHEMA (VLM OUTPUT)
# ==========================================

class TaskPlan(BaseModel):
    """The sequential action plan to complete the user request in a Dual-Arm setup."""
    reasoning: str = Field(
        description="Provide a step-by-step logical explanation for the chosen plan. "
                    "Evaluate spatial reachability based on the starting object and final destination. "
                    "If a single arm can reach both locations comfortably, explicitly state this and plan a direct single-arm PICK and PLACE sequence. "
                    "If they are out of the kinematic reach of a single arm, explicitly reason that a bimanual handover is required, "
                    "and ONLY THEN plan an indirect transfer sequence (e.g., Arm 1 Picks and Places in the 'shared_workspace', "
                    "Arm 1 Moves Home, Arm 2 Finds the new object, Arm 2 Picks and Places at destination, Arm 2 Moves Home)."
    )
    sequence: List[RobotSkill] = Field(
        description="The ordered list of robotic actions to strictly execute. Do not invent new actions."
    )