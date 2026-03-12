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
    """Releases the currently held object at a specified target location."""
    action: Literal["PLACE"] = "PLACE"
    target_location: str = Field(
        description="The name of the container (e.g., 'blue box') or area (e.g., 'shared_workspace', 'table_corner') where the object should be released."
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
                    "DUAL-ARM RULES: If an object needs to be moved to an area out of reach from the arm currently closest to the object, "
                    "you MUST plan an indirect transfer sequence: "
                    "1) PICK with the first arm, 2) PLACE in the 'shared_workspace', 3) MOVE_HOME the first arm to make room, "
                    "4) FIND_OBJECT on the newly moved object, 5) PICK with the second arm, 6) PLACE at the final destination."
    )
    sequence: List[RobotSkill] = Field(
        description="The ordered list of robotic actions to strictly execute. Do not invent new actions."
    )