import os

# --- WORKSPACE CONFIGURATION ---
# Automatic discovery of workspace root
try:
    # Assuming config.py is in src/franka_bimanual_skills/franka_bimanual_skills/
    _current_dir = os.path.dirname(os.path.abspath(__file__))
    WORKSPACE_ROOT = os.path.abspath(os.path.join(_current_dir, "../../../"))
    SRC_PATH = os.path.join(WORKSPACE_ROOT, "src")
    
    # Docker/Container override
    if os.path.exists("/mm_ws/src"):
        WORKSPACE_ROOT = "/mm_ws"
        SRC_PATH = "/mm_ws/src"
except Exception:
    WORKSPACE_ROOT = "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis"
    SRC_PATH = os.path.join(WORKSPACE_ROOT, "src")

MOVEIT_ERROR_CODES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "START_STATE_VIOLATES_VELOCITY_LIMITS",
    -13: "GOAL_IN_COLLISION",
    -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -15: "GOAL_VIOLATES_VELOCITY_LIMITS",
    -16: "GOAL_CONSTRAINTS_VIOLATED",
    -17: "INVALID_GROUP_NAME",
    -18: "INVALID_GOAL_CONSTRAINTS",
    -19: "INVALID_ROBOT_STATE",
    -20: "INVALID_LINK_NAME",
    -21: "INVALID_OBJECT_NAME",
    -31: "NO_IK_SOLUTION"
}

# --- DEFINITIVE REFERENCE SYSTEM (Table Top Center = 0,0,0) ---
# Values from robot_poses.yaml. Z=0 is the table surface.
TABLE_CONFIG = {
    "center": (0.0, 0.0, 0.0), 
    "size": (1.215, 0.6, 0.04),
    "height_from_floor": 0.73
}

ROBOT_BASES = {
    "franka1": {"x": 0.4375, "y": 0.205, "z": 0.06, "yaw": 3.14159265359}, # Right Arm
    "franka2": {"x": -0.4175, "y": 0.205, "z": 0.06, "yaw": 0.0}            # Left Arm
}

# --- TARGETS IN TABLE REFERENCE FRAME (Z=0 is surface) ---
# Table margins: X [-0.6, 0.6], Y [-0.3, 0.3]
PREDEFINED_TARGETS = {
    "shared":        (0.0, -0.2, 0.0),      # Shared zone (at table level)
    "box":           (-0.4, -0.25, 0.0),   # In front of Left Arm (Franka2), at table level
    "target_object": (0.4, -0.25, 0.0),    # In front of Right Arm (Franka1), at table level
    "mid_air":       (0.0, 0.0, 0.4)       # Safe transition point
}

# NOTE: There is a ~15cm projection offset between the robot flange (link hand) 
# and the TCP (link tcp) when the robot is rotated for handover. 
# MoveIt targets the TCP. 

# --- LAB CALIBRATED OFFSETS & PARAMETERS ---
DEFAULT_OFFSETS = {
    # --- Skill Offsets ---
    'approach_clearance': 0.10,
    'pick_z_offset': 0.13,
    'pick_x_offset': 0.0,
    'pick_y_offset': 0.0,
    'place_z_offset': 0.15,
    'safety_pause_short': 0.2,   
    'safety_pause_long': 0.5,
    'settling_time': 0.5,                # Generic settling time for LIN/PTP
    
    # --- Gripper Parameters ---
    'gripper_open_width': 0.075,
    'gripper_grasp_width': 0.028,
    'gripper_max_effort': 10.0,
    'gripper_safe_width_limit': 0.075,   # Hardware safety limit for FR3
    
    # --- MoveIt / Planning Parameters ---
    'planning_attempts': 5,
    'planning_time': 10.0,
    'velocity_scaling': 0.3,
    'acceleration_scaling': 0.1,
    'joint_tolerance': 0.01,
    'position_tolerance': 0.01,
    'orientation_tolerance': 0.05
}

# --- TARGET-SPECIFIC OVERRIDES ---
# Use these to tune approach/offsets for specific objects or locations
TARGET_OFFSETS = {
    "box": {
        "place_z_offset": 0.20,       # Posa la scatola a 5cm dal tavolo (più delicato)
        "approach_clearance": 0.1    # Spazio verticale standard
    },
    "bottle": {
        "pick_z_offset": 0.20,        # Prendi la bottiglia a circa metà corpo (12cm)
        "gripper_grasp_width": 0.055, # Presa sicura per bottiglia standard
        "approach_clearance": 0.1     # 10cm sopra la bottiglia
    },
    "cup": {
        "pick_z_offset": 0.15,        # Prendi la tazza a 6cm dal tavolo
        "gripper_grasp_width": 0.055,  # Un po' più largo per sicurezza
        "approach_clearance": 0.1     # 10cm sopra la tazza
    },
    "sports": {
        "pick_z_offset": 0.16,
        "gripper_grasp_width": 0.055,
        "pick_y_offset": 0.03,
        "approach_clearance": 0.1
    }
}

# --- SPLIT READY POSES (Joint Configurations) ---
READY_POSE_VALUES_RIGHT = [1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]
READY_POSE_VALUES_LEFT  = [-1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]

# Aggressive 'Parallelismo Spinto' Poses (Joint1 shifted 45 deg towards shared zone)
MIDWAY_POSE_VALUES_RIGHT = [1.35, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]
MIDWAY_POSE_VALUES_LEFT  = [-1.35, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]

# Safe Offset Poses (Visible 20-30 deg shift on Joint 6 for clear parallel testing)
OFFSET_POSE_VALUES_RIGHT = [1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.9, 0.785398]
OFFSET_POSE_VALUES_LEFT  = [-1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.2, 0.785398]

# Compatibility for old code
READY_POSE_VALUES = [0.0, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]

WORLD_FRAME = "table"

def parse_error_code(code_val):
    return MOVEIT_ERROR_CODES.get(code_val, f"UNKNOWN_ERROR_CODE_{code_val}")

def get_arm_config(request_arm, logger=None):
    if request_arm == "left_arm":
        return "franka2_arm", "franka2_fr3_hand_tcp"
    elif request_arm == "right_arm":
        return "franka1_arm", "franka1_fr3_hand_tcp"
    else:
        if logger:
            logger.error(f"❌ Unrecognized arm request: '{request_arm}'")
        return None, None

def apply_top_down_orientation(pose):
    """Standard Pick/Place orientation (TCP down)."""
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

def apply_rotated_top_down_orientation(pose):
    """Pick/Place orientation rotated 90 deg around Z axis."""
    pose.orientation.x = 0.7071
    pose.orientation.y = 0.7071
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

