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

PREDEFINED_TARGETS = {
    "base_pose": (1.1, 0.2, 0.225),
    "shared":    (0.6, 0.5, 0.225),    # Table crossover point (aligned to Z=0.225)
    "box":       (0.1, 0.1, 0.140),    # Receptacle
    # "mid_air":   (0.5, 0.4, 0.5)     # REMOVED: Handover disabled
}

# NOTE: There is a ~15cm projection offset between the robot flange (link hand) 
# and the TCP (link tcp) when the robot is rotated for handover. 
# MoveIt targets the TCP. Gazebo GUI shows the flange. 
# Target (0.6, 0.4) in code will show as approx (0.45, 0.25) in Gazebo GUI.

# Actual positions extracted from bimanual_custom.world for tracking discrepancies
GAZEBO_WORLD_POSES = {
    "red_cube": (1.1, 0.2, 0.225), # Matches base_pose
    "open_box": (0.1, 0.1, 0.23)   # Difference with 'box': (-0.1, -0.2, +0.09)
}

DEFAULT_OFFSETS = {
    'approach_clearance': 0.1,
    'pick_z_offset': 0.09,      # Deeper engagement to avoid landing 'on top' of the cube
    'place_z_offset': 0.110,     # release 5mm above table
    'gripper_open_width': 0.08,
    'gripper_grasp_width': 0.048,
    'safety_pause_short': 1.5,   # Increased for physics settlement
    'safety_pause_long': 1.0,
    'handover_safety_offset': 0.25,
    'handover_donor_z_offset': 0.20,     # Reduced slightly to keep elbow comfortable
    'handover_recipient_x_offset': -0.25, # Adjusted to avoid self-collision at X=0.15
    'handover_timeout_sec': 120.0        # rendezvous timeout
}

READY_POSE_VALUES_RIGHT = [1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]
READY_POSE_VALUES_LEFT  = [-1.570796, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]

WORLD_FRAME = "world"

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

def apply_donor_handover_orientation(pose):
    """Donor: Points along -X, fingers Vertical."""
    # Canonical Handover Orientation for Franka Right
    pose.orientation.x = 0.7071
    pose.orientation.y = 0.7071
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

def apply_recipient_handover_orientation(pose):
    """Recipient: Points along +X, fingers Horizontal."""
    # Canonical Handover Orientation for Franka Left
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.7071
    pose.orientation.w = 0.7071

def apply_top_down_orientation(pose):
    """Standard Pick/Place orientation (TCP down)."""
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
