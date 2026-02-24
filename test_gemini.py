import os
import json
import argparse
from pydantic import BaseModel, Field
from typing import Literal
from PIL import Image
from google import genai
from google.genai import types

# ---------------------------------------------------------
# 1. Schema Definition (Data Contract for ROS2/Behavior Trees)
# ---------------------------------------------------------
class RobotDecision(BaseModel):
    selected_arm: Literal["left_arm", "right_arm"] = Field(
        description="The arm selected based on the spatial position of the object."
    )
    action_type: Literal["PICK", "PLACE", "POUR"] = Field(
        description="The type of primitive action the Motion Planner (MTC) should execute."
    )
    reasoning: str = Field(
        description="Brief justification for the choice (e.g., 'object is on the right')."
    )

# ---------------------------------------------------------
# 2. VLM Inference Function
# ---------------------------------------------------------
def run_vlm_query():
    # Argument Setup (Matching Yigit's workflow)
    parser = argparse.ArgumentParser(description="VLM Query for Bimanual Robot Decisions.")
    parser.add_argument("--image", default="/home/falco_robotics/mm_ws/target-image.jpeg", help="Path to input image")
    parser.add_argument("--model", default="models/gemini-3-flash-preview", help="Full model name string")
    parser.add_argument("--max_side", type=int, default=112, help="Resize image to save quota/tokens")
    parser.add_argument("--thinking", default="LOW", choices=["LOW", "MEDIUM", "HIGH"], help="Reasoning effort level")
    parser.add_argument("--jpeg_quality", type=int, default=75, help="JPEG compression quality")
    parser.add_argument("--output", default="/home/falco_robotics/mm_ws/vlm_decision.json", help="Path to save JSON result")
    args = parser.parse_args()

    # API Key Verification
    if "GEMINI_API_KEY" not in os.environ:
        print("ERROR: GEMINI_API_KEY environment variable not set.")
        return

    # Client Initialization
    client = genai.Client()
    
    # Image Loading and Pre-processing (Critical to avoid 429 Errors)
    try:
        img = Image.open(args.image)
        if args.max_side > 0:
            img.thumbnail((args.max_side, args.max_side))
            print(f"Image resized to: {img.size}")
    except Exception as e:
        print(f"Error loading image: {e}")
        return

    # Optimized Prompt for Bimanual Manipulation
    # Note: We explicitly target 'the mug' to avoid picking the 'plant' again.
    prompt = """
    You are the High-Level Planning module for a bimanual Franka FR3 robot.
    Analyze the image and identify the target object: a mug.
    
    RULES:
    - If the mug is in the LEFT half of the image, use 'left_arm'.
    - If the mug is in the RIGHT half of the image, use 'right_arm'.
    - If the mug is not found, set action_type to 'IDLE'.
    
    Return the result strictly in JSON format according to the provided schema.
    """

    # Generation Configuration
    cfg = types.GenerateContentConfig(
        response_mime_type="application/json",
        response_schema=RobotDecision,
        thinking_config=types.ThinkingConfig(thinking_level=args.thinking),
        temperature=0.1, # Low temperature for deterministic results
    )

    print(f"Sending request to model: {args.model}...")
    
    try:
        # API Call 
        response = client.models.generate_content(
            model=args.model,
            contents=[prompt, img],
            config=cfg
        )
        
        # Parse the JSON response
        decision_data = json.loads(response.text)
        
        # Display Results
        print("\n--- VLM RESULT ---")
        print(json.dumps(decision_data, indent=4))
        
        # Save to physical file for the ROS2 Node/BT to read
        with open(args.output, "w") as f:
            json.dump(decision_data, f, indent=4)
        print(f"\n[SUCCESS] Decision saved to: {args.output}")
        
    except Exception as e:
        print(f"\nINFERENCE ERROR: {e}")

if __name__ == "__main__":
    run_vlm_query()