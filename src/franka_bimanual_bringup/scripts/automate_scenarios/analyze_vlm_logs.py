import os
import json

base_dir = "/home/falco_robotics/vf_projects_portfolio/mm_ws/src/franka_bimanual_bringup/scripts/automate_scenarios/experiment_logs/VLM_INTEGRATED_EXPERIMENTS"

results = {
    "total": 0,
    "success": 0,
    "fail_syntactic": 0,
    "fail_semantic": 0,
    "failures": []
}

for root, dirs, files in os.walk(base_dir):
    for f in files:
        if f.endswith(".json"):
            results["total"] += 1
            path = os.path.join(root, f)
            with open(path, "r") as file:
                try:
                    data = json.load(file)
                    success = data.get("rsr_vlm_success", False)
                    vlm_plan = data.get("vlm_output_plan", None)
                    
                    if success:
                        results["success"] += 1
                    else:
                        is_string = isinstance(vlm_plan, str)
                        if is_string:
                            results["fail_syntactic"] += 1
                            failure_type = "Syntactic (Malformed JSON/Extra Keys)"
                        else:
                            results["fail_semantic"] += 1
                            failure_type = "Semantic (Valid JSON, but failed logic/schema)"
                            
                        results["failures"].append({
                            "file": f,
                            "type": failure_type,
                            "plan_preview": str(vlm_plan)[:400] + "..." if vlm_plan else "None"
                        })
                except Exception as e:
                    print(f"Error parsing {path}: {e}")

print("=== VLM LOGS ANALYSIS ===")
print(f"Total Logs: {results['total']}")
print(f"Successful Plans: {results['success']}")
print(f"Failed Plans: {results['fail_syntactic'] + results['fail_semantic']}")
print(f"  - Syntactic Errors (String output): {results['fail_syntactic']}")
print(f"  - Semantic Errors (Dict output): {results['fail_semantic']}")
print("\n--- FAILURE DETAILS ---")
for fail in results["failures"]:
    print(f"\nFile: {fail['file']}")
    print(f"Type: {fail['type']}")
    print(f"Preview: {fail['plan_preview']}")
