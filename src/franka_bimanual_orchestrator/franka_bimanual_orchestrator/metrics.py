import json
import os
import time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class MetricsLogger:
    def __init__(self, experiment_name="experiment"):
        self.experiment_name = experiment_name.replace(" ", "_")
        self.start_time = time.time()
        self.end_time = None
        
        self.metrics = {
            'mission_name': self.experiment_name,
            'start_timestamp': self.start_time,
            'end_timestamp': 0.0,
            'rsr_vlm_success': False,
            'rsr_bt_success': False,
            'vlm_input_prompt': None,
            'vlm_output_plan': None,
            'psr_perception_attempts': 0,
            'psr_perception_successes': 0,
            'psr_grasp_attempts': 0,
            'psr_grasp_successes': 0,
            'fsr_execution_attempts': 0,
            'fsr_execution_successes': 0,
            'scene_inventory': [],
            'arm_actions': {
                'left_arm': [],
                'right_arm': []
            }
        }

    def mark_vlm_success(self, success):
        self.metrics['rsr_vlm_success'] = success

    def mark_bt_success(self, success):
        self.metrics['rsr_bt_success'] = success

    def log_vlm_plan(self, plan_str):
        self.metrics['vlm_output_plan'] = plan_str

    def log_vlm_prompt(self, prompt_str):
        self.metrics['vlm_input_prompt'] = prompt_str

    def log_scene_inventory(self, scene_inventory):
        self.metrics['scene_inventory'] = scene_inventory

    def log_perception(self, success):
        self.metrics['psr_perception_attempts'] += 1
        if success:
            self.metrics['psr_perception_successes'] += 1

    def log_grasp(self, success):
        self.metrics['psr_grasp_attempts'] += 1
        if success:
            self.metrics['psr_grasp_successes'] += 1

    def log_execution(self, success):
        self.metrics['fsr_execution_attempts'] += 1
        if success:
            self.metrics['fsr_execution_successes'] += 1

    def log_action(self, arm_name, action_name, start_t, end_t, success):
        if arm_name not in self.metrics['arm_actions']:
            self.metrics['arm_actions'][arm_name] = []
        
        self.metrics['arm_actions'][arm_name].append({
            'action': action_name,
            'start': start_t,
            'end': end_t,
            'success': success
        })

    def save_log(self, base_path):
        self.end_time = time.time()
        self.metrics['end_timestamp'] = self.end_time
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.experiment_name}_{timestamp_str}.json"
        
        # Ensure directory exists
        exp_dir = os.path.join(base_path, self.experiment_name)
        os.makedirs(exp_dir, exist_ok=True)
        
        filepath = os.path.join(exp_dir, filename)
        with open(filepath, 'w') as f:
            json.dump(self.metrics, f, indent=4)
        
        return filepath
