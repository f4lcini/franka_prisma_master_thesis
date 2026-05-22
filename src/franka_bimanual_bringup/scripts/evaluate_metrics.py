#!/usr/import/env python3
import os
import json
import numpy as np

def compute_overlap(left_intervals, right_intervals):
    # Overlap is the intersection of all left_intervals with all right_intervals
    overlap_time = 0.0
    for l_start, l_end in left_intervals:
        for r_start, r_end in right_intervals:
            o_start = max(l_start, r_start)
            o_end = min(l_end, r_end)
            if o_end > o_start:
                overlap_time += (o_end - o_start)
    return overlap_time

def compute_union(intervals):
    if not intervals:
        return 0.0
    # Sort and merge intervals
    intervals.sort(key=lambda x: x[0])
    merged = [intervals[0]]
    for current in intervals[1:]:
        prev = merged[-1]
        if current[0] <= prev[1]:
            merged[-1] = (prev[0], max(prev[1], current[1]))
        else:
            merged.append(current)
    return sum([end - start for start, end in merged])

def print_metrics(title, metrics, counts, fails, detail, total_logs):
    print(f"\n==================================================")
    print(f"{title} ({total_logs} logs)")
    print(f"==================================================")
    
    for key in ['RSR', 'PSR_Mission', 'PSR_Action', 'FSR_Mission', 'FSR_Action', 'PEO_Percent', 'PEO_Seconds', 'Mission_Time']:
        values = metrics.get(key, [])
        if values:
            if key in ['PEO_Seconds', 'Mission_Time']:
                mean_val = np.mean(values)
                std_val = np.std(values)
                print(f"{key:12s}: {mean_val:6.2f}s ± {std_val:5.2f}s")
            elif key == 'PEO_Percent':
                mean_val = np.mean(values) * 100.0
                std_val = np.std(values) * 100.0
                print(f"{key:12s}: {mean_val:6.2f}% ± {std_val:5.2f}%")
            else:
                mean_val = np.mean(values) * 100.0
                if key == 'RSR':
                    tot, suc = counts['RSR_total'], counts['RSR_succ']
                elif key == 'PSR_Mission':
                    tot, suc = counts['PSR_M_total'], counts['PSR_M_succ']
                elif key == 'PSR_Action':
                    tot, suc = counts['PSR_A_total'], counts['PSR_A_succ']
                elif key == 'FSR_Mission':
                    tot, suc = counts['FSR_M_total'], counts['FSR_M_succ']
                elif key == 'FSR_Action':
                    tot, suc = counts['FSR_A_total'], counts['FSR_A_succ']
                else:
                    tot, suc = 0, 0
                print(f"{key:12s}: {mean_val:6.2f}%  ({suc}/{tot} success)")
        else:
            print(f"{key:12s}: N/A")

    # --- Action-Type Breakdown (PICK / PLACE / MOVE_HOME) ---
    print("\n--- Action-Type Breakdown ---")
    for atype in ['PICK', 'PLACE', 'MOVE_HOME', 'HANDOVER']:
        att = detail['action_attempts'].get(atype, 0)
        suc = detail['action_successes'].get(atype, 0)
        durs = detail['action_durations'].get(atype, [])
        if att > 0:
            pct = 100.0 * suc / att
            avg_dur = np.mean(durs) if durs else 0.0
            print(f"  {atype:<12s}: {pct:5.1f}%  ({suc}/{att})   avg duration: {avg_dur:.2f}s")

    # --- Per-Arm Breakdown ---
    print("\n--- Per-Arm Breakdown ---")
    for arm in ['left_arm', 'right_arm']:
        att = detail['arm_attempts'].get(arm, 0)
        suc = detail['arm_successes'].get(arm, 0)
        if att > 0:
            pct = 100.0 * suc / att
            label = 'Left ' if arm == 'left_arm' else 'Right'
            print(f"  {label} Arm         : {pct:5.1f}%  ({suc}/{att} actions succeeded)")

    # --- Failure Breakdown ---
    print("\n--- Failure Breakdown (Causa → Effetto) ---")
    tot_action_fails = fails['perc'] + fails['grasp'] + fails['exec']
    recovered = tot_action_fails - fails['bt']
    recovery_rate = 100.0 * recovered / tot_action_fails if tot_action_fails > 0 else 100.0
    print(f"Total Action Fails     : {tot_action_fails}")
    print(f"  ├─ Perception Fails  : {fails['perc']} Vision/Camera, {fails['grasp']} Grasp/Slip")
    print(f"  └─ Execution Fails   : {fails['exec']} MoveIt IK/Hardware aborts")
    print(f"BT Recovery Rate       : {recovered}/{tot_action_fails} action fails recovered by Fallback ({recovery_rate:.1f}%)")
    print(f"Mission Aborts (RSR)   : {fails['bt']} unrecoverable cascades → mission abort")
    print(f"VLM Logic Errors       : {fails['vlm']} (pure planning logic errors)")


def main():
    base_path = os.path.join(os.path.dirname(__file__), 'automate_scenarios', 'experiment_logs')
    
    if not os.path.exists(base_path):
        print(f"Directory {base_path} not found.")
        return

    exp_dirs = []
    for root, dirs, files in os.walk(base_path):
        if any(f.endswith('.json') for f in files):
            exp_dirs.append(root)
    
    if not exp_dirs:
        print("No log files found in any subdirectories.")
        return

    global_metrics = {k: [] for k in ['RSR', 'PSR_Mission', 'PSR_Action', 'FSR_Mission', 'FSR_Action', 'PEO_Percent', 'PEO_Seconds', 'Mission_Time']}
    global_counts = {k: 0 for k in ['RSR_total', 'RSR_succ', 'PSR_M_total', 'PSR_M_succ', 'PSR_A_total', 'PSR_A_succ', 'FSR_M_total', 'FSR_M_succ', 'FSR_A_total', 'FSR_A_succ']}
    global_fails = {'vlm': 0, 'bt': 0, 'perc': 0, 'grasp': 0, 'exec': 0}
    global_detail = {
        'action_attempts':  {},
        'action_successes': {},
        'action_durations': {},
        'arm_attempts':     {},
        'arm_successes':    {},
    }
    
    for exp_path in exp_dirs:
        files = [f for f in os.listdir(exp_path) if f.endswith('.json')]
        if not files:
            continue
            
        metrics = {k: [] for k in global_metrics.keys()}
        counts = {k: 0 for k in global_counts.keys()}
        fails = {'vlm': 0, 'bt': 0, 'perc': 0, 'grasp': 0, 'exec': 0}
        detail = {
            'action_attempts':  {},
            'action_successes': {},
            'action_durations': {},
            'arm_attempts':     {},
            'arm_successes':    {},
        }
        
        for f in files:
            filepath = os.path.join(exp_path, f)
            try:
                with open(filepath, 'r') as fp:
                    data = json.load(fp)
                    
                # RSR
                rsr_vlm = data.get('rsr_vlm_success', False)
                rsr_bt = data.get('rsr_bt_success', False)
                rsr_success = int(rsr_vlm and rsr_bt)
                
                if not rsr_vlm: fails['vlm'] += 1; global_fails['vlm'] += 1
                elif not rsr_bt: fails['bt'] += 1; global_fails['bt'] += 1
                
                metrics['RSR'].append(rsr_success)
                global_metrics['RSR'].append(rsr_success)
                counts['RSR_total'] += 1; global_counts['RSR_total'] += 1
                if rsr_success: counts['RSR_succ'] += 1; global_counts['RSR_succ'] += 1

                # PSR
                p_att = data.get('psr_perception_attempts', 0)
                p_succ = data.get('psr_perception_successes', 0)
                g_att = data.get('psr_grasp_attempts', 0)
                g_succ = data.get('psr_grasp_successes', 0)
                
                fails['perc'] += (p_att - p_succ); global_fails['perc'] += (p_att - p_succ)
                fails['grasp'] += (g_att - g_succ); global_fails['grasp'] += (g_att - g_succ)
                
                tot_p = p_att + g_att
                suc_p = p_succ + g_succ
                
                counts['PSR_A_total'] += tot_p; global_counts['PSR_A_total'] += tot_p
                counts['PSR_A_succ'] += suc_p; global_counts['PSR_A_succ'] += suc_p
                
                if tot_p > 0:
                    metrics['PSR_Action'].append(suc_p / tot_p)
                    global_metrics['PSR_Action'].append(suc_p / tot_p)
                    m_val = 1 if suc_p == tot_p else 0
                    metrics['PSR_Mission'].append(m_val)
                    global_metrics['PSR_Mission'].append(m_val)
                    counts['PSR_M_total'] += 1; global_counts['PSR_M_total'] += 1
                    if m_val: counts['PSR_M_succ'] += 1; global_counts['PSR_M_succ'] += 1
                else:
                    metrics['PSR_Action'].append(1.0); global_metrics['PSR_Action'].append(1.0)
                    metrics['PSR_Mission'].append(1); global_metrics['PSR_Mission'].append(1)
                    counts['PSR_M_total'] += 1; global_counts['PSR_M_total'] += 1
                    counts['PSR_M_succ'] += 1; global_counts['PSR_M_succ'] += 1

                # FSR
                f_att = data.get('fsr_execution_attempts', 0)
                f_succ = data.get('fsr_execution_successes', 0)
                fails['exec'] += (f_att - f_succ); global_fails['exec'] += (f_att - f_succ)
                
                counts['FSR_A_total'] += f_att; global_counts['FSR_A_total'] += f_att
                counts['FSR_A_succ'] += f_succ; global_counts['FSR_A_succ'] += f_succ
                
                if f_att > 0:
                    metrics['FSR_Action'].append(f_succ / f_att)
                    global_metrics['FSR_Action'].append(f_succ / f_att)
                    m_val = 1 if f_succ == f_att else 0
                    metrics['FSR_Mission'].append(m_val)
                    global_metrics['FSR_Mission'].append(m_val)
                    counts['FSR_M_total'] += 1; global_counts['FSR_M_total'] += 1
                    if m_val: counts['FSR_M_succ'] += 1; global_counts['FSR_M_succ'] += 1
                else:
                    metrics['FSR_Action'].append(1.0); global_metrics['FSR_Action'].append(1.0)
                    metrics['FSR_Mission'].append(1); global_metrics['FSR_Mission'].append(1)
                    counts['FSR_M_total'] += 1; global_counts['FSR_M_total'] += 1
                    counts['FSR_M_succ'] += 1; global_counts['FSR_M_succ'] += 1
                    
                # PEO & Timing
                if rsr_success:
                    left_intervals = []
                    right_intervals = []
                    
                    for action in data.get('arm_actions', {}).get('left_arm', []):
                        left_intervals.append((action['start'], action['end']))
                    for action in data.get('arm_actions', {}).get('right_arm', []):
                        right_intervals.append((action['start'], action['end']))
                        
                    t_left = compute_union(left_intervals)
                    t_right = compute_union(right_intervals)
                    t_overlap = compute_overlap(left_intervals, right_intervals)
                    
                    t_seq = t_left + t_right
                    peo = (t_overlap / t_seq) if t_seq > 0 else 0.0
                    
                    start_ts = data.get('start_timestamp', 0)
                    end_ts = data.get('end_timestamp', 0)
                    mission_time = (end_ts - start_ts) if (end_ts > start_ts) else 0.0
                    
                    metrics['PEO_Percent'].append(peo); global_metrics['PEO_Percent'].append(peo)
                    metrics['PEO_Seconds'].append(t_overlap); global_metrics['PEO_Seconds'].append(t_overlap)
                    metrics['Mission_Time'].append(mission_time); global_metrics['Mission_Time'].append(mission_time)

                # Arm-level and action-type breakdown
                for arm_key, arm_actions in data.get('arm_actions', {}).items():
                    for act in arm_actions:
                        atype = act.get('action', 'UNKNOWN')
                        success = act.get('success', False)
                        duration = act.get('end', 0) - act.get('start', 0)
                        # per action type
                        detail['action_attempts'][atype]  = detail['action_attempts'].get(atype, 0) + 1
                        detail['action_successes'][atype] = detail['action_successes'].get(atype, 0) + (1 if success else 0)
                        detail['action_durations'].setdefault(atype, []).append(duration)
                        # per arm
                        detail['arm_attempts'][arm_key]  = detail['arm_attempts'].get(arm_key, 0) + 1
                        detail['arm_successes'][arm_key] = detail['arm_successes'].get(arm_key, 0) + (1 if success else 0)
                        # global detail
                        global_detail['action_attempts'][atype]  = global_detail['action_attempts'].get(atype, 0) + 1
                        global_detail['action_successes'][atype] = global_detail['action_successes'].get(atype, 0) + (1 if success else 0)
                        global_detail['action_durations'].setdefault(atype, []).append(duration)
                        global_detail['arm_attempts'][arm_key]  = global_detail['arm_attempts'].get(arm_key, 0) + 1
                        global_detail['arm_successes'][arm_key] = global_detail['arm_successes'].get(arm_key, 0) + (1 if success else 0)

            except Exception as e:
                print(f"Error reading {f}: {e}")

        exp_name = os.path.relpath(exp_path, base_path)
        print_metrics(f"Experiment: {exp_name}", metrics, counts, fails, detail, len(files))

    if len(exp_dirs) > 1 or (len(exp_dirs) == 1 and exp_dirs[0] != base_path):
        total_logs = len(global_metrics['RSR'])
        if total_logs > 0:
            print_metrics("GLOBAL AGGREGATE RESULTS", global_metrics, global_counts, global_fails, global_detail, total_logs)

if __name__ == '__main__':
    main()
