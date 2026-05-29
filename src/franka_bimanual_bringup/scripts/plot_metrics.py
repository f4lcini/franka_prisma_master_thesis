#!/usr/bin/env python3
import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# --- Academic Plot Configuration ---
plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm", 
    "font.size": 11,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linestyle": "--",
    "axes.linewidth": 0.8,
    "lines.linewidth": 1.5,
})

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = "/home/hargalaten/vfalcini_demos/Thesis/images/plots"
os.makedirs(OUTPUT_DIR, exist_ok=True)

COLORS = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9"]

def get_acronym(path):
    p = path.lower()
    if "vlm_not_integrated" in p:
        if "exp1" in p: return "B-EXP1"
        if "exp2" in p: return "B-EXP2"
        if "exp3" in p: return "B-EXP3"
        return "B-OTHER"
    if "vlm_integrated" in p:
        if "sort" in p: return "V-EXP1"
        if "transfer" in p or "handover" in p: return "V-EXP2"
        if "clear" in p: return "V-EXP3"
        return "V-OTHER"
    return "UNKNOWN"

def get_data(base_path):
    experiments = {}
    if not os.path.exists(base_path):
        return experiments
    for root, dirs, files in os.walk(base_path):
        json_files = [f for f in files if f.endswith('.json')]
        if json_files:
            exp_name = get_acronym(os.path.relpath(root, base_path))
            if exp_name not in experiments:
                experiments[exp_name] = []
            for f in json_files:
                try:
                    with open(os.path.join(root, f), 'r') as fp:
                        data = json.load(fp)
                        data['_filename'] = f
                        experiments[exp_name].append(data)
                except:
                    pass
                    
    # Sort dictionary keys alphabetically so B- comes before V-
    return dict(sorted(experiments.items()))

def compute_union(intervals):
    if not intervals: return 0.0
    intervals.sort(key=lambda x: x[0])
    merged = [intervals[0]]
    for current in intervals[1:]:
        prev = merged[-1]
        if current[0] <= prev[1]:
            merged[-1] = (prev[0], max(prev[1], current[1]))
        else:
            merged.append(current)
    return sum([end - start for start, end in merged])

def compute_overlap(left_intervals, right_intervals):
    overlap_time = 0.0
    for l_start, l_end in left_intervals:
        for r_start, r_end in right_intervals:
            o_start = max(l_start, r_start)
            o_end = min(l_end, r_end)
            if o_end > o_start:
                overlap_time += (o_end - o_start)
    return overlap_time

def inject_sync_barriers(log_data):
    # If there's a VLM plan, we can infer the exact duration of the SYNC_BARRIER
    # by looking at the gap between the preceding and succeeding actions.
    plan = log_data.get('vlm_output_plan')
    if not plan: return log_data
    
    for arm in ['left_arm', 'right_arm']:
        seq_key = f"{arm}_sequence"
        if seq_key not in plan: continue
        planned_seq = plan[seq_key]
        
        executed = log_data.get('arm_actions', {}).get(arm, [])
        if not executed: continue
        
        new_executed = []
        exec_idx = 0
        
        for p_action in planned_seq:
            act_type = p_action.get('action')
            if act_type == 'FIND_OBJECT':
                continue
                
            if act_type == 'SYNC_BARRIER':
                if exec_idx > 0 and exec_idx < len(executed):
                    prev_act = executed[exec_idx - 1]
                    next_act = executed[exec_idx]
                    gap_start = prev_act['end']
                    gap_end = next_act['start']
                    if gap_end > gap_start:
                        new_executed.append({
                            'action': 'SYNC_BARRIER',
                            'start': gap_start,
                            'end': gap_end,
                            'success': True
                        })
            else:
                if exec_idx < len(executed) and executed[exec_idx]['action'] == act_type:
                    new_executed.append(executed[exec_idx])
                    exec_idx += 1
                    
        # Add any remaining executed actions just in case
        while exec_idx < len(executed):
            new_executed.append(executed[exec_idx])
            exec_idx += 1
            
        log_data['arm_actions'][arm] = new_executed
    return log_data

def plot_gantt(log_data, exp_name):
    # Inject sync barriers into the log data dynamically
    
    fig, ax = plt.subplots(figsize=(6, 3))
    
    left_actions = log_data.get('arm_actions', {}).get('left_arm', [])
    right_actions = log_data.get('arm_actions', {}).get('right_arm', [])
    
    start_ts = log_data.get('start_timestamp', 0)
    if start_ts == 0:
        if left_actions: start_ts = left_actions[0]['start']
        elif right_actions: start_ts = right_actions[0]['start']
    
    colors = {"PICK": "#0072B2", "PLACE": "#D55E00", "MOVE_HOME": "#009E73", "SYNC_BARRIER": "#CC79A7", "UNKNOWN": "gray"}
    
    l_intervals = []
    r_intervals = []
    
    def plot_arm_actions(actions, y_pos, label):
        for act in actions:
            act_type = act.get('action', 'UNKNOWN')
            s = act['start'] - start_ts
            e = act['end'] - start_ts
            ax.barh(y_pos, e - s, left=s, height=0.4, color=colors.get(act_type, "gray"), edgecolor="black", alpha=0.8)
            if y_pos == 1: l_intervals.append((s, e))
            else: r_intervals.append((s, e))
            
    plot_arm_actions(left_actions, 1, "Left Arm")
    plot_arm_actions(right_actions, 0, "Right Arm")
    
    for l_s, l_e in l_intervals:
        for r_s, r_e in r_intervals:
            o_s, o_e = max(l_s, r_s), min(l_e, r_e)
            if o_e > o_s:
                ax.axvspan(o_s, o_e, color="gray", alpha=0.2, zorder=0)

    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Right Arm", "Left Arm"])
    ax.set_xlabel("Time (s)")
    ax.set_title(f"Bimanual Timeline ({exp_name})")
    
    handles = [mpatches.Patch(color=c, label=k) for k, c in colors.items() if k in ["PICK", "PLACE", "MOVE_HOME", "SYNC_BARRIER"]]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.3), ncol=4)
    
    plt.tight_layout()
    # Save with exp_name in the filename so they don't overwrite each other
    safe_name = exp_name.replace(" ", "_")
    plt.savefig(os.path.join(OUTPUT_DIR, f"gantt_chart_{safe_name}.pdf"), format='pdf', bbox_inches='tight')
    plt.close()

def export_latex_table(experiments_dict):
    tex_path = os.path.join(OUTPUT_DIR, "metrics_table.tex")
    
    with open(tex_path, 'w') as f:
        f.write("\\begin{table}[h!]\n")
        f.write("\\centering\n")
        f.write("\\caption{Aggregate Performance Metrics across Experimental Scenarios}\n")
        f.write("\\label{tab:performance_metrics}\n")
        f.write("\\begin{tabular}{l c c c c c}\n")
        f.write("\\hline\n")
        f.write("\\textbf{Scenario} & \\textbf{RSR (\\%)} & \\textbf{PSR (\\%)} & \\textbf{FSR (\\%)} & \\textbf{PEO (\\%)} & \\textbf{Time (s)} \\\\\n")
        f.write("\\hline\n")
        
        for exp_name, logs in experiments_dict.items():
            rsr_s, rsr_t = 0, 0
            psr_a_s, psr_a_t = 0, 0
            fsr_a_s, fsr_a_t = 0, 0
            
            m_times = []
            p_percents = []
            
            for d in logs:
                rsr_vlm = d.get('rsr_vlm_success', False)
                rsr_bt = d.get('rsr_bt_success', False)
                rsr_t += 1; rsr_s += int(rsr_vlm and rsr_bt)
                
                p_att = d.get('psr_perception_attempts', 0) + d.get('psr_grasp_attempts', 0)
                p_succ = d.get('psr_perception_successes', 0) + d.get('psr_grasp_successes', 0)
                psr_a_t += p_att; psr_a_s += p_succ
                
                f_att = d.get('fsr_execution_attempts', 0)
                f_succ = d.get('fsr_execution_successes', 0)
                fsr_a_t += f_att; fsr_a_s += f_succ
                
                if rsr_vlm and rsr_bt:
                    left_intervals = [(a['start'], a['end']) for a in d.get('arm_actions', {}).get('left_arm', [])]
                    right_intervals = [(a['start'], a['end']) for a in d.get('arm_actions', {}).get('right_arm', [])]
                    t_seq = compute_union(left_intervals) + compute_union(right_intervals)
                    peo = (compute_overlap(left_intervals, right_intervals) / t_seq * 100.0) if t_seq > 0 else 0.0
                    
                    start_ts = d.get('start_timestamp', 0)
                    end_ts = d.get('end_timestamp', 0)
                    mission_time = (end_ts - start_ts) if (end_ts > start_ts) else 0.0
                    
                    if mission_time > 0: m_times.append(mission_time)
                    p_percents.append(peo)
            
            rsr_rate = (rsr_s/rsr_t * 100) if rsr_t else 0
            psr_rate = (psr_a_s/psr_a_t * 100) if psr_a_t else 0
            fsr_rate = (fsr_a_s/fsr_a_t * 100) if fsr_a_t else 0
            
            peo_mean = np.mean(p_percents) if p_percents else 0
            peo_std = np.std(p_percents) if p_percents else 0
            
            t_mean = np.mean(m_times) if m_times else 0
            t_std = np.std(m_times) if m_times else 0
            
            f.write(f"\\textbf{{{exp_name}}} & {rsr_rate:.1f} & {psr_rate:.1f} & {fsr_rate:.1f} & ${peo_mean:.1f} \\pm {peo_std:.1f}$ & ${t_mean:.1f} \\pm {t_std:.1f}$ \\\\\n")
            
        f.write("\\hline\n")
        f.write("\\end{tabular}\n")
        f.write("\\end{table}\n")

def plot_action_durations(experiments_dict):
    actions = ["PICK", "PLACE", "MOVE_HOME"]
    x = np.arange(len(actions))
    
    fig, ax = plt.subplots(figsize=(6, 3.5))
    
    num_exps = len(experiments_dict)
    width = 0.8 / num_exps if num_exps > 0 else 0.8
    offsets = np.linspace(-0.4 + width/2, 0.4 - width/2, num_exps) if num_exps > 1 else [0]
    
    handles = []
    
    for idx, (exp_name, logs) in enumerate(experiments_dict.items()):
        data = {a: [] for a in actions}
        for d in logs:
            for arm, acts in d.get('arm_actions', {}).items():
                for act in acts:
                    atype = act.get('action', 'UNKNOWN')
                    if atype in actions:
                        dur = act.get('end', 0) - act.get('start', 0)
                        if dur > 0: data[atype].append(dur)
        
        pos = x + offsets[idx]
        
        for i, a in enumerate(actions):
            if not data[a]: continue
            parts = ax.violinplot(data[a], positions=[pos[i]], showmeans=True, widths=width*0.9)
            for pc in parts['bodies']:
                pc.set_facecolor(COLORS[idx%len(COLORS)])
                pc.set_edgecolor('black')
                pc.set_alpha(0.7)
            parts['cmeans'].set_color('black')
            parts['cmins'].set_color('black')
            parts['cmaxes'].set_color('black')
            parts['cbars'].set_color('black')
            
        handles.append(mpatches.Patch(color=COLORS[idx%len(COLORS)], label=exp_name))

    ax.set_xticks(x)
    ax.set_xticklabels(actions)
    ax.set_ylabel("Execution Time (s)")
    ax.set_title("Action Durations Distribution")
    if num_exps > 1:
        ax.legend(handles=handles, loc="upper right")
        
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "action_durations_violin.pdf"), format='pdf', bbox_inches='tight')
    plt.close()

def plot_detailed_counts(experiments_dict):
    # Prepare data including GLOBAL
    exp_keys = list(experiments_dict.keys()) + ["GLOBAL"]
    all_logs = []
    for logs in experiments_dict.values():
        all_logs.extend(logs)
        
    data_to_plot = {}
    for key in exp_keys:
        logs = all_logs if key == "GLOBAL" else experiments_dict[key]
        
        vlm_att, vlm_succ = 0, 0
        perc_att, perc_succ = 0, 0
        grasp_att, grasp_succ = 0, 0
        mtc_att, mtc_succ = 0, 0
        
        for d in logs:
            vlm_att += 1
            if d.get('rsr_vlm_success', False): vlm_succ += 1
            
            perc_att += d.get('psr_perception_attempts', 0)
            perc_succ += d.get('psr_perception_successes', 0)
            
            grasp_att += d.get('psr_grasp_attempts', 0)
            grasp_succ += d.get('psr_grasp_successes', 0)
            
            mtc_att += d.get('fsr_execution_attempts', 0)
            mtc_succ += d.get('fsr_execution_successes', 0)
            
        data_to_plot[key] = {
            "Plan": (vlm_att, vlm_succ),
            "Vision": (perc_att, perc_succ),
            "Grasp": (grasp_att, grasp_succ),
            "Action": (mtc_att, mtc_succ)
        }
        
    num_plots = len(exp_keys)
    cols = min(3, num_plots)
    rows = int(np.ceil(num_plots / cols))
    
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3.5 * rows), squeeze=False)
    fig.suptitle("Detailed Breakdown: Attempts vs Successes", fontsize=14, fontweight='bold', y=1.02)
    
    categories = ["Plan", "Vision", "Grasp", "Action"]
    x = np.arange(len(categories))
    width = 0.35
    
    for idx, key in enumerate(exp_keys):
        r = idx // cols
        c = idx % cols
        ax = axes[r, c]
        
        vals = data_to_plot[key]
        attempts = [vals[cat][0] for cat in categories]
        successes = [vals[cat][1] for cat in categories]
        
        rects1 = ax.bar(x - width/2, attempts, width, label='Attempts', color='lightgray', edgecolor='black')
        rects2 = ax.bar(x + width/2, successes, width, label='Successes', color='#0072B2', edgecolor='black')
        
        ax.set_title(key, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(categories)
        if c == 0: ax.set_ylabel("Count")
        
        # Add integer labels on top
        for rect in rects1:
            h = rect.get_height()
            if h > 0: ax.annotate(f'{int(h)}', xy=(rect.get_x() + rect.get_width()/2, h), xytext=(0,2), textcoords="offset points", ha='center', va='bottom', fontsize=9)
        for rect in rects2:
            h = rect.get_height()
            if h > 0: ax.annotate(f'{int(h)}', xy=(rect.get_x() + rect.get_width()/2, h), xytext=(0,2), textcoords="offset points", ha='center', va='bottom', fontsize=9)
            
        # Error text inside plot
        max_y = max(max(attempts), 1)
        ax.set_ylim(0, max_y * 1.2)
        
        if idx == 0:
            ax.legend(loc='upper right')

    # Hide unused subplots
    for idx in range(num_plots, rows * cols):
        r = idx // cols
        c = idx % cols
        fig.delaxes(axes[r, c])
        
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "detailed_counts_bar.pdf"), format='pdf', bbox_inches='tight')
    plt.close()

def plot_vlm_vs_baseline_success(experiments_dict):
    baseline_att, baseline_succ = 0, 0
    vlm_att, vlm_succ = 0, 0
    
    for exp_name, logs in experiments_dict.items():
        is_vlm = exp_name.startswith('V-EXP')
        
        for d in logs:
            succ = d.get('rsr_vlm_success', False) and d.get('rsr_bt_success', False)
            if is_vlm:
                vlm_att += 1
                if succ: vlm_succ += 1
            else:
                baseline_att += 1
                if succ: baseline_succ += 1
                
    categories = ["Baseline (Without VLM)", "Integrated (With VLM)"]
    attempts = [baseline_att, vlm_att]
    successes = [baseline_succ, vlm_succ]
    
    x = np.arange(len(categories))
    width = 0.35
    
    fig, ax = plt.subplots(figsize=(5, 4))
    
    rects1 = ax.bar(x - width/2, attempts, width, label='Total Missions', color='lightgray', edgecolor='black')
    rects2 = ax.bar(x + width/2, successes, width, label='Successful Missions', color='#009E73', edgecolor='black')
    
    ax.set_ylabel('Number of Missions')
    ax.set_title('Mission Success: Baseline vs VLM Integrated', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontweight='bold')
    ax.legend(loc='upper right')
    
    for rect in rects1:
        h = rect.get_height()
        if h > 0: ax.annotate(f'{int(h)}', xy=(rect.get_x() + rect.get_width()/2, h), xytext=(0,3), textcoords="offset points", ha='center', va='bottom', fontsize=10)
    for rect in rects2:
        h = rect.get_height()
        if h > 0: ax.annotate(f'{int(h)}', xy=(rect.get_x() + rect.get_width()/2, h), xytext=(0,3), textcoords="offset points", ha='center', va='bottom', fontsize=10, fontweight='bold')
            
    if baseline_att > 0: ax.annotate(f'{baseline_succ/baseline_att*100:.1f}%', xy=(x[0] + width/2, max(5, baseline_succ/2)), ha='center', va='center', color='white', fontweight='bold')
    if vlm_att > 0: ax.annotate(f'{vlm_succ/vlm_att*100:.1f}%', xy=(x[1] + width/2, max(5, vlm_succ/2)), ha='center', va='center', color='white', fontweight='bold')
                    
    ax.set_ylim(0, max(attempts) * 1.2)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "vlm_vs_baseline_success.pdf"), format='pdf', bbox_inches='tight')
    plt.close()

if __name__ == "__main__":
    logs_dir = os.path.join(SCRIPT_DIR, "automate_scenarios", "experiment_logs")
    experiments = get_data(logs_dir)
    
    if not experiments:
        print("No logs found!")
    else:
        # Clear out old plots to avoid confusion
        for f in os.listdir(OUTPUT_DIR):
            if f.endswith('.pdf') or f.endswith('.tex'):
                os.remove(os.path.join(OUTPUT_DIR, f))
                
        # Generate new concise outputs
        export_latex_table(experiments)
        plot_vlm_vs_baseline_success(experiments)
        plot_action_durations(experiments)
        plot_detailed_counts(experiments)
        
        # Generate a Gantt for the best performing FULL successful log of EVERY experiment
        for exp_name, logs in experiments.items():
            successful_logs = [l for l in logs if l.get('rsr_vlm_success') and l.get('rsr_bt_success')]
            if successful_logs:
                best_log = None
                best_metric = (-1, float('inf')) # (num_actions, duration)
                for l in successful_logs:
                    num_acts = len(l.get('arm_actions', {}).get('left_arm', [])) + len(l.get('arm_actions', {}).get('right_arm', []))
                    start_ts = l.get('start_timestamp', 0)
                    end_ts = l.get('end_timestamp', 0)
                    duration = end_ts - start_ts if end_ts > start_ts else float('inf')
                    
                    if num_acts > best_metric[0] or (num_acts == best_metric[0] and duration < best_metric[1]):
                        best_metric = (num_acts, duration)
                        best_log = l
                
                if best_log is None:
                    best_log = successful_logs[0]
                    
                plot_gantt(best_log, exp_name)
                time_str = f"{best_metric[1]:.2f}s" if best_metric[1] != float('inf') else "N/A"
                print(f"Gantt generated from BEST FULL mission in {exp_name} ({best_metric[0]} actions, Time: {time_str}).")
        
        print(f"Data mapping complete. Output saved to {OUTPUT_DIR}")
