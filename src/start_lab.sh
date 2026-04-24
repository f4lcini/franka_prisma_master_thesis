#!/bin/bash
SESSION="bimanual"
WS_DIR="/mm_ws"
LEFT_IP="192.168.9.11"
RIGHT_IP="192.168.9.12"

# Franka Bimanual Lab - Full TMUX Launch Script
# Layout: Hardware (TL), Camera (TR), Planner&Skills (BL), Vision (BR), Reasoning (BR-bottom)

# Clean up previous session
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n "Main" -c "$WS_DIR"

# --- 1. HARDWARE & MOVEIT (Top Left) ---
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION "ros2 launch franka_bimanual_config lab_bringup_final.launch.py left_ip:=$LEFT_IP right_ip:=$RIGHT_IP" C-m

# --- 2. CAMERA (Top Right) ---
tmux split-window -h -t $SESSION -c "$WS_DIR"
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION "sleep 5 && ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true" C-m

# --- 3. PLANNER & SKILLS (Bottom Left) ---
tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION -c "$WS_DIR"
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION "sleep 10 && ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py" C-m

# --- 4. VISION (Bottom Right - Top Half) ---
tmux select-pane -t $SESSION:0.1
tmux split-window -v -t $SESSION -c "$WS_DIR"
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION "sleep 12 && ros2 launch franka_bimanual_skills perception.launch.py use_hardware:=true" C-m

# --- 5. REASONING (Bottom Right - Bottom Half) ---
tmux split-window -v -t $SESSION -c "$WS_DIR"
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION "export GEMINI_API_KEY=IL_TUO_KEY_QUI" C-m
tmux send-keys -t $SESSION "sleep 15 && ros2 launch franka_bimanual_skills reasoning.launch.py" C-m

# Default view and attach
tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION
