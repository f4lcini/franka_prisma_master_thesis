#!/bin/bash

SESSION="bimanual"
BASHRC="/mm_ws/local_bashrc"

echo "♻️ Resetting Simulation Only (Keeping Backend/Tests active)..."

# 1. Kill only the Simulation Pane (Pane 0)
tmux send-keys -t $SESSION:Bimanual.0 C-c
sleep 3

# 2. Relaunch Simulation in the same pane
tmux send-keys -t $SESSION:Bimanual.0 "source $BASHRC && ros_source" C-m
tmux send-keys -t $SESSION:Bimanual.0 "ros2 launch franka_manipulation_env demo_moveit_bimanual.launch.py use_gazebo:=true" C-m

echo "✅ Simulation Relaunched. Backend & Tests should recover after Gazebo initializes."
