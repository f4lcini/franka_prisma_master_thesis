#!/bin/bash
SESSION="bimanual"

echo "================================================="
echo "🛑 Stopping Franka Bimanual Framework..."
echo "================================================="

# 1. Kill the tmux session
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "Killing tmux session: $SESSION"
    tmux kill-session -t $SESSION
else
    echo "No active tmux session '$SESSION' found."
fi

# 2. Force kill any stray processes
echo "Cleaning up stray processes..."
pkill -9 -f "ros2"
pkill -9 -f "controller_manager"
pkill -9 -f "move_group"
pkill -9 -f "rviz2"
pkill -9 -f "franka"
pkill -9 -f "realsense"

# 3. Clean up ROS 2 daemon
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

echo "✅ All systems stopped and cleaned."
