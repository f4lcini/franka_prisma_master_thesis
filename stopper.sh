#!/bin/bash

SESSION="bimanual"

echo "🛑 Stopping Bimanual System..."

# 1. Kill the tmux session
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "Closing tmux session: $SESSION"
    tmux kill-session -t $SESSION
fi

# 2. Kill lingering ROS 2 and Gazebo processes
echo "Cleaning up ROS 2 nodes and Gazebo processes..."
pkill -9 -f ros2
pkill -9 -f gz
pkill -9 -f ign
pkill -9 -f ruby
pkill -9 -f rviz2
pkill -9 -f robot_state_publisher
pkill -9 -f controller_manager

# 3. Final wait
sleep 2
echo "✅ All clear. You can now rebuild or restart."
