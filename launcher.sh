#!/bin/bash

# Configuration
SESSION="bimanual"
WS_PATH="/mm_ws"
BASHRC="/mm_ws/local_bashrc"

# TIMERS (Modifica qui per regolare i tempi di attesa)
SIM_WAIT=20      # Secondi di attesa dopo l'avvio della simulazione
BACKEND_WAIT=30   # Secondi di attesa dopo l'avvio dei backend C++

# 1. Check/Install Tmux (Auto-fix for Docker)
if ! command -v tmux &> /dev/null; then
    echo "📦 Tmux not found. Attempting automatic installation..."
    sudo apt update && sudo apt install -y tmux
fi

# 2. Kill previous session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# 3. Create a layout with 3 panes
# Pane 0 (Top): Simulation
tmux new-session -d -s $SESSION -n "Bimanual"
tmux send-keys -t $SESSION:Bimanual "source $BASHRC && ros_source && ros2 launch franka_manipulation_env demo_moveit_bimanual.launch.py use_gazebo:=true" C-m

# Pane 1 (Bottom-Left): Backend
tmux split-window -v -t $SESSION:Bimanual
tmux send-keys -t $SESSION:Bimanual.1 "source $BASHRC && ros_source && echo \"Waiting ${SIM_WAIT}s for Gazebo...\" && sleep ${SIM_WAIT} && ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py" C-m

# Identify target scenario
SCENARIO="atom_bimanual_handshake.py"
if [ "$1" == "optimize" ]; then
    SCENARIO="handover_optimizer.py"
elif [ "$1" == "stress" ]; then
    SCENARIO="atom_heterogeneous_stress.py"
elif [ "$1" == "sync" ]; then
    SCENARIO="atom_synch_home.py"
fi

# Pane 2 (Bottom-Right): Tests
tmux split-window -h -t $SESSION:Bimanual.1
tmux send-keys -t $SESSION:Bimanual.2 "source $BASHRC && ros_source && pip install seaborn && echo \"Waiting ${BACKEND_WAIT}s for Backend...\" && sleep ${BACKEND_WAIT} && ros2 run franka_bimanual_bringup ${SCENARIO}" C-m

# 4. Attach to session
tmux attach-session -t $SESSION
