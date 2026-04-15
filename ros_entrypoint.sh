#!/bin/bash
set -e

# 1. Carica l'ambiente base di ROS 2 Humble
source "/opt/ros/humble/setup.bash"

# 2. Carica il tuo workspace locale (se è già stato compilato almeno una volta)
# Questo permette al container di vedere i tuoi pacchetti fr3_application, ecc.
if [ -f "/mm_ws/install/setup.bash" ]; then
    source "/mm_ws/install/setup.bash"
fi

# 3. Configurazione specifica per Franka FR3 (Networking)
# Spesso l'FR3 richiede CycloneDDS per evitare problemi di latenza
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Esegue il comando passato (di default 'bash')
exec "$@"