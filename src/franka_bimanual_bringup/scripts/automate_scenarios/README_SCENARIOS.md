# Parallel Architecture Verification Walkthrough

Questo walkthrough spiega come utilizzare i diversi script di test per verificare che l'architettura bimanuale gestisca correttamente l'esecuzione in parallelo.

## 1. Prerequisiti
Avvia la simulazione (Terminale 1) e i backend necessari (Terminale 2):

```bash
# Terminale 1: Simulazione + MoveIt
ros2 launch franka_manipulation_env demo_moveit_bimanual.launch.py

# Terminale 2: Backend (Planner C++ + Skill Servers Python)
ros2 launch franka_bimanual_bringup parallel_test_backends.launch.py
```

## 2. Scenari di Test

### Scenario A: Sync Home (Standard)
Testa entrambi i bracci che tornano a casa simultaneamente usando il sistema di piani standard.
- **Comando**:
```bash
ros2 run franka_bimanual_bringup scenario_synch_home.py
```

### Scenario B: Parallel Waves (Indipendenza)
Dimostra che i bracci sono indipendenti: uno aspetta mentre l'altro si muove.
- **Comando**:
```bash
ros2 run franka_bimanual_bringup scenario_parallel_waves.py
```

### Scenario C: ATOMIC Sync Home (Albero Minimale)
Crea il Behavior Tree più semplice possibile (solo 3 nodi: Radice + 2 bracci). **Usa questo se vuoi vedere un albero ultra-pulito nel viewer.**
- **Comando**:
```bash
ros2 run franka_bimanual_bringup atom_synch_home.py
```

## 3. Visualizzare l'Albero
Tutti gli script supportano il `py-trees-tree-viewer`. Per vedere l'esecuzione live:
1. Avvia uno degli script sopra.
2. In un altro terminale, esegui:
```bash
ros2 run py_trees_ros_viewer py_trees_ros_viewer
```
Vedrai i nodi cambiare colore (Verde per SUCCESS, Blu per RUNNING).

## 4. Ricapitolazione Semplificazione
- **`scenario_engine.py`**: Versione modulare dell'orchestratore per caricare piani hardcoded.
- **`atom_synch_home.py`**: Test "atomico" che riduce al minimo la complessità del Behavior Tree per una comprensione immediata.
