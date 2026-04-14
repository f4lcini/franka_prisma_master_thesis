# Panoramica del Framework Bimanuale Franka FR3

Questo documento riassume l’architettura e il flusso operativo del framework sviluppato per la tesi, organizzando i componenti in ordine logico dall'input dell'utente all'esecuzione fisica.

## 1. Percezione (Perception)
La percezione trasforma i dati grezzi dei sensori in informazioni spaziali utilizzabili.

- **Sensore**: RealSense D435 (RGB-D).
- **Rilevamento**: Utilizza **YOLOv8** per identificare oggetti in tempo reale all'interno del frame RGB, estraendo bounding box e label semantiche.
- **Localizzazione 3D**:
    - **Deproiezione Pinhole**: Converte le coordinate 2D (pixel) del centro dell'oggetto in coordinate 3D metriche (X, Y, Z) utilizzando la matrice intrinseca della camera e i dati di profondità allineati.
    - **Stima dell'Orientamento (PCA)**: Esegue un'analisi delle componenti principali (PCA) sulla nuvola di punti locale dell'oggetto per determinarne l'asse principale e calcolare il quaternione di rotazione.
- **Modalità**: Action Server ROS 2 (`/detect_object`), attivato on-demand per risparmiare risorse.

## 2. Ragionamento (Reasoning)
Il "cervello" del sistema che interpreta i comandi e pianifica la strategia.

- **Motore**: **Gemini 2.5 Flash** (Vision-Language Model).
- **Processo**:
    1. Riceve un comando in linguaggio naturale (es. "Passami la bottiglia").
    2. Acquisisce l'ultimo frame dalla camera.
    3. **Chain-of-Thought**: Il modello ragiona seguendo vincoli cinematici iniettati nel prompt (es. divisione dello spazio di lavoro tra braccio destro e sinistro).
- **Output Strutturato**: Produce un JSON che specifica:
    - **Ragionamento critico** (perché ha scelto quell'azione).
    - **Target Label** (l'oggetto da cercare).
    - **Azione primitiva** (`PICK`, `PLACE`, `HANDOVER`, ecc.).
    - **Selezione del braccio** (Sinistro, Destro o Bimanuale).

## 3. Behavior Trees (Orchestrazione)
Il framework utilizza i Behavior Trees (BT) per gestire la logica di alto livello e il flusso delle missioni.

- **Libreria**: `py_trees` con integrazione ROS 2.
- **Architettura del Tree**:
    - **Planning Gate**: Invoca il nodo di Reasoning per generare un piano.
    - **Plan Splitter**: Scompone il piano globale in sequenze di azioni specifiche per il braccio sinistro e quello destro.
    - **Execution Parallel**: Un nodo parallelo che gestisce simultaneamente due "Lanes" (corsie di esecuzione), una per ogni braccio.
- **Vantaggio**: Permette una gestione reattiva degli errori: se un braccio fallisce, l'albero può decidere se riprovare, fermarsi o cambiare strategia.

## 4. Pianificazione del Movimento (Motion Planning)
Traduce le decisioni strategiche in traiettorie sicure prive di collisioni.

- **Software**: **MoveIt2** integrato con **MoveIt Task Constructor (MTC)**.
- **API di Controllo**: Un layer di astrazione (`robot_control_api.py`) semplifica le chiamate a MoveIt.
- **Pianificatori**:
    - **Pilz Industrial**: Utilizzato per movimenti deterministici e lineari (LIN, PTP) critici durante approcci e prese.
    - **OMPL**: Utilizzato per navigazione complessa nello spazio di lavoro per evitare ostacoli.
- **Skill Repertoire**: Include primitive ottimizzate come approach, vertical descent con controllo della pinza, e retreat.

## 5. Meccanismo di Processi in Parallelo
Il sistema è progettato per gestire la complessità bimanuale senza blocchi (asincronia).

- **Ecosistema**: **ROS 2 Humble**.
- **Concorrenza interna**:
    - Utilizzo di `MultiThreadedExecutor` e `ReentrantCallbackGroups` in Python per gestire più Action Server e Telemetria in parallelo nello stesso nodo.
    - **Threading Events**: Utilizzati per il meccanismo di **Rendezvous** (handshake). Durante un handover, i processi che controllano i due bracci si scambiano segnali tramite `threading.Event` per sincronizzare millimetricamente l'incontro delle pinze senza collisioni.
- **Comunicazione**: Gli Action Server permettono un feedback continuo (percentuali di completamento) mentre il processo principale (BT) monitora l'andamento.

---

### Flusso Logico Riassuntivo:
1. **Input**: L'utente dà un comando vocale/testuale.
2. **Reasoning**: Gemini decide l'azione e quale braccio usare.
3. **BT**: L'albero attiva la sequenza parallela.
4. **Perception**: Il braccio scelto cerca l'oggetto e ne ottiene la posa 3D.
5. **Planning**: MoveIt calcola la traiettoria verso quella posa.
6. **Execution**: I motori Franka eseguono il movimento, monitorati dal BT.
