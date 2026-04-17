# Guida alla Configurazione Franka Desk per il Workspace Bimanuale

Questo documento fornisce istruzioni precise per la configurazione dell'interfaccia **Franka Desk** necessaria per il corretto funzionamento del workspace di tesi bimanuale.

---

## 1. Accesso alla Desk
Per ogni robot, apri un browser (Chrome consigliato) e inserisci l'indirizzo IP corrispondente:
*   **Robot Sinistro (Left):** `http://192.168.9.11`
*   **Robot Destro (Right):** `http://192.168.9.12`

*Nota: Il PC deve avere un IP statico nella stessa sottorete (es. `192.168.9.100`).*

---

## 2. Configurazione FCI (Fondamentale)
Senza l'attivazione della Franka Control Interface, ROS 2 non potrà inviare comandi.
1.  Fai il login con le credenziali del laboratorio.
2.  Assicurati che il robot sia in **modalità Execution** (luce blu).
3.  Attiva il selettore **FCI** nel menu in alto a destra. 
    *   *Verifica:* L'icona deve diventare blu/attiva.
    *   *Importante:* Se il robot va in errore, l'FCI potrebbe disattivarsi automaticamente; controlla sempre dopo una collisione.

---

## 3. Impostazioni del Workspace (Safety)
Per permettere a MoveIt di pianificare movimenti fluidi senza interruzioni dalla sicurezza di basso livello del robot:

### A. Limiti di Forza e Coppia
*   Naviga in **Settings > Safety**.
*   Imposta le soglie di collisione a un livello che permetta la manipolazione di oggetti (es. il mug della tesi) ma che protegga l'hardware.
*   **Cartesian Limits:** Assicurati che non ci siano limiti troppo restrittivi che interferiscano con l'area centrale del tavolo dove avviene l'handover.

### B. Virtual Walls (Pareti Virtuali)
*   **ATTENZIONE:** I Virtual Walls impostati sulla Desk hanno priorità assoluta su ROS.
*   Assicurati che il workspace definito sulla Desk includa l'intero raggio d'azione necessario per il task bimanuale (asse Y tra -0.6m e +0.6m).
*   Se MoveIt fallisce ma il robot non sembra aver toccato nulla, verifica sulla Desk se è stata violata una "Keep-out zone".

---

## 4. End-Effector (Gripper/Pump)
Il sistema è configurato per supportare sia la `franka_hand` che la `cobot_pump`.
1.  In **Settings > End-effector**, verifica che sia selezionato il modello corretto.
2.  Se si usa la pinza, esegui il "Grasping Range Learning" sulla Desk se hai cambiato i diti (fingers), per garantire che il ROS topic `/joint_states` della pinza sia accurato.

---

## 5. Procedure di Recovery
Se il robot si blocca (luce rossa o gialla):
1.  **Desk:** Clicca sul tasto "Recovery" nell'angolo in basso a destra.
2.  **Manuale:** Se il robot è in una posizione di singolarità o collisione fisica, usa la modalità "Guiding" per spostarlo leggermente prima di riattivare l'FCI.
3.  **ROS:** Una volta ripristinato sulla Desk, è necessario riavviare i nodi di controllo o assicurarli che i controller siano tornati in stato `active`.

---

## 6. Riepilogo Mappatura (Allineata ROMOTIVE)
| Robot | IP Fisico | Namespace ROS | Ruolo VLM |
| :--- | :--- | :--- | :--- |
| **Sinistro** | `192.168.9.11` | `franka2` | `left_arm` |
| **Destro** | `192.168.9.12` | `franka1` | `right_arm` |

---

*Ultimo aggiornamento: Aprile 2026*
