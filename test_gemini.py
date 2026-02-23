import os
import json
import argparse
from pydantic import BaseModel, Field
from typing import Literal
from PIL import Image
from google import genai
from google.genai import types

# ---------------------------------------------------------
# 1. Definizione dello Schema (Contratto Dati per ROS2)
# ---------------------------------------------------------
class RobotDecision(BaseModel):
    selected_arm: Literal["left_arm", "right_arm"] = Field(
        description="Il braccio selezionato in base alla posizione spaziale dell'oggetto."
    )
    action_type: Literal["PICK", "PLACE", "POUR"] = Field(
        description="Il tipo di azione che il Motion Planner (MTC) dovrà eseguire."
    )
    reasoning: str = Field(
        description="Breve giustificazione della scelta (es. 'oggetto a destra')."
    )

# ---------------------------------------------------------
# 2. Funzione di Inferenza VLM
# ---------------------------------------------------------
def run_vlm_query():
    # Setup degli argomenti (Stile Yigit)
    parser = argparse.ArgumentParser(description="Query VLM per decisioni bimanuali.")
    parser.add_argument("--image", default="/home/falco_robotics/mm_ws/test-image.jpg", help="Percorso immagine")
    parser.add_argument("--model", default="models/gemini-3-flash-preview", help="Nome modello completo")
    parser.add_argument("--max_side", type=int, default=112, help="Ridimensionamento per risparmio quota")
    parser.add_argument("--thinking", default="LOW", choices=["LOW", "MEDIUM", "HIGH"], help="Livello reasoning")
    parser.add_argument("--jpeg_quality", type=int, default=75, help="Qualità JPEG")
    args = parser.parse_args()

    # Verifica API Key
    if "GEMINI_API_KEY" not in os.environ:
        print("ERRORE: Variabile d'ambiente GEMINI_API_KEY non impostata.")
        return

    # Inizializzazione Client
    client = genai.Client()
    
    # Caricamento e pre-elaborazione immagine (Critico per evitare 429)
    try:
        img = Image.open(args.image)
        if args.max_side > 0:
            img.thumbnail((args.max_side, args.max_side))
            print(f"Immagine ridimensionata a: {img.size}")
    except Exception as e:
        print(f"Errore caricamento immagine: {e}")
        return

    # Prompt ottimizzato per bimanualità
    prompt = """
    Sei il modulo di High-Level Planning di un robot Franka FER bimanuale.
    Analizza l'immagine e identifica l'oggetto target.
    REGOLE:
    - Se l'oggetto è nella metà SINISTRA, usa 'left_arm'.
    - Se l'oggetto è nella metà DESTRA, usa 'right_arm'.
    Rispondi esclusivamente in formato JSON seguendo lo schema richiesto.
    """

    # Configurazione generazione
    cfg = types.GenerateContentConfig(
        response_mime_type="application/json",
        response_schema=RobotDecision,
        thinking_config=types.ThinkingConfig(thinking_level=args.thinking),
        temperature=0.1,
    )

    print(f"Inviando richiesta al modello: {args.model}...")
    
    try:
        # Chiamata API (corrisponde alla riga 386 di Yigit)
        response = client.models.generate_content(
            model=args.model,
            contents=[prompt, img],
            config=cfg
        )
        
        # Output dei risultati
        print("\n--- RISULTATO PER IL BEHAVIOR TREE ---")
        print(response.text)
        
        # Validazione formale del JSON
        data = json.loads(response.text)
        print(f"\n[DATO ESTRATTO] Braccio: {data['selected_arm']}")
        
    except Exception as e:
        print(f"\nERRORE DURANTE L'INFERENZA: {e}")

if __name__ == "__main__":
    run_vlm_query()