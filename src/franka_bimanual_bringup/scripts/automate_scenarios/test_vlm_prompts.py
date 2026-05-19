import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import VlmQuery
import json
import sys
import time

class VlmTestClient(Node):
    def __init__(self):
        super().__init__('vlm_test_client')
        self._action_client = ActionClient(self, VlmQuery, 'vlm_query')

    def send_prompt(self, prompt_text):
        self.get_logger().info(f"\n========================================================")
        self.get_logger().info(f"📤 INVIANDO PROMPT ALLA VLM: '{prompt_text}'")
        self.get_logger().info(f"========================================================")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Servizio /vlm_query non disponibile! Assicurati che vlm_server_node sia attivo.")
            return None

        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = prompt_text

        self.get_logger().info("⏳ Generazione del piano bimanuale in corso (chiamata a Gemini)...")
        future = self._action_client.send_goal_async(goal_msg)
        
        # Semplice spin sincrono per attendere l'accettazione del goal
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rifiutato dal server VLM.")
            return None

        self.get_logger().info("✅ Goal accettato. In attesa del risultato finale...")
        result_future = goal_handle.get_result_async()
        
        # Spin sincrono per attendere il risultato finale
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result().result
        
        if result.success:
            self.get_logger().info("🎉 PIANO GENERATO CON SUCCESSO!")
            try:
                # Carichiamo e formattiamo il JSON per una stampa bellissima
                parsed_json = json.loads(result.vlm_plan_json)
                formatted_json = json.dumps(parsed_json, indent=2)
                print(formatted_json)
                
                # Salvataggio automatico su file
                import os
                pkg_path = "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src/franka_bimanual_bringup/scripts/automate_scenarios/json_plans"
                os.makedirs(pkg_path, exist_ok=True)
                
                # Scegliamo il nome del file in base al prompt
                filename = "vlm_plan_exp3.json"
                if "Sort" in prompt_text:
                    filename = "vlm_plan_exp1.json"
                elif "Transfer" in prompt_text:
                    filename = "vlm_plan_exp2.json"
                    
                full_path = os.path.join(pkg_path, filename)
                with open(full_path, 'w') as f:
                    f.write(formatted_json)
                self.get_logger().info(f"💾 Piano salvato in: {full_path}")
                
                return formatted_json
            except Exception as e:
                self.get_logger().error(f"Errore nel parsing o salvataggio del JSON: {e}")
                print(result.vlm_plan_json)
                return result.vlm_plan_json
        else:
            self.get_logger().error(f"❌ VLM fallita: {result.message}")
            return None

def main():
    rclpy.init()
    node = VlmTestClient()
    
    prompts = [
        "Sort all the items inside the designated boxes", # EXP1
        "Transfer the object on the table to the box",     # EXP2
        "Clear the entire workspace."                      # EXP3
    ]
    
    # Se passiamo un argomento numerico (1, 2 o 3) eseguiamo solo quel test
    if len(sys.argv) > 1:
        try:
            choice = int(sys.argv[1])
            if 1 <= choice <= 3:
                node.send_prompt(prompts[choice - 1])
            else:
                print("Scelta non valida. Usa 1, 2 o 3.")
        except ValueError:
            print("Usa: python3 test_vlm_prompts.py [1|2|3]")
    else:
        # Altrimenti li eseguiamo tutti e tre in sequenza
        for p in prompts:
            node.send_prompt(p)
            time.sleep(2.0)
            
    rclpy.shutdown()

if __name__ == '__main__':
    main()
