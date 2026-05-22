#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics 
Code Description: 
Dual-Arm Franka Orchestrator with Logic-based Synchronization and Fixed Looping.

Supports two execution modes:
  1. STATIC  (--plan <file.json>)  : loads a pre-built JSON plan from disk
     Optional: add --scan to query /scan_table and resolve "auto" target names
  2. DYNAMIC (no --plan flag)      : queries the VLM server at runtime and
                                     builds the Behavior Tree on-the-fly
                                     (the VLM calls /scan_table internally)
================================================================================
"""

import rclpy
import py_trees
import py_trees_ros
import sys
import operator
import json

from geometry_msgs.msg import PoseStamped
from franka_custom_interfaces.srv import ScanTable

from franka_bimanual_orchestrator.behaviors.vlm_client import VlmActionClient
from franka_bimanual_orchestrator.behaviors.object_localization_client import ObjectLocalizationClient
from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_bimanual_orchestrator.behaviors.wait_client import WaitActionClient
from franka_bimanual_orchestrator.behaviors.sync_barrier_client import SyncBarrierClient
from franka_bimanual_orchestrator.behaviors.planner_utils import PlanSplitter, DynamicActionIterator, PlanPopper


# ---------------------------------------------------------------------------
# Helper: initialise the py_trees blackboard with all required keys & defaults
# Called identically for both the static-JSON and the dynamic-VLM paths so
# that behaviour nodes never hit a KeyError.
# ---------------------------------------------------------------------------
def _init_blackboard(full_plan: dict, object_override: str = None, metrics_logger=None):
    """Register all blackboard keys and set safe default values."""
    bb = py_trees.blackboard.Client(name="MainConfig")

    keys = [
        "left_target_name", "right_target_name",
        "left_active_arm",  "right_active_arm",
        "left_target_pose_name", "right_target_pose_name",
        "left_target_location",  "right_target_location",
        "left_target_pose",      "right_target_pose",
        "mission_metadata",      "mission_type",
        "handover_starting",     "metrics_logger",
    ]
    for k in keys:
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)
        
    bb.metrics_logger = metrics_logger

    # Defaults
    bb.left_target_name      = "none"
    bb.right_target_name     = "none"
    bb.left_active_arm       = "left_arm"
    bb.right_active_arm      = "right_arm"
    bb.left_target_pose_name = "ready"
    bb.right_target_pose_name = "ready"
    bb.left_target_location  = "box"
    bb.right_target_location = "box"
    bb.left_target_pose      = PoseStamped()
    bb.right_target_pose     = PoseStamped()
    bb.handover_starting     = False

    metadata = full_plan.get("mission_metadata", {})
    bb.mission_metadata = metadata
    bb.mission_type     = metadata.get("type", "SIMPLE")

    # Optional object override (--object flag, only relevant for JSON mode)
    if object_override:
        print(f"🔄 Overriding all targets with: {object_override}")
        bb.left_target_name  = object_override
        bb.right_target_name = object_override

    return bb


# ---------------------------------------------------------------------------
# Tree building
# ---------------------------------------------------------------------------
def create_dynamic_arm_sequence(arm_name, plan_steps):
    """Builds a pure BT sequence from a list of plan steps."""
    prefix = f"{arm_name}_"
    seq = py_trees.composites.Sequence(name=f"Sequence_{arm_name.upper()}", memory=True)

    if not plan_steps:
        return py_trees.behaviours.Success(name=f"No_Task_{arm_name}")

    for i, step in enumerate(plan_steps):
        action = step.get('action')
        target = step.get('target_name') or step.get('target_location')

        if action == "FIND_OBJECT":
            action_server = "/detect_object_left" if arm_name == "left" else "/detect_object_right"
            node = ObjectLocalizationClient(
                name=f"Find_{target}_{i}",
                prefix=prefix,
                target_name=target,
                action_name=action_server
            )
        elif action == "PICK":
            action_server = f"/{arm_name}_arm/pick_object"
            node = PickActionClient(name=f"Pick_{target}_{i}", action_name=action_server, prefix=prefix, target_name=target)
        elif action == "PLACE":
            action_server = f"/{arm_name}_arm/place_object"
            node = PlaceActionClient(name=f"Place_{target}_{i}", action_name=action_server, prefix=prefix, target_location=target)
        elif action == "MOVE_HOME":
            target_pose = step.get('pose_name')
            action_server = f"/{arm_name}_arm/move_home"
            node = MoveHomeClient(name=f"Home_{i}", action_name=action_server, prefix=prefix, target_pose=target_pose)
        elif action == "WAIT":
            duration = step.get('seconds') or step.get('duration')
            node = WaitActionClient(name=f"Wait_{i}", prefix=prefix, duration=duration)
        elif action == "SYNC_BARRIER":
            node = SyncBarrierClient(
                name=f"SyncBarrier_{i}",
                role="donor" if arm_name == "right" else "recipient"
            )
        else:
            continue

        seq.add_child(node)

    return seq


def create_tree(task_description, full_plan):
    """Constructs the bimanual BT dynamically based on the plan."""
    root = py_trees.composites.Sequence(name=f"Mission: {task_description}", memory=True)

    execution_parallel = py_trees.composites.Parallel(
        name="Bimanual_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    left_seq  = create_dynamic_arm_sequence("left",  full_plan.get('left_arm_sequence',  []))
    right_seq = create_dynamic_arm_sequence("right", full_plan.get('right_arm_sequence', []))

    execution_parallel.add_children([left_seq, right_seq])
    root.add_child(execution_parallel)
    return root


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    rclpy.init(args=sys.argv)

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("task",   type=str, nargs="?", default="Bimanual Operation",
                        help="Natural-language task description (used in VLM mode)")
    parser.add_argument("--plan", type=str,
                        help="Path to a JSON file containing a static plan")
    parser.add_argument("--object", type=str,
                        help="Override the target object name in every plan step")
    parser.add_argument("--scan", action="store_true",
                        help="Call /scan_table before execution to resolve 'auto' target names in the JSON plan")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    full_plan = {}
    task_desc = args.task
    
    if args.plan and task_desc == "Bimanual Operation":
        import os
        task_desc = os.path.splitext(os.path.basename(args.plan))[0]
    elif not args.plan:
        task_desc = "vlm_" + task_desc.replace(" ", "_").lower()
    
    from franka_bimanual_orchestrator.metrics import MetricsLogger
    metrics_logger = MetricsLogger(experiment_name=task_desc)

    # -----------------------------------------------------------------------
    # MODE 1 – STATIC JSON
    # -----------------------------------------------------------------------
    if args.plan:
        try:
            with open(args.plan, 'r') as f:
                full_plan = json.load(f)
            print(f"✅ Custom plan loaded from {args.plan}.")
            metrics_logger.log_vlm_plan(full_plan)
        except Exception as e:
            print(f"❌ Failed to load plan: {e}")
            return

        # Initialise blackboard (with optional object override)
        metrics_logger.mark_vlm_success(True)
        _init_blackboard(full_plan, object_override=args.object, metrics_logger=metrics_logger)

        # Apply --object override directly to plan steps as well
        if args.object:
            for arm in ['left_arm_sequence', 'right_arm_sequence']:
                for step in full_plan.get(arm, []):
                    if 'target_name' in step and step['target_name'] not in ['shared', 'box']:
                        step['target_name'] = args.object
                    if 'config' in step and 'target_name' in step['config'] \
                            and step['config']['target_name'] not in ['shared', 'box']:
                        step['config']['target_name'] = args.object

        # ── --scan: chiama /scan_table e risolve i target_name == "auto" ──────
        if args.scan:
            print("\n🔍 --scan: interrogo /scan_table per rilevare gli oggetti sul tavolo...")
            scan_node = rclpy.create_node('scan_query_client')
            scan_client = scan_node.create_client(ScanTable, 'scan_table')

            if not scan_client.wait_for_service(timeout_sec=8.0):
                print("⚠️  /scan_table non disponibile — i target 'auto' rimarranno invariati.")
            else:
                future = scan_client.call_async(ScanTable.Request())
                while rclpy.ok() and not future.done():
                    rclpy.spin_once(scan_node, timeout_sec=0.1)

                resp = future.result()
                scan_node.destroy_node()

                if resp and resp.success:
                    import json as _json
                    scene = _json.loads(resp.scene_json)
                    metrics_logger.log_scene_inventory(scene)
                    print(f"✅ Oggetti rilevati sul tavolo ({len(scene)} unici):")
                    for obj in scene:
                        arm_hint = "left_arm" if obj['side'] == "left_side" else "right_arm"
                        print(f"   · '{obj['label']}' → {obj['side']} "
                              f"(X={obj['x_world']:.3f}m, conf={obj['conf']:.2f}) "
                              f"→ {arm_hint}")

                    # Risolvi target_name == "auto" in base al lato del braccio.
                    # Se nessun oggetto trovato per un lato → sostituisci l'intera
                    # sequenza con MOVE_HOME per parcheggiare il braccio in sicurezza.
                    side_map = {
                        'left_arm_sequence':  ('left_side',  'left_arm'),
                        'right_arm_sequence': ('right_side', 'right_arm'),
                    }
                    for arm_key, (arm_side, arm_id) in side_map.items():
                        arm_objects = [o for o in scene if o['side'] == arm_side]
                        has_auto = any(
                            s.get('target_name') == 'auto'
                            for s in full_plan.get(arm_key, [])
                        )

                        if has_auto and not arm_objects:
                            # Nessun oggetto su questo lato: parcheggia il braccio
                            print(f"   ℹ️  Nessun oggetto su '{arm_side}' — "
                                  f"'{arm_key}' sostituita con MOVE_HOME.")
                            full_plan[arm_key] = [{
                                "action": "MOVE_HOME",
                                "arm": arm_id,
                                "pose_name": "ready"
                            }]
                        else:
                            # Risolvi i passi "auto" con il primo oggetto trovato (best-conf)
                            for step in full_plan.get(arm_key, []):
                                if step.get('target_name') == 'auto':
                                    resolved = arm_objects[0]['label']
                                    step['target_name'] = resolved
                                    print(f"   🔄 '{arm_key}' step '{step['action']}': "
                                          f"'auto' → '{resolved}'")
                else:
                    scan_node.destroy_node()
                    print("⚠️  /scan_table ha risposto con errore — i target 'auto' rimarranno invariati.")
            print()

    # -----------------------------------------------------------------------
    # MODE 2 – DYNAMIC VLM
    # -----------------------------------------------------------------------
    else:
        print(f"\n📡 Querying VLM for online task: '{task_desc}'...")

        temp_node = rclpy.create_node('vlm_temp_query_client')
        from rclpy.action import ActionClient
        from franka_custom_interfaces.action import VlmQuery

        action_client = ActionClient(temp_node, VlmQuery, 'vlm_query')
        if not action_client.wait_for_server(timeout_sec=10.0):
            print("❌ ERROR: VLM Server Node is not running! Cannot execute dynamic commands.")
            temp_node.destroy_node()
            return

        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = task_desc

        print("⏳ Planning with Gemini API...")
        send_goal_future = action_client.send_goal_async(goal_msg)

        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(temp_node, timeout_sec=0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("❌ ERROR: VLM goal was rejected.")
            temp_node.destroy_node()
            return

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(temp_node, timeout_sec=0.1)

        result = result_future.result().result
        temp_node.destroy_node()

        if result.success:
            try:
                full_plan = json.loads(result.vlm_plan_json)
                if "scene_inventory" in full_plan:
                    metrics_logger.log_scene_inventory(full_plan.pop("scene_inventory"))
                if "vlm_input_prompt" in full_plan:
                    metrics_logger.log_vlm_prompt(full_plan.pop("vlm_input_prompt"))
                
                print("🎉 Plan generated by VLM successfully!")
                print(json.dumps(full_plan, indent=2))
                metrics_logger.log_vlm_plan(full_plan)
            except Exception as e:
                print(f"❌ Failed to parse VLM plan JSON: {e}")
                return
        else:
            print(f"❌ VLM failed: {result.message}")
            metrics_logger.mark_vlm_success(False)
            metrics_logger.mark_bt_success(False)
            import os
            log_dir = "VLM_not_Integrated" if args.plan else "VLM_INTEGRATED_EXPERIMENTS"
            base = "/mm_ws/src/franka_bimanual_bringup/scripts/automate_scenarios/experiment_logs" if os.path.exists("/mm_ws") else "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src/franka_bimanual_bringup/scripts/automate_scenarios/experiment_logs"
            base_path = os.path.join(base, log_dir)
            metrics_logger.save_log(base_path)
            return

        # Initialise blackboard identically to the static path
        metrics_logger.mark_vlm_success(True)
        _init_blackboard(full_plan, metrics_logger=metrics_logger)

    # -----------------------------------------------------------------------
    # Build & run the Behaviour Tree (common to both modes)
    # -----------------------------------------------------------------------
    root = create_tree(task_desc, full_plan)
    root = py_trees.decorators.OneShot(
        name="Single Mission",
        child=root,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )

    print("\n" + "="*40)
    print("🎄 GENERATED BEHAVIOR TREE:")
    print("="*40)
    print(py_trees.display.ascii_tree(root))
    print("="*40 + "\n")

    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    # Shared synchronisation flag (handover)
    bb_main = py_trees.blackboard.Client(name="Main")
    bb_main.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    bb_main.handover_ready = False

    try:
        tree.setup(node_name="bimanual_engine", timeout=15.0)
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    print("\n--- Bimanual DYNAMIC Engine Ready ---")

    try:
        tree.tick_tock(period_ms=1000)
        # One-shot loop: exit as soon as the tree reaches SUCCESS or FAILURE
        while rclpy.ok():
            rclpy.spin_once(tree.node, timeout_sec=0.1)
            status = tree.root.status
            if status == py_trees.common.Status.SUCCESS:
                print("\n✅ MISSION COMPLETED: Both arms finished successfully!")
                metrics_logger.mark_bt_success(True)
                break
            if status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSION FAILED: Plan aborted. Check logs for details.")
                metrics_logger.mark_bt_success(False)
                break
    except KeyboardInterrupt:
        print("\n🛑 Manual interruption.")
        metrics_logger.mark_bt_success(False)
    finally:
        tree.shutdown()
        
        import os
        log_dir = "VLM_not_Integrated" if args.plan else "VLM_INTEGRATED_EXPERIMENTS"
        base = "/mm_ws/src/franka_bimanual_bringup/scripts/automate_scenarios/experiment_logs" if os.path.exists("/mm_ws") else "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src/franka_bimanual_bringup/scripts/automate_scenarios/experiment_logs"
        base_path = os.path.join(base, log_dir)
        log_path = metrics_logger.save_log(base_path)
        print(f"\n📊 Metrics saved to: {log_path}")
        
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()