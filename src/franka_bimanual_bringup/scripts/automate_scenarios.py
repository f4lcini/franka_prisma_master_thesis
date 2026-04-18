#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import sys

def run_command(cmd):
    print(f"Executing: {cmd}")
    return subprocess.Popen(cmd, shell=True)

class BimanualTAMPTestSuite:
    def __init__(self, bypass_perception=False):
        self.bypass = bypass_perception
        print("==============================================")
        print("   FRANCA BIMANUAL TAMP - TEST SUITE 🎯      ")
        if self.bypass:
            print("   (MODE: NO-PERCEPTION / BYPASS ACTIVE)     ")
        print("==============================================")

    def _setup_params(self):
        if self.bypass:
            # Set global parameter for behaviors to see
            subprocess.run('ros2 param set /simple_moveit_server bypass_perception true', shell=True)
            subprocess.run('ros2 param set /bimanual_engine bypass_perception true', shell=True)

    def scenario_relay(self):
        self._setup_params()
        print("\n--- SCENARIO 1: CONSECUTIVE SHARED RELAY ---")
        print("Goal: Left arm picks -> Places in 'shared' -> Right arm picks from 'shared' -> Places in 'box'")
        
        cmd = 'ros2 run franka_bimanual_orchestrator main_engine "Pick the red cube from base_pose and place it in the box using the shared zone"'
        if self.bypass:
            cmd += ' --ros-args -p bypass_perception:=true'
        return run_command(cmd)

    def scenario_handover(self):
        self._setup_params()
        print("\n--- SCENARIO 2: SYNCHRONIZED MID-AIR HANDOVER ---")
        print("Goal: Left arm picks -> Mid-air transfer -> Right arm places in box")
        
        cmd = 'ros2 run franka_bimanual_orchestrator main_engine "Pick the red cube and handover it to the right arm at mid_air point"'
        if self.bypass:
            cmd += ' --ros-args -p bypass_perception:=true'
        return run_command(cmd)

    def scenario_recovery(self):
        self._setup_params()
        print("\n--- SCENARIO 3: FORCE-BASED RECOVERY TEST ---")
        print("Goal: Force a missed pick to trigger BT re-planning loop")
        print("ACTION: You must manually remove the object in Gazebo during the 'Pick' stage!")
        
        cmd = 'ros2 run franka_bimanual_orchestrator main_engine "Pick the red cube and place it in the box"'
        if self.bypass:
            cmd += ' --ros-args -p bypass_perception:=true'
        return run_command(cmd)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-perception", action="store_true", help="Bypass YOLO and use known coordinates")
    args = parser.parse_args()

    suite = BimanualTAMPTestSuite(bypass_perception=args.no_perception)
    print("1. Consecutive shared-space relay")
    print("2. Synchronized mid-air handover")
    print("3. Force-feedback recovery test")
    
    try:
        choice = input("\nEnter choice (1-3): ")
    except EOFError:
        print("\nNo input detected. Exiting.")
        return

    proc = None
    if choice == '1':
        proc = suite.scenario_relay()
    elif choice == '2':
        proc = suite.scenario_handover()
    elif choice == '3':
        proc = suite.scenario_recovery()
    else:
        print("Invalid choice.")
        return

    if proc:
        try:
            proc.wait()
        except KeyboardInterrupt:
            proc.terminate()
            print("\nTest interrupted.")

if __name__ == '__main__':
    main()
