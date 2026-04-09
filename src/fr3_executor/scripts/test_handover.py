#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PickObject, PlaceObject, GiveObject, TakeObject, MoveHome
from action_msgs.msg import GoalStatus
import asyncio
import threading

class HandoverTester(Node):
    def __init__(self):
        super().__init__('handover_tester')
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        self.give_client = ActionClient(self, GiveObject, 'give_object')
        self.take_client = ActionClient(self, TakeObject, 'take_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')
        self.home_client = ActionClient(self, MoveHome, 'move_home')

    async def run_test(self):
        self.get_logger().info("🚀 STARTING THREADED HANDOVER TEST (VLM Bypass)...")

        # 1. Right Arm PICK Cube
        self.get_logger().info("--- Phase 1: Right Arm PICK ---")
        goal = PickObject.Goal()
        goal.arm = "right_arm"
        goal.target_pose.header.frame_id = "base_pose"
        res = await self.send_goal(self.pick_client, goal)
        if not res or not res.success:
             self.get_logger().error("Phase 1 FAILED. Aborting.")
             return

        # 2. Parallel GIVE & TAKE
        self.get_logger().info("--- Phase 2: Parallel Handover (GIVE & TAKE) ---")
        give_goal = GiveObject.Goal()
        give_goal.arm = "right_arm"
        give_goal.handover_pose.header.frame_id = "mid_air"

        take_goal = TakeObject.Goal()
        take_goal.arm = "left_arm"
        take_goal.handover_pose.header.frame_id = "mid_air"

        tasks = [
            self.send_goal(self.give_client, give_goal),
            self.send_goal(self.take_client, take_goal)
        ]
        results = await asyncio.gather(*tasks)
        if not all(r and r.success for r in results):
             self.get_logger().error("Phase 2 FAILED. Aborting.")
             return

        # 3. Left Arm PLACE in Box
        self.get_logger().info("--- Phase 3: Left Arm PLACE ---")
        place_goal = PlaceObject.Goal()
        place_goal.arm = "left_arm"
        place_goal.place_pose.header.frame_id = "box"
        await self.send_goal(self.place_client, place_goal)

        # 4. HOME
        self.get_logger().info("--- Phase 4: Resetting Home ---")
        home_goal = MoveHome.Goal()
        home_goal.arm = "left_arm"
        await self.send_goal(self.home_client, home_goal)
        home_goal.arm = "right_arm"
        await self.send_goal(self.home_client, home_goal)

        self.get_logger().info("✅ MANUAL TEST COMPLETED SUCCESSFULLY!")

    async def send_goal(self, client, goal):
        name = client._action_name
        self.get_logger().info(f"⏳ Waiting for {name} server...")
        client.wait_for_server()
        
        self.get_logger().info(f"📤 Sending goal to {name}...")
        send_goal_future = client.send_goal_async(goal)
        
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ Goal REJECTED by {name}")
            return None

        self.get_logger().info(f"✅ Goal ACCEPTED by {name}. Monitoring execution...")
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 {name} SUCCESS!")
            return result.result
        else:
            self.get_logger().error(f"❌ {name} FAILED with status code: {result.status}")
            return result.result

async def main():
    rclpy.init()
    node = HandoverTester()
    
    # Start spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        await node.run_test()
    except Exception as e:
        import traceback
        node.get_logger().error(f"Error during test: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
