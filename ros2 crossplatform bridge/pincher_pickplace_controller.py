#!/usr/bin/env python3
"""
PincherX100 Pick-and-Place Controller with direct gripper control
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import time

class PincherPickPlaceController(Node):
    def __init__(self):
        super().__init__('pincher_pickplace_controller')
        
        # Subscribe to arrival signal
        self.arrival_sub = self.create_subscription(
            String, '/delivery_goal_remote', self.arrival_callback, 10)
        
        # Publisher for direct gripper control
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, '/px100/commands/joint_single', 10)
        
        # Your taught positions
        self.pre_pick_pos = [0.046019, -1.817767, 1.664369, 0.590583]
        self.pick_pos = [1.555457, 1.445010, -1.400524, 1.483359]
        self.lift_pos = [0.035282, -0.170272, -1.405126, 0.144194]
        self.place_pos = [-0.351282, 0.501612, -0.687223, 1.434272]
        
        # Initialize arm
        self.get_logger().info("Initializing PincherX100...")
        try:
            self.bot = InterbotixManipulatorXS(
                robot_model='px100',
                group_name='arm',
                gripper_name='gripper'
            )
            self.get_logger().info("PincherX100 initialized")
            print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
            # Go to sleep position initially
            self.bot.arm.go_to_sleep_pose()
            self.get_logger().info("Sleep position")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize: {e}")
            self.bot = None
        
        self.get_logger().info("Pick-and-Place Controller ready")
        self.get_logger().info("Listening for 'goal_reached' signal...")
    
    def open_gripper(self):
        """Open gripper using direct command"""
        msg = JointSingleCommand()
        msg.name = 'gripper'
        msg.cmd = 0.037  # Open position
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper opening...")
    
    def close_gripper(self):
        """Close gripper using direct command"""
        msg = JointSingleCommand()
        msg.name = 'gripper'
        msg.cmd = -0.037  # Close position
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper closing...")
    
    def arrival_callback(self, msg):
        """React to goal reached signal"""
        if msg.data == "goal_reached" or msg.data == "arrived":
            self.get_logger().info("TurtleBot reached goal!")
            self.get_logger().info("Starting pick-and-place sequence...")
            
            if self.bot:
                self.execute_pick_and_place()
    
    def execute_pick_and_place(self):
        """Execute pick-and-place with taught positions"""
        try:
            # Step 1: Wake up and go to home
            self.get_logger().info("🏠 Moving to home position...")
            self.bot.arm.go_to_home_pose()
            time.sleep(2.0)
            
            # Step 2: Move to pre-pick position
            self.get_logger().info("Moving to pre-pick position...")
            self.bot.arm.set_joint_positions(self.pre_pick_pos, moving_time=1.0)
            self.get_logger().info("Waiting 10 seconds...")
            time.sleep(2.0)
            
            # Step 3: Open gripper
            self.open_gripper()
            time.sleep(2.0)
            
            # Step 4: Move to pick position
            self.get_logger().info("Moving to pick position...")
            self.bot.arm.set_joint_positions(self.pick_pos, moving_time=1.0)
            self.get_logger().info("Waiting 10 seconds...")
            time.sleep(2.0)
            
            # Step 5: Close gripper to grasp
            self.close_gripper()
            time.sleep(2.0)
            
            # Step 6: Lift object
            self.get_logger().info("Lifting object...")
            self.bot.arm.set_joint_positions(self.lift_pos, moving_time=1.0)
            self.get_logger().info("Waiting 10 seconds...")
            time.sleep(10.0)
            
            # Step 7: Move to place position
            self.get_logger().info("Moving to place position...")
            self.bot.arm.set_joint_positions(self.place_pos, moving_time=1.0)
            self.get_logger().info("Waiting 10 seconds...")
            time.sleep(2.0)
            
            # Step 8: Release object
            self.open_gripper()
            time.sleep(2.0)
            
            # Step 9: Return to home
            self.get_logger().info("Returning to home...")
            self.bot.arm.go_to_home_pose()
            time.sleep(1.0)
            
            # Step 10: Go to sleep
            self.get_logger().info("Going to sleep position...")
            self.bot.arm.go_to_sleep_pose()
            
            self.get_logger().info("Pick-and-place complete")
            
        except Exception as e:
            self.get_logger().error(f"Pick-and-place failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def shutdown(self):
        if self.bot:
            self.get_logger().info("Shutting down...")
            self.bot.arm.go_to_sleep_pose()

def main():
    rclpy.init()
    controller = PincherPickPlaceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down...")
        controller.shutdown()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
