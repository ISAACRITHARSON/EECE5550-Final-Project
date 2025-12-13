#!/usr/bin/env python3
"""
TB4 Navigation Controller: Monitor Nav2 goal status and signal arrival
Author: Abdul Rahman - Autonyx Robotics
"""
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String

print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

class TB4NavController(Node):
    def __init__(self):
        super().__init__('tb4_nav_controller')
        
        # Subscribe to Nav2 goal status
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )
        
        # Publish arrival signal through bridge
        self.arrival_pub = self.create_publisher(
            String, '/delivery_goal', 10)
        
        self.goal_reached = False
        
        self.get_logger().info("TB4 Navigation Controller ready")
        self.get_logger().info("Monitoring Nav2 goal status...")
    
    def status_callback(self, msg):
        """Monitor navigation goal status"""
        if not msg.status_list:
            return
        
        # Get latest status
        latest_status = msg.status_list[-1]
        status_code = latest_status.status
        
        # Status codes:
        # 1 = ACCEPTED
        # 2 = EXECUTING
        # 4 = SUCCEEDED
        # 5 = CANCELED
        # 6 = ABORTED
        
        if status_code == 2 and not self.goal_reached:
            self.get_logger().info("Navigating to goal...")
        
        elif status_code == 4 and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info("Navigation goal reached!")
            
            # Publish arrival signal
            arrival_msg = String()
            arrival_msg.data = "arrived"
            self.arrival_pub.publish(arrival_msg)
            self.get_logger().info("Published 'arrived' signal")
            
            # Reset for next goal after delay
            self.create_timer(2.0, self.reset_flag, one_shot=True)
        
        elif status_code == 5:
            self.get_logger().warn("Navigation canceled")
            self.goal_reached = False
        
        elif status_code == 6:
            self.get_logger().warn("Navigation aborted")
            self.goal_reached = False
    
    def reset_flag(self):
        """Reset goal flag after delay"""
        self.goal_reached = False
        self.get_logger().info("Ready for next goal")

def main():
    rclpy.init()
    controller = TB4NavController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
