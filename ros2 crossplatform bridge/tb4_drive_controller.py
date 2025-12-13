#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

class SimpleTB4Controller(Node):
    def __init__(self):
        super().__init__('simple_tb4_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.status_pub = self.create_publisher(String, '/delivery_goal', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.done = False
        self.linear_speed = 0.2
        
        self.timer = self.create_timer(0.1, self.drive)
        
        self.get_logger().info("Simple TB4 Controller started")
        self.get_logger().info("Driving 1 meter...")
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f"Start: x={self.start_x:.3f}, y={self.start_y:.3f}")
    
    def get_distance(self):
        if self.start_x is None:
            return 0.0
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def drive(self):
        if self.done:
            return
        
        distance = self.get_distance()
        
        if distance >= 1.0:
            # Stop
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Publish arrived
            msg = String()
            msg.data = "arrived"
            self.status_pub.publish(msg)
            
            self.get_logger().info(f"Arrived! Distance: {distance:.3f}m")
            self.get_logger().info("Published 'arrived'")
            self.done = True
        else:
            # Keep driving
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(cmd)
            
            if int(distance * 10) % 2 == 0:  # Log every 0.2m
                self.get_logger().info(f"{distance:.2f}m / 1.00m")

def main():
    rclpy.init()
    controller = SimpleTB4Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # Stop on exit
        cmd = Twist()
        controller.cmd_vel_pub.publish(cmd)
        print("\nStopped")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
