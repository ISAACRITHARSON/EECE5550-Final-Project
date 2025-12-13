#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class RobotPositionTracker(Node):
    def __init__(self):
        super().__init__('robot_position_tracker')
        
        # Subscribe to AMCL pose topic (this gives position on map)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Robot Position Tracker started. Listening for position updates...')
        print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    
    def pose_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_degrees = math.degrees(yaw)
        
        # Print current position
        self.get_logger().info(f'\n=== Robot Position on Map ===')
        self.get_logger().info(f'X: {x:.3f} m')
        self.get_logger().info(f'Y: {y:.3f} m')
        self.get_logger().info(f'Z: {z:.3f} m')
        self.get_logger().info(f'Yaw: {yaw:.3f} rad ({yaw_degrees:.1f}°)')
        self.get_logger().info(f'===========================\n')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
