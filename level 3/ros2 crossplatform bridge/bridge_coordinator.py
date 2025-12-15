#!/usr/bin/env python3

import socket
import threading
import json
import sys

print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    print("ROS2 libraries imported")
except ImportError as e:
    print(f"Failed to import ROS2: {e}")
    sys.exit(1)

SERVER_IP = "0.0.0.0"
SERVER_PORT = 50505
PEER_IP = "127.0.0.1"
PEER_PORT = 50505
BUFFER_SIZE = 8192

class BridgeCoordinator(Node):
    def __init__(self, is_server=False):
        super().__init__('bridge_coordinator')
        self.is_server = is_server
        
        # TCP connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if is_server:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((SERVER_IP, SERVER_PORT))
            self.sock.listen(1)
            print(f"Waiting for connection on {SERVER_IP}:{SERVER_PORT}")
            self.conn, addr = self.sock.accept()
            print(f"TCP Connected to {addr}")
        else:
            print(f"Connecting to {PEER_IP}:{PEER_PORT}")
            self.sock.connect((PEER_IP, PEER_PORT))
            self.conn = self.sock
            print(f"TCP Connected")
        
        # Delivery command topics
        self.sub_delivery = self.create_subscription(
            String, '/delivery_goal', self.string_callback, 10)
        self.pub_delivery = self.create_publisher(
            String, '/delivery_goal_remote', 10)
        
        # TurtleBot4 Odometry - subscribe directly from TB4
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/odom_remote', 10)
        
        print("Subscribed to: /delivery_goal, /odom")
        print("Publishing to: /delivery_goal_remote, /odom_remote")
        
        self.thread = threading.Thread(target=self.listen_remote, daemon=True)
        self.thread.start()
        
        self.odom_count = 0
        print("Bridge operational!\n")
    
    def string_callback(self, msg):
        try:
            data = json.dumps({
                "type": "string",
                "topic": "delivery_goal",
                "data": msg.data
            })
            self.conn.sendall(data.encode('utf-8') + b'\n')
            print(f"Command: {msg.data}")
        except Exception as e:
            print(f"Send failed: {e}")
    
    def odom_callback(self, msg):
        try:
            odom_dict = {
                "header": {
                    "frame_id": msg.header.frame_id,
                    "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
                },
                "child_frame_id": msg.child_frame_id,
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w
                    }
                },
                "twist": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y,
                        "z": msg.twist.twist.linear.z
                    },
                    "angular": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z
                    }
                }
            }
            
            data = json.dumps({
                "type": "odometry",
                "topic": "odom",
                "data": odom_dict
            })
            self.conn.sendall(data.encode('utf-8') + b'\n')
            
            self.odom_count += 1
            if self.odom_count % 50 == 0:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                print(f"Odom: x={x:.2f}, y={y:.2f} ({self.odom_count} msgs)")
                
        except Exception as e:
            print(f"Odom send failed: {e}")
    
    def listen_remote(self):
        buffer = ""
        while True:
            try:
                chunk = self.conn.recv(BUFFER_SIZE).decode('utf-8')
                if not chunk:
                    break
                buffer += chunk
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    payload = json.loads(line)
                    msg_type = payload.get("type")
                    topic = payload.get("topic")
                    data = payload.get("data")
                    
                    if msg_type == "string" and topic == "delivery_goal":
                        msg = String()
                        msg.data = data
                        self.pub_delivery.publish(msg)
                        print(f"Command: {data}")
                    
                    elif msg_type == "odometry" and topic == "odom":
                        msg = Odometry()
                        msg.header.frame_id = data["header"]["frame_id"]
                        msg.header.stamp.sec = data["header"]["stamp"]["sec"]
                        msg.header.stamp.nanosec = data["header"]["stamp"]["nanosec"]
                        msg.child_frame_id = data["child_frame_id"]
                        msg.pose.pose.position.x = data["pose"]["position"]["x"]
                        msg.pose.pose.position.y = data["pose"]["position"]["y"]
                        msg.pose.pose.position.z = data["pose"]["position"]["z"]
                        msg.pose.pose.orientation.x = data["pose"]["orientation"]["x"]
                        msg.pose.pose.orientation.y = data["pose"]["orientation"]["y"]
                        msg.pose.pose.orientation.z = data["pose"]["orientation"]["z"]
                        msg.pose.pose.orientation.w = data["pose"]["orientation"]["w"]
                        msg.twist.twist.linear.x = data["twist"]["linear"]["x"]
                        msg.twist.twist.linear.y = data["twist"]["linear"]["y"]
                        msg.twist.twist.linear.z = data["twist"]["linear"]["z"]
                        msg.twist.twist.angular.x = data["twist"]["angular"]["x"]
                        msg.twist.twist.angular.y = data["twist"]["angular"]["y"]
                        msg.twist.twist.angular.z = data["twist"]["angular"]["z"]
                        
                        self.pub_odom.publish(msg)
                        
            except Exception as e:
                print(f"Listen error: {e}")
                break
    
    def destroy_node(self):
        try:
            self.conn.close()
        except:
            pass
        super().destroy_node()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', action='store_true')
    args = parser.parse_args()
    
    rclpy.init()
    node = BridgeCoordinator(is_server=args.server)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
