#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QThread, Qt
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
import math

class ROSThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        
    def run(self):
        rclpy.spin(self.node)

class TurtleBot4GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        rclpy.init()
        self.node = Node('turtlebot4_waypoint_gui')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        self.audio_pub = self.node.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        
        self.ros_thread = ROSThread(self.node)
        self.ros_thread.start()
        
        self.waypoints = {
            'Waypoint 1': (-2.772, -2.717, 0.0240),
            'Waypoint 2': (0.558, 4.200, -1.763),
            'Waypoint 3': (-5.317, 2.428, -0.403)
        }
        
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle('TurtleBot4 Waypoint Navigator')
        self.setGeometry(100, 100, 400, 300)
        
        main_widget = QWidget()
        layout = QVBoxLayout()
        
        title = QLabel('TurtleBot4 Navigation Control')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 18px; font-weight: bold; color: #E63946;')
        layout.addWidget(title)
        
        self.status_label = QLabel('Status: Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 14px; color: #1D3557;')
        layout.addWidget(self.status_label)
        
        for waypoint_name in self.waypoints.keys():
            btn = QPushButton(waypoint_name)
            btn.setStyleSheet('''
                QPushButton {
                    background-color: #E63946;
                    color: white;
                    font-size: 16px;
                    padding: 15px;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #D62839;
                }
                QPushButton:pressed {
                    background-color: #B91F2B;
                }
            ''')
            btn.clicked.connect(lambda checked, name=waypoint_name: self.navigate_to_waypoint(name))
            layout.addWidget(btn)
        
        cancel_btn = QPushButton('Cancel Navigation')
        cancel_btn.setStyleSheet('''
            QPushButton {
                background-color: #1D3557;
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #0F1F35;
            }
        ''')
        cancel_btn.clicked.connect(self.cancel_navigation)
        layout.addWidget(cancel_btn)
        
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)
    
    def play_success_sound(self):
        msg = AudioNoteVector()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        note1 = AudioNote()
        note1.frequency = 523
        note1.max_runtime.sec = 0
        note1.max_runtime.nanosec = 200000000
        
        note2 = AudioNote()
        note2.frequency = 659
        note2.max_runtime.sec = 0
        note2.max_runtime.nanosec = 200000000
        
        note3 = AudioNote()
        note3.frequency = 784
        note3.max_runtime.sec = 0
        note3.max_runtime.nanosec = 300000000
        
        msg.notes = [note1, note2, note3]
        
        for _ in range(3):
            self.audio_pub.publish(msg)
        
        print("turtlebot playing success sound!")
        
    def navigate_to_waypoint(self, waypoint_name):
        x, y, yaw = self.waypoints[waypoint_name]
        self.status_label.setText(f'Status: Navigating to {waypoint_name}...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_label.setText('Status: Goal rejected!')
            return
        
        self.status_label.setText('Status: Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_label.setText('Status: Navigation succeeded!')
            self.play_success_sound()
        else:
            self.status_label.setText(f'Status: Navigation failed (status: {status})')
            
    def cancel_navigation(self):
        self.status_label.setText('Status: Navigation cancelled')
        
    def closeEvent(self, event):
        self.ros_thread.quit()
        self.ros_thread.wait()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = TurtleBot4GUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
