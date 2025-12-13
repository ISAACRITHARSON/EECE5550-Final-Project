# TurtleBot4 Waypoint Navigator with GUI

A PyQt5-based GUI application for navigating a TurtleBot4 to predefined waypoints using Nav2. Built for EECE 5550 Mobile Robotics at Northeastern University.

## Overview

This project provides two ROS2 nodes:

| File | Description |
|------|-------------|
| `waypoint_navigator_gui_sound.py` | PyQt5 GUI for one-click waypoint navigation with audio feedback on success |
| `waypoint_viewer.py` | Utility node that prints the robot's current map position from AMCL |

## Prerequisites

- ROS2 Jazzy
- Ubuntu 24.04
- TurtleBot4 packages
- Nav2 stack
- PyQt5
```bash
sudo apt install python3-pyqt5
```

## Setup

### 1. Launch Localization
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=216.yaml
```

### 2. Launch Nav2
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

### 3. Launch RViz
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

### 4. Run the GUI
```bash
python3 waypoint_navigator_gui_sound.py
```

## Finding New Waypoints

To record positions for new waypoints:
```bash
python3 waypoint_viewer.py
```

Drive the robot to your desired location and note the X, Y, and Yaw values printed in the terminal. Then update the `waypoints` dictionary in `waypoint_navigator_gui_sound.py`:
```python
self.waypoints = {
    'Waypoint 1': (-2.772, -2.717, 0.0240),
    'Waypoint 2': (0.558, 4.200, -1.763),
    'Waypoint 3': (-5.317, 2.428, -0.403)
}
```

## Project Structure
```
.
├── waypoint_navigator_gui_sound.py   # Main GUI application
├── waypoint_viewer.py                # Position tracking utility
└── README.md
```

## Topics Used

| Topic | Type | Purpose |
|-------|------|---------|
| `/navigate_to_pose` | `NavigateToPose` (Action) | Send navigation goals to Nav2 |
| `/cmd_audio` | `AudioNoteVector` | Play sounds on TurtleBot4 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | Get robot position on map |

## Author

Mohammed Abdul Rahman  
MS Robotics, Northeastern University  
EECE 5550 - Mobile Robotics

- TurtleBot4 documentation and community
- Nav2 project
