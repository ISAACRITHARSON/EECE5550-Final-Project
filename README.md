# IDRIS - Indoor Delivery Robot Intelligence System

**EECE 5550 Mobile Robotics Final Project**  
**Fall 2025**

**Team:** IDRIS  
**Institution:** Northeastern University, Seattle  
**Instructor:** Dr. Xian Li

---

## Project Overview

IDRIS is a mobile manipulation platform combining TurtleBot4 autonomous navigation with Pincher X100 robotic arm control for indoor delivery tasks. The system integrates:

- **Autonomous Navigation:** SLAM mapping, AMCL localization, Nav2 path planning
- **Vision-Based Manipulation:** Mediapipe hand tracking for intuitive puppet control
- **Cross-Distribution Integration:** ROS2 Jazzy (TurtleBot4) + Humble (Pincher X100)
- **Intuitive Interfaces:** PyQt5 GUI with audio feedback, gesture-based teleoperation

---

## Achievement Levels

### Level 1: Foundation ✅ COMPLETE
- TurtleBot4 autonomous navigation with SLAM and AMCL
- Pincher X100 ROS2 integration and basic manipulation
- Mediapipe hand tracking implementation
- Waypoint recording and navigation workflow

### Level 2: Integration ✅ COMPLETE
- Complete camera-arm integration with puppet control
- Full system integration with waypoint-based delivery
- PyQt5 GUI with audio feedback
- Thread-safe multi-node architecture
- Real-time gesture recognition

### Level 3: Autonomy ✅ COMPLETE
- Cross-platform TCP bridge for automated coordination
- Navigation-to-manipulation handoff without manual intervention
- Fully autonomous delivery workflow

---

## Repository Structure
```
├── Team_Contract.pdf          # Signed team agreement with Level 1/2/3 goals
├── README.md                  # This file
├── demo/                      # Demonstration videos (external links)
│   └── README.md
├── src/                       # Source code
│   ├── navigation/            # Waypoint tools and GUI
│   │   ├── waypoint_navigator_gui_sound.py
│   │   ├── waypoint_viewer.py
│   │   └── README.md
│   ├── manipulation/          # Puppet control system
│   │   ├── puppet_control.py
│   │   └── README.md
│   └── launch/                # ROS2 launch files
│       ├── mapping.launch.py
│       ├── navigation.launch.py
│       └── README.md
├── data/                      # Maps and experimental data
│   └── maps/
│       ├── Map1.pgm
│       ├── Map1.yaml
│       └── README.md
├── results/                   # Screenshots and outputs
│   ├── puppet_control_screenshot.png
│   ├── gui_screenshot.png
│   └── README.md
└── level3/ros2 crossplatform bridge                    # Cross-platform bridge (Level 3)
    ├── README.md
    ├── bridge_coordinator.py
    ├── navgoalarrival.py
    ├── pincher_pickplace_controller.py
    └── tb4_drive_controller.py
```

---

## Installation and Setup

### Prerequisites

**TurtleBot4 Side (ROS2 Jazzy):**
- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.10+
- PyQt5: `pip install PyQt5`
- TurtleBot4 packages: `sudo apt install ros-jazzy-turtlebot4-navigation`

**Pincher X100 Side (ROS2 Humble in Docker):**
- Ubuntu 22.04
- ROS2 Humble
- Interbotix ROS2 SDK
- Python 3.10+
- OpenCV: `pip install opencv-python`
- Mediapipe: `pip install mediapipe`
- NumPy: `pip install numpy`

---

## How to Launch and Run

### Step 1: SLAM Mapping

Following the [TurtleBot4 mapping tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html):

**Terminal 1: Launch SLAM**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=6
ros2 launch turtlebot4_navigation slam.launch.py
```

**Terminal 2: Launch Rviz2 (on desktop PC)**
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

**Terminal 3: Teleoperate Robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the robot around the environment for 3-5 minutes.

**Save Map:**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'Map1'"
```

Map files (`Map1.pgm`, `Map1.yaml`) will be saved to current directory. Move them to `data/maps/`.

---

### Step 2: Waypoint Recording

**Terminal 1: Launch Localization**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_navigation localization.launch.py map:=data/maps/Map1.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
ros2 launch turtlebot4_viz view_navigation.launch.py
```

**Terminal 2: Run Position Tracker**
```bash
cd src/navigation/
python3 waypoint_viewer.py
```

**Workflow:**
1. Manually drive TurtleBot4 to desired waypoint locations
2. Copy (x, y, yaw) coordinates from terminal output
3. Paste into `waypoint_navigator_gui_sound.py` waypoint dictionary
4. Repeat for all desired waypoints

---

### Step 3: Autonomous Navigation

**Terminal 1: Launch Nav2**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_navigation localization.launch.py map:=data/maps/Map1.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
ros2 launch turtlebot4_viz view_navigation.launch.py
```

**Terminal 2: Launch Waypoint GUI**
```bash
cd src/navigation/
python3 waypoint_navigator_gui_sound.py
```

Click waypoint buttons for autonomous navigation. Audio feedback (C-E-G chord) plays on successful arrival.

---

### Step 4: Puppet Control

**In Docker Container (Humble):**
```bash
docker exec -it <container_name> bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

cd /ros2_ws/src/manipulation/
python3 puppet_control.py
```

**Controls:**
- Move hand in front of camera → arm follows
- Pinch fingers together → gripper closes
- Open hand → gripper opens
- `r` - Reset to home position
- `+/-` - Adjust smoothing factor
- `q` - Quit

---

### Step 5: Automated Delivery (Level 3)

See `level3/ros2 crossplatform bridge/README.md` for complete bridge setup and automated delivery workflow.

---

## Demo Videos

**Note:** Videos exceed 100MB and are hosted on Google Drive.

### 1. Waypoint Navigation Demo
**Link:** https://drive.google.com/file/d/1EHIcZNjYdfdbiXKw84AxZ1VH9d1XXqjn/view?resourcekey

Shows GUI operation, autonomous navigation, and audio feedback.

### 2. Puppet Control Demo
**Link:** https://drive.google.com/file/d/1ae89fkb-uN1fgXB-kwMaPfdTWgGLwCvM/view?resourcekey

Shows hand tracking, arm following, pinch gesture, and object manipulation.

### 3. Automated Delivery (Level 3)
**Link:** https://drive.google.com/file/d/1S99l4ovkFCzmrrP61BnAghYmLGV5P2i0/view?usp=sharing

Shows complete automated delivery workflow with cross-platform bridge coordination.

---

## Technical Highlights

### Recursive Bayesian-Inspired Filtering
Puppet control implements exponential smoothing following predict-update structure from Bayesian state estimation:
- Prediction: $\hat{\theta}_{k|k-1} = \hat{\theta}_{k-1}$
- Update: $\hat{\theta}_k = 0.7 \cdot \hat{\theta}_{k-1} + 0.3 \cdot z_k$

### Cross-Distribution Integration (Level 3)
TCP bridge enables Jazzy ↔ Humble communication, enabling automated delivery without manual coordination.

### Multi-Threaded Architecture
Puppet control uses dual threads for concurrent camera capture and ROS2 callback processing with thread-safe gripper state management.

---

## How to Reproduce

### Mapping
1. Follow TurtleBot4 mapping tutorial (link above)
2. Drive robot for 3-5 minutes in controlled environment
3. Save map as Map1
4. Files generated: Map1.pgm, Map1.yaml

### Navigation
1. Launch localization with Map1
2. Use waypoint_viewer to record coordinates
3. Update GUI waypoint dictionary
4. Launch GUI for autonomous navigation

### Puppet Control
1. Launch Pincher X100 in Docker (Humble)
2. Run puppet_control.py
3. Control arm with hand gestures
4. Pinch to grasp objects

### Level 3 Automation - in progress
1. Start bridge coordinator (server and client)
2. Launch navigation goal monitor
3. Launch pick-place controller
4. Send navigation goal → manipulation executes automatically

---

## Technical Stack

**Navigation:**
- SLAM: slam_toolbox (Karto scan matcher)
- Localization: AMCL (particle filter)
- Planning: Nav2 (Dijkstra + DWA)
- Visualization: Foxglove Bridge

**Manipulation:**
- Arm Control: Interbotix SDK
- Motion Planning: MoveIt2
- Vision: Google Mediapipe
- GUI: PyQt5

**Integration:**
- Cross-Platform: TCP sockets with JSON
- ROS2 Distributions: Jazzy (native) + Humble (Docker)

---

## Team Contributions

**Navigation and Localization:**
- AMCL configuration and parameter tuning
- Waypoint navigation GUI with audio feedback 
- Position tracker tool 
- Nav2 integration testing

**Manipulation Control:**
- Complete puppet control system 
- Recursive state estimation with filtering
- Multi-threaded architecture
- MoveIt2 integration

**SLAM Mapping:**
- slam_toolbox deployment and operation
- Map generation through manual exploration
- Foxglove visualization setup

**Cross-Platform Bridge (Level 3):**
- TCP bridge implementation 
- Automated coordination controllers
- Navigation-manipulation handoff
- Based on original concept by Dr. Xian Li

**Testing and Documentation:**
- System integration testing
- Workflow validation
- Report and presentation development

---

## License

MIT License

---

## Acknowledgments

- Dr. Xian Li for project guidance and original bridge concept
- Northeastern University Seattle for hardware access
- TurtleBot4 documentation and tutorials

---

## Contact

Repository: https://github.com/Jerryzhang258/EECE5550-Final-Project

For questions, contact team members via Northeastern University email.
