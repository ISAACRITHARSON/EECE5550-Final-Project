# ROS2 Cross-Platform Bridge (Jazzy ↔ Humble)

TCP-based communication bridge for seamless message passing between ROS2 Jazzy and ROS2 Humble distros. Enables multi-robot systems where different robots run on incompatible ROS2 distributions.

## Overview

This bridge solves the DDS compatibility issues between ROS2 distros by using plain TCP sockets to relay messages. Perfect for projects like:
- TurtleBot4 (Jazzy) + Pincher X100 (Humble) coordination
- Cross-distro multi-robot systems
- Docker containerized ROS2 applications
- Mixed hardware deployments

**Architecture:**
```
TurtleBot4 (Jazzy, Domain 6) 
    ↓ ROS2 Topics
Bridge Server (Jazzy)
    ↓ TCP Socket
Bridge Client (Docker/Humble, Domain 0)
    ↓ ROS2 Topics
PincherX100 (Humble)
```

## Features

-  Distro-agnostic TCP communication
-  Supports String, Odometry, PoseStamped, Twist messages
-  Bi-directional message forwarding
-  Lightweight (<1ms LAN latency)
-  Easy to extend for custom message types

## Prerequisites

**Host Machine (Jazzy):**
- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.10+

**Docker Container (Humble):**
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## Installation

### 1. Clone Repository
```bash
cd ~/ros2_ws
git clone <your-repo-url>
cd ros2_cross_platform_bridge
```

### 2. Setup on Host (Jazzy)
```bash
# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Copy bridge script
cp bridge_coordinator.py ~/ros2_ws/
chmod +x ~/ros2_ws/bridge_coordinator.py
```

### 3. Setup in Docker (Humble)
```bash
# Enter Docker container
docker exec -it <container_name> bash

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Copy bridge script
cp bridge_coordinator.py /ros2_ws/
chmod +x /ros2_ws/bridge_coordinator.py
```

## Configuration

### Network Setup

For localhost Docker communication, use:
```python
PEER_IP = "127.0.0.1"  # Both host and Docker
```

For separate machines:
```python
# On Server (Jazzy)
PEER_IP = "192.168.1.X"  # IP of Client machine

# On Client (Humble)
PEER_IP = "192.168.1.Y"  # IP of Server machine
```

### Discovery Server (TurtleBot4)

If using TurtleBot4 with discovery server:
```bash
# Uncomment in ~/.bashrc
source /etc/turtlebot4_discovery/setup.bash
export ROS_DOMAIN_ID=6
```

### Topics Configuration

Edit `TOPIC_MAP` in `bridge_coordinator.py`:
```python
TOPIC_MAP = [
    ("/delivery_goal", "/delivery_goal_remote", String, "both"),
    ("/odom", "/odom_remote", Odometry, "both"),
    # Add your custom topics here
]
```

## 🎮 Usage

### Basic Setup (Localhost Docker)

**Terminal 1 - Start Bridge Server (Jazzy/Host):**
```bash
source ~/.bashrc
export ROS_DOMAIN_ID=6  # Or your domain
cd ~/ros2_ws
python3 bridge_coordinator.py --server
```

Expected output:
```
abdul rahman
ROS2 libraries imported
Waiting for connection on 0.0.0.0:50505
```

**Terminal 2 - Start Bridge Client (Humble/Docker):**
```bash
docker exec -it <container_name> bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
cd /ros2_ws
python3 bridge_coordinator.py
```

Expected output:
```
abdul rahman
ROS2 libraries imported
Connecting to 127.0.0.1:50505
TCP Connected
Subscribed to: /delivery_goal, /odom
Publishing to: /delivery_goal_remote, /odom_remote
Bridge operational!
```

## Testing

### Test 1: String Message (Jazzy → Humble)

**On Jazzy (Terminal 3):**
```bash
export ROS_DOMAIN_ID=6
source /opt/ros/jazzy/setup.bash
ros2 topic pub /delivery_goal std_msgs/msg/String "data: 'test_message'" --once
```

**On Humble (Terminal 4):**
```bash
docker exec -it <container_name> bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /delivery_goal_remote
```

You should see: `data: test_message`

### Test 2: Odometry (TurtleBot4 → Humble)

**Check odometry flowing:**
```bash
# In Humble Docker
ros2 topic hz /odom_remote
```

Should show ~30-60 Hz if TurtleBot4 is publishing.

### Test 3: Bi-directional Communication

**Humble → Jazzy:**
```bash
# In Docker
ros2 topic pub /delivery_goal std_msgs/msg/String "data: 'from_humble'" --once

# On Host
ros2 topic echo /delivery_goal_remote
```

## Troubleshooting

### Issue: "Waiting for at least 1 matching subscription(s)..."

**Solution:** Bridge isn't running or not on correct domain.
```bash
# Check if bridge node exists
ros2 node list | grep bridge_coordinator

# Verify domain ID matches
echo $ROS_DOMAIN_ID
```

### Issue: Docker Discovery Server Conflict

**Solution:** TurtleBot4 discovery server can block normal DDS. Comment out:
```bash
# In ~/.bashrc
# source /etc/turtlebot4_discovery/setup.bash
```

### Issue: Connection Refused

**Solution:** Check firewall and network connectivity.
```bash
# Allow port
sudo ufw allow 50505/tcp

# Test connectivity
ping <peer_ip>

# Check if port is already in use
netstat -an | grep 50505
```

### Issue: Bridge Connects but No Topics

**Solution:** ROS not sourced before starting bridge.
```bash
# Always source ROS BEFORE running bridge
source /opt/ros/jazzy/setup.bash  # or humble
export ROS_DOMAIN_ID=X
python3 bridge_coordinator.py --server
```

## Project Structure
```
ros2_cross_platform_bridge/
├── bridge_coordinator.py          # Main bridge script
├── README.md                       # This file
└── examples/
    ├── tb4_nav_controller.py      # Nav2 goal monitoring
    ├── simple_tb4_controller.py   # Simple drive controller
    └── pincher_pickplace_controller.py  # Arm control example
```

## Example Applications

### TurtleBot4 Navigation → Pincher Pick-and-Place

Complete example where TurtleBot4 navigates to a goal and signals Pincher arm to execute pick-and-place:

**Setup:**
1. Start bridge (both server and client)
2. Launch MoveIt for Pincher
3. Start Nav2 on TurtleBot4
4. Run nav controller to monitor Nav2 goals
5. Run Pincher pick-and-place controller

When TB4 reaches goal → Arm executes sequence automatically!


## 🤝 Contributing

Contributions welcome! Areas for improvement:
- Additional message type support
- Heartbeat/reconnection logic
- Configuration file support
- Launch file integration

## 👨‍💻 Author

**Abdul Rahman**  
MS Robotics @ Northeastern University Seattle  
Robotics Special Interest Group

## 🔗 Related Projects

- [TurtleBot4 Documentation](https://turtlebot.github.io/turtlebot4-user-manual/)
- [Interbotix ROS2 Packages](https://github.com/Interbotix/interbotix_ros_manipulators)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/)

## 📧 Support

For issues or questions, please open an issue on GitHub or contact via Northeastern University email.

---

**Note:** This bridge is designed for development and research purposes. For production deployments, consider additional security measures and error handling.
