# IDRIS Demonstration Videos

**Note:** Video files exceed 100MB GitHub limit and are hosted on Google Drive.

---

## Video 1: Waypoint Navigation with GUI

**Link:** https://drive.google.com/file/d/1EHIcZNjYdfdbiXKw84AxZ1VH9d1XXqjn/view?resourcekey

**Description:**
Demonstrates autonomous waypoint navigation with PyQt5 graphical interface and integrated audio feedback system.

**Content:**
- GUI launch and waypoint selection
- TurtleBot4 autonomous navigation
- Real-time status updates
- Audio feedback on goal completion (C-E-G major chord)

**Technical Features:**
- NavigateToPose action client with async callbacks
- QThread integration for non-blocking ROS2
- AMCL localization with Nav2 path planning
- AudioNoteVector message implementation

---

## Video 2: Puppet Control Demonstration

**Link:** https://drive.google.com/file/d/1ae89fkb-uN1fgXB-kwMaPfdTWgGLwCvM/view?resourcekey

**Description:**
Demonstrates Mediapipe-based puppet control system for intuitive robotic arm teleoperation.

**Content:**
- Hand tracking initialization with landmark detection
- Hand position mapping to arm joints
- Real-time arm following behavior
- Pinch gesture triggering gripper closure
- Object manipulation demonstration

**Technical Features:**
- 30fps hand tracking with Google Mediapipe
- Recursive state estimation (exponential smoothing, α=0.7)
- 5-frame moving average for pinch detection
- Multi-threaded architecture (camera + ROS2)
- Thread-safe gripper state machine
- MoveIt2 trajectory execution

---

## Video 3: Automated Delivery Workflow (Level 3)

**Link:** https://drive.google.com/file/d/1S99l4ovkFCzmrrP61BnAghYmLGV5P2i0/view?usp=sharing

**Description:**
Demonstrates fully automated delivery sequence using cross-platform bridge for ROS2 Jazzy-Humble coordination.

**Content:**
- Cross-platform bridge initialization
- TurtleBot4 autonomous navigation to delivery station
- Automatic goal completion detection
- Signal forwarding through TCP bridge
- Automated pick-and-place execution
- Complete end-to-end delivery workflow

**Technical Features:**
- TCP socket communication (port 50505)
- JSON message serialization
- Navigation goal status monitoring
- Automated manipulation triggering
- Level 3 autonomous operation without manual coordination

---

## Access Instructions

All videos are shared via Google Drive with "Anyone with the link can view" permissions.

If you encounter access issues, contact the team via Northeastern University email or open an issue on this repository.
