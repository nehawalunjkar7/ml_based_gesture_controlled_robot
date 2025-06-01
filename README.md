# Project Status
## Implemented So Far:
### 1. Robot Control Module

    Node: robot_node.py

    Subscribes to /cmd_vel (geometry_msgs/Twist)

    Simulates robot motion and publishes visualization markers to RViz

    Launches with robot_control.launch.py

### 2. Gesture Recognition Module (v1)

    Node: gesture_node_v1.py

    Uses webcam + MediaPipe to detect simple static hand gestures

    Maps gestures to commands:

        ✋ (5 fingers) → "stop"

        ☝️ (1 finger) → "left"

        ✌️ (2 fingers) → "right"

        🤟 (3 fingers) → "forward"

    Publishes recognized gestures to /gesture_cmds (std_msgs/String)

# Project Structure

    gesture_controlled_robot/
    ├── gesture_recognition/        # Gesture detection node
    ├── robot_control/              # Robot simulator + RViz
    └── (motion_translator/)        # (Planned) Gesture → velocity translator

# How to Run the Project
## Prerequisites

    ROS 2 Humble (or compatible version)

    Python 3.8+

    MediaPipe (pip install mediapipe)

    OpenCV (pip install opencv-python)

# Steps to Launch Modules

## Step 1: Build your workspace
```
cd ~/ros2_ws2
colcon build --packages-select gesture_recognition robot_control
source install/setup.bash
```
## Step 2: Start the robot simulator + RViz
```
ros2 launch robot_control robot_control.launch.py
```
## Step 3: Run the gesture detection node
```
ros2 launch gesture_recognition gesture_recognition.launch.py
```
### You can monitor the gesture commands:
```
ros2 topic echo /gesture_cmds
```
  📝 Note: You’ll need to manually test the full pipeline (gesture → /cmd_vel) after implementing the motion_translator.

# Next Planned Modules

    motion_translator: Subscribes to /gesture_cmds, outputs /cmd_vel

    Combined launch file to run all nodes together
