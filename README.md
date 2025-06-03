# Gesture-Controlled Robot 🤖✋

A modular ROS2 system that allows a robot to be controlled using static hand gestures from a webcam.

---

## 📦 Modules

### 1. Gesture Recognition (MediaPipe + OpenCV)
- Detects static hand gestures (1–5 fingers)
- Publishes gesture labels to `/gesture_cmds`

### 2. Motion Translator
- Converts gesture labels to velocity commands (`/cmd_vel`)
- Smooths speed changes based on gesture duration

### 3. Robot Control
- Subscribes to `/cmd_vel`
- Simulates robot movement and displays pose in RViz

---

## 🔁 Interfaces

| Topic           | Type                  | Description                        |
|----------------|-----------------------|------------------------------------|
| `/gesture_cmds` | `std_msgs/String`     | Input gesture: `"stop"`, `"left"`, etc. |
| `/cmd_vel`      | `geometry_msgs/Twist` | Robot movement commands            |

---

## ✅ Current Status

| Module               | Status     | Tested With        |
|----------------------|------------|--------------------|
| Gesture Recognition  | ✅ Working | MediaPipe, webcam  |
| Motion Translator    | ✅ Working | CLI & gesture input |
| Robot Control        | ✅ Working | RViz simulation    |

---

## 🚀 How to Run (Module-wise)

```bash
# 1. Gesture Node
ros2 run gesture_recognition gesture_node_v1

# 2. Motion Translator
ros2 run motion_translator motion_translator_node

# 3. Robot Control + RViz
ros2 launch robot_control robot_control.launch.py
