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

### Optional:
- Unified Launch System: Runs all modules together with debug toggles.

---

## 🔁 Interfaces

| Topic           | Type                  | Description                        |
|----------------|-----------------------|------------------------------------|
| `/gesture_cmds` | `std_msgs/String`     | Input gesture: `"stop"`, `"left"`, etc. |
| `/cmd_vel`      | `geometry_msgs/Twist` | Robot movement commands            |

---

## ✅ Current Status

| Module              | Status    | Verified With          |
| ------------------- | --------- | ---------------------- |
| Gesture Recognition | ✅ Working | MediaPipe + Webcam     |
| Motion Translator   | ✅ Working | Gesture + Manual Input |
| Robot Control       | ✅ Working | RViz Simulation        |
| Unified Launch      | ✅ Working | Launches full pipeline |

---

## 🚀 How to Run (Module-wise)

### 🧪 Individual Module Testing
```bash
# 1. Gesture Node
ros2 run gesture_recognition gesture_node_v1

# 2. Motion Translator
ros2 run motion_translator motion_translator_node

# 3. Robot Control + RViz
ros2 launch robot_control robot_control.launch.py
```
### 🧩 Full System Launch
```
ros2 launch unified_launch gesture_controlled_robot.launch.py
```
